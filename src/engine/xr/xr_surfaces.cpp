#include "xr_surfaces.h"

#include <algorithm>
#include <utility>

namespace engine {

const char* xrSurfaceClassName(XrSurfaceClass cls) {
    switch (cls) {
        case XrSurfaceClass::Unknown: return "unknown";
        case XrSurfaceClass::Wall:    return "wall";
        case XrSurfaceClass::Floor:   return "floor";
        case XrSurfaceClass::Ceiling: return "ceiling";
        case XrSurfaceClass::Table:   return "table";
        case XrSurfaceClass::Seat:    return "seat";
        case XrSurfaceClass::Window:  return "window";
        case XrSurfaceClass::Door:    return "door";
        case XrSurfaceClass::Mesh:    return "mesh";
    }
    return "unknown";
}

Vec3 xrSurfaceClassColor(XrSurfaceClass cls) {
    switch (cls) {
        case XrSurfaceClass::Unknown: return Vec3(0.7, 0.7, 0.7);
        case XrSurfaceClass::Wall:    return Vec3(0.8, 0.8, 0.75);
        case XrSurfaceClass::Floor:   return Vec3(0.25, 0.85, 0.35);
        case XrSurfaceClass::Ceiling: return Vec3(0.35, 0.5, 0.9);
        case XrSurfaceClass::Table:   return Vec3(0.95, 0.7, 0.25);
        case XrSurfaceClass::Seat:    return Vec3(0.95, 0.45, 0.2);
        case XrSurfaceClass::Window:  return Vec3(0.3, 0.9, 0.9);
        case XrSurfaceClass::Door:    return Vec3(0.9, 0.4, 0.9);
        case XrSurfaceClass::Mesh:    return Vec3(0.45, 0.45, 0.5);
    }
    return Vec3(0.7, 0.7, 0.7);
}

void XrSurfaceLedger::apply(const std::vector<XrSurfaceUpdate>& updates,
                            const MeshOps& ops) {
    for (const XrSurfaceUpdate& update : updates) {
        auto it = surfaces_.find(update.anchorId);

        if (update.op == XrSurfaceUpdate::Op::Removed) {
            // Unknown or doubly-removed ids are silence, not errors: the
            // runtime's queue and a clear() can race a straggling remove.
            if (it == surfaces_.end()) continue;
            if (it->second.meshToken) ops.destroy(it->second.meshToken);
            surfaces_.erase(it);
            continue;
        }

        // Added and Updated converge on the same upsert: an Updated for an id
        // never seen is an Added that lost the race, and an Added for a live
        // id is a refresh. Either way the old mesh (if any) must be released —
        // uploadMesh has no update-in-place, so a changed anchor is a new mesh
        // and a leaked token per update would bleed GPU memory for as long as
        // the user looks around.
        uint64_t oldToken = (it != surfaces_.end()) ? it->second.meshToken : 0;

        XrSurfaceEntry entry;
        entry.cls = update.cls;
        entry.originFromAnchor = update.originFromAnchor;
        entry.triangleCount = static_cast<uint32_t>(update.indices.size() / 3);
        entry.meshToken = update.indices.empty() ? 0 : ops.create(update);
        surfaces_[update.anchorId] = entry;

        if (oldToken) ops.destroy(oldToken);
    }
}

void XrSurfaceLedger::clear(const MeshOps& ops) {
    for (auto& [id, entry] : surfaces_) {
        (void)id;
        if (entry.meshToken) ops.destroy(entry.meshToken);
    }
    surfaces_.clear();
}

XrSurfaceLedger::Census XrSurfaceLedger::census() const {
    Census c;
    for (const auto& [id, entry] : surfaces_) {
        (void)id;
        c.countByClass[static_cast<int>(entry.cls)]++;
        c.total++;
        c.triangles += entry.triangleCount;
        if (entry.cls == XrSurfaceClass::Floor) {
            const Real y = entry.originFromAnchor.m[1][3];
            if (!c.floorValid || y < c.floorY) c.floorY = y;
            c.floorValid = true;
        }
    }
    return c;
}

std::vector<std::pair<uint32_t, uint32_t>> xrSurfaceBoundaryEdges(
    const std::vector<uint32_t>& indices) {
    // Count each undirected edge; boundary edges are used exactly once. A
    // sorted vector beats a hash map at this size, and the input is small
    // (plane outlines, not the room mesh).
    std::vector<std::pair<uint32_t, uint32_t>> edges;
    const size_t triangles = indices.size() / 3;
    edges.reserve(triangles * 3);
    for (size_t t = 0; t < triangles; t++) {
        for (int e = 0; e < 3; e++) {
            uint32_t a = indices[t * 3 + e];
            uint32_t b = indices[t * 3 + (e + 1) % 3];
            if (a > b) std::swap(a, b);
            edges.emplace_back(a, b);
        }
    }
    std::sort(edges.begin(), edges.end());

    std::vector<std::pair<uint32_t, uint32_t>> boundary;
    for (size_t i = 0; i < edges.size();) {
        size_t j = i + 1;
        while (j < edges.size() && edges[j] == edges[i]) j++;
        if (j - i == 1) boundary.push_back(edges[i]);
        i = j;
    }
    return boundary;
}

XrSurfaceExtent xrSurfaceExtent(const std::vector<Vec3>& positions) {
    XrSurfaceExtent extent;
    for (const Vec3& p : positions) {
        if (!extent.valid) {
            extent.min = extent.max = p;
            extent.valid = true;
            continue;
        }
        extent.min.x = std::min(extent.min.x, p.x);
        extent.min.y = std::min(extent.min.y, p.y);
        extent.min.z = std::min(extent.min.z, p.z);
        extent.max.x = std::max(extent.max.x, p.x);
        extent.max.y = std::max(extent.max.y, p.y);
        extent.max.z = std::max(extent.max.z, p.z);
    }
    return extent;
}

bool xrRaycastTriangles(const Vec3& origin, const Vec3& dir,
                        const std::vector<Vec3>& positions,
                        const std::vector<uint32_t>& indices, Real& tOut) {
    // Moller-Trumbore, two-sided (see header), nearest positive t. Sizes here
    // are plane meshes — tens of triangles — so a linear walk is the right
    // tool; the room mesh deliberately does not come through this path.
    constexpr Real kEpsilon = 1e-9;
    bool hit = false;
    Real best = 0;
    for (size_t t = 0; t + 2 < indices.size(); t += 3) {
        const Vec3& a = positions[indices[t]];
        const Vec3 e1 = positions[indices[t + 1]] - a;
        const Vec3 e2 = positions[indices[t + 2]] - a;
        const Vec3 p = cross(dir, e2);
        const Real det = dot(e1, p);
        if (std::abs(det) < kEpsilon) continue;   // parallel or degenerate
        const Real invDet = 1.0 / det;
        const Vec3 s = origin - a;
        const Real u = dot(s, p) * invDet;
        if (u < 0.0 || u > 1.0) continue;
        const Vec3 q = cross(s, e1);
        const Real v = dot(dir, q) * invDet;
        if (v < 0.0 || u + v > 1.0) continue;
        const Real dist = dot(e2, q) * invDet;
        if (dist <= kEpsilon) continue;            // behind the origin
        if (!hit || dist < best) {
            hit = true;
            best = dist;
        }
    }
    if (hit) tOut = best;
    return hit;
}

void XrColliderPolicy::noteUpdate(uint64_t anchorId, Real now,
                                  Real minInterval) {
    (void)now;   // build time is stamped in drainDue, when the cook happens
    State& state = anchors_[anchorId];
    state.dirty = true;
    if (minInterval > 0) state.interval = minInterval;
}

void XrColliderPolicy::noteRemoved(uint64_t anchorId) {
    anchors_.erase(anchorId);
}

void XrColliderPolicy::invalidateAll() {
    for (auto& [id, state] : anchors_) state.dirty = true;
}

std::vector<uint64_t> XrColliderPolicy::drainDue(Real now, size_t maxCount) {
    std::vector<uint64_t> due;
    for (auto& [id, state] : anchors_) {
        if (due.size() >= maxCount) break;
        if (!state.dirty) continue;
        const Real interval =
            state.interval > 0 ? state.interval : minInterval_;
        if (state.everBuilt && now - state.lastBuild < interval) continue;
        state.dirty = false;
        state.everBuilt = true;
        state.lastBuild = now;
        due.push_back(id);
    }
    return due;
}

size_t XrColliderPolicy::pendingCount() const {
    size_t n = 0;
    for (const auto& [id, state] : anchors_)
        if (state.dirty) n++;
    return n;
}

}  // namespace engine
