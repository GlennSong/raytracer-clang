#include "xr_surface_system.h"

#include <cstdlib>

#include "../../log.h"
#include "../mesh_builder.h"

namespace engine {

namespace {

// MeshHandle <-> the ledger's opaque token. The ledger stays renderer-free
// (and therefore host-testable) by never seeing a MeshHandle; a Handle is two
// uint32s, so it packs losslessly. generation != 0 for any valid handle, so a
// packed token is never the ledger's "no mesh" zero.
uint64_t packMesh(MeshHandle handle) {
    return (static_cast<uint64_t>(handle.generation) << 32) | handle.index;
}
MeshHandle unpackMesh(uint64_t token) {
    MeshHandle handle;
    handle.index = static_cast<uint32_t>(token & 0xffffffffu);
    handle.generation = static_cast<uint32_t>(token >> 32);
    return handle;
}

// An anchor's geometry as the renderer wants it: classification tint in the
// vertex color (the debug material's albedo stays white), flat normals
// accumulated from faces when the runtime sent none.
RenderMesh buildSurfaceMesh(const XrSurfaceUpdate& update) {
    RenderMesh mesh;
    const Vec3 tint = xrSurfaceClassColor(update.cls);
    const bool haveNormals = update.normals.size() == update.positions.size();

    mesh.vertices.reserve(update.positions.size());
    for (size_t i = 0; i < update.positions.size(); i++) {
        Vertex v(update.positions[i],
                 haveNormals ? update.normals[i] : Vec3(0, 0, 0));
        v.color = tint;
        mesh.vertices.push_back(v);
    }
    mesh.indices = update.indices;

    if (!haveNormals) {
        for (size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
            Vertex& a = mesh.vertices[mesh.indices[t]];
            Vertex& b = mesh.vertices[mesh.indices[t + 1]];
            Vertex& c = mesh.vertices[mesh.indices[t + 2]];
            const Vec3 n = cross(b.position - a.position,
                                 c.position - a.position);
            a.normal += n;
            b.normal += n;
            c.normal += n;
        }
        for (Vertex& v : mesh.vertices) {
            const Real len = v.normal.length();
            v.normal = (len > 1e-12) ? v.normal / len : Vec3(0, 1, 0);
        }
    }
    return mesh;
}

Real worldScaleOf(FrameContext& ctx) {
    return (ctx.renderer.xrWorldScale > 0.01f)
        ? static_cast<Real>(ctx.renderer.xrWorldScale) : 1.0;
}

// Reach for gaze placement, real metres. Generous enough for a far wall of a
// normal room, short enough that pinching at the far virtual landscape still
// means teleport.
constexpr Real kPlaceReachMetres = 3.0;
constexpr size_t kMaxMarkers = 64;

}  // namespace

XrSurfaceLedger::MeshOps XrSurfaceSystem::meshOps(FrameContext& ctx) {
    return {
        [&ctx](const XrSurfaceUpdate& update) {
            return packMesh(ctx.renderer.uploadMesh(buildSurfaceMesh(update)));
        },
        [&ctx](uint64_t token) { ctx.renderer.removeMesh(unpackMesh(token)); },
    };
}

void XrSurfaceSystem::onStart(FrameContext&) {
    const char* env = std::getenv("RT_XR_SURFACES");
    visible_ = !(env && env[0] == '0');
}

void XrSurfaceSystem::ingest(FrameContext& ctx) {
    drainScratch_.clear();
    ctx.renderer.xrSurfaceStore()->drain(drainScratch_);
    if (drainScratch_.empty()) return;

    const Real scale = worldScaleOf(ctx);
    for (const XrSurfaceUpdate& u : drainScratch_) {
        if (u.op == XrSurfaceUpdate::Op::Removed) {
            outlines_.erase(u.anchorId);
            planes_.erase(u.anchorId);
            // Orphan this anchor's markers at their last composed world pose
            // rather than deleting them: a marker that silently vanished
            // would read as "placement is broken", when the truth is "the
            // runtime merged/replaced that plane" — worth SEEING.
            auto ledgerIt = ledger_.surfaces().find(u.anchorId);
            if (ledgerIt != ledger_.surfaces().end()) {
                const Mat4 world = xrSurfaceWorldTransform(
                    ctx.xr.originBase, scale,
                    ledgerIt->second.originFromAnchor);
                int orphaned = 0;
                for (Marker& m : markers_) {
                    if (m.anchorId != u.anchorId) continue;
                    m.frozenWorld =
                        world * Mat4::translate(m.anchorPoint.x, m.anchorPoint.y,
                                                m.anchorPoint.z);
                    m.anchorId = 0;
                    orphaned++;
                }
                if (orphaned)
                    LOG_INFO << "[xr] plane removed; " << orphaned
                             << " marker(s) orphaned in place";
            }
            continue;
        }
        if (u.cls == XrSurfaceClass::Mesh || u.indices.empty()) continue;

        // Plane bookkeeping: outline for drawing, triangles for placement
        // raycasts, extent for the dimensions readout.
        auto& segments = outlines_[u.anchorId];
        segments.clear();
        for (const auto& [ia, ib] : xrSurfaceBoundaryEdges(u.indices))
            segments.emplace_back(u.positions[ia], u.positions[ib]);

        PlaneGeometry& plane = planes_[u.anchorId];
        const Vec3 oldExtent = plane.extent;
        plane.positions = u.positions;
        plane.indices = u.indices;
        plane.extent = xrSurfaceExtent(u.positions).size();
        // The dimensions display, by log line (the engine draws no 3D text):
        // announced when a plane appears and whenever refinement moves a
        // dimension by more than 5 cm, so the console shows sizes settling
        // without spamming every refinement tick.
        if (std::abs(plane.extent.x - oldExtent.x) > 0.05 ||
            std::abs(plane.extent.z - oldExtent.z) > 0.05) {
            LOG_INFO("[xr] plane %s %.2fm x %.2fm (id %llx)",
                     xrSurfaceClassName(u.cls), plane.extent.x, plane.extent.z,
                     static_cast<unsigned long long>(u.anchorId));
        }
    }
    ledger_.apply(drainScratch_, meshOps(ctx));
}

void XrSurfaceSystem::placeOnPinch(FrameContext& ctx) {
    // Quick pinch, same window PlayerSystem calls a teleport (< 0.8 s hold).
    if (!ctx.xr.pinchEnded || ctx.xr.pinchHeldSeconds >= 0.8) return;
    if (!ctx.xr.gazeValid || !ctx.xr.originBaseValid) return;

    const Real scale = worldScaleOf(ctx);
    const Vec3 rayOrigin = ctx.xr.originBase + ctx.xr.gazeOrigin;
    const Vec3 rayDir = ctx.xr.gazeDir;

    // Nearest plane hit within reach. t comes back in real metres for every
    // plane (the anchor spaces are unscaled), so hits compare directly.
    bool found = false;
    Real bestT = kPlaceReachMetres;
    uint64_t bestAnchor = 0;
    Vec3 bestPoint;
    for (const auto& [anchorId, plane] : planes_) {
        auto ledgerIt = ledger_.surfaces().find(anchorId);
        if (ledgerIt == ledger_.surfaces().end()) continue;
        const Mat4 world = xrSurfaceWorldTransform(
            ctx.xr.originBase, scale, ledgerIt->second.originFromAnchor);
        const Mat4 inv = world.inverse();
        Vec3 o = inv.transformPoint(rayOrigin);
        Vec3 d = inv.transformDirection(rayDir);
        const Real dLen = d.length();
        if (dLen < 1e-12) continue;
        d /= dLen;
        Real t = 0;
        if (!xrRaycastTriangles(o, d, plane.positions, plane.indices, t))
            continue;
        if (t < bestT) {
            found = true;
            bestT = t;
            bestAnchor = anchorId;
            bestPoint = o + d * t;
        }
    }
    if (!found) return;   // pinch stays; PlayerSystem may teleport with it

    Marker marker;
    marker.anchorId = bestAnchor;
    marker.anchorPoint = bestPoint;
    if (markers_.size() >= kMaxMarkers) markers_.erase(markers_.begin());
    markers_.push_back(marker);

    // CONSUME the pinch so this gesture is a placement, not also a teleport.
    // ctx.xr is the frame's shared state and this system runs before
    // PlayerSystem precisely so this write is seen there.
    ctx.xr.pinchEnded = false;

    const auto& cls = ledger_.surfaces().at(bestAnchor).cls;
    LOG_INFO("[xr] placed marker %zu on %s at %.2fm (anchor %llx)",
             markers_.size(), xrSurfaceClassName(cls), bestT,
             static_cast<unsigned long long>(bestAnchor));
}

void XrSurfaceSystem::update(FrameContext& ctx) {
    XrSurfaceStore* store = ctx.renderer.xrSurfaceStore();
    if (!store) return;

    ingest(ctx);
    if (visible_ && ctx.xr.active) placeOnPinch(ctx);

    // The numeric readout — how surface mapping is verified without seeing the
    // render. Logged when the census changes shape and as a slow heartbeat.
    const auto census = ledger_.census();
    const bool beat = (++frame_ % 450) == 0;   // ~5 s at 90 Hz
    if (census.total != lastLoggedTotal_ || (beat && census.total > 0)) {
        lastLoggedTotal_ = census.total;
        auto line = LOG_INFO;
        line << "[xr] surfaces: total=" << census.total;
        for (int i = 0; i < XR_SURFACE_CLASS_COUNT; i++) {
            if (census.countByClass[i] == 0) continue;
            line << " " << xrSurfaceClassName(static_cast<XrSurfaceClass>(i))
                 << "=" << census.countByClass[i];
        }
        line << " tris=" << census.triangles
             << " markers=" << markers_.size();
        if (census.floorValid) line << " floorY=" << census.floorY << "m";
    }
}

void XrSurfaceSystem::render(FrameContext& ctx) {
    if (!visible_) return;
    if (!ctx.xr.active || !ctx.xr.originBaseValid) return;
    if (ledger_.surfaces().empty() && markers_.empty()) return;

    const Real scale = worldScaleOf(ctx);

    RenderMaterial material;
    material.albedo = Vec3(1, 1, 1);   // tint rides Vertex::color
    material.metallic = 0.0f;
    material.roughness = 1.0f;
    material.flags = RenderMaterial::FLAG_TWO_SIDED;

    for (const auto& [anchorId, entry] : ledger_.surfaces()) {
        const Mat4 world = xrSurfaceWorldTransform(ctx.xr.originBase, scale,
                                                   entry.originFromAnchor);
        if (entry.meshToken)
            ctx.renderer.drawMesh(unpackMesh(entry.meshToken), world, material);

        auto outline = outlines_.find(anchorId);
        if (outline != outlines_.end()) {
            const Vec3 color = xrSurfaceClassColor(entry.cls);
            for (const auto& [a, b] : outline->second)
                ctx.debug.line(world.transformPoint(a), world.transformPoint(b),
                               color);
            // Normal tick: ARKit planes face +Y in anchor space. 10 real cm.
            const Vec3 foot = world.transformPoint(Vec3(0, 0, 0));
            const Vec3 tip = world.transformPoint(Vec3(0, 0.1, 0));
            ctx.debug.line(foot, tip, color);
        }

        // Extent rectangle, dimmed: the measured bounding box the dimension
        // log lines refer to, so "2.40m x 1.95m" has a visible referent.
        auto plane = planes_.find(anchorId);
        if (plane != planes_.end()) {
            const auto extent = xrSurfaceExtent(plane->second.positions);
            if (extent.valid) {
                const Vec3 dim = xrSurfaceClassColor(entry.cls) * 0.45;
                const Vec3 c[4] = {
                    world.transformPoint(Vec3(extent.min.x, 0, extent.min.z)),
                    world.transformPoint(Vec3(extent.max.x, 0, extent.min.z)),
                    world.transformPoint(Vec3(extent.max.x, 0, extent.max.z)),
                    world.transformPoint(Vec3(extent.min.x, 0, extent.max.z)),
                };
                for (int i = 0; i < 4; i++)
                    ctx.debug.line(c[i], c[(i + 1) % 4], dim);
            }
        }
    }

    // Markers: 10 real cm cubes, magenta, riding their plane's anchor — the
    // live probe of anchoring quality. Orphans draw at their frozen pose.
    if (!markers_.empty()) {
        if (!markerMesh_.valid())
            markerMesh_ = ctx.renderer.uploadMesh(
                MeshBuilder::box(Vec3(0.1, 0.1, 0.1)));
        RenderMaterial markerMaterial;
        markerMaterial.albedo = Vec3(0.95, 0.2, 0.9);
        markerMaterial.metallic = 0.0f;
        markerMaterial.roughness = 0.6f;
        const Mat4 lift = Mat4::translate(0, 0.05, 0);   // sit ON the surface
        for (const Marker& m : markers_) {
            Mat4 world;
            if (m.anchorId == 0) {
                world = m.frozenWorld * lift;
            } else {
                auto it = ledger_.surfaces().find(m.anchorId);
                if (it == ledger_.surfaces().end()) continue;
                world = xrSurfaceWorldTransform(ctx.xr.originBase, scale,
                                                it->second.originFromAnchor)
                      * Mat4::translate(m.anchorPoint.x, m.anchorPoint.y,
                                        m.anchorPoint.z)
                      * lift;
            }
            ctx.renderer.drawMesh(markerMesh_, world, markerMaterial);
        }
    }
}

void XrSurfaceSystem::onStop(FrameContext& ctx) {
    ledger_.clear(meshOps(ctx));
    outlines_.clear();
    planes_.clear();
    markers_.clear();
    if (markerMesh_.valid()) {
        ctx.renderer.removeMesh(markerMesh_);
        markerMesh_ = MeshHandle{};
    }
    lastLoggedTotal_ = -1;
}

}  // namespace engine
