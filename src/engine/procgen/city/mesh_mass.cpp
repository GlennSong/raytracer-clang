#include "mesh_mass.h"
#include "implicit.h"          // simplifyLoop
#include "section.h"
#include "shape_grammar.h"   // kCoreTierMin, tierShortSide
#include "../../model_importer.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <mutex>

namespace engine {

namespace {

long long keyOf(const Vec2& p) {
    const long long x = static_cast<long long>(std::llround(p.x * 1000.0));
    const long long z = static_cast<long long>(std::llround(p.y * 1000.0));
    return (x << 32) ^ (z & 0xffffffffLL);
}

}  // namespace

Poly2 resampleLoop(const Poly2& loopIn, int n, const Vec2& from) {
    if (loopIn.size() < 3 || n < 3) return loopIn;
    Poly2 loop = loopIn;
    ensureCCW(loop);
    const std::size_t m = loop.size();
    // THE START is exactly where the ray from the centroid along `from` leaves the loop -- a point on an edge,
    // not the nearest vertex: a start that snapped to a vertex jumped a little from storey to storey as a turning
    // form's vertices passed the bearing, and the mullions above it zig-zagged in a seam up the whole tower.
    const Vec2 c = centroid(loop);
    const Vec2 dir = normalize(from);
    std::size_t start = 0;
    Real startT = 0;      // along edge start -> start + 1
    Real bestDist = -1;   // the farthest crossing wins (the outline of a star-shaped loop has one)
    for (std::size_t i = 0; i < m; ++i) {
        const Vec2 a = loop[i], b = loop[(i + 1) % m];
        const Vec2 e = b - a;
        const Real den = dir.x * e.y - dir.y * e.x;   // cross(dir, e)
        if (std::fabs(den) < 1e-12) continue;
        const Vec2 ac = a - c;
        const Real t = (ac.x * e.y - ac.y * e.x) / den;        // along the ray
        const Real u = (ac.x * dir.y - ac.y * dir.x) / den;    // along the edge
        if (t <= 0 || u < 0 || u >= 1) continue;
        if (t > bestDist) { bestDist = t; start = i; startT = u; }
    }
    if (bestDist < 0) {   // no crossing (a loop not star-shaped from its centroid): the nearest vertex, as before
        Real best = -2;
        for (std::size_t i = 0; i < m; ++i) {
            const Real d = dot(normalize(loop[i] - c), dir);
            if (d > best) { best = d; start = i; }
        }
        startT = 0;
    }
    // the loop re-rooted at the crossing: the crossing point, the rest of its edge, then round
    Poly2 ring;
    ring.reserve(m + 1);
    const Vec2 p0 = loop[start] + (loop[(start + 1) % m] - loop[start]) * startT;
    ring.push_back(p0);
    for (std::size_t i = 1; i <= m; ++i) ring.push_back(loop[(start + i) % m]);
    if ((ring.back() - p0).lengthSquared() < 1e-12) ring.pop_back();   // startT 0: the start vertex itself closes it
    const std::size_t r = ring.size();
    Real total = 0;
    for (std::size_t i = 0; i < r; ++i) total += (ring[(i + 1) % r] - ring[i]).length();
    if (total < 1e-6) return loopIn;
    Poly2 out;
    out.reserve(static_cast<std::size_t>(n));
    std::size_t seg = 0;
    Real segStart = 0;
    Real segLen = (ring[1 % r] - ring[0]).length();
    for (int k = 0; k < n; ++k) {
        const Real s = total * (static_cast<Real>(k) / n);
        while (s > segStart + segLen && seg + 1 < r) {
            segStart += segLen;
            ++seg;
            segLen = (ring[(seg + 1) % r] - ring[seg % r]).length();
        }
        const Vec2 a = ring[seg % r], b = ring[(seg + 1) % r];
        const Real t = segLen > 1e-9 ? std::max(Real(0), std::min(Real(1), (s - segStart) / segLen)) : Real(0);
        out.push_back(a + (b - a) * t);
    }
    return out;
}

Poly2 sliceMesh(const RenderMesh& mesh, Real y, Real tol, int corners) {
    Poly2 none;
    const std::size_t nTri = mesh.indices.size() / 3;
    if (nTri == 0) return none;
    std::vector<std::pair<Vec2, Vec2>> segs;
    segs.reserve(64);
    for (std::size_t t = 0; t < nTri; ++t) {
        const Vec3 p[3] = {mesh.vertices[mesh.indices[3 * t]].position, mesh.vertices[mesh.indices[3 * t + 1]].position,
                           mesh.vertices[mesh.indices[3 * t + 2]].position};
        Vec2 hit[3];
        int nh = 0;
        for (int e = 0; e < 3; ++e) {
            const Vec3 a = p[e], b = p[(e + 1) % 3];
            const double da = a.y - y, db = b.y - y;
            if ((da < 0) == (db < 0)) continue;   // no crossing (both sides, or both on)
            if (std::fabs(db - da) < 1e-12) continue;
            const double tt = da / (da - db);
            if (nh < 3) hit[nh++] = Vec2(a.x + (b.x - a.x) * tt, a.z + (b.z - a.z) * tt);
        }
        if (nh >= 2 && (hit[0] - hit[1]).lengthSquared() > 1e-10) segs.push_back({hit[0], hit[1]});
    }
    if (segs.size() < 3) return none;
    // link into loops: a map from a quantised endpoint to the segments touching it
    std::multimap<long long, std::size_t> at;
    for (std::size_t i = 0; i < segs.size(); ++i) { at.insert({keyOf(segs[i].first), i}); at.insert({keyOf(segs[i].second), i}); }
    std::vector<bool> used(segs.size(), false);
    Poly2 bestLoop;
    Real bestArea = -1;
    for (std::size_t s0 = 0; s0 < segs.size(); ++s0) {
        if (used[s0]) continue;
        Poly2 loop;
        std::size_t k = s0;
        Vec2 cur = segs[s0].first;
        for (std::size_t guard = 0; guard < segs.size() + 2; ++guard) {
            used[k] = true;
            loop.push_back(cur);
            const Vec2 next = (segs[k].first - cur).lengthSquared() < (segs[k].second - cur).lengthSquared() ? segs[k].second : segs[k].first;
            std::size_t nk = k;
            auto range = at.equal_range(keyOf(next));
            for (auto it = range.first; it != range.second; ++it)
                if (it->second != k && !used[it->second]) { nk = it->second; break; }
            cur = next;
            if (nk == k) break;
            k = nk;
        }
        if (loop.size() < 3) continue;
        const Real a = std::fabs(signedArea(loop));
        if (a > bestArea) { bestArea = a; bestLoop = loop; }
    }
    if (bestLoop.size() < 3) return none;
    ensureCCW(bestLoop);
    Poly2 out = simplifyLoop(bestLoop, tol);
    if (corners > 0) out = resampleLoop(out, corners);
    ensureCCW(out);
    return out;
}

Slice MeshMass::slice(Real y) const {
    Slice s;
    if (base.size() < 3) return s;
    int k = 0;
    if (y >= groundH) k = 1 + static_cast<int>(std::floor((y - groundH) / std::max(Real(0.1), floorH) + 1e-6));
    k = std::max(0, std::min(floors, k));
    if (k <= baseStoreys || mesh.indices.size() < 3) {
        s.pieces.push_back(Section::fromLot(base));
        s.tier = 0;
        return s;
    }
    const Poly2 outline = sliceMesh(mesh, groundH + (k - 1) * floorH + 0.5 * floorH, tol, corners);
    s.pieces.push_back(Section::fromLot(outline.size() >= 3 ? outline : base));
    s.tier = 1;
    return s;
}

MassBounds MeshMass::bounds() const {
    MassBounds b;
    bool first = true;
    for (const Vertex& v : mesh.vertices) {
        const Vec2 p(v.position.x, v.position.z);
        if (first) { b.lo = b.hi = p; first = false; continue; }
        b.lo.x = std::min(b.lo.x, p.x); b.lo.y = std::min(b.lo.y, p.y);
        b.hi.x = std::max(b.hi.x, p.x); b.hi.y = std::max(b.hi.y, p.y);
    }
    if (first) { Vec2 lo, hi; if (base.size() >= 3) { ::engine::bounds(base, lo, hi); b.lo = lo; b.hi = hi; } }
    b.y0 = 0;
    b.y1 = height();
    return b;
}

RenderMesh MeshMass::shell(int lod, const Vec3& colour) const {
    if (lod >= 1 && !mesh.indices.empty()) return mesh;
    RenderMesh m;
    std::vector<Poly2> outlines;
    std::vector<Real> heights;
    for (int k = 0; k <= floors; ++k) {
        outlines.push_back(slice(k == 0 ? -1.0 : groundH + (k - 1) * floorH).outline().tessellate());
        heights.push_back(k == 0 ? 0.0 : groundH + (k - 1) * floorH);
    }
    outlines.push_back(outlines.back());
    heights.push_back(height());
    loftSlices(m, outlines, heights, colour, true);
    return m;
}

void fitMeshToPlan(RenderMesh& mesh, const Poly2& planIn, Real height) {
    if (mesh.vertices.empty() || planIn.size() < 3) return;
    Poly2 plan = planIn;
    ensureCCW(plan);
    Vec3 lo = mesh.vertices[0].position, hi = lo;
    for (const Vertex& v : mesh.vertices) {
        lo.x = std::min(lo.x, v.position.x); lo.y = std::min(lo.y, v.position.y); lo.z = std::min(lo.z, v.position.z);
        hi.x = std::max(hi.x, v.position.x); hi.y = std::max(hi.y, v.position.y); hi.z = std::max(hi.z, v.position.z);
    }
    const OBB2 ob = orientedBoundingBox(plan);
    const Real sx = (hi.x - lo.x) > 1e-6 ? 2 * ob.half[0] / (hi.x - lo.x) : 1;
    const Real sz = (hi.z - lo.z) > 1e-6 ? 2 * ob.half[1] / (hi.z - lo.z) : 1;
    const Real sy = (hi.y - lo.y) > 1e-6 ? height / (hi.y - lo.y) : 1;
    const Vec3 mid((lo.x + hi.x) * 0.5, lo.y, (lo.z + hi.z) * 0.5);
    for (Vertex& v : mesh.vertices) {
        const Real u = (v.position.x - mid.x) * sx, w = (v.position.z - mid.z) * sz;
        const Vec2 p = ob.center + ob.axis[0] * u + ob.axis[1] * w;
        v.position = Vec3(p.x, (v.position.y - mid.y) * sy, p.y);
    }
}

void widenMeshForCore(RenderMesh& mesh, const Poly2& planIn, const MeshMass& mm) {
    if (mesh.indices.size() < 3 || planIn.size() < 3) return;
    Poly2 plan = planIn;
    ensureCCW(plan);
    const OBB2 ob = orientedBoundingBox(plan);
    const Real boxShort = 2 * std::min(ob.half[0], ob.half[1]);
    const Real need = std::min(kCoreTierMin + 3.0, boxShort);
    auto narrowest = [&](const RenderMesh& m) {
        Real n = 1e9;
        for (int k = mm.baseStoreys + 1; k <= mm.floors; ++k) {
            const Poly2 s = sliceMesh(m, mm.groundH + (k - 1) * mm.floorH + 0.5 * mm.floorH, mm.tol, mm.corners);
            if (s.size() >= 3) n = std::min(n, tierShortSide(s));
        }
        return n;
    };
    for (int pass = 0; pass < 6; ++pass) {
        const Real n = narrowest(mesh);
        if (n > 1e8 || n >= need - 0.05) break;
        // a point on the box stays; a point inside moves out by the share `f` of the way to it (its box-norm `nb`
        // is 1 on the box), so the narrowest slice grows by about f of the way to the box's short side
        const Real f = std::min(Real(1), (need - n) / std::max(Real(0.1), boxShort - n) + 0.02);
        for (Vertex& v : mesh.vertices) {
            const Vec2 r = Vec2(v.position.x, v.position.z) - ob.center;
            const Real u = dot(r, ob.axis[0]), w = dot(r, ob.axis[1]);
            const Real nb = std::max(ob.half[0] > 1e-6 ? std::fabs(u) / ob.half[0] : Real(1), ob.half[1] > 1e-6 ? std::fabs(w) / ob.half[1] : Real(1));
            if (nb < 1e-6 || nb >= 1) continue;
            const Real g = 1 + f * (1 / nb - 1);
            const Vec2 q = ob.center + ob.axis[0] * (u * g) + ob.axis[1] * (w * g);
            v.position.x = q.x;
            v.position.z = q.y;
        }
    }
}

const std::vector<std::string>& massLibrary() {
    static std::vector<std::string> files;
    static std::once_flag once;
    std::call_once(once, [] {
        namespace fs = std::filesystem;
        const fs::path dir("assets/masses");
        std::error_code ec;
        if (!fs::is_directory(dir, ec)) return;
        for (const auto& entry : fs::directory_iterator(dir, ec)) {
            const std::string ext = entry.path().extension().string();
            if (ext == ".glb" || ext == ".gltf") files.push_back(entry.path().string());
        }
        std::sort(files.begin(), files.end());
    });
    return files;
}

MeshMass meshOf(const Poly2& planIn, const BuildingParams& params) {
    MeshMass mm;
    Poly2 plan = planIn;
    if (plan.size() < 3) return mm;
    ensureCCW(plan);
    mm.base = plan;
    mm.baseStoreys = std::max(1, params.baseFloors);
    mm.groundH = params.groundHeight;
    mm.floorH = params.floorHeight;
    mm.floors = params.floors;
    const std::vector<std::string>& lib = massLibrary();
    if (params.massMesh == 0 || params.massMesh > lib.size()) return mm;
    static std::map<std::string, RenderMesh> cache;
    static std::mutex mu;
    const std::string& path = lib[params.massMesh - 1];
    RenderMesh raw;
    {
        std::lock_guard<std::mutex> lock(mu);
        auto it = cache.find(path);
        if (it == cache.end()) {
            const CpuModel model = ModelImporter::loadCpu(path);
            RenderMesh merged;
            for (const CpuMesh& cm : model.meshes) {
                const uint32_t base = static_cast<uint32_t>(merged.vertices.size());
                merged.vertices.insert(merged.vertices.end(), cm.geometry.vertices.begin(), cm.geometry.vertices.end());
                for (uint32_t i : cm.geometry.indices) merged.indices.push_back(base + i);
            }
            it = cache.emplace(path, std::move(merged)).first;
        }
        raw = it->second;
    }
    if (raw.indices.size() < 3) return mm;
    // onto the box INSIDE the plan (the plan itself when it is a rectangle), as the twist turns in: on an L the
    // lot's own box reaches into the notch, and a slice centred there holds no core the ground storey also holds
    // (tierBoxInside shrinks the plan's box about its centre until its corners are in; on an L centred on its
    // inner corner no such box exists, and the form is not placed) -- a plan with no box inside it, or a box too
    // tight to hold the core (kCoreTierMin + 3 m, the floor widenMeshForCore keeps), cannot carry the form: the
    // building stands as the plan, as the taper and the slab stand on a plan with no box inside it
    const OBB2 ob = orientedBoundingBox(plan);
    Real hw = 0, hd = 0;
    if (!tierBoxInside(plan, ob, hw, hd)) return mm;
    const Poly2 box = tierBox(ob, hw, hd);
    if (tierShortSide(box) < kCoreTierMin + 3.0) return mm;
    fitMeshToPlan(raw, box, mm.height());
    widenMeshForCore(raw, box, mm);
    mm.mesh = std::move(raw);
    return mm;
}

}  // namespace engine
