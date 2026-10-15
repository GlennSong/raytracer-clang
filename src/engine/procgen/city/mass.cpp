#include "mass.h"
#include "triangulate.h"
#include "../../mesh_builder.h"
#include <algorithm>
#include <cmath>

namespace engine {

std::vector<StoreyPlan> sliceStoreys(const Mass& mass, const BuildingParams& params) {
    std::vector<StoreyPlan> out;
    const Slice base = mass.slice(-1.0);
    if (base.empty()) return out;
    out.push_back({base.outline().tessellate(), 0, params.groundHeight, 0});
    Real y = params.groundHeight;
    for (int i = 0; i < params.floors; ++i) {
        const Slice s = mass.slice(y);
        if (s.empty()) break;
        out.push_back({s.outline().tessellate(), y, params.floorHeight, s.tier});
        y += params.floorHeight;
    }
    return out;
}

void loftSlices(RenderMesh& out, const std::vector<Poly2>& outlines, const std::vector<Real>& heights, const Vec3& colour, bool cap) {
    const std::size_t n = std::min(outlines.size(), heights.size());
    if (n == 0) return;
    auto wall = [&](const Poly2& lo, Real y0, const Poly2& hi, Real y1) {
        const std::size_t m = lo.size();
        for (std::size_t i = 0; i < m; ++i) {
            const Vec2 &a = lo[i], &b = lo[(i + 1) % m], &c = hi[(i + 1) % m], &d = hi[i];
            const Vec3 A(a.x, y0, a.y), B(b.x, y0, b.y), C(c.x, y1, c.y), D(d.x, y1, d.y);
            Vec3 nrm = cross(B - A, D - A);
            if (nrm.lengthSquared() < 1e-12) nrm = cross(C - A, D - A);
            if (nrm.lengthSquared() < 1e-12) continue;
            nrm = normalize(nrm);
            MeshBuilder::emitQuad(out, A, B, C, D, nrm, colour);
        }
    };
    for (std::size_t k = 0; k + 1 < n; ++k) {
        const Poly2& lo = outlines[k];
        const Poly2& hi = outlines[k + 1];
        if (lo.size() < 3) continue;
        // A RULED face between outlines with the same corners (a twisted or tapered entry's storeys); else the
        // lower outline's own walls up to the next height (the step between two tiers is the slab's business).
        if (hi.size() == lo.size()) wall(lo, heights[k], hi, heights[k + 1]);
        else wall(lo, heights[k], lo, heights[k + 1]);
    }
    if (cap && outlines[n - 1].size() >= 3) {
        const Poly2& top = outlines[n - 1];
        const Real y = heights[n - 1];
        for (const auto& tri : triangulateIndices(top, {})) {
            const Vec2 &a = top[static_cast<std::size_t>(tri[0])], &b = top[static_cast<std::size_t>(tri[1])], &c = top[static_cast<std::size_t>(tri[2])];
            MeshBuilder::emitTri(out, Vec3(a.x, y, a.y), Vec3(b.x, y, b.y), Vec3(c.x, y, c.y), Vec3(0, 1, 0), colour);
        }
    }
}

}  // namespace engine
