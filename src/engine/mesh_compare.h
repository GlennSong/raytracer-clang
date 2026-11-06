#ifndef RAYTRACER_ENGINE_MESH_COMPARE_H
#define RAYTRACER_ENGINE_MESH_COMPARE_H

#include "../renderer/renderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// GEOMETRY-EQUAL (#191 phase 0): are two meshes the same surface -- the same triangles, each wound the same way, with
// the same corners (position, normal, uv, colour) -- whatever order their vertices and triangles come in? A port onto
// the toolkit that only reorders its output (a box built by emitBox instead of six hand quads) is proved by this when
// a byte hash cannot be: a refactor that should change nothing either passes, or the first triangle that differs is
// named. Attributes compare within `eps`; winding is kept (a triangle and its reverse differ).
struct MeshDiff {
    bool equal = false;
    std::size_t trisA = 0, trisB = 0;
    std::string why;   // empty when equal: else the first difference found
};

namespace meshCompareDetail {
struct Corner {
    std::array<double, 11> k{};   // position 3, normal 3, uv 2, colour 3 -- quantized by eps
    bool operator<(const Corner& o) const { return k < o.k; }
    bool operator==(const Corner& o) const { return k == o.k; }
};
using Tri = std::array<Corner, 3>;

inline double q(double x, double eps) { return std::round(x / eps); }

inline Corner cornerOf(const Vertex& v, double eps) {
    Corner c;
    c.k = {q(v.position.x, eps), q(v.position.y, eps), q(v.position.z, eps), q(v.normal.x, eps * 10), q(v.normal.y, eps * 10),
           q(v.normal.z, eps * 10), q(v.u, eps), q(v.v, eps), q(v.color.x, eps), q(v.color.y, eps), q(v.color.z, eps)};
    return c;
}

// a triangle rotated so its least corner leads (winding kept: (a b c) = (b c a) = (c a b), not (a c b))
inline Tri canonical(Tri t) {
    std::size_t lead = 0;
    for (std::size_t i = 1; i < 3; ++i)
        if (t[i] < t[lead]) lead = i;
    return {t[lead], t[(lead + 1) % 3], t[(lead + 2) % 3]};
}

inline std::vector<Tri> trianglesOf(const RenderMesh& m, double eps) {
    std::vector<Tri> out;
    out.reserve(m.indices.size() / 3);
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3)
        out.push_back(canonical({cornerOf(m.vertices[m.indices[i]], eps), cornerOf(m.vertices[m.indices[i + 1]], eps),
                                 cornerOf(m.vertices[m.indices[i + 2]], eps)}));
    std::sort(out.begin(), out.end());
    return out;
}
}  // namespace meshCompareDetail

inline MeshDiff geometryEqual(const RenderMesh& a, const RenderMesh& b, double eps = 1e-6) {
    using namespace meshCompareDetail;
    MeshDiff d;
    const std::vector<Tri> ta = trianglesOf(a, eps), tb = trianglesOf(b, eps);
    d.trisA = ta.size();
    d.trisB = tb.size();
    if (ta.size() != tb.size()) {
        d.why = "triangle counts differ: " + std::to_string(ta.size()) + " vs " + std::to_string(tb.size());
        return d;
    }
    for (std::size_t i = 0; i < ta.size(); ++i)
        if (!(ta[i] == tb[i])) {
            const auto& p = ta[i][0].k;
            d.why = "triangle " + std::to_string(i) + " differs (first corner near " + std::to_string(p[0] * eps) + ", " +
                    std::to_string(p[1] * eps) + ", " + std::to_string(p[2] * eps) + ")";
            return d;
        }
    d.equal = true;
    return d;
}

}  // namespace engine

#endif
