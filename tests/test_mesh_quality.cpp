// Is the road mesh made of RIBBONS, or of fans?
//
// A road is a swept strip: its natural triangulation is transverse quads, every
// triangle local and well-shaped. weldSolid instead boolean-unions the strips
// into a generic polygon and hands it to earcut, which has no idea it was ever a
// road — ear clipping emits a VALID triangulation with no shape guarantee, so it
// fans the polygon from whatever vertex it reaches first.
//
// User, driving metropolis: "you have a lot of degenerate surfaces on the road. I
// can see a ton of triangles emitting from a single point. Those don't look like
// ribbons at all."
//
// These measure it: how many triangles share one apex, and how sliver-shaped the
// faces are. degenerateTriangles() (zero AREA) already passes — slivers are not
// zero-area, which is why the existing suite never saw this.
#include "test_framework.h"
#include "mesh_invariants.h"

#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/road_mesh.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <vector>

using namespace engine;
using namespace mesh_invariants;

namespace {

// weldSolid flat-shades, so every triangle pushes its OWN vertices — a fan has no
// shared index to count. Cluster by POSITION (1 cm) to find coincident apexes.
long long key1cm(const Vec3& p) {
    const long long x = std::llround(p.x * 100), y = std::llround(p.y * 100),
                    z = std::llround(p.z * 100);
    return (x * 73856093LL) ^ (y * 19349663LL) ^ (z * 83492791LL);
}

// Aspect: longest edge / shortest altitude. An equilateral triangle is ~1.15; a
// sliver is unbounded. >20 is a needle you can see.
double aspect(const Vec3& a, const Vec3& b, const Vec3& c) {
    const double la = (b - a).length(), lb = (c - b).length(), lc = (a - c).length();
    const double longest = std::max({la, lb, lc});
    const double area = cross(b - a, c - a).length() * 0.5;
    if (area < 1e-12) return 1e9;
    const double minAlt = 2.0 * area / longest;   // altitude onto the longest edge
    return longest / std::max(1e-9, minAlt);
}

RoadNet gridCity(int n, double spacing) {
    RoadNet g;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) g.nodes.push_back(Vec2(i * spacing, j * spacing));
    auto id = [&](int i, int j) { return i * n + j; };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            if (i + 1 < n) g.edges.push_back({ id(i, j), id(i + 1, j) });
            if (j + 1 < n) g.edges.push_back({ id(i, j), id(i, j + 1) });
        }
    g.heightAt = [](Real x, Real z) {
        return 1.5 * std::sin(x * 0.02) + 1.2 * std::cos(z * 0.017);
    };
    return g;
}

}  // namespace

// A road's triangles should be LOCAL. Ear clipping fans, so one point ends up as
// the apex of a huge number of faces — the "triangles emitting from a single
// point" you can see on the asphalt.
TEST_CASE(road_mesh_is_ribbons_not_fans) {
    RenderMesh m = buildRoadNetMesh(gridCity(4, 90.0));
    CHECK(triangleCount(m) > 0);
    // BASELINE 2026-07-17: on HILLS (not the flat checkWeld fixtures) the union
    // sheet emits zero-area triangles. The swept lattice has none by construction.
    CHECK(degenerateTriangles(m) <= 400);     // BASELINE 364; STAGE-1/3 GATE: tighten to == 0

    std::map<long long, int> apex;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3)
        for (int k = 0; k < 3; ++k)
            ++apex[key1cm(m.vertices[m.indices[t + k]].position)];
    int worstApex = 0;
    for (const auto& [k, n] : apex) worstApex = std::max(worstApex, n);

    // A vertex in a swept ribbon touches ~6 triangles; a junction corner maybe
    // ~12. Dozens sharing one point is a fan, not a road.
    // Is there ANY connectivity? An indexed quad grid shares vertices (V ~= T/2).
    // An unshared soup pushes 3 fresh vertices per face (V == 3T) — nothing is
    // connected to anything, so there is no edge to subdivide along and no ring
    // to decimate. That is the real disease; the fans are the symptom.
    const double vPerT = (double)m.vertices.size() / std::max(1, triangleCount(m));
    std::size_t distinct = 0;
    { std::map<long long,int> pos;
      for (const Vertex& v : m.vertices) ++pos[key1cm(v.position)];
      distinct = pos.size(); }
    std::printf("[quality] tris=%d verts=%zu (%.2f verts/tri; soup=3.00, shared grid~0.5)"
                "  degenerate=%d\n",
                triangleCount(m), m.vertices.size(), vPerT, degenerateTriangles(m));
    std::printf("[quality] distinct positions=%zu -> %.1fx duplication\n", distinct,
                (double)m.vertices.size() / std::max<std::size_t>(1, distinct));
    std::printf("[quality] worst apex fan=%d triangles at one point\n", worstApex);
    // BASELINE 2026-07-17: worst apex fan=457, ~2.9 verts/tri (unshared soup),
    // 7.5x position duplication. This IS the "triangles emitting from a single
    // point" defect. The swept lattice shares vertices by index (~0.55 verts/tri,
    // apex ~6). STAGE-3 GATE (streets on the lattice): tighten to `< 24`.
    CHECK(worstApex <= 500);
}

// And they should be well-SHAPED. A sliver spanning the road is what chords
// across the profile and rips at junctions.
TEST_CASE(road_mesh_has_no_slivers) {
    RenderMesh m = buildRoadNetMesh(gridCity(4, 90.0));
    int bad = 0, total = 0;
    double worst = 0;
    for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        const double r = aspect(a, b, c);
        ++total;
        worst = std::max(worst, r > 1e8 ? worst : r);
        if (r > 20.0) ++bad;
    }
    std::printf("[quality] tris=%d slivers(aspect>20)=%d (%.1f%%)  worstAspect=%.0f\n",
                total, bad, 100.0 * bad / std::max(1, total), worst);
    // BASELINE 2026-07-17: 68.8% of faces are slivers (aspect>20), worst ~297000.
    // Earcutting a union polygon has no shape guarantee. The swept lattice's
    // aspect is bounded by ring spacing / lane width. STAGE-3 GATE: tighten to `< 5`.
    CHECK(100.0 * bad / std::max(1, total) < 75.0);
}
