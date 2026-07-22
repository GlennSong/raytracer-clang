#include "test_framework.h"

#include "../src/engine/mesh_builder.h"
#include "mesh_invariants.h"

#include <cmath>
#include <cstdint>
#include <map>
#include <utility>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

double triArea(const RenderMesh& m, size_t t) {
    const Vec3& a = m.vertices[m.indices[t]].position;
    const Vec3& b = m.vertices[m.indices[t + 1]].position;
    const Vec3& c = m.vertices[m.indices[t + 2]].position;
    return 0.5 * cross(b - a, c - a).length();
}

// Ratio of largest to smallest triangle area — the area-distortion metric the
// COBE warp exists to shrink.
double areaRatio(const RenderMesh& m) {
    double lo = 1e30, hi = 0.0;
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        double a = triArea(m, t);
        if (a < 1e-12) continue;
        lo = std::min(lo, a);
        hi = std::max(hi, a);
    }
    return hi / lo;
}

}  // namespace

TEST_CASE(cube_sphere_triangle_count_and_validity) {
    const int faceRes = 8;
    RenderMesh m = MeshBuilder::cubeSphere(100.0f, faceRes);

    // 6 faces × faceRes² quads × 2 triangles.
    CHECK(mesh_invariants::triangleCount(m) == 6 * faceRes * faceRes * 2);
    CHECK(mesh_invariants::degenerateTriangles(m) == 0);
    CHECK(!mesh_invariants::hasNonFinite(m));
    CHECK(mesh_invariants::indicesInRange(m));

    // Welding across seams must merge vertices, so the unique count is well below
    // the naive 6·(faceRes+1)² per-face total.
    CHECK(m.vertices.size() < static_cast<size_t>(6 * (faceRes + 1) * (faceRes + 1)));
}

TEST_CASE(cube_sphere_vertices_lie_on_the_sphere) {
    const float R = 250.0f;
    RenderMesh m = MeshBuilder::cubeSphere(R, 6);
    for (const Vertex& v : m.vertices) {
        CHECK_APPROX(v.position.length(), R, 1e-3);
        // Normal is the outward radial.
        CHECK(dot(v.normal, normalize(v.position)) > 0.999);
    }
}

TEST_CASE(cube_sphere_is_a_closed_consistently_wound_manifold) {
    RenderMesh m = MeshBuilder::cubeSphere(100.0f, 8);

    // In a closed, consistently-wound manifold every DIRECTED edge (a->b) appears
    // exactly once, and its reverse (b->a) is present (the neighbouring triangle).
    // This proves both watertightness (no cracks — seams welded) and a single
    // coherent winding across all 6 faces.
    std::map<std::pair<uint32_t, uint32_t>, int> directed;
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        uint32_t tri[3] = {m.indices[t], m.indices[t + 1], m.indices[t + 2]};
        for (int e = 0; e < 3; e++)
            directed[{tri[e], tri[(e + 1) % 3]}]++;
    }
    int doubled = 0, missingReverse = 0;
    for (const auto& kv : directed) {
        if (kv.second != 1) ++doubled;
        if (directed.find({kv.first.second, kv.first.first}) == directed.end())
            ++missingReverse;
    }
    CHECK(doubled == 0);          // no directed edge traversed twice
    CHECK(missingReverse == 0);   // every edge has its opposite-facing neighbour
}

TEST_CASE(cube_sphere_faces_wind_outward) {
    RenderMesh m = MeshBuilder::cubeSphere(100.0f, 6);
    // Engine convention: outward normal = cross(c-a, b-a). On a sphere centred at
    // the origin the outward direction is the vertex position, so every front face
    // must satisfy dot(cross(c-a,b-a), a) > 0.
    int outward = 0, total = 0;
    for (size_t t = 0; t + 2 < m.indices.size(); t += 3) {
        const Vec3& a = m.vertices[m.indices[t]].position;
        const Vec3& b = m.vertices[m.indices[t + 1]].position;
        const Vec3& c = m.vertices[m.indices[t + 2]].position;
        ++total;
        if (dot(cross(c - a, b - a), a) > 0) ++outward;
    }
    CHECK(outward == total);
}

TEST_CASE(cube_sphere_warp_evens_out_triangle_area) {
    RenderMesh warped = MeshBuilder::cubeSphere(100.0f, 8, /*warp=*/true);
    RenderMesh raw = MeshBuilder::cubeSphere(100.0f, 8, /*warp=*/false);

    double warpedRatio = areaRatio(warped);
    double rawRatio = areaRatio(raw);

    // The COBE warp should measurably shrink the corner-to-centre area ratio; the
    // raw cube→sphere projection is ~5:1, the warp brings it toward ~1.5:1.
    CHECK(warpedRatio < rawRatio);
    CHECK(warpedRatio < 2.0);
    CHECK(rawRatio > 3.0);
}

TEST_CASE(cube_sphere_is_deterministic) {
    RenderMesh a = MeshBuilder::cubeSphere(100.0f, 8);
    RenderMesh b = MeshBuilder::cubeSphere(100.0f, 8);
    CHECK(a.vertices.size() == b.vertices.size());
    CHECK(a.indices.size() == b.indices.size());
    bool identical = a.indices.size() == b.indices.size();
    for (size_t i = 0; identical && i < a.indices.size(); i++)
        identical = a.indices[i] == b.indices[i];
    for (size_t i = 0; identical && i < a.vertices.size(); i++)
        identical = (a.vertices[i].position - b.vertices[i].position).lengthSquared() == 0.0;
    CHECK(identical);
}
