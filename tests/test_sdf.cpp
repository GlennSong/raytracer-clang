#include "test_framework.h"

#include "../src/engine/procgen/sdf.h"
#include <cmath>
#include <map>
#include <utility>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(sdf_sphere_distance) {
    Sdf s = sdfSphere(Vec3(0, 0, 0), 1.0);
    CHECK_APPROX(s(Vec3(0, 0, 0)), -1.0, 1e-9);   // center: inside by the radius
    CHECK_APPROX(s(Vec3(1, 0, 0)), 0.0, 1e-9);    // on the surface
    CHECK_APPROX(s(Vec3(3, 0, 0)), 2.0, 1e-9);    // outside by 2
}

TEST_CASE(sdf_capsule_distance) {
    Sdf s = sdfCapsule(Vec3(0, -1, 0), Vec3(0, 1, 0), 0.5);
    CHECK_APPROX(s(Vec3(0, 0, 0)), -0.5, 1e-9);   // on the axis, mid-segment
    CHECK_APPROX(s(Vec3(0.5, 0, 0)), 0.0, 1e-9);  // at radius from the axis
}

TEST_CASE(sdf_boolean_combinators) {
    Sdf a = sdfSphere(Vec3(-0.5, 0, 0), 1.0);
    Sdf b = sdfSphere(Vec3(0.5, 0, 0), 1.0);
    Vec3 p(2, 0, 0);
    CHECK_APPROX(sdfUnion(a, b)(p), std::min(a(p), b(p)), 1e-12);
    CHECK_APPROX(sdfIntersect(a, b)(p), std::max(a(p), b(p)), 1e-12);
    CHECK_APPROX(sdfSubtract(a, b)(p), std::max(a(p), -b(p)), 1e-12);
}

TEST_CASE(sdf_smooth_union_blends_at_the_seam) {
    Sdf a = sdfSphere(Vec3(-0.5, 0, 0), 0.7);
    Sdf b = sdfSphere(Vec3(0.5, 0, 0), 0.7);
    // k=0 degenerates to a hard min; k>0 pulls the seam outward (more negative).
    Vec3 seam(0, 0, 0);
    CHECK_APPROX(sdfSmoothUnion(a, b, 0.0)(seam), std::min(a(seam), b(seam)), 1e-9);
    CHECK(sdfSmoothUnion(a, b, 0.5)(seam) < std::min(a(seam), b(seam)));
}

TEST_CASE(sdf_gradient_points_outward) {
    Sdf s = sdfSphere(Vec3(0, 0, 0), 1.0);
    Vec3 g = sdfGradient(s, Vec3(1, 0, 0));
    CHECK(dot(g, Vec3(1, 0, 0)) > 0.9);   // outward radial on the +x surface
}

TEST_CASE(sdf_polygonize_sphere_is_on_surface) {
    Sdf s = sdfSphere(Vec3(0, 0, 0), 1.0);
    SdfBounds b{Vec3(-1.5, -1.5, -1.5), Vec3(1.5, 1.5, 1.5)};
    int res = 24;
    RenderMesh m = polygonizeSdf(s, b, res);
    CHECK(m.vertices.size() > 50);
    CHECK(m.indices.size() % 3 == 0);

    double cell = 3.0 / res;
    double outwardDotSum = 0.0;
    bool allNearSurface = true, allInBounds = true;
    uint32_t maxIdx = 0;
    for (uint32_t i : m.indices) maxIdx = std::max(maxIdx, i);
    for (const Vertex& v : m.vertices) {
        double r = v.position.length();
        if (std::fabs(r - 1.0) > 1.5 * cell) allNearSurface = false;
        if (std::fabs(v.position.x) > 1.5 || std::fabs(v.position.y) > 1.5 ||
            std::fabs(v.position.z) > 1.5) allInBounds = false;
        outwardDotSum += dot(v.normal, normalize(v.position));
    }
    CHECK(maxIdx < m.vertices.size());            // indices in range
    CHECK(allNearSurface);                        // vertices sit on the sphere
    CHECK(allInBounds);
    CHECK(outwardDotSum / m.vertices.size() > 0.9);  // normals radial-outward
}

TEST_CASE(sdf_polygonize_is_deterministic) {
    Sdf s = sdfSphere(Vec3(0, 0, 0), 1.0);
    SdfBounds b{Vec3(-1.5, -1.5, -1.5), Vec3(1.5, 1.5, 1.5)};
    RenderMesh a = polygonizeSdf(s, b, 16);
    RenderMesh c = polygonizeSdf(s, b, 16);
    CHECK(a.vertices.size() == c.vertices.size());
    CHECK(a.indices.size() == c.indices.size());
}

TEST_CASE(sdf_polygonize_is_consistently_wound_and_watertight) {
    // The signature of the "stripey holes" bug was inconsistent winding: two
    // triangles sharing a directed edge (both wound the same way) — half get
    // backface-culled. A consistently wound, watertight surface has every
    // directed edge appear exactly once, with its reverse present.
    Sdf s = sdfSphere(Vec3(0, 0, 0), 1.0);
    SdfBounds b{Vec3(-1.4, -1.4, -1.4), Vec3(1.4, 1.4, 1.4)};
    RenderMesh m = polygonizeSdf(s, b, 24);

    std::map<std::pair<uint32_t, uint32_t>, int> directed;
    for (size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        uint32_t a = m.indices[i], b2 = m.indices[i + 1], c = m.indices[i + 2];
        directed[{a, b2}]++;
        directed[{b2, c}]++;
        directed[{c, a}]++;
    }
    int sameDirDuplicates = 0, unmatched = 0, total = 0;
    for (const auto& kv : directed) {
        total++;
        if (kv.second > 1) sameDirDuplicates++;                 // inconsistent winding
        if (!directed.count({kv.first.second, kv.first.first})) unmatched++;  // a hole/boundary
    }
    CHECK(total > 0);
    CHECK(sameDirDuplicates == 0);          // consistent winding (the bug)
    CHECK(unmatched <= total / 20);         // watertight to within ~5%
}

TEST_CASE(sdf_smooth_union_welds_into_one_blob) {
    // Two overlapping spheres, smooth-unioned, polygonize into a single bounded
    // surface (the welding the kit-bashed trees lack).
    Sdf blob = sdfSmoothUnion(sdfSphere(Vec3(-0.6, 0, 0), 0.7),
                              sdfSphere(Vec3(0.6, 0, 0), 0.7), 0.4);
    SdfBounds b{Vec3(-2, -2, -2), Vec3(2, 2, 2)};
    RenderMesh m = polygonizeSdf(blob, b, 32);
    CHECK(m.vertices.size() > 100);
    bool inBounds = true;
    for (const Vertex& v : m.vertices)
        if (v.position.length() > 2.0) inBounds = false;
    CHECK(inBounds);
}
