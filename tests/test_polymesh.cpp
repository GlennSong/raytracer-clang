// PolyMesh: the hard-surface modelling kit (ADR-0139). Each op keeps the mesh a closed, outward-facing
// solid, and subdivision honours creases -- the properties every recipe leans on.
#include "test_framework.h"
#include "../src/engine/procgen/polymesh.h"

#include <cmath>

using namespace engine;

namespace {
// a unit cube (-0.5..0.5) as a loft of two square sections
PolyMesh cube() {
    const std::vector<Vec2> sq = {Vec2(-0.5, -0.5), Vec2(0.5, -0.5), Vec2(0.5, 0.5), Vec2(-0.5, 0.5)};
    return loft({sq, sq}, {-0.5, 0.5}, true, true);
}
void bounds(const PolyMesh& m, Vec3& lo, Vec3& hi) {
    lo = Vec3(1e30, 1e30, 1e30);
    hi = Vec3(-1e30, -1e30, -1e30);
    for (const Vec3& p : m.pts) {
        lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
        hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
    }
}
}  // namespace

TEST_CASE(polymesh_loft_makes_a_closed_outward_solid) {
    PolyMesh c = cube();
    CHECK(c.faces.size() == 6);
    CHECK(isClosed(c));
    CHECK_APPROX(signedVolume(c), 1.0, 1e-9);
    // a rounded, resampled section lofted over three stations stays closed and outward
    const auto sec = resampleClosed(roundedPolygon({Vec2(-1, 0), Vec2(1, 0), Vec2(0.8, 1), Vec2(-0.8, 1)}, {0.2}, 3), 24);
    CHECK(sec.size() == 24);
    // every corner emits segs + 1 points, even square and straight ones (recipes index faces by corner)
    CHECK(roundedPolygon({Vec2(0, 0), Vec2(1, 0), Vec2(2, 0), Vec2(2, 1), Vec2(0, 1)}, {0.0, 0.3, 0.2, 0.2, 0.0}, 3).size() == 20u);
    PolyMesh b = loft({sec, sec, sec}, {-2, 0, 2}, true, true);
    CHECK(isClosed(b));
    CHECK(signedVolume(b) > 0);
}

TEST_CASE(polymesh_extrude_and_inset_keep_the_solid_closed) {
    PolyMesh c = cube();
    const auto top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    CHECK(top.size() == 1);
    extrude(c, top, 1.0);
    CHECK(c.faces.size() == 10);
    CHECK(isClosed(c));
    CHECK_APPROX(signedVolume(c), 2.0, 1e-9);
    Vec3 lo, hi;
    bounds(c, lo, hi);
    CHECK_APPROX(hi.y, 1.5, 1e-9);
    const auto front = selectWhere(c, [](const Vec3& ctr, const Vec3& n) { return n.z > 0.9 && ctr.y < 0.1; });
    CHECK(front.size() == 1);
    const auto in = inset(c, front, 0.1);
    CHECK(in.size() == 1);
    CHECK(c.faces.size() == 14);
    CHECK(isClosed(c));
    // pushing the inset face in makes a recess (a window): still closed, less volume
    const double v0 = signedVolume(c);
    extrude(c, in, -0.05);
    CHECK(isClosed(c));
    CHECK(signedVolume(c) < v0);
}

TEST_CASE(polymesh_mirror_welds_the_seam) {
    // the +x half of a box, open on x = 0
    PolyMesh h;
    h.pts = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1, 1, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 0, 1), Vec3(1, 1, 1), Vec3(0, 1, 1)};
    auto q = [&](int a, int b, int c, int d) { PolyMesh::Face f; f.v = {a, b, c, d}; h.faces.push_back(f); };
    q(0, 3, 2, 1);   // z = 0 (faces -z)
    q(4, 5, 6, 7);   // z = 1
    q(1, 2, 6, 5);   // x = 1
    q(0, 1, 5, 4);   // y = 0
    q(3, 7, 6, 2);   // y = 1
    CHECK(!isClosed(h));
    mirrorX(h);
    CHECK(isClosed(h));
    CHECK_APPROX(signedVolume(h), 2.0, 1e-9);
}

TEST_CASE(polymesh_subdivision_smooths_and_creases_hold) {
    PolyMesh c = cube();
    c.creases.clear();   // loft creases its cap rings; the blob wants none
    PolyMesh s = subdivide(c, 3);
    CHECK(s.faces.size() == 6u * 64u);
    CHECK(isClosed(s));
    CHECK(signedVolume(s) > 0.3 && signedVolume(s) < 0.4);   // a smooth blob well inside the cage
    Vec3 lo, hi;
    bounds(s, lo, hi);
    CHECK(hi.x < 0.5 && hi.x > 0.25);
    // every edge fully creased: the cube keeps its shape exactly
    PolyMesh k = cube();
    creaseFaces(k, selectAll(k), PolyMesh::kInfCrease);
    PolyMesh ks = subdivide(k, 2);
    CHECK(isClosed(ks));
    CHECK_APPROX(signedVolume(ks), 1.0, 1e-9);
    // sharpness is a dial: every edge at 0 (smooth), 0.5 (semi-sharp), 1 (one level sharp) -- the volume
    // grows with it. (loft creases its cap rings at 1, so clear them for the smooth reference.)
    auto vol = [](float s) { PolyMesh h = cube(); creaseFaces(h, selectAll(h), s); h.creases.erase(h.creases.begin(), h.creases.end()); if (s > 0) creaseFaces(h, selectAll(h), s); return signedVolume(subdivide(h, 2)); };
    CHECK(vol(0.0f) < vol(0.5f));
    CHECK(vol(0.5f) < vol(1.0f));
    CHECK(vol(1.0f) < 1.0);
}

TEST_CASE(polymesh_to_parts_splits_by_material_in_engine_winding) {
    PolyMesh c = cube();
    const auto top = selectWhere(c, [](const Vec3&, const Vec3& n) { return n.y > 0.9; });
    for (int f : top) c.faces[static_cast<std::size_t>(f)].mat = c.matId("glass");
    const auto parts = toParts(subdivide(c, 1), 35.0);
    CHECK(parts.size() == 2);
    CHECK(parts.count("body") && parts.count("glass"));
    for (const auto& [name, rm] : parts) {
        CHECK(!rm.indices.empty());
        // engine winding: the geometric normal cross(c-a, b-a) agrees with the vertex normal
        int agree = 0, total = 0;
        for (std::size_t i = 0; i + 2 < rm.indices.size(); i += 3) {
            const Vertex& a = rm.vertices[rm.indices[i]];
            const Vertex& b = rm.vertices[rm.indices[i + 1]];
            const Vertex& cc = rm.vertices[rm.indices[i + 2]];
            agree += dot(cross(cc.position - a.position, b.position - a.position), a.normal) > 0;
            ++total;
            CHECK_APPROX(a.normal.length(), 1.0, 1e-6);
        }
        CHECK(agree == total);
    }
}
