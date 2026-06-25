#include "test_framework.h"

#include "../src/engine/procgen/city/road_offset.h"
#include <cmath>

using namespace engine;

namespace {
double dist(const Vec2& a, const Vec2& b) { return (a - b).length(); }
}

// A straight centerline offsets to a parallel line at the offset distance, on the left.
TEST_CASE(offset_straight_is_parallel) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(10, 0) };       // heading +x; left normal is +y
    std::vector<Vec2> o = offsetPolyline(cl, 2.0);
    CHECK(o.size() == 2);
    CHECK(std::fabs(o[0].x - 0.0) < 1e-9);  CHECK(std::fabs(o[0].y - 2.0) < 1e-9);
    CHECK(std::fabs(o[1].x - 10.0) < 1e-9); CHECK(std::fabs(o[1].y - 2.0) < 1e-9);
}

// A 90-degree corner miters to the bisector at distance d/cos(45) = d*sqrt(2) from the vertex.
TEST_CASE(offset_right_angle_miters) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(10, 0), Vec2(10, 10) };   // left turn at (10,0)
    std::vector<Vec2> inner = offsetPolyline(cl, 2.0);        // inside the turn
    std::vector<Vec2> outer = offsetPolyline(cl, -2.0);       // outside the turn
    CHECK(std::fabs(dist(inner[1], Vec2(10, 0)) - 2.0 * std::sqrt(2.0)) < 1e-6);
    CHECK(dist(inner[1], Vec2(8, 2)) < 1e-6);                 // pulled in
    CHECK(dist(outer[1], Vec2(12, -2)) < 1e-6);               // pushed out
}

// A near-spike corner is clamped to miterLimit*|d| instead of shooting out to infinity.
TEST_CASE(offset_sharp_corner_clamps_to_miter_limit) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(10, 0), Vec2(0, 0.5) };   // very sharp reversal
    double d = 2.0, miterLimit = 4.0;
    std::vector<Vec2> o = offsetPolyline(cl, d, miterLimit);
    double m = dist(o[1], Vec2(10, 0));
    CHECK(m <= miterLimit * d + 1e-6);                        // clamped, not a spike
    CHECK(std::fabs(m - miterLimit * d) < 1e-3);              // and it hit the cap
}

// A straight ribbon outline is the expected rectangle: correct area, CCW, four corners.
TEST_CASE(ribbon_outline_straight_rectangle) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(10, 0) };
    Poly2 poly = ribbonOutline(cl, 3.0);
    CHECK(poly.size() == 4);
    CHECK(std::fabs(area(poly) - 60.0) < 1e-6);              // length 10 * width 6
    CHECK(signedArea(poly) > 0);                             // canonical CCW
}

// The ribbon of a bent road is a single closed loop whose area is at least the straight-sum
// (the outer miter adds a little); it is non-degenerate.
TEST_CASE(ribbon_outline_bend_is_closed_and_nonempty) {
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(20, 0), Vec2(20, 20) };
    Poly2 poly = ribbonOutline(cl, 2.0);
    CHECK(poly.size() == 6);                                 // 3 left + 3 right
    CHECK(area(poly) > 2.0 * 20.0 * 4.0 * 0.9);              // ~ two 20m legs * 4m width
    CHECK(signedArea(poly) > 0);
}

// --- the junction weld: boolean union of ribbon outlines ---

namespace {
double loopAreaSum(const std::vector<Poly2>& loops) {
    double a = 0; for (const Poly2& p : loops) a += area(p); return a;
}
}

// Two ribbons crossing at right angles weld into ONE cross-shaped loop with the right area
// (each ribbon 80, overlap 16 -> 144).
TEST_CASE(union_welds_a_cross) {
    Poly2 h = ribbonOutline({ Vec2(-10, 0), Vec2(10, 0) }, 2.0);
    Poly2 v = ribbonOutline({ Vec2(0, -10), Vec2(0, 10) }, 2.0);
    std::vector<Poly2> loops = polygonUnion({ h, v });
    CHECK(loops.size() == 1);
    CHECK(std::fabs(area(loops[0]) - 144.0) < 1e-2);
}

// Two overlapping squares merge into one loop; area = 100 + 100 - 25 overlap = 175.
TEST_CASE(union_merges_overlapping_squares) {
    Poly2 s1 = { Vec2(0, 0), Vec2(10, 0), Vec2(10, 10), Vec2(0, 10) };
    Poly2 s2 = { Vec2(5, 5), Vec2(15, 5), Vec2(15, 15), Vec2(5, 15) };
    std::vector<Poly2> loops = polygonUnion({ s1, s2 });
    CHECK(loops.size() == 1);
    CHECK(std::fabs(area(loops[0]) - 175.0) < 1e-2);
}

// Two disjoint squares stay two loops; total area preserved.
TEST_CASE(union_keeps_disjoint_polys_separate) {
    Poly2 a = { Vec2(0, 0), Vec2(10, 0), Vec2(10, 10), Vec2(0, 10) };
    Poly2 b = { Vec2(20, 0), Vec2(30, 0), Vec2(30, 10), Vec2(20, 10) };
    std::vector<Poly2> loops = polygonUnion({ a, b });
    CHECK(loops.size() == 2);
    CHECK(std::fabs(loopAreaSum(loops) - 200.0) < 1e-2);
}
