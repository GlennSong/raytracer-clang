#include "test_framework.h"

#include "../src/engine/procgen/city/road_mesh.h"
#include <cmath>

using namespace engine;

namespace {
// Circumcenter of three 2D points — lets a test recover the arc's centre and confirm it
// is a TRUE circle (every sample equidistant) without the helper exposing its internals.
Vec2 circumcenter(const Vec2& a, const Vec2& b, const Vec2& c) {
    double d = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
    double a2 = a.x * a.x + a.y * a.y, b2 = b.x * b.x + b.y * b.y, c2 = c.x * c.x + c.y * c.y;
    double ux = (a2 * (b.y - c.y) + b2 * (c.y - a.y) + c2 * (a.y - b.y)) / d;
    double uy = (a2 * (c.x - b.x) + b2 * (a.x - c.x) + c2 * (b.x - a.x)) / d;
    return {ux, uy};
}
// Perpendicular distance from p to the line through q with unit direction u.
double lineDist(const Vec2& p, const Vec2& q, const Vec2& u) {
    return std::fabs(cross(p - q, normalize(u)));
}
double len(const Vec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// Assert `arc` is a true circular fillet of `radius` tangent to both lines (each line
// through the corner with the given back-direction), starting on A and ending on B.
void checkFillet(const std::vector<Vec2>& arc, const Vec2& C, const Vec2& uA, const Vec2& uB,
                 double radius) {
    CHECK(arc.size() >= 3);
    if (arc.size() < 3) return;
    Vec2 ctr = circumcenter(arc.front(), arc[arc.size() / 2], arc.back());
    for (const Vec2& pt : arc) CHECK_APPROX(len(pt - ctr), radius, 1e-6);   // one circle
    CHECK_APPROX(lineDist(ctr, C, uA), radius, 1e-6);                       // tangent to A
    CHECK_APPROX(lineDist(ctr, C, uB), radius, 1e-6);                       // tangent to B
    CHECK(lineDist(arc.front(), C, uA) < 1e-6);                            // starts on A
    CHECK(lineDist(arc.back(), C, uB) < 1e-6);                             // ends on B
}
}

TEST_CASE(curb_fillet_right_angle_is_a_true_quarter_circle) {
    Vec2 C(0, 0), uA(-1, 0), uB(0, -1);                 // 90 deg corner
    std::vector<Vec2> arc = curbReturnFillet(C, uA, uB, 2.0, 100.0);
    checkFillet(arc, C, uA, uB, 2.0);
    // Tangent points sit at t = r/tan45 = r = 2 back along each kerb.
    CHECK_APPROX(len(arc.front() - C), 2.0, 1e-6);
    CHECK_APPROX(len(arc.back() - C), 2.0, 1e-6);
}

TEST_CASE(curb_fillet_holds_at_acute_and_obtuse_angles) {
    Vec2 C(0, 0);
    // Acute: kerbs 60 deg apart. (The old code rejected anything past ~75 deg.)
    Vec2 aA(1, 0), aB(std::cos(PI / 3), std::sin(PI / 3));
    checkFillet(curbReturnFillet(C, aA, aB, 1.5, 100.0), C, aA, aB, 1.5);
    // Obtuse: kerbs 140 deg apart.
    Vec2 oA(1, 0), oB(std::cos(PI * 140 / 180), std::sin(PI * 140 / 180));
    checkFillet(curbReturnFillet(C, oA, oB, 1.5, 100.0), C, oA, oB, 1.5);
}

TEST_CASE(curb_fillet_shrinks_to_fit_the_tangent_cap) {
    Vec2 C(0, 0), uA(-1, 0), uB(0, -1);                 // 90 deg: t == effective radius
    std::vector<Vec2> arc = curbReturnFillet(C, uA, uB, 10.0, 2.0);   // want r=10, only 2 of room
    CHECK(arc.size() >= 3);
    if (arc.size() < 3) return;
    Vec2 ctr = circumcenter(arc.front(), arc[arc.size() / 2], arc.back());
    CHECK_APPROX(len(arc.front() - ctr), 2.0, 1e-6);    // radius clamped to 2, not 10
    CHECK_APPROX(len(arc.front() - C), 2.0, 1e-6);      // tangent point at the cap
}

TEST_CASE(curb_fillet_degenerates_to_empty) {
    Vec2 C(0, 0);
    CHECK(curbReturnFillet(C, Vec2(1, 0), Vec2(1, 0.001), 2.0, 100.0).empty());   // folded/parallel
    CHECK(curbReturnFillet(C, Vec2(1, 0), Vec2(-0.999, 0.045), 2.0, 100.0).empty()); // near-straight
    CHECK(curbReturnFillet(C, Vec2(-1, 0), Vec2(0, -1), 0.0, 100.0).empty());     // no radius
    CHECK(curbReturnFillet(C, Vec2(-1, 0), Vec2(0, -1), 2.0, 0.0).empty());       // no room
}
