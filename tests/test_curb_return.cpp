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

namespace {
// The tightest bend along a polyline as a radius (circumradius of each triple; +inf for a
// straight run). Mirrors the metric fairHermite caps against.
double minCircum(const std::vector<Vec2>& poly) {
    double worst = 1e30;
    for (std::size_t i = 1; i + 1 < poly.size(); ++i) {
        const Vec2 &a = poly[i - 1], &b = poly[i], &c = poly[i + 1];
        double ab = len(b - a), bc = len(c - b), ca = len(a - c);
        double area2 = std::fabs(cross(b - a, c - a));
        if (area2 < 1e-9) continue;
        worst = std::min(worst, (ab * bc * ca) / (2.0 * area2));
    }
    return worst;
}
}

TEST_CASE(fair_hermite_relaxes_an_over_tight_curve_to_the_min_radius) {
    // A short edge with huge tangents loops back on itself (radius far below 6 m).
    Vec2 p0(0, 0), p1(4, 0), m0(0, 30), m1(0, -30);
    std::vector<Vec2> tight = fairHermite(p0, m0, p1, m1, 24, 0.0);   // no cap: stays tight
    CHECK(minCircum(tight) < 6.0);
    std::vector<Vec2> fair = fairHermite(p0, m0, p1, m1, 24, 6.0);    // capped at 6 m
    CHECK(minCircum(fair) >= 6.0 - 0.5);                              // no longer over-tight
    CHECK(fair.front().x == 0.0 && fair.back().x == 4.0);            // endpoints preserved
}

TEST_CASE(fair_hermite_leaves_a_gentle_curve_untouched) {
    // Gentle tangents -> a wide curve already well above the limit: returned as-is.
    Vec2 p0(0, 0), p1(40, 0), m0(20, 8), m1(20, -8);
    std::vector<Vec2> ref = fairHermite(p0, m0, p1, m1, 16, 0.0);
    std::vector<Vec2> capped = fairHermite(p0, m0, p1, m1, 16, 6.0);
    CHECK(capped.size() == ref.size());
    CHECK(minCircum(capped) > 6.0);
    for (std::size_t i = 0; i < ref.size(); ++i)
        CHECK(len(capped[i] - ref[i]) < 1e-9);          // identical samples (scale stayed 1)
}

namespace {
double triArea(const Vec2& a, const Vec2& b, const Vec2& c) {
    return std::fabs(cross(b - a, c - a)) * 0.5;
}
double shoelace(const std::vector<Vec2>& p) {
    double a = 0.0;
    int n = static_cast<int>(p.size());
    for (int i = 0; i < n; ++i) a += cross(p[i], p[(i + 1) % n]);
    return std::fabs(a) * 0.5;
}
double sumTriAreas(const std::vector<Vec2>& p, const std::vector<std::array<int, 3>>& tris) {
    double a = 0.0;
    for (const auto& t : tris) a += triArea(p[t[0]], p[t[1]], p[t[2]]);
    return a;
}
}

namespace {
double maxGradeOf(const std::vector<double>& y, const std::vector<double>& s) {
    double g = 0.0;
    for (std::size_t i = 1; i < y.size(); ++i) {
        double ds = s[i] - s[i - 1];
        if (ds > 1e-9) g = std::max(g, std::fabs(y[i] - y[i - 1]) / ds);
    }
    return g;
}
std::vector<double> arcLengths(int n, double step) {
    std::vector<double> s(n);
    for (int i = 0; i < n; ++i) s[i] = i * step;
    return s;
}
}

TEST_CASE(road_profile_ramps_a_step_within_the_grade_budget) {
    // A 4 m step over 70 m — climbable at 10% (budget 7 m) — but concentrated as a cliff
    // between two samples. The profile spreads it so no step exceeds the grade.
    std::vector<double> ground = { 0, 0, 0, 0, 4, 4, 4, 4 };
    std::vector<double> s = arcLengths(8, 10.0);          // 10 m spacing
    std::vector<double> y = roadProfile(ground, s, 0.10); // max 10% grade
    CHECK(maxGradeOf(y, s) <= 0.10 + 1e-6);               // drivable everywhere
    CHECK_APPROX(y.front(), 0.0, 0.6);
    CHECK_APPROX(y.back(), 4.0, 0.6);                    // the achievable rise is kept
    CHECK((y[4] - y[3]) < 4.0);                          // the cliff step is gone
}

TEST_CASE(road_profile_clamps_an_unclimbable_slope_for_cut) {
    // A 20 m rise over 70 m needs ~29% grade; at 10% the road can only climb ~7 m, so it
    // deliberately falls short of the terrain — the terrain is cut down to meet it.
    std::vector<double> ground = { 0, 0, 0, 0, 20, 20, 20, 20 };
    std::vector<double> s = arcLengths(8, 10.0);
    std::vector<double> y = roadProfile(ground, s, 0.10);
    CHECK(maxGradeOf(y, s) <= 0.10 + 1e-6);               // still drivable
    CHECK(y.back() < 12.0);                               // can't reach 20 -> needs a cut
}

TEST_CASE(road_profile_leaves_flat_and_gentle_ground_alone) {
    std::vector<double> s = arcLengths(6, 20.0);
    std::vector<double> flat = { 5, 5, 5, 5, 5, 5 };
    std::vector<double> yf = roadProfile(flat, s, 0.10);
    CHECK(maxGradeOf(yf, s) < 1e-9);                      // stays flat
    // A gentle 4% ramp is under the 10% limit -> essentially preserved.
    std::vector<double> ramp = { 0, 0.8, 1.6, 2.4, 3.2, 4.0 };
    std::vector<double> yr = roadProfile(ramp, s, 0.10);
    CHECK(maxGradeOf(yr, s) <= 0.045);
    CHECK_APPROX(yr.back() - yr.front(), 4.0, 0.5);       // overall rise preserved
}

TEST_CASE(road_conform_carves_terrain_to_the_profile) {
    // A straight road climbing 0 -> 5 m over 40 m. On the centerline the terrain must be
    // graded to the road profile; far off the road it must stay natural ground.
    std::vector<Vec2> cl = { Vec2(0, 0), Vec2(10, 0), Vec2(20, 0), Vec2(30, 0), Vec2(40, 0) };
    std::vector<double> prof = { 0, 1.25, 2.5, 3.75, 5 };
    auto regions = roadConformRegions(cl, prof, 4.0, 1.0, 6.0);   // hw 4 + shoulder 1
    CHECK(static_cast<int>(regions.size()) == 4);
    double base = 100.0;                                          // natural ground far above
    CHECK_APPROX(applyFlatten(regions, 20, 0, base), 2.5, 0.3);  // on road -> the profile
    CHECK_APPROX(applyFlatten(regions, 20, 40, base), base, 1e-6);  // far off road -> natural
}

TEST_CASE(triangulate_convex_tiles_the_polygon) {
    std::vector<Vec2> sq = { Vec2(0, 0), Vec2(4, 0), Vec2(4, 4), Vec2(0, 4) };
    auto tris = triangulatePolygon(sq);
    CHECK(tris.size() == 2);                                  // n-2 triangles
    CHECK_APPROX(sumTriAreas(sq, tris), 16.0, 1e-9);         // exact tiling, no overlap/gap
}

TEST_CASE(triangulate_nonconvex_notch_tiles_without_overlap) {
    // A square with the top edge pushed into a V — (3,3) is a reflex vertex, exactly the
    // case where fanning from an interior point self-overlaps. Ear clipping must tile it.
    std::vector<Vec2> notch = { Vec2(0, 0), Vec2(6, 0), Vec2(6, 6), Vec2(3, 3), Vec2(0, 6) };
    auto tris = triangulatePolygon(notch);
    CHECK(static_cast<int>(tris.size()) == static_cast<int>(notch.size()) - 2);
    CHECK_APPROX(sumTriAreas(notch, tris), shoelace(notch), 1e-9);   // tiles exactly (=27)
    for (const auto& t : tris)
        CHECK(triArea(notch[t[0]], notch[t[1]], notch[t[2]]) > 1e-9);   // none degenerate
}

TEST_CASE(triangulate_dedupes_a_ring_with_coincident_vertices) {
    // A sharp junction's ring repeats coincident mouth corners (cornerRadius 0). The
    // triangulator must dedupe and still tile the underlying square (covers the centre).
    std::vector<Vec2> ring = { Vec2(5, -5), Vec2(5, 5), Vec2(5, 5), Vec2(-5, 5),
                               Vec2(-5, 5), Vec2(-5, -5), Vec2(-5, -5), Vec2(5, -5) };
    auto tris = triangulatePolygon(ring);
    CHECK(tris.size() == 2);                                  // dedups to a 4-vert square
    CHECK_APPROX(sumTriAreas(ring, tris), 100.0, 1e-9);     // tiles the full 10x10
}

TEST_CASE(triangulate_handles_cw_and_degenerate) {
    // Clockwise input is normalized to CCW internally and still tiles.
    std::vector<Vec2> cw = { Vec2(0, 0), Vec2(0, 4), Vec2(4, 4), Vec2(4, 0) };
    CHECK_APPROX(sumTriAreas(cw, triangulatePolygon(cw)), 16.0, 1e-9);
    CHECK(triangulatePolygon({ Vec2(0, 0), Vec2(1, 1) }).empty());     // < 3 verts
}

TEST_CASE(curb_fillet_degenerates_to_empty) {
    Vec2 C(0, 0);
    CHECK(curbReturnFillet(C, Vec2(1, 0), Vec2(1, 0.001), 2.0, 100.0).empty());   // folded/parallel
    CHECK(curbReturnFillet(C, Vec2(1, 0), Vec2(-0.999, 0.045), 2.0, 100.0).empty()); // near-straight
    CHECK(curbReturnFillet(C, Vec2(-1, 0), Vec2(0, -1), 0.0, 100.0).empty());     // no radius
    CHECK(curbReturnFillet(C, Vec2(-1, 0), Vec2(0, -1), 2.0, 0.0).empty());       // no room
}
