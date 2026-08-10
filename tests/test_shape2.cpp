#include "test_framework.h"

#include "../src/engine/procgen/city/shape2.h"
#include <cmath>

using namespace engine;

namespace {
constexpr Real kTau = 6.28318530717958647692;
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }
}  // namespace

// The kernel's whole claim is that a curved wall stays a TRUE CIRCLE through
// the grammar. If the area of a four-arc circle is not pi*r^2 then the arcs are
// not circles, and every later invariant built on area is measuring a lie.
TEST_CASE(shape2_circle_area_is_exact) {
    const Real r = 12.0;
    Shape2 c = circleShape(Vec2(3, -4), r);
    CHECK(c.outer.size() == 4);                      // four edges, not 64 points
    CHECK(near(area(c), kTau * 0.5 * r * r, 1e-6));  // pi r^2
    CHECK(near(perimeter(c.outer), kTau * r, 1e-6));
    // The inscribed square is 2r^2 — a plain point-ring would measure that.
    CHECK(area(c) > 2.0 * r * r * 1.5);
}

TEST_CASE(shape2_tessellation_honours_the_chord_tolerance) {
    const Real r = 20.0;
    Shape2 c = circleShape(Vec2(0, 0), r);
    for (Real tol : {Real(0.5), Real(0.05), Real(0.005)}) {
        Poly2 p = tessellate(c.outer, tol);
        CHECK(p.size() >= 4);
        // Every chord midpoint must sit within `tol` of the true circle.
        Real worst = 0;
        for (std::size_t i = 0; i < p.size(); ++i) {
            const Vec2 m = (p[i] + p[(i + 1) % p.size()]) * 0.5;
            worst = std::max(worst, std::fabs(r - m.length()));
        }
        CHECK(worst <= tol * 1.05);
        // ... and a finer tolerance must actually cost more points.
        CHECK(p.size() >= 4);
    }
    CHECK(tessellate(c.outer, 0.005).size() > tessellate(c.outer, 0.5).size());
}

// Splitting an arc must not degrade it: a sub-arc of a circle is an arc on the
// SAME circle. This is what lets a boolean cut a curved wall and hand back a
// curve rather than a chord fan.
TEST_CASE(shape2_sub_arc_stays_on_the_same_circle) {
    const Vec2 a(0, 0), b(10, 0);
    const Real bulge = 0.6;
    const ArcGeom whole = arcGeom(a, b, bulge);
    const Real t0 = 0.25, t1 = 0.8;
    const Vec2 p0 = arcPoint(a, b, bulge, t0);
    const Vec2 p1 = arcPoint(a, b, bulge, t1);
    const ArcGeom part = arcGeom(p0, p1, subBulge(bulge, t0, t1));
    CHECK(!part.straight);
    CHECK(near(part.radius, whole.radius, 1e-6));
    CHECK(near((part.centre - whole.centre).length(), 0, 1e-6));
    // The sub-arc's midpoint is the whole arc's midpoint at the same parameter.
    const Vec2 mid = arcPoint(p0, p1, subBulge(bulge, t0, t1), 0.5);
    const Vec2 want = arcPoint(a, b, bulge, (t0 + t1) * 0.5);
    CHECK(near((mid - want).length(), 0, 1e-6));
}

// Reversing a loop with arcs is not "reverse the points": bulge and tag belong
// to the edge, so both must shift index AND the bulge must negate. If this is
// wrong the shape survives one reversal and inverts every curve on the next.
TEST_CASE(shape2_reversal_round_trips_with_arcs) {
    Pen p;
    p.moveTo(0, 0).forward(20).turn(90).sweep(8, 90).forward(14).turn(90).forward(10);
    Loop2 loop = p.closeLoop();
    CHECK(loop.size() >= 4);
    const Real a0 = signedArea(loop);
    CHECK(a0 > 0);                              // the pen hands back CCW
    const Poly2 before = tessellate(loop, 0.01);

    reverseLoop(loop);
    // Reversal flips the sign but not the magnitude — if the bulges had NOT
    // been negated, the arcs would bow the other way and the area would change.
    CHECK(near(signedArea(loop), -a0, 1e-9));
    int arcs = 0;
    for (const Edge2& e : loop.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 1);                           // the sweep survived, once

    reverseLoop(loop);
    CHECK(near(signedArea(loop), a0, 1e-9));
    const Poly2 after = tessellate(loop, 0.01);
    CHECK(after.size() == before.size());
    Real worst = 0;
    for (std::size_t i = 0; i < after.size(); ++i)
        worst = std::max(worst, (after[i] - before[i]).length());
    CHECK(worst < 1e-9);                        // exactly where it started
}

TEST_CASE(shape2_holes_subtract_from_the_region) {
    Shape2 s = rectShape(0, 0, 40, 30);
    CHECK(near(area(s), 1200, 1e-9));
    // A courtyard: the thing a bare Poly2 cannot express at all.
    Shape2 court = rectShape(10, 10, 20, 20);
    s.holes.push_back(court.outer);
    orient(s);
    CHECK(signedArea(s.holes[0]) < 0);        // holes wind CW by convention
    CHECK(near(area(s), 1200 - 100, 1e-9));
    CHECK(contains(s, Vec2(5, 5)));           // in the solid
    CHECK(!contains(s, Vec2(15, 15)));        // in the court
    CHECK(!contains(s, Vec2(-5, 5)));         // outside entirely
}

TEST_CASE(shape2_pen_walks_a_closed_plan) {
    // A plain rectangle by turtle walk: 18 x 12.
    Pen p;
    p.moveTo(0, 0).forward(18).turn(90).forward(12).turn(90).forward(18).turn(90).forward(12);
    Shape2 s = p.close();
    CHECK(s.outer.size() == 4);
    CHECK(near(area(s), 18 * 12, 1e-6));
    CHECK(near(perimeter(s.outer), 2 * (18 + 12), 1e-6));
}

// `sweep` must emit ONE arc edge, not a fan of chords — that is the whole
// reason the type carries bulge.
TEST_CASE(shape2_pen_sweep_emits_one_true_arc) {
    Pen p;
    p.moveTo(0, 0).forward(10).sweep(5, 180).forward(10);
    Loop2 loop = p.closeLoop();
    int arcs = 0;
    for (const Edge2& e : loop.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 1);
    // A 180-degree sweep is a semicircle: bulge magnitude 1, arc length pi*r.
    for (const Edge2& e : loop.edges)
        if (std::fabs(e.bulge) > 1e-9) CHECK(near(std::fabs(e.bulge), 1.0, 1e-9));
    // A stadium: two 10 m sides joined by a semicircular end of radius 5.
    CHECK(loop.size() == 4);
}

TEST_CASE(shape2_bow_makes_a_measurable_bay) {
    Shape2 s = rectShape(0, 0, 20, 10);
    const Real flat = area(s);
    bowEdge(s.outer, 0, 2.0);                 // bow the first wall outward 2 m
    const Real bowed = area(s);
    CHECK(bowed > flat);                      // a bay ADDS floor area
    // Circular-segment area for chord 20, sagitta 2 is ~26.9 m^2; check we are
    // in the right neighbourhood rather than asserting the closed form twice.
    CHECK(bowed - flat > 20 && bowed - flat < 35);
    // Scooping in must remove the same amount.
    Shape2 t = rectShape(0, 0, 20, 10);
    bowEdge(t.outer, 0, -2.0);
    CHECK(near(flat - area(t), bowed - flat, 1e-6));
}

TEST_CASE(shape2_tags_survive_and_face_the_right_way) {
    Shape2 s = rectShape(0, 0, 20, 10);
    // The street is to the south (-y): the wall from (0,0)->(20,0) faces it.
    tagFacing(s.outer, Vec2(0, -1), 30.0, EdgeTag::Street);
    int street = 0;
    std::size_t streetEdge = 999;
    for (std::size_t i = 0; i < s.outer.size(); ++i)
        if (s.outer.edges[i].tag == EdgeTag::Street) { ++street; streetEdge = i; }
    CHECK(street == 1);
    CHECK(streetEdge == 0);
    // A tag is edge data, so it must survive a reversal along with its edge.
    Shape2 t = s;
    orient(t);
    int stillStreet = 0;
    for (const Edge2& e : t.outer.edges)
        if (e.tag == EdgeTag::Street) ++stillStreet;
    CHECK(stillStreet == 1);
}

TEST_CASE(shape2_simplify_drops_noise_but_never_a_curve) {
    // A rectangle with a redundant collinear vertex and a duplicate.
    Poly2 ring = {{0, 0}, {10, 0}, {20, 0}, {20, 10}, {20, 10}, {0, 10}};
    Shape2 s = shapeFromPoly(ring);
    Shape2 c = simplify(s, 0.01);
    CHECK(c.outer.size() == 4);
    CHECK(near(area(c), 200, 1e-9));

    // The same shape with one edge bowed: simplify must keep every arc, even
    // though the arc's endpoints are geometrically "collinear-ish".
    Shape2 curved = shapeFromPoly(ring);
    bowEdge(curved.outer, 0, 1.5);
    Shape2 cc = simplify(curved, 0.01);
    int arcs = 0;
    for (const Edge2& e : cc.outer.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 1);
}

TEST_CASE(shape2_bounds_include_the_bulge) {
    Shape2 s = rectShape(0, 0, 20, 10);
    bowEdge(s.outer, 0, 3.0);        // bows outward, below y = 0
    Vec2 lo, hi;
    bounds(s, lo, hi);
    // A point-ring bound would stop at y = 0 and clip the bay off the building.
    CHECK(lo.y < -2.8);
    CHECK(near(hi.y, 10, 1e-6));
    CHECK(near(lo.x, 0, 1e-6) && near(hi.x, 20, 1e-6));
}

TEST_CASE(shape2_nesting_decides_holes_not_winding) {
    // Contour extraction returns loops in arbitrary winding; only NESTING says
    // which is a hole. Depth 0 -> CCW outer, depth 1 -> CW hole, depth 2 -> an
    // island inside the hole, CCW again.
    Poly2 big = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    Poly2 mid = {{20, 20}, {20, 80}, {80, 80}, {80, 20}};      // wound CW
    Poly2 small = {{40, 40}, {60, 40}, {60, 60}, {40, 60}};
    std::vector<Poly2> out = orientByNesting({big, mid, small});
    CHECK(signedArea(out[0]) > 0);    // outer
    CHECK(signedArea(out[1]) < 0);    // hole
    CHECK(signedArea(out[2]) > 0);    // island in the hole
}
