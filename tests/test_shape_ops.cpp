#include "test_framework.h"

#include "../src/engine/procgen/city/shape_ops.h"
#include <cmath>

using namespace engine;

namespace {
constexpr Real kTau = 6.28318530717958647692;
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }

// Total area across a shape list — the quantity every boolean invariant is
// written in.
Real totalArea(const std::vector<Shape2>& v) {
    Real a = 0;
    for (const Shape2& s : v) a += area(s);
    return a;
}
}  // namespace

// --- booleans --------------------------------------------------------------

TEST_CASE(ops_union_conserves_area_on_overlap) {
    Shape2 a = rectShape(0, 0, 20, 20);
    Shape2 b = rectShape(10, 10, 30, 30);
    std::vector<Shape2> u = unite(a, b);
    CHECK(u.size() == 1);
    // 400 + 400 - 100 overlap.
    CHECK(near(totalArea(u), 700, 0.01));
}

TEST_CASE(ops_union_of_disjoint_stays_two_shapes) {
    Shape2 a = rectShape(0, 0, 10, 10);
    Shape2 b = rectShape(50, 50, 60, 60);
    std::vector<Shape2> u = unite(a, b);
    CHECK(u.size() == 2);
    CHECK(near(totalArea(u), 200, 0.01));
}

TEST_CASE(ops_subtract_punches_a_real_hole) {
    Shape2 plate = rectShape(0, 0, 40, 40);
    Shape2 well = rectShape(15, 15, 25, 25);      // fully interior: a light well
    std::vector<Shape2> r = subtract(plate, well);
    CHECK(r.size() == 1);
    // The result must be a shape WITH A HOLE, not two shapes and not a shape
    // whose outline detours around the well.
    CHECK(r[0].holes.size() == 1);
    CHECK(near(area(r[0]), 1600 - 100, 0.01));
    CHECK(contains(r[0], Vec2(5, 5)));
    CHECK(!contains(r[0], Vec2(20, 20)));
}

TEST_CASE(ops_subtract_can_split_one_shape_into_two) {
    Shape2 plate = rectShape(0, 0, 40, 20);
    Shape2 bar = rectShape(18, -5, 22, 25);       // a bar clean across the middle
    std::vector<Shape2> r = subtract(plate, bar);
    // The caller MUST be able to see that one lot became two buildable pieces.
    CHECK(r.size() == 2);
    CHECK(near(totalArea(r), 40 * 20 - 4 * 20, 0.01));
}

TEST_CASE(ops_intersect_is_the_common_part) {
    Shape2 a = rectShape(0, 0, 20, 20);
    Shape2 b = rectShape(10, 10, 30, 30);
    std::vector<Shape2> r = intersect(a, b);
    CHECK(r.size() == 1);
    CHECK(near(totalArea(r), 100, 0.01));
    CHECK(contains(r[0], Vec2(15, 15)));
    CHECK(!contains(r[0], Vec2(5, 5)));
}

TEST_CASE(ops_boolean_works_at_any_angle) {
    // The rectilinear case can be faked; a pentagon's walls are at 72 degrees.
    Pen p;
    p.moveTo(0, 0);
    for (int i = 0; i < 5; ++i) p.forward(20).turn(72);
    Shape2 penta = p.close();
    const Real a0 = area(penta);
    CHECK(a0 > 0);
    Shape2 knife = rectShape(-50, -50, 50, 0.0001);   // trims nothing much
    std::vector<Shape2> u = unite(penta, rectShape(5, 5, 8, 8));
    // A small rect wholly inside must not change the area at all.
    CHECK(u.size() == 1);
    CHECK(near(totalArea(u), a0, 0.05));
    // ... and cutting it out must remove exactly its area.
    std::vector<Shape2> s = subtract(penta, rectShape(5, 5, 8, 8));
    CHECK(near(totalArea(s), a0 - 9, 0.05));
    (void)knife;
}

// The headline claim of the kernel: a boolean must not permanently flatten a
// curve it passed over.
TEST_CASE(ops_boolean_preserves_untouched_arcs) {
    Shape2 disc = circleShape(Vec2(0, 0), 10);
    const Real full = area(disc);
    CHECK(near(full, kTau * 0.5 * 100, 1e-6));

    // Unite with a small rectangle that pokes out of one side. The three
    // quarter-arcs it never touches must come back as ARCS, not chord fans.
    Shape2 tab = rectShape(9, -2, 14, 2);
    std::vector<Shape2> u = unite(disc, tab);
    CHECK(u.size() == 1);
    int arcs = 0;
    for (const Edge2& e : u[0].outer.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs >= 2);                      // the untouched quarters survived
    // Area must still be dominated by the true disc, not by an inscribed
    // polygon: a chord-fan result would lose several square metres.
    CHECK(totalArea(u) > full);
    CHECK(totalArea(u) < full + 40);
}

TEST_CASE(ops_boolean_split_arc_stays_on_its_circle) {
    Shape2 disc = circleShape(Vec2(0, 0), 10);
    // Cut the disc in half. The surviving semicircle must still measure like a
    // half-disc (157.08), not like a half-polygon.
    Shape2 knife = rectShape(-20, -20, 20, 0);
    std::vector<Shape2> r = subtract(disc, knife);
    CHECK(r.size() == 1);
    CHECK(near(totalArea(r), kTau * 0.25 * 100, 0.5));
    int arcs = 0;
    for (const Edge2& e : r[0].outer.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs >= 1);                      // came back curved
}

TEST_CASE(ops_boolean_preserves_holes_in_the_operand) {
    Shape2 donut = rectShape(0, 0, 40, 40);
    donut.holes.push_back(rectShape(15, 15, 25, 25).outer);
    orient(donut);
    CHECK(near(area(donut), 1500, 1e-6));
    // Uniting a small disjoint square must leave the court untouched.
    std::vector<Shape2> u = unite(donut, rectShape(50, 0, 60, 10));
    CHECK(u.size() == 2);
    CHECK(near(totalArea(u), 1500 + 100, 0.01));
    bool foundCourt = false;
    for (const Shape2& s : u) if (!s.holes.empty()) foundCourt = true;
    CHECK(foundCourt);
}

TEST_CASE(ops_unite_all_folds_a_pile) {
    std::vector<Shape2> parts;
    parts.push_back(rectShape(0, 0, 10, 10));
    parts.push_back(rectShape(8, 0, 18, 10));      // overlaps the first by 2
    parts.push_back(rectShape(16, 0, 26, 10));     // overlaps the second by 2
    std::vector<Shape2> u = uniteAll(parts);
    CHECK(u.size() == 1);
    CHECK(near(totalArea(u), 26 * 10, 0.01));
}

// --- offset ----------------------------------------------------------------

TEST_CASE(ops_offset_round_trips_on_a_convex_shape) {
    Shape2 s = rectShape(0, 0, 40, 30);
    std::vector<Shape2> grown = offsetShape(s, 5);
    CHECK(grown.size() == 1);
    CHECK(near(area(grown[0]), 50 * 40, 0.5));
    std::vector<Shape2> back = offsetShape(grown[0], -5);
    CHECK(back.size() == 1);
    CHECK(near(area(back[0]), 40 * 30, 1.0));      // returned to where it began
}

TEST_CASE(ops_offset_collapses_a_shape_it_consumes) {
    Shape2 s = rectShape(0, 0, 10, 10);
    std::vector<Shape2> r = offsetShape(s, -6);    // deeper than the half-width
    CHECK(r.empty());
}

// The plan's motivating case: an inward offset that CHANGES TOPOLOGY. The old
// inset() returns a fixed vertex count and simply cannot express this.
TEST_CASE(ops_offset_splits_a_u_into_two) {
    // A U: 40 wide, 30 tall, with a 16-wide slot cut down the middle from the
    // top leaving 7 m arms.
    Shape2 u = rectShape(0, 0, 40, 30);
    std::vector<Shape2> cut = subtract(u, rectShape(12, 8, 28, 31));
    CHECK(cut.size() == 1);
    // Shrinking by 4 m eats the 8 m base bridge and leaves the two arms.
    std::vector<Shape2> r = offsetShape(cut[0], -4.5);
    CHECK(r.size() == 2);
    for (const Shape2& s : r) CHECK(area(s) > 1.0);
}

TEST_CASE(ops_offset_merges_two_neighbours_into_one) {
    Shape2 a = rectShape(0, 0, 10, 10);
    Shape2 b = rectShape(13, 0, 23, 10);           // a 3 m gap
    std::vector<Shape2> pair = unite(a, b);
    CHECK(pair.size() == 2);
    std::vector<Shape2> grown = fieldBool({a}, {b}, BoolOp::Unite, 0.2);
    CHECK(grown.size() == 2);                      // still apart at zero offset
    // Grow each by 2 m and they touch: the union becomes ONE shape.
    std::vector<Shape2> ga = offsetShape(a, 2), gb = offsetShape(b, 2);
    CHECK(ga.size() == 1 && gb.size() == 1);
    std::vector<Shape2> merged = unite(ga[0], gb[0]);
    CHECK(merged.size() == 1);
}

TEST_CASE(ops_per_edge_offset_makes_setbacks_a_program_property) {
    Shape2 lot = rectShape(0, 0, 20, 30);
    // front (south) 7, sides 3, rear 8 — the whole point of per-edge offsets.
    lot.outer.edges[0].tag = EdgeTag::Street;   // y = 0
    lot.outer.edges[1].tag = EdgeTag::Side;     // x = 20
    lot.outer.edges[2].tag = EdgeTag::Rear;     // y = 30
    lot.outer.edges[3].tag = EdgeTag::Side;     // x = 0
    TagOffsets t;
    t.street = -7; t.side = -3; t.rear = -8;
    Shape2 pad = offsetByTag(lot, t);
    CHECK(pad.outer.size() == 4);
    // 20 - 3 - 3 wide, 30 - 7 - 8 deep.
    CHECK(near(area(pad), 14.0 * 15.0, 0.01));
    Vec2 lo, hi;
    bounds(pad, lo, hi);
    CHECK(near(lo.x, 3, 0.01) && near(hi.x, 17, 0.01));
    CHECK(near(lo.y, 7, 0.01) && near(hi.y, 22, 0.01));
}

TEST_CASE(ops_offset_keeps_a_curve_concentric) {
    Shape2 disc = circleShape(Vec2(0, 0), 10);
    std::vector<Shape2> grown = offsetShape(disc, 3);
    CHECK(grown.size() == 1);
    // An offset circle is a CIRCLE of radius 13 — if the arcs had been
    // flattened first, the area would fall short of pi*13^2.
    CHECK(near(area(grown[0]), kTau * 0.5 * 169, 1.0));
    int arcs = 0;
    for (const Edge2& e : grown[0].outer.edges)
        if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 4);
}

// --- corners ---------------------------------------------------------------

TEST_CASE(ops_fillet_rounds_with_a_true_arc) {
    Shape2 s = rectShape(0, 0, 20, 20);
    Shape2 f = filletAll(s, 4);
    CHECK(f.outer.size() == 8);            // each corner became arc + wall
    int arcs = 0;
    for (const Edge2& e : f.outer.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 4);
    // Area lost per corner is r^2 - (pi/4)r^2 = r^2(1 - pi/4) = 3.43 for r = 4.
    CHECK(near(area(s) - area(f), 4.0 * 16.0 * (1.0 - kTau / 8.0), 0.01));
}

TEST_CASE(ops_round_to_circle_actually_reaches_a_circle) {
    Shape2 sq = rectShape(0, 0, 20, 20);
    Shape2 half = roundToCircle(sq, 0.5);
    Shape2 full = roundToCircle(sq, 1.0);
    CHECK(area(full) < area(half));
    CHECK(area(half) < area(sq));
    // At t = 1 a square of side 20 becomes a circle of radius 10: pi*100.
    CHECK(near(area(full), kTau * 0.5 * 100, 0.5));
}

TEST_CASE(ops_chamfer_cuts_the_corner_straight) {
    Shape2 s = rectShape(0, 0, 20, 20);
    Shape2 c = chamferAll(s, 4);
    CHECK(c.outer.size() == 8);
    for (const Edge2& e : c.outer.edges) CHECK(std::fabs(e.bulge) < 1e-9);
    // A 4 m setback on each leg removes a 4x4 right triangle per corner.
    CHECK(near(area(s) - area(c), 4 * 0.5 * 16, 0.01));
}

TEST_CASE(ops_fillet_never_overruns_a_short_edge) {
    // A radius far larger than the shape must clamp, not invert the loop.
    Shape2 s = rectShape(0, 0, 6, 6);
    Shape2 f = filletAll(s, 100);
    CHECK(area(f) > 0);
    CHECK(area(f) < area(s));
    Vec2 lo, hi;
    bounds(f, lo, hi);
    CHECK(lo.x > -0.5 && hi.x < 6.5);      // still inside the original box
}

// --- field path ------------------------------------------------------------

TEST_CASE(ops_field_boolean_agrees_with_the_exact_one) {
    Shape2 a = rectShape(0, 0, 20, 20);
    Shape2 b = rectShape(10, 10, 30, 30);
    const Real exact = totalArea(unite(a, b));
    const Real field = totalArea(fieldBool({a}, {b}, BoolOp::Unite, 0.25));
    // The field is a sampled approximation; agreeing to ~1% is the contract.
    CHECK(near(field, exact, exact * 0.02));
}

TEST_CASE(ops_smooth_union_adds_a_fillet_of_material) {
    Shape2 a = rectShape(0, 0, 20, 10);
    Shape2 b = rectShape(18, 0, 38, 10);
    const Real hard = totalArea(unite(a, b));
    std::vector<Shape2> blended = smoothUnion(a, b, 3.0, 0.2);
    CHECK(!blended.empty());
    // A blend ADDS material in the joint — measured, not assumed. (The Lot Lab
    // found the same thing: a smooth-min weld grows the shape.)
    CHECK(totalArea(blended) > hard);
    CHECK(totalArea(blended) < hard * 1.3);
}

TEST_CASE(ops_contour_orients_holes_by_nesting) {
    Shape2 donut = rectShape(0, 0, 40, 40);
    donut.holes.push_back(rectShape(15, 15, 25, 25).outer);
    orient(donut);
    Field2 f = rasterize({donut}, 0.4, 2.0);
    std::vector<Poly2> loops = contour(f, 0.0);
    CHECK(loops.size() == 2);
    // Exactly one outer (CCW) and one hole (CW), decided by nesting depth.
    int ccw = 0, cw = 0;
    for (const Poly2& l : loops) (signedArea(l) > 0 ? ccw : cw)++;
    CHECK(ccw == 1 && cw == 1);
}

TEST_CASE(ops_determinism_same_input_same_output) {
    Shape2 a = rectShape(0, 0, 20, 20);
    Shape2 b = circleShape(Vec2(18, 18), 8);
    std::vector<Shape2> r1 = unite(a, b);
    std::vector<Shape2> r2 = unite(a, b);
    CHECK(r1.size() == r2.size());
    for (std::size_t i = 0; i < r1.size(); ++i) {
        CHECK(r1[i].outer.size() == r2[i].outer.size());
        for (std::size_t j = 0; j < r1[i].outer.size(); ++j) {
            CHECK((r1[i].outer.edges[j].a - r2[i].outer.edges[j].a).length() < 1e-12);
            CHECK(near(r1[i].outer.edges[j].bulge, r2[i].outer.edges[j].bulge, 1e-12));
        }
    }
}

TEST_CASE(ops_growing_a_donut_shrinks_its_court) {
    // The Minkowski view: growing the solid makes it encroach on the court, so
    // a donut offset outward has a SMALLER hole, not a bigger one. Getting this
    // sign backwards is invisible on a solid shape and wrong on every courtyard
    // building in the city.
    Shape2 donut = rectShape(0, 0, 40, 40);
    donut.holes.push_back(rectShape(15, 15, 25, 25).outer);
    orient(donut);
    const Real court0 = area(donut.holes[0]);
    CHECK(near(court0, 100, 1e-6));

    Shape2 grown = offsetEdges(donut, std::vector<Real>(donut.outer.size(), 2.0));
    CHECK(grown.outer.size() == 4);
    CHECK(grown.holes.size() == 1);
    CHECK(near(area(grown.outer), 44 * 44, 0.01));       // outer grew
    CHECK(area(grown.holes[0]) < court0);                // court shrank
    CHECK(near(area(grown.holes[0]), 6 * 6, 0.01));      // by 2 m on each side
}

TEST_CASE(ops_tag_offsets_reach_the_courtyard_walls) {
    Shape2 block = rectShape(0, 0, 60, 60);
    for (Edge2& e : block.outer.edges) e.tag = EdgeTag::Street;
    Shape2 court = rectShape(20, 20, 40, 40);
    block.holes.push_back(court.outer);
    orient(block);
    for (Edge2& e : block.holes[0].edges) e.tag = EdgeTag::Court;

    TagOffsets t;
    t.street = -4;      // pull the street wall in
    t.court = 3;        // and push the courtyard wall out (a bigger court)
    Shape2 r = offsetByTag(block, t);
    CHECK(r.outer.size() == 4);
    CHECK(r.holes.size() == 1);
    CHECK(near(area(r.outer), 52 * 52, 0.01));
    // A positive offset on a hole grows the SOLID into the court, so +3 shrinks
    // the court to 14x14 — the same convention the donut test pins down.
    CHECK(near(area(r.holes[0]), 14 * 14, 0.01));
}
