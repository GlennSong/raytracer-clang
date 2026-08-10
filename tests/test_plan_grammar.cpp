#include "test_framework.h"

#include "../src/engine/procgen/city/plan_grammar.h"
#include "../src/engine/procgen/city/mass_stack.h"
#include <cmath>
#include <string>

using namespace engine;

namespace {
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }
Real totalArea(const std::vector<Shape2>& v) {
    Real a = 0;
    for (const Shape2& s : v) a += area(s);
    return a;
}
}  // namespace

// --- selectors -------------------------------------------------------------

TEST_CASE(plan_selectors_name_walls_without_vertex_numbers) {
    Shape2 s = rectShape(0, 0, 30, 10);      // long walls run along x
    CHECK(EdgeSelector::all().select(s.outer).size() == 4);
    CHECK(EdgeSelector::longest(2).select(s.outer).size() == 2);
    // The two longest are the 30 m walls: indices 0 and 2.
    std::vector<std::size_t> lng = EdgeSelector::longest(2).select(s.outer);
    CHECK(lng[0] == 0 && lng[1] == 2);
    CHECK(EdgeSelector::shortest(1).select(s.outer).size() == 1);
    CHECK(EdgeSelector::longerThan(20).select(s.outer).size() == 2);
    // Facing: edge 0 (y = 0) has outward normal (0,-1).
    std::vector<std::size_t> south = EdgeSelector::facing(Vec2(0, -1), 30).select(s.outer);
    CHECK(south.size() == 1 && south[0] == 0);
    // Tags.
    s.outer.edges[2].tag = EdgeTag::Rear;
    CHECK(EdgeSelector::byTag(EdgeTag::Rear).select(s.outer).size() == 1);
}

TEST_CASE(plan_selectors_survive_a_changed_vertex_count) {
    // The point of selectors: the SAME rule must work after the plan's shape
    // has changed under it. A chamfer doubles the vertex count.
    Shape2 s = rectShape(0, 0, 30, 10);
    Shape2 c = chamferAll(s, 2);
    CHECK(c.outer.size() == 8);
    // Still exactly two walls longer than 20 m, without knowing any indices.
    CHECK(EdgeSelector::longerThan(20).select(c.outer).size() == 2);
}

// --- quality invariant -----------------------------------------------------

TEST_CASE(plan_quality_rejects_knife_edges_and_slivers) {
    PlanQuality q;
    CHECK(q.ok(rectShape(0, 0, 20, 12)));

    // A knife edge: a triangle with a near-zero interior angle.
    Poly2 knife = {{0, 0}, {40, 0}, {0, 1.2}};
    std::string why;
    CHECK(!q.ok(shapeFromPoly(knife), &why));

    // A wall shorter than minEdge.
    Poly2 stub = {{0, 0}, {20, 0}, {20, 12}, {19.5, 12}, {0, 12}};
    CHECK(!q.ok(shapeFromPoly(stub)));

    // Too small to be a building at all.
    CHECK(!q.ok(rectShape(0, 0, 2, 2)));
}

TEST_CASE(plan_quality_accepts_deliberate_curves) {
    // A bowed wall's CHORD can be short while the wall is long. Measuring the
    // chord would forbid apses and bow fronts, which are legitimate designs.
    PlanQuality q;
    Shape2 s = rectShape(0, 0, 20, 12);
    bowEdge(s.outer, 0, 3.0);
    CHECK(q.ok(s));
    Shape2 disc = circleShape(Vec2(0, 0), 9);
    CHECK(q.ok(disc));
}

// --- the ops ---------------------------------------------------------------

TEST_CASE(plan_outset_adds_material_on_the_selected_wall) {
    Shape2 s = rectShape(0, 0, 20, 12);
    PlanOp op;
    op.kind = PlanOpKind::Outset;
    op.on = EdgeSelector::facing(Vec2(0, -1), 30);
    op.depth = 2;
    Shape2 r = applyPlanOp(s, op);
    CHECK(near(area(r), 20 * 12 + 20 * 2, 0.05));
    Vec2 lo, hi;
    bounds(r, lo, hi);
    CHECK(near(lo.y, -2, 0.01));           // it grew SOUTH, the way it faced
}

TEST_CASE(plan_wing_is_an_outset_over_part_of_a_wall) {
    Shape2 s = rectShape(0, 0, 20, 12);
    PlanOp op;
    op.kind = PlanOpKind::Wing;
    op.on = EdgeSelector::facing(Vec2(0, -1), 30);
    op.depth = 6;
    op.width = 5;
    op.along = 0.5;
    Shape2 r = applyPlanOp(s, op);
    CHECK(near(area(r), 20 * 12 + 5 * 6, 0.05));
    Vec2 lo, hi;
    bounds(r, lo, hi);
    CHECK(near(lo.y, -6, 0.01));
    CHECK(near(lo.x, 0, 0.01) && near(hi.x, 20, 0.01));   // the limb is narrow
}

TEST_CASE(plan_court_bites_a_u_out_of_a_box) {
    Shape2 s = rectShape(0, 0, 20, 12);
    PlanOp op;
    op.kind = PlanOpKind::Court;
    op.on = EdgeSelector::facing(Vec2(0, 1), 30);    // bite from the north
    op.depth = 7;
    op.width = 8;
    Shape2 r = applyPlanOp(s, op);
    CHECK(near(area(r), 20 * 12 - 8 * 7, 0.2));
    CHECK(r.holes.empty());                          // a U, not a courtyard
    CHECK(!contains(r, Vec2(10, 11)));               // the bite is really gone
}

TEST_CASE(plan_ops_are_total_functions) {
    // Every op must return something usable even when it cannot do its job, so
    // a script never has to guard. A selector that matches nothing, a zero
    // depth, a degenerate shape: all return the input.
    Shape2 s = rectShape(0, 0, 20, 12);
    PlanOp op;
    op.kind = PlanOpKind::Wing;
    op.on = EdgeSelector::byTag(EdgeTag::Party);     // no edge carries it
    op.depth = 5;
    CHECK(near(area(applyPlanOp(s, op)), area(s), 1e-9));
    Shape2 empty;
    CHECK(applyPlanOp(empty, op).outer.size() == 0);
}

// --- scripts ---------------------------------------------------------------

TEST_CASE(plan_script_is_deterministic_for_its_seed) {
    PlanScript sc;
    PlanStep step;
    PlanOp wing;
    wing.kind = PlanOpKind::Wing;
    wing.on = EdgeSelector::longest(1);
    wing.depth = 5;
    wing.width = 6;
    wing.weight = 3;
    PlanOp noop;
    noop.kind = PlanOpKind::Noop;
    noop.weight = 1;
    step.choices = {wing, noop};
    sc.steps = {step, step};

    Shape2 seed = rectShape(0, 0, 24, 14);
    PlanQuality q;
    Shape2 a = runPlanScript(seed, sc, 12345, q);
    Shape2 b = runPlanScript(seed, sc, 12345, q);
    CHECK(near(area(a), area(b), 1e-12));
    CHECK(a.outer.size() == b.outer.size());
}

// The plan's §17.5 point: `noop` is a WEIGHTED OP, so "this face does not grow"
// is a designed outcome with a probability rather than an accident. If noop
// never wins, every building gets every feature.
TEST_CASE(plan_noop_is_a_real_weighted_choice) {
    PlanScript sc;
    PlanStep step;
    PlanOp wing;
    wing.kind = PlanOpKind::Wing;
    wing.on = EdgeSelector::longest(1);
    wing.depth = 5;
    wing.width = 6;
    wing.weight = 1;
    PlanOp noop;
    noop.kind = PlanOpKind::Noop;
    noop.weight = 1;          // an even coin
    step.choices = {wing, noop};
    sc.steps = {step};

    Shape2 seed = rectShape(0, 0, 24, 14);
    PlanQuality q;
    int grew = 0, same = 0;
    for (std::uint32_t s = 1; s <= 60; ++s) {
        Shape2 r = runPlanScript(seed, sc, s * 7919u, q);
        (area(r) > area(seed) + 0.5 ? grew : same)++;
    }
    // Both outcomes must actually occur across seeds — a noop that never fires
    // is the same bug as one that always does.
    CHECK(grew > 5);
    CHECK(same > 5);
}

TEST_CASE(plan_script_never_accepts_a_plan_that_fails_quality) {
    // An op deep enough to slice the plan into a sliver must be REVERTED, not
    // allowed through to be amplified up a sixty-storey shaft.
    PlanScript sc;
    PlanStep step;
    PlanOp court;
    court.kind = PlanOpKind::Court;
    court.on = EdgeSelector::longest(1);
    court.depth = 11.6;      // out of a 12 m deep plan: leaves 0.4 m
    court.width = 18;
    court.weight = 1;
    step.choices = {court};
    sc.steps = {step};

    Shape2 seed = rectShape(0, 0, 20, 12);
    PlanQuality q;
    Shape2 r = runPlanScript(seed, sc, 99, q);
    CHECK(q.ok(r));
    CHECK(near(area(r), area(seed), 1e-9));    // reverted, so unchanged
}

TEST_CASE(plan_envelope_clip_is_a_containment_guarantee) {
    // Containment is a GUARANTEE here, not the progressive-inset search it
    // replaces: whatever the script does, the result is inside the envelope.
    Shape2 envelope = rectShape(0, 0, 20, 12);
    PlanScript sc;
    PlanStep step;
    PlanOp out;
    out.kind = PlanOpKind::Outset;
    out.on = EdgeSelector::all();
    out.depth = 8;                       // tries to burst out on every side
    step.choices = {out};
    sc.steps = {step, step, step};

    Shape2 seed = rectShape(4, 3, 16, 9);
    PlanQuality q;
    Shape2 r = runPlanScript(seed, sc, 7, q, &envelope);
    CHECK(area(r) <= area(envelope) + 1e-6);
    for (const Edge2& e : r.outer.edges) {
        CHECK(e.a.x >= -1e-6 && e.a.x <= 20 + 1e-6);
        CHECK(e.a.y >= -1e-6 && e.a.y <= 12 + 1e-6);
    }
}

// --- templates -------------------------------------------------------------

TEST_CASE(plan_every_template_is_valid_at_many_sizes_and_seeds) {
    PlanQuality q;
    for (PlanTemplate t : allPlanTemplates()) {
        for (Real w : {Real(14), Real(30), Real(60)}) {
            for (std::uint32_t s = 1; s <= 4; ++s) {
                Shape2 p = planTemplate(t, w, w * 0.68, s);
                std::string why;
                // A template must be valid at ANY size it is asked for — it is
                // the template's job to scale its own proportions, not the
                // caller's job to know each one's minimum frame.
                CHECK(q.ok(p, &why));
                CHECK(area(p) > 0);
            }
        }
    }
}

TEST_CASE(plan_courtyard_template_has_a_real_hole) {
    Shape2 c = planTemplate(PlanTemplate::Courtyard, 40, 30, 3);
    CHECK(c.holes.size() == 1);
    CHECK(area(c) < 40 * 30);
    // The court is enclosed: its centre is NOT part of the building.
    CHECK(!contains(c, centroid(c)) || area(c.holes[0]) > 1);
}

TEST_CASE(plan_curved_templates_carry_true_arcs) {
    Shape2 slab = planTemplate(PlanTemplate::CurvedCornerSlab, 30, 20, 2);
    int arcs = 0;
    for (const Edge2& e : slab.outer.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 1);
    Shape2 bow = planTemplate(PlanTemplate::BowFront, 30, 20, 2);
    arcs = 0;
    for (const Edge2& e : bow.outer.edges) if (std::fabs(e.bulge) > 1e-9) ++arcs;
    CHECK(arcs == 1);
    // The bow bulges toward the street but STAYS INSIDE its own w x d frame:
    // a template that overhangs the frame it was sized to is trimmed straight
    // back off by the envelope, which is what made every bow-front collapse.
    Vec2 blo, bhi;
    bounds(bow, blo, bhi);
    CHECK(blo.x >= -1e-6 && blo.y >= -1e-6);
    CHECK(bhi.x <= 30 + 1e-6 && bhi.y <= 20 + 1e-6);
    // The bow reserves a band inside the frame and then bulges back into most
    // of it, so the plan recovers nearly all the frame's area without ever
    // leaving it.
    CHECK(area(bow) > 30 * 20 * 0.85);
    CHECK(area(bow) < 30 * 20);
}

// --- mass stack ------------------------------------------------------------

TEST_CASE(stack_expands_to_levels_with_per_band_heights) {
    Shape2 plan = rectShape(0, 0, 20, 14);
    MassStack s = simpleStack(plan, 5, 3.0);
    std::vector<Level> lv = expandLevels(s);
    CHECK(lv.size() == 5);
    // The ground floor is its own band and is taller — the thing six existing
    // recipes fake by overloading floorHeight.
    CHECK(near(lv[0].y1 - lv[0].y0, 3.75, 1e-9));
    CHECK(near(lv[1].y1 - lv[1].y0, 3.0, 1e-9));
    CHECK(near(lv.back().y1, s.height(), 1e-9));
    for (std::size_t i = 1; i < lv.size(); ++i)
        CHECK(near(lv[i].y0, lv[i - 1].y1, 1e-12));   // no gaps, no overlaps
}

TEST_CASE(stack_support_rule_defaults_to_containment) {
    std::vector<Shape2> below{rectShape(0, 0, 20, 20)};
    std::vector<Shape2> above{rectShape(10, 10, 40, 40)};    // half hangs off
    CHECK(near(supportRatio(above, below), 100.0 / 900.0, 0.01));

    SupportRule strict;                       // minSupport 1, maxOverhang 0
    std::vector<Shape2> clipped = enforceSupport(above, below, strict);
    CHECK(near(totalArea(clipped), 100, 0.5));
    CHECK(near(supportRatio(clipped, below), 1.0, 0.01));
}

// The §17.4 case the plan exists to allow: Fallingwater must be possible.
TEST_CASE(stack_support_rule_permits_a_bounded_cantilever) {
    std::vector<Shape2> below{rectShape(0, 0, 20, 20)};
    std::vector<Shape2> above{rectShape(0, -4, 20, 20)};     // 4 m juts out
    CHECK(supportRatio(above, below) < 1.0);
    CHECK(near(overhangDistance(above, below), 4.0, 0.3));

    SupportRule cantilever;
    cantilever.minSupport = 0.7;
    cantilever.maxOverhang = 5.0;
    std::vector<Shape2> kept = enforceSupport(above, below, cantilever);
    // It survives INTACT: the overhang is within budget and enough is held up.
    CHECK(near(totalArea(kept), totalArea(above), 0.5));

    // Push it too far and the same rule clips it back.
    std::vector<Shape2> far{rectShape(0, -14, 20, 20)};
    std::vector<Shape2> cut = enforceSupport(far, below, cantilever);
    CHECK(totalArea(cut) < totalArea(far) - 1.0);
}

TEST_CASE(stack_profile_curves_shape_the_silhouette) {
    // A pyramid tapers monotonically; a Gherkin bulges then noses in. Both come
    // from ONE plan and a scalar curve — no loft, no twist.
    CHECK(near(profileAt(ProfileKind::Constant, 0.5, 1.0), 1.0, 1e-9));
    CHECK(near(profileAt(ProfileKind::Taper, 0.0, 0.1), 1.0, 1e-9));
    CHECK(near(profileAt(ProfileKind::Taper, 1.0, 0.1), 0.1, 1e-9));

    const Real low = profileAt(ProfileKind::Swell, 0.15, 1.0);
    const Real mid = profileAt(ProfileKind::Swell, 0.45, 1.0);
    const Real top = profileAt(ProfileKind::Swell, 1.0, 1.0);
    CHECK(mid > low);            // bulges
    CHECK(top < mid);            // then noses in
}

TEST_CASE(stack_gherkin_floor_plates_grow_then_shrink) {
    MassStack g = gherkin(circleShape(Vec2(0, 0), 14), 30, 3.6);
    std::vector<Level> lv = expandLevels(g);
    CHECK(lv.size() == 30);
    Real first = area(lv.front().plans[0]);
    Real widest = 0;
    std::size_t widestAt = 0;
    for (std::size_t i = 0; i < lv.size(); ++i) {
        const Real a = area(lv[i].plans[0]);
        if (a > widest) { widest = a; widestAt = i; }
    }
    CHECK(widestAt > 0);                       // the widest floor is not the base
    CHECK(widestAt < lv.size() - 1);           // ... nor the top
    CHECK(area(lv.back().plans[0]) < first);   // and it noses in
}

TEST_CASE(stack_pyramid_shrinks_every_floor) {
    MassStack p = pyramid(rectShape(0, 0, 40, 40), 12, 4.0);
    std::vector<Level> lv = expandLevels(p);
    CHECK(lv.size() == 12);
    for (std::size_t i = 1; i < lv.size(); ++i)
        CHECK(area(lv[i].plans[0]) < area(lv[i - 1].plans[0]));
}

TEST_CASE(stack_podium_tower_reclads_at_the_transition) {
    Shape2 podium = rectShape(0, 0, 40, 30);
    Shape2 tower = rectShape(8, 6, 32, 24);
    MassStack s = podiumTower(podium, tower, 3, 20, 3.4);
    std::vector<Level> lv = expandLevels(s);
    CHECK(lv.size() == 23);
    CHECK(lv[0].materialSet == 0);
    CHECK(lv.back().materialSet == 1);
    // The tower plate really is smaller than the podium's.
    CHECK(area(lv.back().plans[0]) < area(lv[0].plans[0]));
}

TEST_CASE(stack_wedding_cake_steps_in) {
    MassStack s = weddingCake(rectShape(0, 0, 50, 40), 4, 5, 0.18, 3.4);
    std::vector<Level> lv = expandLevels(s);
    CHECK(lv.size() >= 15);
    CHECK(area(lv.back().plans[0]) < area(lv.front().plans[0]) * 0.8);
}

// The Singapore arch: two legs that MERGE into one mass going up. A
// vertex-to-vertex morph cannot do this — only the field can change topology.
TEST_CASE(stack_arch_tower_merges_two_legs_into_one_mass) {
    Shape2 a = rectShape(0, 0, 12, 14);
    Shape2 b = rectShape(30, 0, 42, 14);
    MassStack s = archTower(a, b, 14, 3.5);
    std::vector<Level> lv = expandLevels(s);
    CHECK(lv.size() == 14);
    // Bottom: two separate plates. Top: one.
    CHECK(lv.front().plans.size() == 2);
    CHECK(lv.back().plans.size() == 1);
    CHECK(area(lv.back().plans[0]) > area(lv.front().plans[0]));
}

TEST_CASE(stack_levels_are_deterministic) {
    Shape2 plan = planTemplate(PlanTemplate::U, 30, 22, 5);
    MassStack a = weddingCake(plan, 3, 4);
    MassStack b = weddingCake(plan, 3, 4);
    std::vector<Level> la = expandLevels(a), lb = expandLevels(b);
    CHECK(la.size() == lb.size());
    for (std::size_t i = 0; i < la.size(); ++i) {
        CHECK(near(la[i].y0, lb[i].y0, 1e-12));
        CHECK(la[i].plans.size() == lb[i].plans.size());
        CHECK(near(totalArea(la[i].plans), totalArea(lb[i].plans), 1e-9));
    }
}
