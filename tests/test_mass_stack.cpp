// THE MASS AND THE STACK (mass.h, mass_stack.h, issue #105 step 4): sections over storey ranges under keyed
// transforms; today's tiers as the first preset, byte-identical through the slice bridge; the Twist and Stack
// envelopes as the first transforms, every storey inside the lot.
#include "test_framework.h"

#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/mass_stack.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {
BuildingParams towerParams(int floors) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    return p;
}
bool samePoly(const Poly2& a, const Poly2& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (a[i].x != b[i].x || a[i].y != b[i].y) return false;
    return true;
}
bool inside(const Poly2& outer, const Poly2& q, Real nudge = 0.02) {
    const Vec2 c = centroid(q);
    for (const Vec2& v : q)
        if (!pointInPolygon(outer, v + (c - v) * nudge)) return false;
    return true;
}
}  // namespace

TEST_CASE(curves_and_transforms_do_what_they_say) {
    Curve c = Curve::linear(0, 90);
    CHECK(std::fabs(c.at(0.5, 0) - 45.0) < 1e-9);
    CHECK(c.at(-1, 0) == 0 && c.at(2, 0) == 90);
    CHECK(Curve{}.at(0.3, 7) == 7);
    CHECK(Curve{}.flat(0) && !c.flat(0) && Curve::constant(1).flat(1));
    Transform id;
    CHECK(id.identity());
    const Section sq = Section::rect(10, 6, Vec2(3, 4));
    CHECK(samePoly(id.apply(sq, 0.5).tessellate(), sq.tessellate()));
    // A quarter turn about the rectangle's own centre swaps its extents.
    Transform rot;
    rot.pivot = Vec2(3, 4);
    rot.rot = Curve::constant(90);
    const Poly2 r = rot.apply(sq, 0).tessellate();
    Vec2 lo, hi;
    bounds(r, lo, hi);
    CHECK(std::fabs((hi.x - lo.x) - 6.0) < 1e-9 && std::fabs((hi.y - lo.y) - 10.0) < 1e-9);
    CHECK(std::fabs(centroid(r).x - 3.0) < 1e-9 && std::fabs(centroid(r).y - 4.0) < 1e-9);
    // A slide along v moves it, a scale about the pivot shrinks it about the pivot.
    Transform sl;
    sl.pivot = Vec2(3, 4);
    sl.slideV = Curve::constant(2.5);
    CHECK(std::fabs(centroid(sl.apply(sq, 0).tessellate()).y - 6.5) < 1e-9);
    Transform sc;
    sc.pivot = Vec2(3, 4);
    sc.scaleU = Curve::linear(1, 0.5);
    const Poly2 half = sc.apply(sq, 1).tessellate();
    bounds(half, lo, hi);
    CHECK(std::fabs((hi.x - lo.x) - 5.0) < 1e-9 && std::fabs((hi.y - lo.y) - 6.0) < 1e-9);
}

TEST_CASE(the_stack_preset_slices_to_todays_storeys_bit_for_bit) {
    // Every envelope's tiers as a stack: the slices ARE the storey stack, point for point, tier for tier.
    const Poly2 plans[3] = {{{0, 0}, {36, 0}, {36, 44}, {0, 44}},
                            {{-14, -12}, {14, -12}, {14, 0}, {0, 0}, {0, 12}, {-14, 12}},
                            {{-20, -20}, {20, -20}, {20, 20}, {-20, 20}}};
    using Env = BuildingParams::Envelope;
    const Env envs[] = {Env::None, Env::StreetWallSetback, Env::SkyExposure, Env::Taper, Env::Slab, Env::Feathered};
    int checked = 0;
    for (const Poly2& plan : plans)
        for (Env env : envs) {
            BuildingParams p = towerParams(34);
            p.envelope = env;
            p.setbackFloors = env == Env::None ? 8 : 0;
            p.setbackEvery = env == Env::None ? 2.0 : 0;
            p.baseFloors = 4; p.setback1 = 5; p.stepFloors = 8; p.stepDepth = 3; p.towerFrac = 0.4; p.towerFloor = 20;
            const Stack st = stackOf(plan, p);
            const std::vector<MassTier> tiers = envelopeTiers(plan, p);
            const std::vector<MassTier> viaStack = st.tiers();
            CHECK(viaStack.size() == tiers.size());
            for (std::size_t i = 0; i < tiers.size() && i < viaStack.size(); ++i) {
                CHECK(viaStack[i].floor0 == tiers[i].floor0);
                CHECK(samePoly(viaStack[i].plan, tiers[i].plan));
            }
            const std::vector<StoreyPlan> sliced = sliceStoreys(st, p);
            const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
            CHECK(sliced.size() == storeys.size() && sliced.size() == static_cast<std::size_t>(p.floors + 1));
            for (std::size_t k = 0; k < sliced.size() && k < storeys.size(); ++k) {
                CHECK(samePoly(sliced[k].plan, storeys[k].plan));
                CHECK(sliced[k].y0 == storeys[k].y0 && sliced[k].h == storeys[k].h && sliced[k].tier == storeys[k].tier);
            }
            for (const StackEntry& e : st.entries) CHECK(!e.varies());
            ++checked;
        }
    std::printf("    [stack] %d plan x envelope pairs sliced identically\n", checked);
}

TEST_CASE(a_twisted_tower_turns_inside_its_lot_storey_by_storey) {
    const Poly2 plan = {{-20, -16}, {20, -16}, {20, 16}, {-20, 16}};
    BuildingParams p = towerParams(30);
    p.envelope = BuildingParams::Envelope::Twist;
    p.baseFloors = 1;
    p.twistDeg = 90;
    const Stack st = stackOf(plan, p);
    CHECK(st.entries.size() == 2);
    const StackEntry& tw = st.entries.back();
    CHECK(tw.varies() && tw.rel == Relation::Support && tw.k0 == 2 && tw.k1 == 30);
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CHECK(storeys.size() == 31);
    // The base storeys are the lot; above, every storey inside the lot, each turned on from the last, the top a
    // quarter turn from the first; the whole twist is ONE tier.
    CHECK(samePoly(storeys[0].plan, plan) && samePoly(storeys[1].plan, plan));
    Real lastAngle = -1;
    for (int k = 2; k <= 30; ++k) {
        const Poly2& sp = storeys[static_cast<std::size_t>(k)].plan;
        CHECK(sp.size() == 4);
        CHECK(inside(plan, sp));
        CHECK(storeys[static_cast<std::size_t>(k)].tier == 1);
        const Vec2 d = normalize(sp[1] - sp[0]);
        const Real ang = std::atan2(d.y, d.x) * 180.0 / 3.14159265358979323846;
        if (k > 2) CHECK(ang > lastAngle + 1.0);
        lastAngle = ang;
    }
    const Vec2 d0 = normalize(storeys[2].plan[1] - storeys[2].plan[0]), d1 = normalize(storeys[30].plan[1] - storeys[30].plan[0]);
    CHECK(std::fabs(dot(d0, d1)) < 0.05);   // a quarter turn
    // The tiers for the core's fit: one per turned storey; the collider: one per storey, no deck between them.
    const std::vector<MassTier> tiers = massStack(plan, p);
    CHECK(tiers.size() == 1 + 29);
    const std::vector<ColliderTier> ct = colliderTiers(plan, p);
    CHECK(ct.size() == 1 + 29);
    CHECK(ct.front().deck && ct.front().parapet > 0);
    for (std::size_t i = 1; i + 1 < ct.size(); ++i) { CHECK(!ct[i].deck); CHECK(ct[i].parapet == 0); CHECK(ct[i + 1].y0 == ct[i].y1); }
    CHECK(ct.back().deck);
    // The whole thing walks: the gate.
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(unreachableCells(g).empty());
    std::printf("    [twist] %zu storeys, %zu tiers, %zu collider tiers, %zu cells, %zu unreachable\n", storeys.size(), tiers.size(),
                ct.size(), g.cells.size(), unreachableCells(g).size());
}

TEST_CASE(stacked_modules_slide_alternately_and_stay_inside_the_lot) {
    const Poly2 plan = {{-20, -14}, {20, -14}, {20, 14}, {-20, 14}};
    BuildingParams p = towerParams(16);
    p.envelope = BuildingParams::Envelope::Stack;
    p.baseFloors = 1;
    p.stepFloors = 4;
    p.stackShift = 3.0;
    const Stack st = stackOf(plan, p);
    CHECK(st.entries.size() == 1 + 4);   // the base, then modules at storeys 2-5, 6-9, 10-13, 14-16
    Real lastX = 0;
    for (std::size_t i = 1; i < st.entries.size(); ++i) {
        const StackEntry& m = st.entries[i];
        CHECK(m.rel == Relation::Support);
        CHECK(m.k0 == 2 + 4 * static_cast<int>(i - 1));
        CHECK(!m.xf.rot.keys.size());
        const Poly2 outline = st.at(m.k0).tessellate();
        CHECK(inside(plan, outline));
        const Real cx = centroid(outline).x;
        if (i == 1) CHECK(std::fabs(cx) < 1e-9);
        if (i == 2) CHECK(std::fabs(cx - 3.0) < 1e-9);
        if (i == 3) CHECK(std::fabs(cx) < 1e-9);
        if (i == 4) CHECK(std::fabs(cx + 3.0) < 1e-9);
        if (i > 1) CHECK(std::fabs(cx - lastX) <= 3.0 + 1e-9);   // never more than the shift past the one below
        lastX = cx;
    }
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CHECK(storeys.size() == 17);
    for (int k = 2; k <= 16; ++k) CHECK(storeys[static_cast<std::size_t>(k)].tier == 1 + (k - 2) / 4);
    // Tiers for the core: one per module (a constant slide is one tier); the collider: decks between modules.
    CHECK(massStack(plan, p).size() == 5);
    const std::vector<ColliderTier> ct = colliderTiers(plan, p);
    CHECK(ct.size() == 5);
    for (const ColliderTier& t : ct) CHECK(t.deck);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    CHECK(unreachableCells(g).empty());
    std::printf("    [stack] %zu modules, %zu cells, %zu unreachable\n", st.entries.size() - 1, g.cells.size(), unreachableCells(g).size());
}

TEST_CASE(a_lens_tower_widens_to_its_waist_and_narrows_to_its_top) {
    Poly2 drum;
    for (int k = 0; k < 16; ++k) {
        const Real a2 = 2.0 * 3.14159265358979323846 * k / 16;
        drum.push_back(Vec2(std::cos(a2), std::sin(a2)) * 18.0);
    }
    BuildingParams p = towerParams(30);
    p.envelope = BuildingParams::Envelope::Bulge;
    p.baseFloors = 1;
    p.bulge = 0.45;
    p.taperTop = 0.4;
    const Stack st = stackOf(drum, p);
    CHECK(st.entries.size() == 2 && st.entries.back().varies() && st.entries.back().rel == Relation::Support);
    const std::vector<StoreyPlan> storeys = storeyPlans(drum, p);
    CHECK(storeys.size() == 31);
    auto radius = [&](int k) { return (storeys[static_cast<std::size_t>(k)].plan[0] - centroid(storeys[static_cast<std::size_t>(k)].plan)).length(); };
    // 0.8 of the drum at its foot, the full drum at the waist, the top no narrower than the core needs (0.4 asked,
    // 17 m across given); widening up to the waist, narrowing above; every storey inside the lot, the same 16 corners.
    CHECK(std::fabs(radius(2) - 18.0 * 0.8) < 1e-6);
    int waist = 2;
    for (int k = 3; k <= 30; ++k) if (radius(k) > radius(waist)) waist = k;
    CHECK(std::fabs(radius(waist) - 18.0) < 0.3);
    CHECK(radius(30) > 18.0 * 0.4 && radius(30) < 18.0 * 0.5);
    CHECK(2 * radius(30) * std::cos(3.14159265358979323846 / 16) >= 17.0 - 1e-6);
    for (int k = 3; k <= 30; ++k) {
        CHECK(storeys[static_cast<std::size_t>(k)].plan.size() == 16);
        CHECK(inside(drum, storeys[static_cast<std::size_t>(k)].plan));
        if (k <= waist) CHECK(radius(k) >= radius(k - 1) - 1e-9);
        else CHECK(radius(k) <= radius(k - 1) + 1e-9);
        CHECK(storeys[static_cast<std::size_t>(k)].tier == 1);
    }
    // The core fits every storey, so the lens is climbed by its lifts and stairs: the gate.
    CHECK(coreFor(drum, p, entranceEdgeFor(drum, p)).valid);
    const SpaceGraph g = spaceGraphOf(drum, p, 0.0);
    for (int c : unreachableCells(g)) {
        const Cell& cell = g.cells[static_cast<std::size_t>(c)];
        Vec2 lo, hi;
        bounds(cell.plan, lo, hi);
        std::printf("    [lens] unreachable %s %s storey %d (%.1f,%.1f)-(%.1f,%.1f) free %.1f\n", cellKindName(cell.kind), cellUseName(cell.use),
                    cell.k0, lo.x, lo.y, hi.x, hi.y, cell.freeArea);
    }
    CHECK(unreachableCells(g).empty());
    std::printf("    [lens] foot r %.2f, waist r %.2f at storey %d, top r %.2f; %zu cells, %zu unreachable\n", radius(2), radius(waist), waist,
                radius(30), g.cells.size(), unreachableCells(g).size());
}

TEST_CASE(the_stack_is_a_mass_with_slices_bounds_and_a_shell) {
    const Poly2 plan = {{-12, -10}, {12, -10}, {12, 10}, {-12, 10}};
    BuildingParams p = towerParams(10);
    p.envelope = BuildingParams::Envelope::Twist;
    p.baseFloors = 1;
    p.twistDeg = 60;
    const Stack st = stackOf(plan, p);
    const Mass& m = st;
    CHECK(std::fabs(m.height() - (p.groundHeight + 10 * p.floorHeight)) < 1e-9);
    CHECK(samePoly(m.slice(-1).outline().tessellate(), plan));
    CHECK(samePoly(m.slice(1.0).outline().tessellate(), plan));           // the ground storey
    CHECK(m.slice(p.groundHeight + 0.5).tier == 0);                       // storey 1: still the base
    CHECK(m.slice(p.groundHeight + p.floorHeight + 0.5).tier == 1);       // storey 2: the twist
    const MassBounds b = m.bounds();
    CHECK(b.lo.x >= -12 - 1e-9 && b.hi.x <= 12 + 1e-9 && b.y1 == m.height());
    const RenderMesh shell = m.shell(0, Vec3(0.5, 0.5, 0.5));
    CHECK(shell.vertices.size() > 4 * 4 * 10);   // ruled quads storey by storey and the cap
    CHECK(shell.indices.size() % 3 == 0);
}
