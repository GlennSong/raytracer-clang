// THE ENVELOPE (envelope.h, issue #105 step 6; ADR-0178): a form as a skin round straight floors -- the floors the
// plan's prism inside it, the form a glass veil or the floor plates' contour, Glenn's straight-inside rule as a word.
#include "test_framework.h"

#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/envelope.h"
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
bool inside(const Poly2& outer, const Poly2& inner, Real nudge = 0.01) {
    const Vec2 c = centroid(outer);
    for (const Vec2& q : inner) if (!pointInPolygon(outer, q + (c - q) * nudge)) return false;
    return true;
}
}  // namespace

TEST_CASE(a_double_skin_stands_straight_floors_inside_a_turned_skin) {
    // A twist turns inside its lot scaled to ~0.7, and straight floors inside a turning rectangle are ~0.7 of it
    // again (the square inscribed in a 45-degree turn): a twisted veil needs a big lot. 44 x 40 holds one; on a
    // 30 x 24 lot the same recipe wears the twist as its floors (the fallback the third test proves).
    const Poly2 plan = {{-22, -20}, {22, -20}, {22, 20}, {-22, 20}};
    BuildingParams p = towerParams(20);
    p.envelope = BuildingParams::Envelope::Twist;
    p.twistDeg = 50;
    p.baseFloors = 1;
    p.skin = kSkinDouble;
    p.skinGap = 1.2;
    CHECK(skinOf(p) == kSkinDouble);
    const Poly2 inner = skinFloorsPlan(plan, p);
    CHECK(inner.size() == 4);
    CHECK(tierShortSide(inner) >= kCoreTierMin);
    // the floors: the plan's prism up to the base, the inner rectangle above, every storey the same
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    CHECK(storeys.size() == 21);
    CHECK(storeys[0].plan.size() == 4 && storeys[1].plan.size() == 4);
    for (std::size_t k = 2; k < storeys.size(); ++k) {
        CHECK(storeys[k].plan.size() == 4);
        CHECK(storeys[k].tier == 1);
        CHECK(std::fabs(area(storeys[k].plan) - area(inner)) < 1e-6);
    }
    // the skin: the twist's own turned rectangles, every one holding the floors with the gap to spare
    const std::vector<Poly2> skin = skinSlices(plan, p);
    CHECK(skin.size() == 21);
    CHECK(skin[0].empty() && skin[1].empty());
    int turned = 0;
    for (std::size_t k = 2; k < skin.size(); ++k) {
        CHECK(skin[k].size() == 4);
        if (skin[k].size() < 4) continue;
        CHECK(inside(skin[k], offsetPlanPublic(inner, -1.2 + 0.05)));
        if (k > 2 && skin[k - 1].size() == 4 && (skin[k][0] - skin[k - 1][0]).length() > 0.05) ++turned;
    }
    CHECK(turned >= 17);
    // the same twist on a 30 x 24 lot: no straight floors the core needs fit inside it, so it is the floors
    const Poly2 small = {{-15, -12}, {15, -12}, {15, 12}, {-15, 12}};
    CHECK(skinFloorsPlan(small, p).empty());
    CHECK(skinSlices(small, p).empty());
    CHECK(skinPlates(plan, p).empty());   // a double skin alone: no plates
    // the tiers the core sees are the floors', and the building is climbed
    CHECK(massStack(plan, p).size() == 2);
    CHECK(coreFor(plan, p, entranceEdgeFor(plan, p)).valid);
    CHECK(unreachableCells(spaceGraphOf(plan, p, 0.0)).empty());
    std::printf("    [skin] floors %.1f x %.1f inside a %d-degree twist on 44 x 40, %d storeys turned; on 30 x 24 the twist is the floors\n",
                tierShortSide(inner), 2 * std::max(orientedBoundingBox(inner).half[0], orientedBoundingBox(inner).half[1]), 50, turned);
}

TEST_CASE(the_wave_keeps_its_plates_as_its_default_skin) {
    const Poly2 plan = {{-20, -10}, {20, -10}, {20, 10}, {-20, 10}};
    BuildingParams p = towerParams(20);
    p.envelope = BuildingParams::Envelope::Wave;
    p.baseFloors = 1;
    p.wave = 1.5;
    p.waveLen = 18.0;
    CHECK(p.skin == 0 && skinOf(p) == kSkinPlates);
    CHECK(skinFloorsPlan(plan, p).size() == 4);
    Stack st;
    CHECK(!skinFloorsStack(plan, p, st));   // the floors are the plan: the path the wave always took
    const std::vector<Poly2> plates = skinPlates(plan, p), waves = wavePlates(plan, p);
    CHECK(plates.size() == waves.size() && plates.size() == 21);
    for (std::size_t k = 0; k < plates.size(); ++k) CHECK(plates[k].size() == waves[k].size());
    CHECK(skinSlices(plan, p).empty());
    // a wavy VEIL: the floors inset by the wave and the gap, the skin's contours resampled to one corner count
    p.skin = kSkinDouble;
    const Poly2 inner = skinFloorsPlan(plan, p);
    CHECK(inner.size() == 4);
    Vec2 lo, hi;
    bounds(inner, lo, hi);
    CHECK(std::fabs(hi.y - (10.0 - 1.5 - 1.2)) < 0.05);
    const std::vector<Poly2> veil = skinSlices(plan, p);
    CHECK(veil.size() == 21);
    std::size_t n = 0;
    for (std::size_t k = 2; k < veil.size(); ++k) { if (k == 2) n = veil[k].size(); CHECK(veil[k].size() == n && n >= 32); }
    CHECK(unreachableCells(spaceGraphOf(plan, p, 0.0)).empty());
    std::printf("    [skin] the wave's plates %zu, its veil %zu corners a storey, floors across %.1f m\n", plates.size(), n, hi.y - lo.y);
}

TEST_CASE(a_skin_too_tight_for_its_core_gives_way_to_the_form) {
    // A lens on an 18 x 16 lot: the bulge's foot is 0.8 of the plan, 12.8 m, and no straight floor the core needs
    // (14 m) fits inside it with a gap -- so the form is the floors, as if skin were 0.
    const Poly2 plan = {{-9, -8}, {9, -8}, {9, 8}, {-9, 8}};
    BuildingParams p = towerParams(16);
    p.envelope = BuildingParams::Envelope::Bulge;
    p.bulge = 0.5;
    p.taperTop = 0.7;
    p.baseFloors = 1;
    p.skin = kSkinDouble | kSkinPlates;
    CHECK(skinFloorsPlan(plan, p).empty());
    CHECK(skinSlices(plan, p).empty() && skinPlates(plan, p).empty());
    const std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    BuildingParams q = p;
    q.skin = 0;
    const std::vector<StoreyPlan> form = storeyPlans(plan, q);
    CHECK(storeys.size() == form.size());
    for (std::size_t k = 0; k < storeys.size(); ++k) CHECK(std::fabs(area(storeys[k].plan) - area(form[k].plan)) < 1e-6);
    // and on a lot with room, the same lens carries plates AND a veil
    const Poly2 wide = {{-16, -14}, {16, -14}, {16, 14}, {-16, 14}};
    CHECK(skinFloorsPlan(wide, p).size() == 4);
    const std::vector<Poly2> plates = skinPlates(wide, p), veil = skinSlices(wide, p);
    CHECK(plates.size() == 17 && veil.size() == 17);
    int both = 0;
    for (std::size_t k = 2; k < 17; ++k) if (plates[k].size() >= 3 && veil[k].size() >= 3) ++both;
    CHECK(both == 15);
    CHECK(unreachableCells(spaceGraphOf(wide, p, 0.0)).empty());
    std::printf("    [skin] 18 x 16: the form is the floors; 32 x 28: %d storeys with a plate and a veil\n", both);
}
