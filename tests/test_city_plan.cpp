#include "test_framework.h"
#include "../src/engine/procgen/city/plan/city_plan.h"

#include <cmath>

using namespace engine;
using namespace engine::plan;

// The city is designed as DATA before it is built: a brief makes a plan, and the
// plan is scored with the engine's own parcel walk and router. These are the
// promises a plan has to keep for the generator to be handed it — a grid core
// whose blocks are rectangles, one connected street network, every block with a
// use it can actually hold, a freeway commuters choose, and the same plan every
// time from the same brief. Pinned on a small brief so it runs in a second.

namespace {
Brief testBrief() {
    Brief b;
    b.name = "test_city";
    b.seed = 11;
    b.size = 1800;
    b.coreRadius = 280;
    b.midRadius = 480;
    b.freewayRadius = 620;
    b.spokes = 8;
    b.outerMargin = 100;
    return b;
}
}  // namespace

TEST_CASE(city_plan_is_the_same_plan_every_time) {
    // Same brief in, same city out: variants are compared side by side and hand
    // edits are diffed against a re-run, so the planner may not drift.
    CityPlan a = generatePlan(testBrief());
    CityPlan b = generatePlan(testBrief());
    CHECK(a.streets.nodes.size() == b.streets.nodes.size());
    CHECK(a.streets.edges.size() == b.streets.edges.size());
    CHECK(a.freeway.edges.size() == b.freeway.edges.size());
    CHECK(a.interchanges.size() == b.interchanges.size());
    CHECK(a.blocks.size() == b.blocks.size());
    const PlanScore sa = evaluatePlan(a), sb = evaluatePlan(b);
    CHECK(sa.blocks == sb.blocks);
    CHECK(sa.lots == sb.lots);
    CHECK(sa.predictedBuildings == sb.predictedBuildings);
    CHECK_APPROX(sa.freewayCommuteShare, sb.freewayCommuteShare, 1e-9);
    for (std::size_t i = 0; i < a.blocks.size() && i < b.blocks.size(); ++i) {
        CHECK(a.blocks[i].use == b.blocks[i].use);
        CHECK(a.blocks[i].lots == b.blocks[i].lots);
    }
}

TEST_CASE(city_plan_core_is_a_grid_of_rectangles) {
    // Glenn, 2026-09-22: "the core of the city should be gridlike ... they can't
    // be weird shaped blocks ... maybe in the outskirts the roads can become more
    // wedge like and curvy". Rectangularity is buildable area over its oriented
    // box, so a rectangle scores 1 whatever its aspect or angle. The CORE is held
    // to being rectangles outright; midtown is the grid already warping, and the
    // outskirts are meant to curve.
    CityPlan plan = generatePlan(testBrief());
    const PlanScore s = evaluatePlan(plan);
    CHECK(s.gridRectilinearShare >= 0.95);
    CHECK(s.coreRectilinearShare >= 0.75);
    CHECK(s.rectilinearShare >= 0.5);
    CHECK(s.streetComponents == 1);     // one drivable street network
    int core = 0;
    for (const PlanBlock& b : plan.blocks)
        if (b.district == 0 && b.use != BlockUse::RightOfWay) ++core;
    CHECK(core > 0);
}

TEST_CASE(city_plan_gives_every_block_a_use_it_can_hold) {
    // A block is several buildings or one block-sized landmark; it is never a
    // block that parcels into a dinky lot or two.
    CityPlan plan = generatePlan(testBrief());
    const PlanScore s = evaluatePlan(plan);
    CHECK(s.blocks > 0);
    CHECK(s.lotBlocks > 0);
    CHECK(s.predictedBuildings > 0);
    for (const PlanBlock& b : plan.blocks) {
        if (b.use == BlockUse::Lots) {
            CHECK(b.lots >= 3);
            CHECK(b.predictedBuildings >= 1);
        } else if (b.use == BlockUse::Landmark) {
            CHECK(b.lots <= 2);
            CHECK(b.predictedBuildings == 1);
        }
    }
    CHECK(s.meanLotsPerLotBlock >= 4);
}

TEST_CASE(city_plan_does_not_draw_two_roads_on_one_line) {
    // A grid clipped to a ring, the ring's own boulevard, spokes leaving it and the
    // first ring road outside it all land in the same annulus, and each generator
    // knows only its own geometry. Drawn over each other they build as lanes that do
    // not own their footprint — so the plan measures it: two roads inside each other's
    // corridor, running within 20 degrees of parallel, for 25 m or more. Crossings and
    // continuations are neither. The test brief measured 11, worst 13 m, when this was
    // pinned; it was 122 before the rim and the frontage roads learned to keep clear.
    CityPlan plan = generatePlan(testBrief());
    const PlanScore s = evaluatePlan(plan);
    CHECK(s.corridorOverlaps <= 15);
    CHECK(s.worstOverlap < 20);
}

TEST_CASE(city_plan_freeway_is_worth_driving_to) {
    // The ring alone left 1% of metro_lanes' commutes on the freeway. A plan has
    // to show its freeway earning its traffic before anything is built.
    // At city scale: the small brief's trips are all short enough to walk past
    // the ring, so this one case pays for a full 3 km plan (about a second).
    Brief b;
    b.name = "test_metro";
    b.seed = 7;
    CityPlan plan = generatePlan(b);
    const PlanScore s = evaluatePlan(plan);
    CHECK(s.freewayKm > 0);
    CHECK(plan.interchanges.size() >= 2);
    CHECK(s.commutesSampled > 0);
    CHECK(s.freewayCommuteShare >= 0.15);
}

// A CITY SHAPED BY ITS LAND (ADR-0106): on a coastal plain with the sea to the west and a steep
// range rising to the east, the city grows to its area over the plain -- down to the shore, not up
// the slope -- and its depth field is deepest inside it, with contours and descent lines to lay it by.
#include "../src/engine/procgen/city/plan/land_shape.h"
TEST_CASE(city_shape_grows_over_the_plain_not_up_the_range) {
    // x < -800 sea; a gentle plain; x > 600 a range climbing at 30%
    const HeightField ground = [](double x, double) { return x < -800 ? -5.0 : x < 600 ? 8.0 + 0.01 * (x + 800) : 22.0 + 0.3 * (x - 600); };
    auto buildable = [&](const Vec2& p) {
        const double h = ground(p.x, p.y), s = std::fabs(ground(p.x + 5, p.y) - ground(p.x - 5, p.y)) / 10.0;
        return h > 3.0 && s < 0.15;
    };
    LandShapeParams sp;
    sp.targetArea = 2.0e6;
    sp.seaLevel = 0.0;
    const LandShape s = growLandShape(ground, buildable, Vec2(0, 0), 1600.0, sp);
    CHECK(std::fabs(s.area - sp.targetArea) < 0.25 * sp.targetArea);
    CHECK(!s.limits.empty());
    CHECK(s.inside(s.heart));
    CHECK(!s.inside(Vec2(900, 0)));          // not up the range
    CHECK(s.inside(Vec2(-700, 0)));          // down to the shore (the waterfront pulls)
    CHECK(!s.inside(Vec2(-900, 0)));         // not in the sea
    // a contour inside it, and a street down the depth from it ending near the edge
    const double rim = s.depthHolding(0.35);
    const auto c = s.contour(rim);
    CHECK(!c.empty());
    const std::vector<Vec2> line = descendDepth(s, c.front().front() + s.gradient(c.front().front()) * 8.0, 40.0);
    CHECK(line.size() > 3);
    CHECK(s.depthAt(line.back()) < 45.0);
}
