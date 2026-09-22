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
    // be weird shaped blocks". Rectangularity is buildable area over its
    // oriented box, so a rectangle scores 1 whatever its aspect or angle.
    CityPlan plan = generatePlan(testBrief());
    const PlanScore s = evaluatePlan(plan);
    CHECK(s.coreRectilinearShare >= 0.85);
    CHECK(s.rectilinearShare >= 0.5);   // the outskirts curve, and may
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
