#include "test_framework.h"
#include "../src/engine/procgen/city/plan/city_plan.h"
#include "../src/engine/procgen/city/plan/plan_scene.h"
#include "planned_city.h"

#include <cmath>
#include <fstream>
#include <string>

using namespace engine;
using namespace engine::plan;

// The city is designed as DATA before it is built: a brief makes a plan, and the
// plan is scored with the engine's own parcel walk and router. These are the
// promises a plan has to keep for the generator to be handed it — a grid core
// whose blocks are rectangles, one connected street network, every block with a
// use it can actually hold, a freeway commuters choose, and the same plan every
// time from the same brief. Pinned on a small brief so it runs in a second.

namespace {
Brief testBrief() { return citytest::roundBrief(); }
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

TEST_CASE(a_shaped_city_meets_its_boulevard_in_junctions_not_stars) {
    // Glenn, 2026-10-09: "some streets look like a mess... like there are many roads that merged together". A city
    // laid by its land's depth (world.land.shape, the island's cities) lays its grid, its boulevard and rings, its
    // spokes and its wedge streets separately; they used to MISS each other by a few metres -- a grid street short of
    // the boulevard, a wedge street starting 20 m along it -- and collapseShortLinks chained those near-misses into
    // stars (17 streets at one junction on island 8). Now the grid ends on the boulevard, the streets beyond start
    // where it arrives, and a merge can't move a junction further than the link it removes is long.
    // Then the triangles (Glenn: "can you look at the small triangles?"): grid streets meeting the boulevard at a
    // glance or as a corner just inside it, stubs left by the cuts, junctions 30-45 m apart on a contour (which the
    // merge then kinked together), a contour loop too thin to hold a block.
    // Measured: worst junction 4 streets, no pair leaving one under 15 degrees apart; before all this, 9 and 36.
    Brief b = testBrief();
    b.world = {{"base", {{"heightScale", 0.0}, {"mountainHeight", 0.0}}},
               {"seaLevel", -50.0},
               {"land", {{"shape", true}, {"area", 2.4e6}}}};
    CityPlan plan = generatePlan(b);
    CHECK(!plan.limits.empty());
    const PlanScore s = evaluatePlan(plan);
    std::printf("    [shaped] %.0f m paved into another, worst junction %d streets, %d pairs under 15 degrees\n",
                static_cast<double>(s.pavedOverlapM), s.worstJunction, s.tightTurns);
    CHECK(s.worstJunction <= 5);
    CHECK(s.tightTurns == 0);
    CHECK(s.pavedOverlapM < 60);
}

TEST_CASE(the_plan_score_measures_streets_merging) {
    // The score's merged-street numbers (PlanScore: worstJunction / crowdedJunctions / tightTurns / pavedOverlapM),
    // on the round test city -- which lays its grid, ring and spokes the older way and was never shaped. Pinned as
    // measured (587 m, worst junction 4, none under 15 degrees), so a change that merges its streets shows up here,
    // not first in a walk-through. Was 799 m until streets that stopped inside another's pavement were joined to it
    // in a T, or pruned where no T may stand (joinNearMisses, ADR-0266); the rest is where the round layout's ring
    // meets its grid and spokes -- owed, like metro_planned's 1.4 km and metro_rivers' 2.4 km.
    CityPlan plan = generatePlan(testBrief());
    const PlanScore s = evaluatePlan(plan);
    std::printf("    [round] %.0f m paved into another, worst junction %d streets (%d of 5+), %d pairs under 15 degrees\n",
                static_cast<double>(s.pavedOverlapM), s.worstJunction, s.crowdedJunctions, s.tightTurns);
    CHECK(s.worstJunction <= 5);
    CHECK(s.tightTurns == 0);
    CHECK(s.pavedOverlapM < 650);
}

namespace {
// Street ends standing inside another street's pavement (within its half-width): the dead ends no car
// can turn out of that the planner used to leave where a street stopped a few metres short of the one
// it ran into.
int endsInsideAStreet(const RoadGraph& g) {
    std::vector<int> degree(g.nodes.size(), 0);
    for (const RoadEdge& e : g.edges) { ++degree[static_cast<std::size_t>(e.a)]; ++degree[static_cast<std::size_t>(e.b)]; }
    int n = 0;
    for (std::size_t v = 0; v < g.nodes.size(); ++v) {
        if (degree[v] != 1) continue;
        const Vec2 p = g.nodes[v].pos;
        for (const RoadEdge& e : g.edges) {
            if (e.a == static_cast<int>(v) || e.b == static_cast<int>(v)) continue;
            const Vec2 a = g.nodes[static_cast<std::size_t>(e.a)].pos, ab = g.nodes[static_cast<std::size_t>(e.b)].pos - a;
            const Real L2 = ab.lengthSquared();
            const Real t = L2 > 1e-12 ? std::clamp(dot(p - a, ab) / L2, Real(0), Real(1)) : Real(0);
            if ((a + ab * t - p).length() < e.width * Real(0.5)) { ++n; break; }
        }
    }
    return n;
}

int hangingLoops(const RoadGraph& g) {
    int n = 0;
    for (char c : hangingLoopEdges(g)) n += c;
    return n;
}

Brief coastBrief() {
    std::ifstream in("assets/city_plans/metro_v2_test.json");
    return briefFromJson(nlohmann::json::parse(in));
}
}  // namespace

TEST_CASE(city_plan_never_hangs_a_loop_off_one_junction) {
    // Glenn, 2026-10-29: "We should prune weird looped single roads." A cycle of streets whose only way
    // to the rest of the city is one junction -- a scrap of a crossing the planariser left, or a street
    // round a block a keep-out cut every other link to -- is pruned (dropHangingLoops). Every seed:
    // before, 9 of 18 seeds across three briefs had one (river_town's seed 41 a 526 m one).
    int seeds = 0;
    for (int seed = 1; seed <= 12; ++seed) {
        Brief b = testBrief();
        b.seed = static_cast<uint32_t>(seed * 7919);
        const CityPlan plan = generatePlan(b);
        CHECK(hangingLoops(plan.streets) == 0);
        ++seeds;
    }
    for (int seed = 1; seed <= 4; ++seed) {
        Brief b = coastBrief();
        b.seed = static_cast<uint32_t>(seed * 104729);
        const CityPlan plan = generatePlan(b);
        CHECK(hangingLoops(plan.streets) == 0);
        ++seeds;
    }
    CHECK(seeds == 16);
}

TEST_CASE(city_plan_never_ends_a_street_inside_another) {
    // A street that stopped 1-5 m short of the one it ran into ended inside that street's pavement and was
    // never joined to it: 13 of metro_v2_test's 17 dead ends, 20 of metro_planned's 29. Now it is carried
    // straight on to a T, or -- where a T would stand too near a junction and be fused into it -- pruned
    // back to its own last junction. Every seed of the round brief and of a coast-clipped one.
    for (int seed = 1; seed <= 12; ++seed) {
        Brief b = testBrief();
        b.seed = static_cast<uint32_t>(seed * 7919);
        const CityPlan plan = generatePlan(b);
        CHECK(endsInsideAStreet(plan.streets) == 0);
    }
    for (int seed = 1; seed <= 4; ++seed) {
        Brief b = coastBrief();
        b.seed = static_cast<uint32_t>(seed * 104729);
        const CityPlan plan = generatePlan(b);
        CHECK(endsInsideAStreet(plan.streets) == 0);
    }
}

TEST_CASE(a_coast_city_builds_its_ring_as_a_c_and_lands_its_ends) {
    // Where the coast opens the ring into a C (plan.ringArc), the scene used to build only a closed ring:
    // metro_v2_test's planned 4.1 km of freeway came out as a 2-edge stub. The C is now chains, carriageways
    // and system interchanges like the ring, and each end comes down to a street as a boulevard pair (bc*)
    // or, with none in reach, turns back on itself (tc*). Every seed of the coast brief.
    for (int seed = 1; seed <= 4; ++seed) {
        Brief b = coastBrief();
        b.seed = static_cast<uint32_t>(seed * 104729);
        const CityPlan plan = generatePlan(b);
        if (plan.ringArc.empty()) continue;   // a seed whose ring stayed on land, or became a strip city
        const nlohmann::json scene = planToLanesScene(plan);
        int freeway = 0;
        bool end0 = false, end1 = false;
        for (const nlohmann::json& e : scene["edges"]) {
            const std::string id = e.value("id", std::string());
            if (e.value("class", std::string()) == "freeway") ++freeway;
            end0 = end0 || id.rfind("bc0", 0) == 0 || id == "tc0";
            end1 = end1 || id.rfind("bc1", 0) == 0 || id == "tc1";
        }
        std::printf("    [coast seed %d] %d freeway edges; ends %s / %s\n", seed, freeway, end0 ? "landed" : "LOOSE", end1 ? "landed" : "LOOSE");
        CHECK(freeway >= 4);
        CHECK(end0);
        CHECK(end1);
    }
}
