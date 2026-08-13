// No building may stand in a carriageway — on a road net with MIXED widths.
//
// Why this file exists (docs/knowledge-retention-plan.md): "a building corner
// must clear edge_width/2 of every centreline" was recorded as two arguments on
// growLotBuildings, with the device report in the comment, and the rule was lost
// anyway when a replacement pass was written with a narrower signature. Nothing
// failed to compile; buildings stood in the carriageway again.
//
// Two things separate this from the clearance tests that already exist
// (test_city_lots.cpp `lot_buildings_keep_clear_of_roads`, test_ground_truth.cpp
// `park_pads_and_fences_stay_out_of_carriageways`), and both are the reason
// those two stayed green:
//
//  1. MIXED WIDTHS. Both existing tests use ONE width for the whole net (a 13 m
//     arterial). A single scalar setback can serve one width; the failure needs
//     a narrow street and a wide arterial meeting at the same junction, so that
//     any margin picked for one is wrong for the other.
//
//  2. PRODUCTION'S OWN DERIVATION. Both existing tests hand-pick the margin they
//     then assert against — the test does the road-half arithmetic that the
//     shipping code is supposed to do. Here the margins come from
//     readLotGrowParams (level_params.cpp), the SAME function the loader calls,
//     fed a level's "city" block. If that derivation is wrong, this fails; if
//     someone fixes it, this follows. A test that supplies the input production
//     gets wrong can only ever prove the formula it wrote itself.

#include "test_framework.h"

#include "../src/engine/level_params.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cmath>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <vector>

using namespace engine;

namespace {

constexpr double STREET_WIDTH = 12.0;
constexpr double ARTERIAL_WIDTH = 17.0;

// A generated metro whose edges alternate street / arterial, so a narrow and a
// wide road meet at essentially every junction and share block frontage.
RoadNet mixedWidthMetro() {
    RoadNet net;
    net.width = STREET_WIDTH;
    net.autoRoundabout = false;
    nlohmann::json gen = { { "kind", "metro" },    { "radius", 600 },
                           { "hotspots", 4 },      { "block_size", 130 },
                           { "seed", 21 },         { "freeways", false },
                           { "min_road_len", 24 }, { "terrain_aware", false } };
    applyGenerateRecipe(net, gen);
    net.edgeWidths.assign(net.edges.size(), STREET_WIDTH);
    for (std::size_t i = 1; i < net.edgeWidths.size(); i += 2)
        net.edgeWidths[i] = ARTERIAL_WIDTH;
    return net;
}

// Distance from `p` to the segment a-b (world XZ).
double distanceToSegment(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    const Real l2 = ab.lengthSquared();
    Real t = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return (a + ab * t - p).length();
}

}  // namespace

// The gate. Every vertex of every grown building PLAN stays out of every
// carriageway: at least edge_width/2 from that edge's centreline.
TEST_CASE(lot_buildings_clear_every_carriageway_on_a_mixed_width_net) {
    RoadNet net = mixedWidthMetro();
    RoadGraph rg = navRoadGraph(net);

    // The setup must actually be mixed — otherwise a change to the generator
    // could quietly make this a uniform-width net again and the case would pass
    // for the wrong reason. Count it, then assert it.
    int narrow = 0, wide = 0;
    for (const RoadEdge& e : rg.edges) {
        if (e.width < (STREET_WIDTH + ARTERIAL_WIDTH) * 0.5) ++narrow;
        else ++wide;
    }
    CHECK(narrow > 0);
    CHECK(wide > 0);

    // Production's derivation, not a number chosen here: this is the block a
    // level's shape:"city" entity carries, and readLotGrowParams is what the
    // loader calls on it (level_loader.cpp -> level_params.cpp).
    nlohmann::json cityJson = { { "seed", 21 }, { "sidewalk", 4.0 },
                                { "buildChance", 0.9 } };
    EdgeBlockParams ep;
    LotParams lp;
    readLotGrowParams(cityJson, ep, lp);

    // The one derivation the loader keeps at its own call site rather than in
    // level_params (level_loader.cpp:2038). Duplicated here knowingly: it is a
    // mirror, and mirrors drift — worth moving into readLotGrowParams so this
    // test and the loader cannot disagree.
    const Real roadClear = cityJson.value("sidewalk", 4.0) + 0.6;

    NetLotResult grown = growLotBuildingsOnNets({ net }, lp, ep, roadClear, &rg);
    CHECK(!grown.lots.empty());

    int built = 0, intruding = 0;
    double worstOverlap = 0.0;
    std::string worstType;
    for (const LotBuilding& lb : grown.lots) {
        if (lb.plan.empty()) continue;             // parks/greens carry a pad, not a plan
        if (lb.recipe.rfind("underfreeway", 0) == 0) continue;   // in the ROW by design
        ++built;
        bool hit = false;
        for (const Vec2& v : lb.plan) {
            for (const RoadEdge& e : rg.edges) {
                if (e.a < 0 || e.b < 0) continue;
                const double d = distanceToSegment(v, rg.nodes[e.a].pos,
                                                   rg.nodes[e.b].pos);
                const double overlap = e.width * 0.5 - d;
                if (overlap <= 0) continue;
                hit = true;
                if (overlap > worstOverlap) {
                    worstOverlap = overlap;
                    worstType = lb.type + "/" + lb.recipe;
                }
            }
        }
        if (hit) ++intruding;
    }

    std::printf("    [clearance] %d buildings, %d intruding, worst %.2f m into "
                "the carriageway (%s)\n",
                built, intruding, worstOverlap, worstType.c_str());
    CHECK(built > 0);
    CHECK(intruding == 0);
}

// The LOT LINE, one level up from the building.
//
// Buildings survive a wrong setback because growLotBuildings runs a width-aware
// corner test on top of it (`clearOfRoads`, city_lots.cpp:917 — it knows each
// edge's real width, and the SAMPLED centreline, and shrinks the massing to
// fit). The block interior the parcel pass subdivides gets no such second
// chance. It is planned against a cruder model of the road in two ways:
//
//  1. ONE SCALAR SETBACK. LotParams::roadMargin, which level_params.cpp:178
//     derives as `4.0 + sidewalk` — the 4.0 standing in for "road half". That is
//     the half-width of an 8 m road, not of the 17 m arterial this net carries
//     (8.5). The plan (docs/knowledge-retention-plan.md, "Carried over") names
//     this one.
//  2. THE WRONG CENTRELINE. Block faces are extracted from the nets' own
//     nodes/edges — the control chords — because "the sampled navRoadGraph loses
//     faces" (city_lots.cpp:2250). The asphalt is meshed from the SAMPLED
//     spline, and the same comment notes "a curvy road bows off its control
//     chords". Where it bows, the block edge is inside the real carriageway.
//
// The cost is not cosmetic: lots planned inside a road are thrown away
// downstream as `rejClear`, so the error shows up as density that never got
// built. That count is printed with the failure.
//
// This gate is written BEFORE the fix, which is what the plan asks for.
TEST_CASE(lot_block_interiors_clear_every_carriageway_on_a_mixed_width_net) {
    RoadNet net = mixedWidthMetro();
    RoadGraph rg = navRoadGraph(net);

    nlohmann::json cityJson = { { "seed", 21 }, { "sidewalk", 4.0 },
                                { "buildChance", 0.9 } };
    EdgeBlockParams ep;
    LotParams lp;
    readLotGrowParams(cityJson, ep, lp);
    const Real roadClear = cityJson.value("sidewalk", 4.0) + 0.6;

    NetLotResult grown = growLotBuildingsOnNets({ net }, lp, ep, roadClear, &rg);
    CHECK(!grown.plan.blocks.empty());

    int intruding = 0;
    double worstOverlap = 0.0;
    double worstRoadWidth = 0.0;
    for (const Poly2& block : grown.plan.blocks) {
        bool hit = false;
        for (const Vec2& v : block) {
            for (const RoadEdge& e : rg.edges) {
                if (e.a < 0 || e.b < 0) continue;
                const double d = distanceToSegment(v, rg.nodes[e.a].pos,
                                                   rg.nodes[e.b].pos);
                const double overlap = e.width * 0.5 - d;
                if (overlap <= 1e-6) continue;
                hit = true;
                if (overlap > worstOverlap) {
                    worstOverlap = overlap;
                    worstRoadWidth = e.width;
                }
            }
        }
        if (hit) ++intruding;
    }

    std::printf("    [lot line] %zu blocks, %d intruding, worst %.2f m inside a "
                "%.1f m carriageway (roadMargin=%.2f) | %d lots discarded as "
                "rejClear\n",
                grown.plan.blocks.size(), intruding, worstOverlap, worstRoadWidth,
                static_cast<double>(lp.roadMargin), grown.plan.rejClear);
    CHECK(intruding == 0);
}
