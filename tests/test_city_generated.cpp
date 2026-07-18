#include "test_framework.h"

#include "../src/engine/procgen/city/city.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/ai/nav_graph.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/apps/citysim/places.h"

#include <nlohmann/json.hpp>

using namespace engine;
using namespace citysim;

// Living City, ADR-0066: the generated-city → citysim bridge. living_city.json
// runs a shape:"city" generator whose road graph feeds the sim. This pins the
// exact path the level_loader takes (CityModel.roadGraph → RoadNet → navRoadGraph
// → buildNavGraph → CitySim) so "a generated city is drivable and its buildings
// are places" is a headless regression, not something only a browser can reveal.

namespace {
NavGraph navFromCity(const CityModel& m) {
    // Mirror level_loader: build a RoadNet from the surfaced road graph, then the
    // same navRoadGraph → buildNavGraph the render bridge runs.
    RoadNet net;
    for (const auto& n : m.roadGraph.nodes) net.nodes.push_back(n.pos);
    for (const auto& e : m.roadGraph.edges) net.edges.push_back({e.a, e.b});
    net.width = 12.0;
    return buildNavGraph(navRoadGraph(net));
}
}  // namespace

TEST_CASE(generated_city_surfaces_a_road_graph) {
    CityParams cp;
    cp.extent = 120; cp.cellSize = 70; cp.seed = 3; cp.buildChance = 0.85;
    CityModel m = generateCity(cp);
    CHECK(!m.roadGraph.nodes.empty());
    CHECK(!m.roadGraph.edges.empty());
    CHECK(!m.buildings.empty());   // and it has buildings to become places
}

TEST_CASE(generated_city_is_drivable_by_the_sim) {
    CityParams cp;
    cp.extent = 120; cp.cellSize = 70; cp.seed = 3; cp.buildChance = 0.85;
    CityModel m = generateCity(cp);
    NavGraph nav = navFromCity(m);
    CHECK(nav.nodeCount() > 0);
    CHECK(nav.linkCount() > 0);   // the sim needs links to place + move agents

    CitySim sim;
    sim.build(nav, /*cars*/ 12, /*peds*/ 12, /*seed*/ 5);
    CHECK(sim.agents().size() == 24u);   // agents actually spawned

    // Drive it a while; on a real road network agents must get MOVING (this is
    // exactly what "no cars in the city" would fail).
    for (int i = 0; i < 300; ++i) sim.step(0.1);
    int moving = 0;
    for (const Agent& a : sim.agents())
        if (a.moving || a.speed > 0.05) ++moving;
    CHECK(moving > 0);
}

TEST_CASE(district_city_composes_real_roads_lots_and_buildings) {
    // The Living City proper: streets from the district road tech (arterials +
    // irregular streets), lots subdivided from ITS blocks, buildings on the lots —
    // all one system, and the same graph the sim drives.
    // Mirrors living_city.json: wide sidewalks (roadInset = 8 + sidewalk) need
    // blocks big enough that lots still carry a building after the setback.
    CityParams cp;
    cp.districtRoads = true;
    cp.extent = 150; cp.arterials = 3;
    cp.blockSizeMin = 38; cp.blockSizeMax = 64;
    cp.sidewalk = 5.0; cp.buildChance = 0.9; cp.seed = 4;
    CityModel m = generateCity(cp);
    CHECK(!m.roadGraph.edges.empty());   // real roads, surfaced for the sim
    CHECK(m.blockCount > 0);             // blocks came from the district net
    CHECK(!m.buildings.empty());         // wide sidewalks still leave room for buildings

    NavGraph nav = navFromCity(m);
    CHECK(nav.linkCount() > 0);
    CitySim sim;
    sim.build(nav, 20, 20, 6);
    for (int i = 0; i < 300; ++i) sim.step(0.1);
    int moving = 0;
    for (const Agent& a : sim.agents())
        if (a.moving || a.speed > 0.05) ++moving;
    CHECK(moving > 0);                   // agents drive the district streets
}

TEST_CASE(living_city_traffic_never_gridlocks) {
    // Regression for the device jam: living_city's curvy arterial crossing gets
    // sampled into a KNOT of ~4.5 m links whose junction boxes (arterial
    // half-width) used to overlap — every held car sat inside someone else's box,
    // a circular wait, permanent standstill. Rebuild the exact level pipeline and
    // assert traffic keeps flowing through the whole commute cycle. Fixed by (a)
    // capping the box radius at 60% of the shortest incident link and (b) the
    // gridlock escape (a car held >~6 s by only STALLED occupants creeps through).
    RoadNet net;
    net.width = 7.0;
    net.sidewalk = 1.8;
    nlohmann::json gen = {{"kind", "district"}, {"radius", 170}, {"arterials", 3},
                          {"artery_width", 13}, {"street_width", 7},
                          {"block_size", 82},   {"curviness", 0.22}, {"seed", 5}};
    applyGenerateRecipe(net, gen);
    NavGraph nav = buildNavGraph(navRoadGraph(net));
    CHECK(nav.linkCount() > 0);

    RoadGraph rg;
    for (const Vec2& n : net.nodes) rg.nodes.push_back({n});
    for (const auto& e : net.edges)
        rg.edges.push_back(RoadEdge{e[0], e[1], 8, RoadClass::Local, 0});
    LotParams lp;
    lp.seed = 5u ^ 0x10c5u;
    lp.buildChance = 0.9;
    lp.roadMargin = 8.0;
    PlaceMap places;
    for (const LotBuilding& lb : growLotBuildings(extractBlocks(rg), lp)) {
        PlaceType t;
        if (parsePlaceType(lb.type, t)) places.add(t, lb.site, nav);
    }
    CHECK(places.size() > 10);

    CitySim sim;
    sim.build(nav, 22, 26, 5);
    sim.assignPlaces(places, nav);
    for (int i = 0; i < 400; ++i) sim.step(0.1);   // the level's warm-up

    // Run through the day. Track the longest streak where EVERY driving car is
    // pinned at zero — before the fix this hit 328 sim-seconds (permanent).
    long streak = 0, worst = 0;
    for (int i = 0; i < 60 * 60 * 2 * 2; ++i) {
        sim.step(0.25);
        int driving = 0, stopped = 0;
        for (const Agent& a : sim.agents()) {
            if (a.mode != Agent::Mode::Driver) continue;
            if (a.state == Agent::State::Resting || !a.moving) continue;
            ++driving;
            if (a.speed < 0.05) ++stopped;
        }
        streak = (driving >= 3 && stopped == driving) ? streak + 1 : 0;
        worst = std::max(worst, streak);
    }
    CHECK(worst * 0.25 < 30.0);   // never fully stalled for half a minute
}

TEST_CASE(generated_city_buildings_snap_as_places) {
    CityParams cp;
    cp.extent = 120; cp.cellSize = 70; cp.seed = 3; cp.buildChance = 0.85;
    CityModel m = generateCity(cp);
    NavGraph nav = navFromCity(m);

    // Each building snaps onto a walkable edge (what the loader does per building).
    PlaceMap places;
    for (const CityBuilding& b : m.buildings)
        places.add(PlaceType::Home, b.site, nav);
    CHECK(places.size() == static_cast<int>(m.buildings.size()));
    for (const Place& p : places.places()) CHECK(p.entranceLink >= 0);
}

TEST_CASE(traffic_soak_no_collisions_idm) {
    // Roads-v2 §5.7 soak gate (scaled for the suite; the full-hour run is the
    // release check): a generated city's traffic, driven by IDM car-following
    // + signals + junction rules, runs a sustained commute with ZERO car
    // contacts. Before IDM the follower law was a linear speed ramp with
    // instantaneous braking — pile-ups were arbitrated after the fact by the
    // fender-bender freeze; the backbone now prevents them.
    RoadNet net;
    net.width = 7.0;
    net.sidewalk = 1.8;
    nlohmann::json gen = {{"kind", "district"}, {"radius", 170}, {"arterials", 3},
                          {"artery_width", 13}, {"street_width", 7},
                          {"block_size", 82},   {"curviness", 0.22}, {"seed", 5}};
    applyGenerateRecipe(net, gen);
    NavGraph nav = buildNavGraph(navRoadGraph(net));
    CHECK(nav.linkCount() > 0);

    CitySim sim;
    sim.build(nav, 26, 20, 5);
    for (int i = 0; i < 400; ++i) sim.step(0.1);       // warm-up
    const int afterWarmup = sim.crashEvents();
    for (int i = 0; i < 6000; ++i) sim.step(0.1, 0.02);   // 10 sim-minutes
    std::printf("[soak] warmup crashes=%d run crashes=%d\n", afterWarmup,
                sim.crashEvents() - afterWarmup);
    // RATCHET (deterministic sim, exact count). Pre-S7 baseline: 115/10 min
    // under the proximity-disc rule. Slice 2 (oriented capsule contact): 95.
    // Slice 3 (corridor vision wedge -> IDM leader): 52. The remainder is
    // dominated by junction crossing conflict — the crossing-course slice's
    // target. The plan's end gate is 0.
    CHECK(sim.crashEvents() - afterWarmup <= 52);
}
