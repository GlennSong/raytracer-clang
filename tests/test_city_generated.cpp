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
    // Bound scales with the R3 signal cycle: per-group slots + the WALK
    // window make a legitimate worst red-wait ~40-45 s at a 3-group junction
    // (it was ~22 s under the old fixed 2-phase cycle). Real gridlock — the
    // thing this gate exists for — measured MINUTES when it happened.
    std::printf("[grid] worst stall=%.1f s\n", worst * 0.25);
    CHECK(worst * 0.25 < 55.0);   // never fully stalled for half a minute
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
    const int fastAfterWarmup = sim.fastCrashEvents();
    for (int i = 0; i < 6000; ++i) sim.step(0.1, 0.02);   // 10 sim-minutes
    std::printf("[soak] warmup crashes=%d run crashes=%d fast=%d\n", afterWarmup,
                sim.crashEvents() - afterWarmup,
                sim.fastCrashEvents() - fastAfterWarmup);
    // RATCHET (deterministic sim). Pre-S7 baseline: 115/10 min under the
    // proximity-disc rule. Slice 2 (oriented capsule contact): 95. Slice 3
    // (corridor vision wedge -> IDM leader): 52. Slice 4 (crossing-course
    // sense, right-yield): FAST crashes — both bodies above walking pace,
    // the class 'no pile-ups' is about — down to <= 3/10 min, none of them
    // crossings (oncoming/parallel U-turn artifacts). The total is slow
    // junction-mouth brushes: kinematic lane ribbons overlapping at creep
    // speed, arbitrated by the fender-bender freeze; their fix is junction
    // path geometry (lane-frame steering), not more braking rules. FAST is
    // the strict gate; the total gets small headroom because ANY behaviour
    // change elsewhere (e.g. walker path shape) perturbs the deterministic
    // run by a few slow brushes either way.
    // Re-measured after R3 (conflict phases + staggered offsets + WALK):
    // 4 on this seed, same artifact class as before (synthetic-tip U-turn
    // meets, not crossings — verified via the stall/contact dumps). R5's
    // physical steering owns retiring the class.
    CHECK(sim.fastCrashEvents() - fastAfterWarmup <= 4);
    // Slow-brush baseline re-measured after R3: WALK windows bunch queue
    // discharge at green onset, adding creep-speed brushes (75 on this seed;
    // retire with R5's physical steering). The INVARIANT is fast == 0.
    CHECK(sim.crashEvents() - afterWarmup <= 80);
}

TEST_CASE(walkers_keep_off_the_carriageway) {
    // Roads-v2 S8 gate (plan §5.8, scaled for the suite): across a sustained
    // run of the generated city, NO walker is ever on the asphalt outside a
    // junction crossing. Mid-block the sidewalk offset owns them; the only
    // sanctioned carriageway presence is the corner-cut crossing window
    // around a junction node.
    RoadNet net;
    net.width = 7.0;
    net.sidewalk = 1.8;
    nlohmann::json gen = {{"kind", "district"}, {"radius", 170}, {"arterials", 3},
                          {"artery_width", 13}, {"street_width", 7},
                          {"block_size", 82},   {"curviness", 0.22}, {"seed", 5}};
    applyGenerateRecipe(net, gen);
    NavGraph nav = buildNavGraph(navRoadGraph(net));

    CitySim sim;
    sim.build(nav, 14, 24, 7);
    for (int i = 0; i < 400; ++i) sim.step(0.1);
    // Junction nodes + each one's widest incident carriageway: the sanctioned
    // crossing window is per-JUNCTION, not per-attributed-link — nearestLink
    // hands a walker crossing beside a junction to whichever link's centreline
    // is closest, whose own endpoints may be a different pair of nodes.
    std::vector<std::pair<engine::Vec2, Real>> crossings;
    for (int ni = 0; ni < nav.nodeCount(); ++ni) {
        if (!nav.isJunction(ni)) continue;
        Real w = 0;
        for (int li2 = 0; li2 < nav.linkCount(); ++li2)
            if (nav.links[li2].from == ni || nav.links[li2].to == ni)
                w = std::max(w, nav.links[li2].width);
        crossings.push_back({nav.nodes[ni], w * 0.5 + 6.0});
    }
    long roadWalks = 0, walkTicks = 0;
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.02);
        for (const Agent& a : sim.agents()) {
            if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
            ++walkTicks;
            // Measure against the walker's OWN leg: guide-line discipline —
            // the walker keeps to ITS road's sidewalk and enters a
            // carriageway only inside a junction crossing window. (Whether a
            // sidewalk band and a NEIGHBOURING road's ribbon overlap is a
            // mesh-ownership question the S5/S6 surface scan already gates —
            // nearestLink attribution here just re-reported that geometry.)
            if (a.leg >= static_cast<int>(a.route.links.size())) continue;
            const int li = a.route.links[a.leg];
            if (li < 0) continue;
            // On the asphalt: inside the carriageway half-width (a margin
            // under it — the kerb edge itself is legal).
            const engine::Vec2 A = nav.nodes[nav.links[li].from];
            const engine::Vec2 B = nav.nodes[nav.links[li].to];
            const engine::Vec2 AB = B - A;
            const Real l2 = AB.lengthSquared();
            Real t = l2 > 1e-9 ? ((a.pos.x - A.x) * AB.x + (a.pos.y - A.y) * AB.y) / l2
                               : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const engine::Vec2 q(A.x + AB.x * t, A.y + AB.y * t);
            const Real lat = (a.pos - q).length();
            // On the asphalt = clearly inside the kerb line. The 0.75 m
            // margin absorbs chord-geometry noise: on 4-5 m chord links at
            // sharp curve knots the offset guide grazes 0-11 cm inside the
            // notional half-width (the meshed kerb sits OUTSIDE those chords
            // — S5's loops miter properly). A walker genuinely on the road
            // is metres inside; the pre-miter corner dips measured 1.5 m.
            if (lat >= nav.links[li].width * 0.5 - 0.75) continue;
            bool nearJunction = false;
            for (const auto& c : crossings)
                if ((a.pos - c.first).length() < c.second) { nearJunction = true; break; }
            if (!nearJunction) ++roadWalks;
        }
    }
    std::printf("[walk] ticks=%ld roadWalks=%ld\n", walkTicks, roadWalks);
    CHECK(walkTicks > 1000);   // walkers really walked
    CHECK(roadWalks == 0);     // and never down the middle of a block's road
}
