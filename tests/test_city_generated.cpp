#include "test_framework.h"

#include "../src/engine/procgen/city/city.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/ai/nav_graph.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/apps/citysim/places.h"

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
