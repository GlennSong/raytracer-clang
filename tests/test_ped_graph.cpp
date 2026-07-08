#include "test_framework.h"

#include "../src/apps/citysim/ped_graph.h"
#include "../src/apps/citysim/places.h"
#include "city_test_util.h"

using namespace engine;
using namespace citysim;

// Living City, ADR-0066 Phase 2: the pedestrian nav-graph — a walkable layer,
// separate from the shared car/ped road graph, whose routes begin and end at
// building DOORS. Pure + headless; live-sim integration is a later step.

namespace {
// A grid town with a home and a shop on opposite arms of a 4-way cross.
PedGraph townGraph(NavGraph& navOut, PlaceMap& placesOut) {
    navOut = citytest::cross4(45);
    placesOut = PlaceMap{};
    placesOut.add(PlaceType::Home, Vec2(-40, 5), navOut);
    placesOut.add(PlaceType::Shop, Vec2(40, -5), navOut, 8, 20);
    return buildPedGraph(navOut, placesOut);
}
}  // namespace

TEST_CASE(ped_graph_has_a_door_per_place) {
    NavGraph nav;
    PlaceMap places;
    PedGraph g = townGraph(nav, places);

    CHECK(g.nodeCount() == nav.nodeCount() + 2);   // corners + two doors
    for (const Place& p : places.places()) {
        const int door = g.doorOf(p.id);
        CHECK(door >= 0);
        // The door node sits exactly at the place's snapped entrance.
        CHECK_APPROX(g.nodes[door].x, p.entrance.x, 1e-9);
        CHECK_APPROX(g.nodes[door].y, p.entrance.y, 1e-9);
    }
}

TEST_CASE(ped_route_runs_door_to_door) {
    NavGraph nav;
    PlaceMap places;
    PedGraph g = townGraph(nav, places);

    const int home = g.doorOf(0);   // Home
    const int shop = g.doorOf(1);   // Shop
    std::vector<int> path = pedRoute(g, home, shop);

    CHECK(path.size() >= 2u);
    CHECK(path.front() == home);    // starts at the home door
    CHECK(path.back() == shop);     // ends at the shop door
    // Every hop is a real edge of the graph (route never leaves the walkable net).
    for (std::size_t i = 1; i < path.size(); ++i) {
        bool linked = false;
        for (int ei : g.adj[path[i - 1]]) {
            const PedEdge& e = g.edges[ei];
            if ((e.a == path[i - 1] && e.b == path[i]) ||
                (e.b == path[i - 1] && e.a == path[i])) { linked = true; break; }
        }
        CHECK(linked);
    }
}

TEST_CASE(ped_route_degenerate_cases) {
    NavGraph nav;
    PlaceMap places;
    PedGraph g = townGraph(nav, places);
    const int home = g.doorOf(0);
    CHECK(pedRoute(g, home, home).size() == 1u);   // same node → itself
    CHECK(pedRoute(g, home, -1).empty());          // invalid target
    CHECK(pedRoute(g, 999, home).empty());         // out of range
}

TEST_CASE(ped_graph_is_deterministic) {
    NavGraph a, b;
    PlaceMap pa, pb;
    PedGraph ga = townGraph(a, pa);
    PedGraph gb = townGraph(b, pb);
    CHECK(ga.nodeCount() == gb.nodeCount());
    CHECK(ga.edgeCount() == gb.edgeCount());
    std::vector<int> r1 = pedRoute(ga, ga.doorOf(0), ga.doorOf(1));
    std::vector<int> r2 = pedRoute(gb, gb.doorOf(0), gb.doorOf(1));
    CHECK(r1 == r2);   // same input → same route
}
