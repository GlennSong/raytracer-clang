#include "test_framework.h"

#include "../src/engine/ai/pathfind.h"
#include "../src/engine/procgen/city/road_net.h"   // S8: band-model walk tests

using namespace engine;

namespace {

// A symmetric diamond from S(0,0) to G(20,0): a northern arm through M1 is
// Arterial, a southern arm through M2 is Local. Both arms are the same LENGTH,
// so a shortest-distance planner would tie — but A* costs by travel TIME, so it
// must pick the faster (arterial) arm.
//   node 0 = S, 1 = G, 2 = M1 (north, arterial), 3 = M2 (south, local)
RoadGraph diamond() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(20, 0)}, {Vec2(10, 5)}, {Vec2(10, -5)} };
    g.edges = {
        RoadEdge{0, 2, 12, RoadClass::Arterial, 0},   // S -> M1
        RoadEdge{2, 1, 12, RoadClass::Arterial, 0},   // M1 -> G
        RoadEdge{0, 3, 8,  RoadClass::Local, 0},      // S -> M2
        RoadEdge{3, 1, 8,  RoadClass::Local, 0},      // M2 -> G
    };
    return g;
}

}  // namespace

TEST_CASE(pathfind_finds_a_route) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 0, 1);
    CHECK(r.valid());
    // Reaches the goal: the last link's head is node 1.
    CHECK(nav.links[r.links.back()].to == 1);
}

TEST_CASE(pathfind_prefers_faster_road) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 0, 1);
    CHECK(r.valid());
    // Every link on the chosen route is Arterial (the faster arm via M1=node 2).
    bool allArterial = true;
    bool viaM1 = false;
    for (int li : r.links) {
        if (nav.links[li].klass != RoadClass::Arterial) allArterial = false;
        if (nav.links[li].from == 2 || nav.links[li].to == 2) viaM1 = true;
    }
    CHECK(allArterial);
    CHECK(viaM1);
}

TEST_CASE(pathfind_route_length) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 0, 1);
    // Two arterial arms, each sqrt(10^2 + 5^2) = 11.1803...
    CHECK_APPROX(r.length(nav), 2.0 * 11.1803398875, 1e-6);
}

TEST_CASE(pathfind_same_node_is_empty) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 1, 1);
    CHECK(!r.valid());
}

TEST_CASE(pathfind_unreachable_is_empty) {
    RoadGraph g = diamond();
    g.nodes.push_back({Vec2(100, 100)});   // node 4: isolated, no edges
    NavGraph nav = buildNavGraph(g);
    Route r = findRoute(nav, 0, 4);
    CHECK(!r.valid());
}

TEST_CASE(pathfind_class_speed_ordering) {
    // Faster classes must report higher free-speeds (the routing cost basis).
    CHECK(classSpeed(RoadClass::Freeway) > classSpeed(RoadClass::Arterial));
    CHECK(classSpeed(RoadClass::Arterial) > classSpeed(RoadClass::Local));
}

TEST_CASE(pathfind_walkers_need_sidewalk_bands) {
    // Roads-v2 S8: walk permission comes from the BAND MODEL, not just class.
    // Two routes from node 0 to node 1: a direct street whose spec has NO
    // sidewalk bands (a country road through the block), and a longer L-shaped
    // pair of ordinary streets with sidewalks. Cars take the short cut; a
    // pedestrian must go around.
    engine::RoadEntity net;
    net.look.defaultWidth = 7.0;
    net.graph.nodes = { engine::RoadNode{engine::Vec2(0, 0)},
                        engine::RoadNode{engine::Vec2(100, 0)},
                        engine::RoadNode{engine::Vec2(0, 90)},
                        engine::RoadNode{engine::Vec2(100, 90)} };
    net.graph.addEdge(0, 1, 7.0);      // direct: dirt (no sidewalk band)
    net.graph.addEdge(0, 2, 7.0);      // around: local streets
    net.graph.addEdge(2, 3, 7.0);
    net.graph.addEdge(3, 1, 7.0);
    net.graph.specs = { engine::roadSpecPreset("dirt"),
                        engine::roadSpecPreset("local") };
    net.graph.edges[0].spec = 0;
    net.graph.edges[1].spec = 1;
    net.graph.edges[2].spec = 1;
    net.graph.edges[3].spec = 1;
    NavGraph nav = buildNavGraph(engine::navRoadGraph(net, nullptr));

    const int s = nav.nearestNode(engine::Vec2(0, 0));
    const int g = nav.nearestNode(engine::Vec2(100, 0));
    Route car = findRoute(nav, s, g, /*onFoot=*/false);
    Route ped = findRoute(nav, s, g, /*onFoot=*/true);
    CHECK(car.valid());
    CHECK(ped.valid());
    CHECK(car.length(nav) < 120.0);        // driver takes the direct road
    CHECK(ped.length(nav) > 200.0);        // walker goes around the block
    for (int li : ped.links) CHECK(nav.links[li].walkable);
}

TEST_CASE(pathfind_walkers_never_route_the_authored_freeway) {
    // An AUTHORED freeway-class net (weld_freeway_lab-style: edge_classes +
    // freeway3 spec, NOT baked) reaches navRoadGraph directly — a BAKED
    // corridor never does (roadNetStreetsOnly strips it; its nav comes from
    // the corridor fragments, where pathfind's class backstop rules). Assert
    // the DATA level here: the freeway3 spec has no Sidewalk band, so the
    // edge is unwalkable BEFORE the class rule even looks, and an on-foot
    // route between its endpoints goes around on the streets.
    engine::RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.graph.nodes = { engine::RoadNode{engine::Vec2(0, 0)},
                        engine::RoadNode{engine::Vec2(200, 0)},
                        engine::RoadNode{engine::Vec2(0, 80)},
                        engine::RoadNode{engine::Vec2(200, 80)} };
    net.graph.addEdge(0, 1, 8.0, engine::RoadClass::Freeway);   // direct: freeway carriageway
    net.graph.addEdge(0, 2, 8.0, engine::RoadClass::Local);     // around: local streets
    net.graph.addEdge(2, 3, 8.0, engine::RoadClass::Local);
    net.graph.addEdge(3, 1, 8.0, engine::RoadClass::Local);
    net.graph.specs = { engine::roadSpecPreset("freeway3"),
                        engine::roadSpecPreset("local") };
    net.graph.edges[0].spec = 0;
    net.graph.edges[1].spec = 1;
    net.graph.edges[2].spec = 1;
    net.graph.edges[3].spec = 1;

    engine::RoadGraph rg = engine::navRoadGraph(net, nullptr);
    bool sawFreewayUnwalkable = false;
    for (const engine::RoadEdge& re : rg.edges)
        if (re.klass == engine::RoadClass::Freeway) {
            CHECK(!re.walkable);       // the band model shut it, not the class
            sawFreewayUnwalkable = true;
        }
    CHECK(sawFreewayUnwalkable);

    NavGraph nav = buildNavGraph(rg);
    const int s = nav.nearestNode(engine::Vec2(0, 0));
    const int g = nav.nearestNode(engine::Vec2(200, 0));
    Route ped = findRoute(nav, s, g, /*onFoot=*/true);
    CHECK(ped.valid());
    for (int li : ped.links) {
        CHECK(nav.links[li].walkable);
        CHECK(nav.links[li].klass != engine::RoadClass::Freeway);
    }
    CHECK(ped.length(nav) > 350.0);    // around the block, not down the deck
}

// --- routePolyline (possession round, ADR-0079) ------------------------------
// The pure sampler that turns a Route into LaneFollower input: lane centres
// for cars, sidewalk offsets for walkers, per-point class speeds alongside.

TEST_CASE(pathfind_route_polyline_is_dense_and_reaches_both_ends) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 0, 1);
    CHECK(r.valid());

    std::vector<Vec2> pts = routePolyline(nav, r, 3.0);
    CHECK(pts.size() >= 2u);
    // Starts near S(0,0) and ends near G(20,0) — lane offset keeps it within a
    // couple of lane widths of the node, never somewhere else on the map.
    auto near = [](const Vec2& a, Real x, Real y, Real tol) {
        const Real dx = a.x - x, dy = a.y - y;
        return dx * dx + dy * dy <= tol * tol;
    };
    CHECK(near(pts.front(), 0, 0, 8.0));
    CHECK(near(pts.back(), 20, 0, 8.0));
    // Dense: consecutive points at most ~step apart (lane-offset kinks at the
    // junction add a little; 1.5x bounds it).
    for (std::size_t i = 1; i < pts.size(); ++i) {
        const Real dx = pts[i].x - pts[i - 1].x, dy = pts[i].y - pts[i - 1].y;
        CHECK(std::sqrt(dx * dx + dy * dy) <= 4.5 + 1e-9);
    }
}

TEST_CASE(pathfind_route_polyline_speeds_track_link_class) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 0, 1);   // prefers the arterial arm
    std::vector<Real> speeds;
    std::vector<Vec2> pts = routePolylineWithSpeeds(nav, r, speeds);
    CHECK(pts.size() == speeds.size());
    for (Real s : speeds) CHECK(std::fabs(s - classSpeed(RoadClass::Arterial)) < 1e-9);
}

TEST_CASE(pathfind_route_polyline_sidewalk_offsets_off_the_centreline) {
    NavGraph nav = buildNavGraph(diamond());
    Route r = findRoute(nav, 0, 1);
    std::vector<Vec2> lane = routePolyline(nav, r, 3.0, /*sidewalk=*/false);
    std::vector<Vec2> walk = routePolyline(nav, r, 3.0, /*sidewalk=*/true);
    CHECK(lane.size() == walk.size());
    // The sidewalk offset puts every sample clear of the lane centre — a
    // walker polyline that coincided with the carriageway would march the
    // pedestrian down the middle of the road.
    for (std::size_t i = 0; i < lane.size(); ++i) {
        const Real dx = lane[i].x - walk[i].x, dy = lane[i].y - walk[i].y;
        CHECK(dx * dx + dy * dy > 1.0);
    }
}
