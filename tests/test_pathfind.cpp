#include "test_framework.h"

#include "../src/engine/ai/pathfind.h"

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
