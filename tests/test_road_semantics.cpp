// The road-graph SEMANTIC LAYER (roads-v2.2, issue #17): classifyRoadGraph
// stamps JunctionKind on nodes and access bits on edges; the bake stamps its
// gores and landings as hints. These tests pin the classification rules and
// the carriers (netGraph hint copy, streetsOnly degradation, nav knot-merge
// priority) BEFORE any consumer reads the fields — stage S1 must be
// behaviorally invisible.
#include "test_framework.h"

#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/procgen/city/road_semantics.h"

#include <cmath>
#include <cstdio>

using namespace engine;

namespace {

RoadGraph plainCross() {
    RoadGraph g;
    g.nodes.push_back({ Vec2(0, 0) });
    g.nodes.push_back({ Vec2(80, 0) });
    g.nodes.push_back({ Vec2(-80, 0) });
    g.nodes.push_back({ Vec2(0, 80) });
    g.nodes.push_back({ Vec2(0, -80) });
    g.addEdge(0, 1, 10);
    g.addEdge(0, 2, 10);
    g.addEdge(0, 3, 10);
    g.addEdge(0, 4, 10);
    return g;
}

}  // namespace

TEST_CASE(semantics_classifies_street_topology) {
    RoadGraph g = plainCross();
    classifyRoadGraph(g);
    CHECK(g.nodes[0].kind == JunctionKind::Intersection);
    for (int i = 1; i <= 4; ++i) CHECK(g.nodes[i].kind == JunctionKind::DeadEnd);
    for (const RoadEdge& e : g.edges)
        CHECK(e.access == road_access::kAllStreet);

    // Degree-2 through nodes are None; idempotent on re-run.
    RoadGraph line;
    line.nodes.push_back({ Vec2(0, 0) });
    line.nodes.push_back({ Vec2(50, 0) });
    line.nodes.push_back({ Vec2(100, 0) });
    line.addEdge(0, 1);
    line.addEdge(1, 2);
    classifyRoadGraph(line);
    CHECK(line.nodes[1].kind == JunctionKind::None);
    RoadGraph again = line;
    classifyRoadGraph(again);
    CHECK(again.nodes[1].kind == line.nodes[1].kind);
}

TEST_CASE(semantics_freeway_edges_lose_street_access) {
    RoadGraph g;
    g.nodes.push_back({ Vec2(0, 0) });
    g.nodes.push_back({ Vec2(100, 0) });
    g.addEdge(0, 1, 12, RoadClass::Freeway);
    g.addEdge(0, 1, 6, RoadClass::Ramp);
    classifyRoadGraph(g);
    for (const RoadEdge& e : g.edges) CHECK(e.access == 0);
}

TEST_CASE(semantics_nav_carries_kind_through_knot_merge) {
    // Two junction nodes 4 m apart (inside the 7 m merge radius): a hinted
    // Landing glued to a street Intersection — the cluster keeps Landing.
    RoadGraph g;
    g.nodes.push_back({ Vec2(0, 0) });     // street intersection
    g.nodes.push_back({ Vec2(4, 0) });     // landing (hinted)
    g.nodes.push_back({ Vec2(-60, 0) });
    g.nodes.push_back({ Vec2(0, 60) });
    g.nodes.push_back({ Vec2(0, -60) });
    g.nodes.push_back({ Vec2(64, 0) });
    g.nodes.push_back({ Vec2(4, 60) });
    g.addEdge(0, 2, 10);
    g.addEdge(0, 3, 10);
    g.addEdge(0, 4, 10);
    g.addEdge(0, 1, 10);
    g.addEdge(1, 5, 10);
    g.addEdge(1, 6, 6, RoadClass::Ramp);
    g.nodes[1].kind = JunctionKind::Landing;   // the bake's hint
    NavGraph nav = buildNavGraph(g);
    int landings = 0;
    for (int i = 0; i < nav.nodeCount(); ++i)
        if (nav.kindOf(i) == JunctionKind::Landing) ++landings;
    CHECK(landings == 1);
    // And access bits ride the links.
    bool sawRampNoAccess = false;
    for (const NavLink& l : nav.links)
        if (l.klass == RoadClass::Ramp && l.access == 0) sawRampNoAccess = true;
    CHECK(sawRampNoAccess);
}

// [1] Review S4b: the audit's spatial hash must register every cell a
// segment passes through (DDA supercover), not point-sample at cell width.
// Two SHORT edges crossing at their midpoints on a cell corner used to
// bucket into disjoint cells and be silently missed.
TEST_CASE(semantics_audit_catches_short_edge_crossing) {
    // 15 m edges (< the 16 m cell), an X centred on the grid corner (0,0) at
    // 45 degrees — the exact disjoint-bucket case the review constructed.
    RoadGraph g;
    g.nodes.push_back({ Vec2(-5.3, -5.3) });
    g.nodes.push_back({ Vec2(5.3, 5.3) });
    g.nodes.push_back({ Vec2(5.3, -5.3) });
    g.nodes.push_back({ Vec2(-5.3, 5.3) });
    g.addEdge(0, 1, 6, RoadClass::Ramp);
    g.addEdge(2, 3, 6, RoadClass::Ramp);
    auto viols = auditRoadGraph(g);
    std::printf("[sem] short-edge X violations = %zu\n", viols.size());
    CHECK(viols.size() == 1);   // the crossing is found

    // And a long edge crossing a short one (mixed lengths, arbitrary offset)
    // is also caught — the DDA walk covers the whole span.
    RoadGraph g2;
    g2.nodes.push_back({ Vec2(-200, 3) });
    g2.nodes.push_back({ Vec2(200, 3) });
    g2.nodes.push_back({ Vec2(37, -6) });
    g2.nodes.push_back({ Vec2(37, 12) });
    g2.addEdge(0, 1, 8, RoadClass::Ramp);
    g2.addEdge(2, 3, 6, RoadClass::Ramp);
    CHECK(auditRoadGraph(g2).size() == 1);
}

