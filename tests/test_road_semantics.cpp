// The road-graph SEMANTIC LAYER (roads-v2.2, issue #17): classifyRoadGraph
// stamps JunctionKind on nodes and access bits on edges; the bake stamps its
// gores and landings as hints. These tests pin the classification rules and
// the carriers (netGraph hint copy, streetsOnly degradation, nav knot-merge
// priority) BEFORE any consumer reads the fields — stage S1 must be
// behaviorally invisible.
#include "test_framework.h"

#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/procgen/city/corridor_bake.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/corridor_plan.h"
#include "../src/engine/procgen/city/road_net.h"
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

// The corridor bake stamps its gores and landings — the fixture (borrowed
// from test_freeway_graph) has an EXIT at s=400 and an ON-RAMP at s=300, so
// both gore flavours and both landings appear in one net.
TEST_CASE(semantics_bake_stamps_gores_and_landings) {
    // The exact test_freeway_graph fixture: streets under the corridor, one
    // EXIT at s=400 and one ON-RAMP at s=300 — both gore flavours + both
    // landings in one net.
    RoadNet net;
    net.nodes = { Vec2(60, -120),  Vec2(200, -120), Vec2(340, -120),
                  Vec2(460, -120), Vec2(600, -120), Vec2(740, -120) };
    net.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 } };
    net.width = 8.0;
    net.autoRoundabout = false;
    net.heightAt = [](double, double) { return 0.0; };

    CorridorDef def;
    def.horizontal =
        Alignment::fromPolyline({ Vec2(-100, 0), Vec2(800, 0) }, 300.0, 20.0);
    const Real L = def.horizontal.length();
    def.vertical.pvis = { { 0.0, 9.0, 0.0 }, { L, 9.0, 0.0 } };
    def.lanes.throughLanes = 3;
    ExitDef e;
    e.station = 400.0;
    e.upStation = true;
    e.target = Vec2(600, -120);
    e.targetY = 0.0;
    def.exits.push_back(e);
    ExitDef on;
    on.station = 300.0;
    on.upStation = true;
    on.onRamp = true;
    on.target = Vec2(60, -120);
    on.targetY = 0.0;
    def.exits.push_back(on);

    CorridorAuthoring au = corridorAuthor(def, [](Real, Real) { return 0.0; });
    std::vector<int> mainline = bakeCorridorIntoNet(net, def, au.rampPaths);
    CHECK(!mainline.empty());

    RoadGraph g = roadNetFullGraph(net);
    int merges = 0, diverges = 0, landings = 0;
    for (const RoadNode& n : g.nodes) {
        if (n.kind == JunctionKind::Merge) ++merges;
        if (n.kind == JunctionKind::Diverge) ++diverges;
        if (n.kind == JunctionKind::Landing) ++landings;
    }
    std::printf("[sem] merges=%d diverges=%d landings=%d\n", merges, diverges,
                landings);
    CHECK(merges >= 1);     // the on-ramp's gore
    CHECK(diverges >= 1);   // the exit's gore
    CHECK(landings >= 2);   // BOTH ramp feet — including the street-tip one
                            // (degree 2: the hint must survive there or the
                            // approach rule (#20) never fires on feeders)

    // Ramp/freeway edges carry no street access; plain streets keep all.
    for (const RoadEdge& e : g.edges) {
        if (e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp)
            CHECK(e.access == 0);
    }

    // STREETS-ONLY re-derivation: the ramp edges are stripped, so a hinted
    // landing must DEGRADE to what its remaining street arms are (here the
    // straight street run: a through node, not a junction).
    const RoadNet so = roadNetStreetsOnly(net);
    RoadGraph gs = navRoadGraph(so);
    for (const RoadNode& n : gs.nodes) {
        CHECK(n.kind != JunctionKind::Merge);
        CHECK(n.kind != JunctionKind::Diverge);
    }
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
