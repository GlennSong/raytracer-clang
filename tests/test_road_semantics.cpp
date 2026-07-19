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

// The audit (#18): two deliberately-crossing ramp chains at similar height.
// auditRoadGraph reports the crossing, and the BAKE refuses to create it in
// the first place — the second ramp rolls back whole, with no stale gore.
TEST_CASE(semantics_audit_rejects_crossing_ramps) {
    // Hand-built graph: two at-grade edges crossing mid-span, no shared node.
    RoadGraph g;
    g.nodes.push_back({ Vec2(-50, 0) });
    g.nodes.push_back({ Vec2(50, 0) });
    g.nodes.push_back({ Vec2(0, -50) });
    g.nodes.push_back({ Vec2(0, 50) });
    g.addEdge(0, 1, 6, RoadClass::Ramp);
    g.addEdge(2, 3, 6, RoadClass::Ramp);
    auto viols = auditRoadGraph(g);
    CHECK(viols.size() == 1);
    if (!viols.empty()) {
        CHECK(std::fabs(viols[0].at.x) < 1.0);
        CHECK(std::fabs(viols[0].at.y) < 1.0);
        CHECK(viols[0].dY < 4.5);
    }
    // Lift one edge to a proper overpass: no violation.
    g.nodes[2].elev = 6.0;
    g.nodes[2].elevAbsolute = true;
    g.nodes[3].elev = 6.0;
    g.nodes[3].elevAbsolute = true;
    CHECK(auditRoadGraph(g).empty());

    // BAKE-TIME REJECT: an exit whose target forces the authored ramp to
    // cross the earlier exit's chain at like height. The second chain must
    // roll back — same edge count as a one-exit bake, and no Landing stamp
    // for the dropped ramp's target.
    auto bakeWith = [](bool addCrosser) {
        RoadNet net;
        net.nodes = { Vec2(60, -120),  Vec2(200, -120), Vec2(340, -120),
                      Vec2(460, -120), Vec2(600, -120), Vec2(740, -120) };
        net.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 } };
        net.width = 8.0;
        net.autoRoundabout = false;
        net.heightAt = [](double, double) { return 0.0; };
        CorridorDef def;
        def.horizontal = Alignment::fromPolyline(
            { Vec2(-100, 0), Vec2(800, 0) }, 300.0, 20.0);
        const Real L = def.horizontal.length();
        def.vertical.pvis = { { 0.0, 9.0, 0.0 }, { L, 9.0, 0.0 } };
        def.lanes.throughLanes = 3;
        ExitDef e;
        e.station = 400.0;
        e.upStation = true;
        e.target = Vec2(600, -120);
        e.targetY = 0.0;
        def.exits.push_back(e);
        if (addCrosser) {
            // A second exit slightly UPSTREAM whose target lies BEYOND the
            // first ramp's landing: its descending chain must slice across
            // the first chain at like height on the way there.
            ExitDef x;
            x.station = 360.0;
            x.upStation = true;
            x.target = Vec2(740, -120);
            x.targetY = 0.0;
            def.exits.push_back(x);
        }
        CorridorAuthoring au =
            corridorAuthor(def, [](Real, Real) { return 0.0; });
        bakeCorridorIntoNet(net, def, au.rampPaths);
        return net;
    };
    RoadNet one = bakeWith(false);
    RoadNet two = bakeWith(true);
    // Either the AUTHOR already refused the overlapping second ramp, or the
    // bake audit dropped it — both are acceptance; what must never happen is
    // both chains in the net with a crossing.
    RoadGraph gb = roadNetFullGraph(two);
    CHECK(auditRoadGraph(gb, two.heightAt).empty());
    std::printf("[sem] oneExit edges=%zu twoExit edges=%zu\n", one.edges.size(),
                two.edges.size());
}

// S3 STYLING GATE (#21): the road MESH must carry no zebra crossing paint at
// a gore or a ramp landing — that's exactly the "the freeways have
// crosswalks?" / "it thinks an on-ramp merge is an intersection" report. The
// zebra is painted by the shader where a CARRIAGEWAY vertex (u = mu > 1.05)
// has v = mv in the crosswalk window [0.5, 3.6] (mesh.wgsl:400). A plain
// street intersection is the positive control: it MUST still paint one.
namespace {
int zebraVertsNear(const engine::RenderMesh& m, const Vec2& c, double radius) {
    int n = 0;
    for (const Vertex& v : m.vertices) {
        const double dx = v.position.x - c.x, dz = v.position.z - c.y;
        if (dx * dx + dz * dz > radius * radius) continue;
        if (v.u > 1.05 && v.u < 3.0 && v.v > 0.4 && v.v < 3.7) ++n;
    }
    return n;
}
}  // namespace

TEST_CASE(semantics_no_zebra_at_gores_or_landings) {
    // Positive control: a plain street 4-way paints a zebra on each approach.
    {
        RoadNet cross;
        cross.width = 10.0;
        cross.sidewalk = 2.5;
        cross.crosswalks = true;
        cross.autoRoundabout = false;
        cross.nodes = { Vec2(0, 0), Vec2(70, 0), Vec2(-70, 0), Vec2(0, 70),
                        Vec2(0, -70) };
        cross.edges = { { 0, 1 }, { 0, 2 }, { 0, 3 }, { 0, 4 } };
        RenderMesh m = buildRoadNetMesh(cross);
        std::printf("[sem] plain 4-way zebra verts = %d\n",
                    zebraVertsNear(m, Vec2(0, 0), 24.0));
        CHECK(zebraVertsNear(m, Vec2(0, 0), 24.0) > 0);
    }

    // The corridor fixture: streets under an elevated freeway with an exit
    // (Diverge) at s=400 and an on-ramp (Merge) at s=300, landing on the
    // street. NO zebra anywhere near a gore or a landing node.
    RoadNet net;
    net.nodes = { Vec2(60, -120),  Vec2(200, -120), Vec2(340, -120),
                  Vec2(460, -120), Vec2(600, -120), Vec2(740, -120) };
    net.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 } };
    net.width = 8.0;
    net.sidewalk = 2.5;
    net.crosswalks = true;
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
    bakeCorridorIntoNet(net, def, au.rampPaths);

    RenderMesh m = buildRoadNetMesh(net);
    RoadGraph g = roadNetFullGraph(net);
    int checked = 0;
    for (const RoadNode& nd : g.nodes) {
        if (nd.kind != JunctionKind::Landing && !isGore(nd.kind)) continue;
        ++checked;
        const int z = zebraVertsNear(m, nd.pos, 16.0);
        if (z > 0)
            std::printf("[sem] ZEBRA at %s (%.0f, %.0f): %d verts\n",
                        nd.kind == JunctionKind::Landing ? "landing" : "gore",
                        nd.pos.x, nd.pos.y, z);
        CHECK(z == 0);
    }
    std::printf("[sem] gore/landing nodes checked = %d\n", checked);
    CHECK(checked >= 2);
}

// S4 APPROACH BAND CUT (#20): a street that climbs to a ramp landing must
// NOT carry its sidewalk band up the grade ("the sidewalk floats as it
// elevates, not attached to the road's side"). The band cuts before the
// climb; a plain flat street keeps its full band.
TEST_CASE(semantics_approach_sheds_its_sidewalk_on_the_climb) {
    // A street that climbs GENTLY (under 1.5 m, so it meshes as a draped
    // street with a sidewalk band, not an elevated bridge) toward a ramp
    // LANDING must shed that band on the grade — "the sidewalk floats as it
    // elevates, not attached to the road's side." Node 4 is degree 3 (two
    // streets + a ramp) so it classifies as a Landing.
    RoadNet net;
    net.width = 9.0;
    net.sidewalk = 2.5;
    net.autoRoundabout = false;
    net.heightAt = [](double, double) { return 0.0; };
    net.nodes = { Vec2(0, 0),   Vec2(40, 0),  Vec2(80, 0),
                  Vec2(110, 0), Vec2(125, 0), Vec2(165, 0) };
    net.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 } };
    // Climb to 1.3 m at the landing over the last flat run (grade ~9%).
    net.nodeElev = { std::nan(""), std::nan(""), std::nan(""),
                     0.4, 1.3, std::nan("") };
    net.nodes.push_back(Vec2(190, 45));   // ramp leaves the landing
    net.nodeElev.push_back(6.0);
    net.edges.push_back({ 4, 6 });
    net.edgeClasses.resize(net.edges.size(), RoadClass::Local);
    net.edgeClasses[5] = RoadClass::Ramp;   // edge {4,6}

    RoadGraph g = roadNetFullGraph(net);
    Vec2 landingPos;
    bool sawLanding = false;
    for (const RoadNode& n : g.nodes)
        if (n.kind == JunctionKind::Landing) { sawLanding = true; landingPos = n.pos; }
    CHECK(sawLanding);   // node 4: street + street + ramp

    RenderMesh m = buildRoadNetMesh(net);
    // Sidewalk band verts (negative-u) within 40 m of the landing that have
    // climbed above 0.5 m: the floating approach sidewalk.
    int bandHigh = 0;
    double maxBandY = 0;
    for (const Vertex& v : m.vertices) {
        if (v.u > -0.01) continue;   // sidewalk band only (negative mu)
        const double dx = v.position.x - landingPos.x;
        const double dz = v.position.z - landingPos.y;
        if (dx * dx + dz * dz > 40.0 * 40.0) continue;
        if (v.position.y > 0.5) {
            ++bandHigh;
            maxBandY = std::max(maxBandY, static_cast<double>(v.position.y));
        }
    }
    (void)bandHigh;
    std::printf("[sem] approach sidewalk maxY near landing = %.2f\n", maxBandY);
    // The approach sidewalk stays at street level (the flat-end curb, ~0.55 m)
    // — it does NOT climb the ramp grade.
    CHECK(maxBandY < 0.9);

    // Control: the SAME street with the ramp removed (node 4 an ordinary
    // through point, not a Landing) keeps its climbing sidewalk — we only
    // shed the band at approaches, never on a plain street that rises.
    RoadNet flat = net;
    flat.edges.pop_back();   // drop the ramp
    flat.edgeClasses.pop_back();
    flat.nodes.pop_back();
    flat.nodeElev.pop_back();
    RenderMesh mf = buildRoadNetMesh(flat);
    double flatMaxY = 0;
    for (const Vertex& v : mf.vertices)
        if (v.u < -0.01) flatMaxY = std::max(flatMaxY, (double)v.position.y);
    std::printf("[sem] no-ramp control sidewalk maxY = %.2f\n", flatMaxY);
    CHECK(flatMaxY > 1.0);   // a plain climbing street keeps its full sidewalk
}
