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
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.look.autoRoundabout = false;
    net.graph.nodes = { RoadNode{Vec2(60, -120)},  RoadNode{Vec2(200, -120)},
                        RoadNode{Vec2(340, -120)}, RoadNode{Vec2(460, -120)},
                        RoadNode{Vec2(600, -120)}, RoadNode{Vec2(740, -120)} };
    for (int i = 0; i < 5; ++i) net.graph.addEdge(i, i + 1, net.look.defaultWidth);
    const RoadGroundFn ground = [](double, double) { return 0.0; };

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
    std::vector<int> mainline = bakeCorridorIntoNet(net, def, au.rampPaths, {}, ground);
    CHECK(!mainline.empty());

    RoadGraph g = roadNetFullGraph(net, ground);
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
    const RoadEntity so = roadNetStreetsOnly(net);
    RoadGraph gs = navRoadGraph(so, ground);
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
    const RoadGroundFn ground = [](double, double) { return 0.0; };  // the flat level terrain
    auto bakeWith = [&ground](bool addCrosser) {
        RoadEntity net;
        net.look.defaultWidth = 8.0;
        net.look.autoRoundabout = false;
        net.graph.nodes = { RoadNode{Vec2(60, -120)},  RoadNode{Vec2(200, -120)},
                            RoadNode{Vec2(340, -120)}, RoadNode{Vec2(460, -120)},
                            RoadNode{Vec2(600, -120)}, RoadNode{Vec2(740, -120)} };
        for (int i = 0; i < 5; ++i) net.graph.addEdge(i, i + 1, net.look.defaultWidth);
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
        bakeCorridorIntoNet(net, def, au.rampPaths, {}, ground);
        return net;
    };
    RoadEntity one = bakeWith(false);
    RoadEntity two = bakeWith(true);
    // Either the AUTHOR already refused the overlapping second ramp, or the
    // bake audit dropped it — both are acceptance; what must never happen is
    // both chains in the net with a crossing.
    RoadGraph gb = roadNetFullGraph(two, ground);
    CHECK(auditRoadGraph(gb, ground).empty());
    std::printf("[sem] oneExit edges=%zu twoExit edges=%zu\n",
                one.graph.edges.size(), two.graph.edges.size());
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
        RoadEntity cross;
        cross.look.defaultWidth = 10.0;
        cross.look.sidewalk = 2.5;
        cross.look.crosswalks = true;
        cross.look.autoRoundabout = false;
        cross.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(70, 0)},
                              RoadNode{Vec2(-70, 0)}, RoadNode{Vec2(0, 70)},
                              RoadNode{Vec2(0, -70)} };
        for (int i = 1; i <= 4; ++i) cross.graph.addEdge(0, i, cross.look.defaultWidth);
        RenderMesh m = buildRoadNetMesh(cross, nullptr);
        std::printf("[sem] plain 4-way zebra verts = %d\n",
                    zebraVertsNear(m, Vec2(0, 0), 24.0));
        CHECK(zebraVertsNear(m, Vec2(0, 0), 24.0) > 0);
    }

    // The corridor fixture: streets under an elevated freeway with an exit
    // (Diverge) at s=400 and an on-ramp (Merge) at s=300, landing on the
    // street. NO zebra anywhere near a gore or a landing node.
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.look.sidewalk = 2.5;
    net.look.crosswalks = true;
    net.look.autoRoundabout = false;
    net.graph.nodes = { RoadNode{Vec2(60, -120)},  RoadNode{Vec2(200, -120)},
                        RoadNode{Vec2(340, -120)}, RoadNode{Vec2(460, -120)},
                        RoadNode{Vec2(600, -120)}, RoadNode{Vec2(740, -120)} };
    for (int i = 0; i < 5; ++i) net.graph.addEdge(i, i + 1, net.look.defaultWidth);
    const RoadGroundFn ground = [](double, double) { return 0.0; };
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
    bakeCorridorIntoNet(net, def, au.rampPaths, {}, ground);

    RenderMesh m = buildRoadNetMesh(net, ground);
    RoadGraph g = roadNetFullGraph(net, ground);
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
    RoadEntity net;
    net.look.defaultWidth = 9.0;
    net.look.sidewalk = 2.5;
    net.look.autoRoundabout = false;
    const RoadGroundFn ground = [](double, double) { return 0.0; };
    net.graph.nodes = { RoadNode{Vec2(0, 0)},   RoadNode{Vec2(40, 0)},
                        RoadNode{Vec2(80, 0)},  RoadNode{Vec2(110, 0)},
                        RoadNode{Vec2(125, 0)}, RoadNode{Vec2(165, 0)} };
    for (int i = 0; i < 5; ++i) net.graph.addEdge(i, i + 1, net.look.defaultWidth);
    // Climb to 1.3 m at the landing over the last flat run (grade ~9%).
    net.graph.nodes[3].elev = 0.4; net.graph.nodes[3].elevAbsolute = true;
    net.graph.nodes[4].elev = 1.3; net.graph.nodes[4].elevAbsolute = true;
    RoadNode rampEnd{Vec2(190, 45)};      // ramp leaves the landing
    rampEnd.elev = 6.0;
    rampEnd.elevAbsolute = true;
    net.graph.nodes.push_back(rampEnd);
    net.graph.addEdge(4, 6, net.look.defaultWidth, RoadClass::Ramp);   // edge {4,6}

    RoadGraph g = roadNetFullGraph(net, ground);
    Vec2 landingPos;
    bool sawLanding = false;
    for (const RoadNode& n : g.nodes)
        if (n.kind == JunctionKind::Landing) { sawLanding = true; landingPos = n.pos; }
    CHECK(sawLanding);   // node 4: street + street + ramp

    RenderMesh m = buildRoadNetMesh(net, ground);
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
    RoadEntity flat = net;
    flat.graph.edges.pop_back();   // drop the ramp (class travels with the edge)
    flat.graph.nodes.pop_back();   // and its far node (elev travels with the node)
    RenderMesh mf = buildRoadNetMesh(flat, ground);
    double flatMaxY = 0;
    for (const Vertex& v : mf.vertices)
        if (v.u < -0.01) flatMaxY = std::max(flatMaxY, (double)v.position.y);
    std::printf("[sem] no-ramp control sidewalk maxY = %.2f\n", flatMaxY);
    CHECK(flatMaxY > 1.0);   // a plain climbing street keeps its full sidewalk
}

// Review S4b regressions (adversarial review of S0-S4).

// [0]: a ramp landing on a MID-street node (degree 3: two collinear street
// edges + the ramp) must DEGRADE to a through node once roadNetStreetsOnly
// strips the ramp — a stale Landing hint on a plain street point would drive
// phantom sidewalk trimming and cluster mis-promotion.
TEST_CASE(semantics_mid_street_landing_degrades_without_its_ramp) {
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.look.autoRoundabout = false;
    const RoadGroundFn ground = [](double, double) { return 0.0; };
    // A straight street; node 2 is interior (collinear neighbours).
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(60, 0)},
                        RoadNode{Vec2(120, 0)}, RoadNode{Vec2(180, 0)} };
    for (int i = 0; i < 3; ++i) net.graph.addEdge(i, i + 1, net.look.defaultWidth);
    net.graph.nodes.push_back(RoadNode{Vec2(140, 60)});   // a BAKED ramp leaves node 2
    net.graph.addEdge(2, 4, net.look.defaultWidth, RoadClass::Ramp);
    net.graph.edges.back().baked = true;   // baked -> roadNetStreetsOnly strips it
    net.graph.nodes[2].kind = JunctionKind::Landing;   // the bake's hint

    // Full graph: node 2 is a real Landing (ramp present).
    RoadGraph full = navRoadGraph(net, ground);
    int fullLandings = 0;
    for (const RoadNode& n : full.nodes)
        if (n.kind == JunctionKind::Landing) ++fullLandings;
    CHECK(fullLandings >= 1);

    // Streets-only: the ramp is gone, so node 2 is a plain through node — NOT
    // a Landing (the stale hint must not survive).
    RoadGraph so = navRoadGraph(roadNetStreetsOnly(net), ground);
    for (const RoadNode& n : so.nodes)
        CHECK(n.kind != JunctionKind::Landing);
}

// [2]: applyGenerateRecipe must not leave stale node-kind hints — a leftover
// baked hint would otherwise land on an arbitrary node of the fresh grid.
TEST_CASE(semantics_regenerate_clears_stale_node_hints) {
    RoadEntity net;
    // Simulate a net that carried a baked corridor: leftover hints on its nodes.
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(50, 0)},
                        RoadNode{Vec2(100, 0)} };
    net.graph.nodes[0].kind = JunctionKind::Merge;
    net.graph.nodes[1].kind = JunctionKind::Landing;
    net.graph.nodes[2].kind = JunctionKind::Diverge;
    nlohmann::json gen = { { "kind", "metro" }, { "radius", 300 },
                           { "hotspots", 3 },   { "block_size", 120 },
                           { "seed", 4 },       { "freeways", false } };
    applyGenerateRecipe(net, gen, nullptr);
    // The regenerated graph replaced the hinted nodes wholesale: no stale
    // gore/landing hint may survive on any node of the fresh grid.
    for (const RoadNode& n : net.graph.nodes) {
        CHECK(!isGore(n.kind));
        CHECK(n.kind != JunctionKind::Landing);
    }

    // And the derived graph carries no phantom gores/landings.
    RoadGraph g = navRoadGraph(net, nullptr);
    for (const RoadNode& n : g.nodes) {
        CHECK(!isGore(n.kind));
        CHECK(n.kind != JunctionKind::Landing);
    }
}

// [4]: roadNetDeleteNode must keep per-edge fields (spec/baked/layer/class)
// with their edge — a delete shifts edge indices, and a desync bakes the
// wrong cross-section onto surviving edges. The fields now LIVE on RoadEdge,
// so the invariant is that each surviving edge keeps its own values.
TEST_CASE(semantics_delete_node_keeps_edge_arrays_parallel) {
    RoadEntity net;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(50, 0)},
                        RoadNode{Vec2(100, 0)}, RoadNode{Vec2(150, 0)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth, RoadClass::Local);
    net.graph.addEdge(1, 2, net.look.defaultWidth, RoadClass::Arterial);
    net.graph.addEdge(2, 3, net.look.defaultWidth, RoadClass::Local);
    net.graph.edges[0].spec = 10;
    net.graph.edges[1].spec = 20; net.graph.edges[1].baked = true; net.graph.edges[1].layer = 2;
    net.graph.edges[2].spec = 30;
    // Delete node 0 -> drops edge {0,1}; edges {1,2},{2,3} survive and shift.
    CHECK(roadNetDeleteNode(net, 0));
    CHECK(net.graph.edges.size() == 2);
    // The surviving arterial edge (was index 1) kept its spec 20 / baked /
    // layer 2 — not a shifted-in neighbour's value.
    CHECK(net.graph.edges[0].klass == RoadClass::Arterial);
    CHECK(net.graph.edges[0].spec == 20);
    CHECK(net.graph.edges[0].baked == true);
    CHECK(net.graph.edges[0].layer == 2);
    // And its neighbour (was index 2) kept its own values too.
    CHECK(net.graph.edges[1].klass == RoadClass::Local);
    CHECK(net.graph.edges[1].spec == 30);
    CHECK(!net.graph.edges[1].baked);
    CHECK(net.graph.edges[1].layer == 0);
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

// [6] Review S4b: the sidewalk band must GAP across a street's mouth at a
// ramp landing — otherwise the street's closed-capsule ribbon caps with a
// curb wall straight across the carriageway, sealing the ramp entrance. A
// curb cap is a sidewalk-band vertex (negative-u) sitting INSIDE the
// carriageway; near a landing there must be none.
// [5] Review S4b: the zebra gate reads PER-END access. A street running
// Intersection A <-> Landing B, welded from the LANDING end (B has the lower
// node index), must still paint its crosswalk at A — the old code used the
// chain's first-edge access (the approach edge, no kCrossable) for BOTH ends
// and dropped the Intersection-end zebra.
TEST_CASE(semantics_zebra_survives_at_the_intersection_end_of_an_approach) {
    // Node 0 = a 4-way street Intersection (lowest index, so weldChainSpines
    // starts the chain here — but we also need the LANDING to have a low
    // index to exercise the back-end path). Build so the chain 5<->0 welds
    // from node 0's side and the far end is the intersection... simplest:
    // one street chain from a Landing (node 1) to an Intersection (node 0),
    // climbing toward the landing.
    RoadEntity net;
    net.look.defaultWidth = 10.0;
    net.look.sidewalk = 2.5;
    net.look.crosswalks = true;
    net.look.autoRoundabout = false;
    const RoadGroundFn ground = [](double, double) { return 0.0; };
    // Node 0: 4-way intersection. Node 4: ramp landing reached by a climbing
    // street from node 0's east arm.
    net.graph.nodes = { RoadNode{Vec2(0, 0)},   RoadNode{Vec2(0, 70)},
                        RoadNode{Vec2(0, -70)}, RoadNode{Vec2(-70, 0)},
                        RoadNode{Vec2(60, 0)},  RoadNode{Vec2(120, 0)} };
    for (int i = 1; i <= 4; ++i) net.graph.addEdge(0, i, net.look.defaultWidth);
    net.graph.addEdge(4, 5, net.look.defaultWidth);
    // Climb east to the landing at node 4.
    net.graph.nodes[4].elev = 0.6; net.graph.nodes[4].elevAbsolute = true;
    net.graph.nodes[5].elev = 1.4; net.graph.nodes[5].elevAbsolute = true;
    RoadNode rampEnd{Vec2(150, 45)};      // ramp leaves node 4
    rampEnd.elev = 6.0;
    rampEnd.elevAbsolute = true;
    net.graph.nodes.push_back(rampEnd);
    net.graph.addEdge(4, 6, net.look.defaultWidth, RoadClass::Ramp);

    RoadGraph g = roadNetFullGraph(net, ground);
    CHECK(g.nodes[0].kind == JunctionKind::Intersection);
    bool landing = false;
    for (const RoadNode& n : g.nodes)
        if (n.kind == JunctionKind::Landing) landing = true;
    CHECK(landing);

    RenderMesh m = buildRoadNetMesh(net, ground);
    // Zebra verts on each of node 0's four approaches — the intersection must
    // still paint them despite one arm being an approach to a landing.
    int zebra = 0;
    for (const Vertex& v : m.vertices) {
        const double dx = v.position.x, dz = v.position.z;
        if (dx * dx + dz * dz > 24.0 * 24.0) continue;
        if (v.u > 1.05 && v.u < 3.0 && v.v > 0.4 && v.v < 3.7) ++zebra;
    }
    std::printf("[sem] intersection zebra verts (with an approach arm) = %d\n",
                zebra);
    CHECK(zebra > 0);   // per-end gate keeps the intersection's crosswalks
}
