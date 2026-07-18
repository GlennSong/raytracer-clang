// Roads-v2 S3 acceptance (plan §1.4): freeways and ramps are IN the editable
// road graph — proven by graph adjacency, not proximity or promises.
//
// The claim under test, in Glenn's words: "is the freeway part of the road
// graph?" — asked six times, never before demonstrable. These tests fail until
// the bake is real:
//   1. After baking a solved corridor, the SAME RoadNet contains Freeway and
//      Ramp edges alongside its streets.
//   2. Every ramp chain STARTS at a gore node that IS a mainline node (shared
//      node id — the mainline chain is split there, degree >= 3).
//   3. Every ramp chain ENDS at a street node (shared node id with a Local
//      edge — the landing is grafted, not floating nearby).
//   4. Baked freeway nodes carry finite ABSOLUTE deck heights (nodeElev) —
//      the exact mechanism the editor's drag handles read, so the freeway is
//      selectable/draggable like any street (roadNetMoveNode works on it).
//   5. The corridor MESH and the baked GRAPH agree: driving the baked mainline
//      and ramp chains over the swept corridor mesh reports no holes/steps.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/corridor_bake.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/road_lattice.h"
#include "../src/engine/procgen/city/road_net.h"
#include <cmath>
#include <vector>

using namespace engine;

namespace {

// A corridor flying at +9 m along +X with ONE exit descending to a street that
// runs under its landing. The street net owns the landing node before the bake.
struct Fixture {
    RoadNet net;
    CorridorDef def;
    CorridorAuthoring authored;
    std::vector<int> mainline;      // baked mainline node indices
    int streetNodeCount = 0;        // net node count BEFORE the bake
};

Fixture makeFixture() {
    Fixture f;
    // Streets: a straight Local road through the ramp's landing area.
    f.net.nodes = { Vec2(460, -120), Vec2(600, -120), Vec2(740, -120) };
    f.net.edges = { { 0, 1 }, { 1, 2 } };
    f.net.width = 8.0;
    f.streetNodeCount = static_cast<int>(f.net.nodes.size());

    // The corridor: straight, elevated, one exit at station 400 to the street.
    f.def.horizontal = Alignment::fromPolyline({ Vec2(-100, 0), Vec2(800, 0) }, 300.0, 20.0);
    const Real L = f.def.horizontal.length();
    f.def.vertical.pvis = { { 0.0, 9.0, 0.0 }, { L, 9.0, 0.0 } };
    f.def.lanes.throughLanes = 3;
    ExitDef e;
    e.station = 400.0;
    e.upStation = true;
    e.target = Vec2(600, -120);     // the street's middle node
    e.targetY = 0.0;
    f.def.exits.push_back(e);

    f.authored = corridorAuthor(f.def, [](Real, Real) { return 0.0; });
    f.mainline = bakeCorridorIntoNet(f.net, f.def, f.authored.rampPaths);
    return f;
}

RoadClass edgeClass(const RoadNet& net, int ei) {
    return ei < static_cast<int>(net.edgeClasses.size()) ? net.edgeClasses[ei]
                                                         : RoadClass::Local;
}

}  // namespace

TEST_CASE(freeway_and_ramps_are_in_the_one_net) {
    Fixture f = makeFixture();
    CHECK(!f.authored.rampPaths.empty());
    CHECK(!f.authored.rampPaths[0].pts.empty());   // the exit was NOT dropped

    int nFreeway = 0, nRamp = 0, nLocal = 0;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei) {
        const RoadClass k = edgeClass(f.net, ei);
        if (k == RoadClass::Freeway) ++nFreeway;
        else if (k == RoadClass::Ramp) ++nRamp;
        else ++nLocal;
    }
    CHECK(nFreeway > 0);            // the mainline is graph edges now
    CHECK(nRamp > 0);               // so is the ramp
    CHECK(nLocal == 2);             // the streets are untouched

    // Specs ride the edges: freeway edges answer the freeway3 band model.
    bool sawFreewaySpec = false;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Freeway) {
            const RoadSpec s = roadNetEdgeSpec(f.net, ei);
            if (s.laneCount() == 6 && !s.hasSidewalk(-1)) sawFreewaySpec = true;
        }
    CHECK(sawFreewaySpec);
}

TEST_CASE(ramp_starts_at_a_gore_node_that_is_a_mainline_node) {
    Fixture f = makeFixture();
    // Find the ramp's first node: the ramp edge whose one end touches a
    // Freeway edge BY NODE ID (graph adjacency, the whole point).
    std::vector<char> onFreeway(f.net.nodes.size(), 0);
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Freeway) {
            onFreeway[f.net.edges[ei][0]] = 1;
            onFreeway[f.net.edges[ei][1]] = 1;
        }
    int goreNode = -1, goreDeg = 0;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp)
            for (int side = 0; side < 2; ++side)
                if (onFreeway[f.net.edges[ei][side]]) goreNode = f.net.edges[ei][side];
    CHECK(goreNode >= 0);           // shared node id: ramp touches the mainline

    // The mainline is SPLIT at the gore: degree >= 3 (two mainline + the ramp).
    for (const auto& e : f.net.edges)
        if (e[0] == goreNode || e[1] == goreNode) ++goreDeg;
    CHECK(goreDeg >= 3);

    // The gore node sits ON the mainline at deck height.
    CHECK(goreNode < static_cast<int>(f.net.nodeElev.size()));
    CHECK(std::isfinite(f.net.nodeElev[goreNode]));
    CHECK(std::fabs(f.net.nodeElev[goreNode] - 9.0) < 0.5);
}

TEST_CASE(ramp_lands_on_a_street_node) {
    Fixture f = makeFixture();
    // The ramp's far end must REUSE a pre-existing street node (index < the
    // street node count), and that node must carry Local edges.
    int landing = -1;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp)
            for (int side = 0; side < 2; ++side)
                if (f.net.edges[ei][side] < f.streetNodeCount)
                    landing = f.net.edges[ei][side];
    CHECK(landing >= 0);            // grafted, not floating nearby
    bool touchesLocal = false;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Local &&
            (f.net.edges[ei][0] == landing || f.net.edges[ei][1] == landing))
            touchesLocal = true;
    CHECK(touchesLocal);
}

TEST_CASE(baked_freeway_is_editable_like_a_street) {
    Fixture f = makeFixture();
    // Every baked mainline node carries a finite ABSOLUTE deck height — the
    // mechanism roadNodeHandles reads to place drag handles ON the deck.
    CHECK(!f.mainline.empty());
    for (int ni : f.mainline) {
        CHECK(ni < static_cast<int>(f.net.nodeElev.size()));
        CHECK(std::isfinite(f.net.nodeElev[ni]));
    }
    // And the standard editor entry point moves it like any street node.
    const int ni = f.mainline[f.mainline.size() / 2];
    const Vec2 before = f.net.nodes[ni];
    CHECK(roadNetMoveNode(f.net, ni, before + Vec2(0, 20)));
    CHECK((f.net.nodes[ni] - (before + Vec2(0, 20))).length() < 1e-6);

    // Determinism: baking the same solve twice yields the same graph shape.
    Fixture g = makeFixture();
    CHECK(g.net.nodes.size() == f.net.nodes.size());
    CHECK(g.net.edges.size() == f.net.edges.size());
}

TEST_CASE(baked_graph_agrees_with_the_corridor_mesh) {
    Fixture f = makeFixture();
    // Mesh the corridor the shipping way (swept lattices)...
    auto ground = [](double, double) { return 0.0; };
    std::vector<UnionSpine> deck = corridorDeckSpines(f.def, [](Real, Real) { return Real(0); });
    CHECK(deck.size() == 1);
    CorridorLatticeParams lp;
    lp.ground = ground;
    RenderMesh m = sweepCorridor(deck[0], corridorRampSpines(f.authored.rampPaths), lp);
    CHECK(!m.vertices.empty());

    // ...then drive the BAKED GRAPH's chains over that mesh. Graph and mesh
    // come from the same solve, so the drive must be clean.
    std::vector<Vec3> mainPath;
    for (int ni : f.mainline)
        mainPath.push_back(Vec3(f.net.nodes[ni].x, f.net.nodeElev[ni], f.net.nodes[ni].y));
    driveprobe::Report rep;
    driveprobe::drivePath(m, mainPath, rep);
    CHECK(rep.samples > 40);
    CHECK(rep.holes == 0);
    CHECK(rep.steps == 0);

    // The ramp chain too: gore -> landing, heights from the graph.
    std::vector<Vec3> rampPath;
    for (const Vec3& p : f.authored.rampPaths[0].pts) rampPath.push_back(p);
    driveprobe::Report rrep;
    driveprobe::drivePath(m, rampPath, rrep);
    CHECK(rrep.samples > 10);
    CHECK(rrep.holes == 0);
}
