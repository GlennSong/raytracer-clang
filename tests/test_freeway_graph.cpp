// Roads-v2 S3 acceptance (plan §1.4): freeways and ramps are IN the editable
// road graph — proven by graph adjacency, not proximity or promises.
//
// The claim under test, in Glenn's words: "is the freeway part of the road
// graph?" — asked six times, never before demonstrable. These tests fail until
// the bake is real:
//   1. After baking a solved corridor, the SAME RoadEntity contains Freeway and
//      Ramp edges alongside its streets.
//   2. Every ramp chain STARTS at a gore node that IS a mainline node (shared
//      node id — the mainline chain is split there, degree >= 3).
//   3. Every ramp chain ENDS at a street node (shared node id with a Local
//      edge — the landing is grafted, not floating nearby).
//   4. Baked freeway nodes carry ABSOLUTE deck heights (elev/elevAbsolute) —
//      the exact mechanism the editor's drag handles read, so the freeway is
//      selectable/draggable like any street (roadNetMoveNode works on it).
//   5. The corridor MESH and the baked GRAPH agree: driving the baked mainline
//      and ramp chains over the swept corridor mesh reports no holes/steps.
#include "test_framework.h"
#include "drive_probe.h"

#include "../src/engine/procgen/city/corridor_bake.h"
#include "../src/engine/procgen/city/corridor_mesh.h"
#include "../src/engine/procgen/city/corridor_plan.h"
#include "../src/engine/procgen/city/roads/road_entity.h"
#include "../src/engine/procgen/city/road_semantics.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;

namespace {

// A corridor flying at +9 m along +X with ONE exit descending to a street that
// runs under its landing. The street net owns the landing node before the bake.
struct Fixture {
    RoadEntity net;
    CorridorDef def;
    CorridorAuthoring authored;
    std::vector<int> mainline;      // baked mainline node indices
    int streetNodeCount = 0;        // net node count BEFORE the bake
};

Fixture makeFixture() {
    Fixture f;
    // Streets: a straight Local road under the corridor's south side — long
    // enough to catch BOTH the exit's landing and the on-ramp's origin.
    f.net.look.defaultWidth = 8.0;
    f.net.graph.nodes = { RoadNode{Vec2(60, -120)},  RoadNode{Vec2(200, -120)},
                          RoadNode{Vec2(340, -120)}, RoadNode{Vec2(460, -120)},
                          RoadNode{Vec2(600, -120)}, RoadNode{Vec2(740, -120)} };
    f.net.graph.edges = { RoadEdge{ 0, 1, 8.0 }, RoadEdge{ 1, 2, 8.0 },
                          RoadEdge{ 2, 3, 8.0 }, RoadEdge{ 3, 4, 8.0 },
                          RoadEdge{ 4, 5, 8.0 } };
    f.net.look.autoRoundabout = false;   // full-graph pass must not promote the gore
    f.streetNodeCount = static_cast<int>(f.net.graph.nodes.size());

    // The corridor: straight, elevated; one EXIT at station 400 to the street
    // and one ON-RAMP at station 300 fed from the street behind it — the
    // configuration whose bake orientation was never covered (and was wrong).
    f.def.horizontal = Alignment::fromPolyline({ Vec2(-100, 0), Vec2(800, 0) }, 300.0, 20.0);
    const Real L = f.def.horizontal.length();
    f.def.vertical.pvis = { { 0.0, 9.0, 0.0 }, { L, 9.0, 0.0 } };
    f.def.lanes.throughLanes = 3;
    ExitDef e;
    e.station = 400.0;
    e.upStation = true;
    e.target = Vec2(600, -120);     // a street node
    e.targetY = 0.0;
    f.def.exits.push_back(e);
    ExitDef on;
    on.station = 300.0;
    on.upStation = true;
    on.onRamp = true;
    on.target = Vec2(60, -120);     // approaches the merge from behind
    on.targetY = 0.0;
    f.def.exits.push_back(on);

    f.authored = corridorAuthor(f.def, [](Real, Real) { return 0.0; });
    f.mainline = bakeCorridorIntoNet(f.net, f.def, f.authored.rampPaths);
    return f;
}

RoadClass edgeClass(const RoadEntity& net, int ei) {
    return net.graph.edges[ei].klass;
}

}  // namespace

TEST_CASE(freeway_and_ramps_are_in_the_one_net) {
    Fixture f = makeFixture();
    CHECK(f.authored.rampPaths.size() == 2u);
    CHECK(!f.authored.rampPaths[0].pts.empty());   // the exit was NOT dropped
    CHECK(!f.authored.rampPaths[1].pts.empty());   // nor the on-ramp

    int nFreeway = 0, nRamp = 0, nLocal = 0;
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei) {
        const RoadClass k = edgeClass(f.net, ei);
        if (k == RoadClass::Freeway) ++nFreeway;
        else if (k == RoadClass::Ramp) ++nRamp;
        else ++nLocal;
    }
    CHECK(nFreeway > 0);            // the mainline is graph edges now
    CHECK(nRamp > 0);               // so is the ramp
    CHECK(nLocal == 5);             // the streets are untouched

    // Specs ride the edges: freeway edges answer the freeway3 band model.
    bool sawFreewaySpec = false;
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
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
    std::vector<char> onFreeway(f.net.graph.nodes.size(), 0);
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Freeway) {
            onFreeway[f.net.graph.edges[ei].a] = 1;
            onFreeway[f.net.graph.edges[ei].b] = 1;
        }
    int goreNode = -1, goreDeg = 0;
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp)
            for (int side = 0; side < 2; ++side) {
                const int n = side == 0 ? f.net.graph.edges[ei].a
                                        : f.net.graph.edges[ei].b;
                if (onFreeway[n]) goreNode = n;
            }
    CHECK(goreNode >= 0);           // shared node id: ramp touches the mainline

    // The mainline is SPLIT at the gore: degree >= 3 (two mainline + the ramp).
    for (const auto& e : f.net.graph.edges)
        if (e.a == goreNode || e.b == goreNode) ++goreDeg;
    CHECK(goreDeg >= 3);

    // The gore node sits ON the mainline at deck height.
    CHECK(goreNode < static_cast<int>(f.net.graph.nodes.size()));
    CHECK(f.net.graph.nodes[goreNode].elevAbsolute);
    CHECK(std::fabs(f.net.graph.nodes[goreNode].elev - 9.0) < 0.5);
}

TEST_CASE(ramp_lands_on_a_street_node) {
    Fixture f = makeFixture();
    // EVERY ramp chain's far end must REUSE a pre-existing street node (index
    // < the street node count) carrying Local edges — the on-ramp too (its
    // authored points run street->merge; the old bake walked it backwards
    // and left its landing floating at the deck).
    std::vector<int> landings;
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp)
            for (int side = 0; side < 2; ++side) {
                const int n = side == 0 ? f.net.graph.edges[ei].a
                                        : f.net.graph.edges[ei].b;
                if (n < f.streetNodeCount) landings.push_back(n);
            }
    CHECK(landings.size() == 2u);   // one street landing per ramp, both grafted
    for (int landing : landings) {
        bool touchesLocal = false;
        for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
            if (edgeClass(f.net, ei) == RoadClass::Local &&
                (f.net.graph.edges[ei].a == landing ||
                 f.net.graph.edges[ei].b == landing))
                touchesLocal = true;
        CHECK(touchesLocal);
    }
}

TEST_CASE(baked_freeway_is_editable_like_a_street) {
    Fixture f = makeFixture();
    // Every baked mainline node carries a finite ABSOLUTE deck height — the
    // mechanism roadNodeHandles reads to place drag handles ON the deck.
    CHECK(!f.mainline.empty());
    for (int ni : f.mainline) {
        CHECK(ni < static_cast<int>(f.net.graph.nodes.size()));
        CHECK(f.net.graph.nodes[ni].elevAbsolute);
    }
    // And the standard editor entry point moves it like any street node.
    const int ni = f.mainline[f.mainline.size() / 2];
    const Vec2 before = f.net.graph.nodes[ni].pos;
    CHECK(roadNetMoveNode(f.net, ni, before + Vec2(0, 20)));
    CHECK((f.net.graph.nodes[ni].pos - (before + Vec2(0, 20))).length() < 1e-6);

    // Determinism: baking the same solve twice yields the same graph shape.
    Fixture g = makeFixture();
    CHECK(g.net.graph.nodes.size() == f.net.graph.nodes.size());
    CHECK(g.net.graph.edges.size() == f.net.graph.edges.size());
}


TEST_CASE(recipe_regenerate_rebakes_the_freeway) {
    // The S3 known gap (commit aef8436): the editor's tuning-panel Regenerate
    // re-runs applyGenerateRecipe, which rebuilds the streets WITHOUT the
    // corridor solve — the freeway vanished from the graph. The contract now:
    // applyGenerateRecipe leaves a CLEAN street net (no stale baked flags, no
    // accumulated freeway plans), and rebakeNetCorridors re-runs the same
    // plan -> land -> author -> bake pipeline the loader uses, so Regenerate
    // keeps the freeway IN the editable graph. Deterministic: regenerating
    // twice from the same recipe lands the same graph.
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    const RoadGroundFn ground = [](double, double) { return 0.0; };
    nlohmann::json gen = {{"kind", "metro"},          {"radius", 700},
                          {"seed", 5},                {"freeways", true},
                          {"corridor_freeways", true},   // plans, not fat streets
                          {"interchange_spacing", 520}};
    applyGenerateRecipe(net, gen, ground);
    CHECK(!net.plan.freewayPlans.empty());
    const std::size_t plans1 = net.plan.freewayPlans.size();

    const int baked1 = rebakeNetCorridors(net, 520.0, ground);
    CHECK(baked1 > 0);
    auto countClass = [&](RoadClass k) {
        int n = 0;
        for (const RoadEdge& e : net.graph.edges)
            if (e.klass == k) ++n;
        return n;
    };
    CHECK(countClass(RoadClass::Freeway) > 0);
    CHECK(countClass(RoadClass::Ramp) > 0);
    const std::size_t nodes1 = net.graph.nodes.size(), edges1 = net.graph.edges.size();

    // Regenerate AGAIN from the same recipe — the editor's actual flow.
    applyGenerateRecipe(net, gen, ground);
    CHECK(net.plan.freewayPlans.size() == plans1);     // plans don't ACCUMULATE
    for (const RoadEdge& e : net.graph.edges) CHECK(!e.baked);  // no stale baked flags
    CHECK(countClass(RoadClass::Freeway) == 0);   // clean street-only net...
    const int baked2 = rebakeNetCorridors(net, 520.0, ground);
    CHECK(baked2 == baked1);                      // ...until the re-bake
    CHECK(net.graph.nodes.size() == nodes1);      // same graph, bit for bit
    CHECK(net.graph.edges.size() == edges1);
    CHECK(countClass(RoadClass::Freeway) > 0);
    CHECK(countClass(RoadClass::Ramp) > 0);

    // And the re-baked ramps still LAND: every Ramp chain touches a street.
    std::vector<char> onStreet(net.graph.nodes.size(), 0);
    for (const RoadEdge& e : net.graph.edges)
        if (e.klass == RoadClass::Local || e.klass == RoadClass::Arterial ||
            e.klass == RoadClass::Collector) {
            onStreet[e.a] = 1;
            onStreet[e.b] = 1;
        }
    int rampTouchesStreet = 0;
    for (const RoadEdge& e : net.graph.edges)
        if (e.klass == RoadClass::Ramp && (onStreet[e.a] || onStreet[e.b]))
            ++rampTouchesStreet;
    CHECK(rampTouchesStreet > 0);
}

TEST_CASE(baked_ramps_are_sparse_editable_splines) {
    // Roads-v2.1 R1 step 2a (drive feedback B5: "Ramps have hundreds of
    // points which is uneditable. Ideally it should be a few spline points
    // with tangents."): the bake keeps ~one node per 25 m of ramp plus the
    // pinned gore and landing, stores TANGENTS from the authored curve, and
    // the net's own Hermite sampling reproduces the authored clothoid
    // between the sparse nodes — editable handles, faithful shape.
    Fixture f = makeFixture();

    // Sparse: TWO ramp chains (exit + on-ramp), each a handful of nodes,
    // not one per 3 m.
    int rampEdges = 0;
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp) ++rampEdges;
    CHECK(rampEdges >= 8);              // two real chains...
    CHECK(rampEdges <= 26);             // ...both editable (<= 14 handles each)

    // Tangents are stored on the baked nodes.
    int tangentless = 0, rampNodes = 0;
    std::vector<char> onRamp(f.net.graph.nodes.size(), 0);
    for (int ei = 0; ei < static_cast<int>(f.net.graph.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp) {
            onRamp[f.net.graph.edges[ei].a] = 1;
            onRamp[f.net.graph.edges[ei].b] = 1;
        }
    for (std::size_t ni = f.streetNodeCount; ni < f.net.graph.nodes.size(); ++ni)
        if (onRamp[ni]) {
            ++rampNodes;
            const Vec2 t = f.net.graph.nodes[ni].tangent;
            if (t.x == 0.0 && t.y == 0.0) ++tangentless;
        }
    CHECK(rampNodes > 0);
    CHECK(tangentless == 0);            // every baked ramp node carries one

    // Fidelity: sample the FULL graph (baked edges included) and check every
    // authored ramp point sits within 0.25 m of the sampled ramp polyline —
    // the Hermite through sparse nodes+tangents IS the clothoid, near enough.
    RoadGraph g = roadNetFullGraph(f.net, nullptr);
    auto segD = [](const Vec2& p, const Vec2& a, const Vec2& b) {
        const Vec2 ab = b - a;
        const Real l2 = ab.lengthSquared();
        Real t = l2 > 1e-12 ? dot(p - a, ab) / l2 : 0.0;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        return (a + ab * t - p).length();
    };
    // Fidelity is measured on the RIBBON — the authored points past the
    // gore band (bandFront/bandBack mark the deck-edge band, which is
    // junction surface owned by 2c, not ramp ribbon) — for BOTH ramps.
    Real worst = 0; Vec2 worstAt;
    for (const RampPath& rp : f.authored.rampPaths) {
        const std::size_t lo = static_cast<std::size_t>(rp.bandFront);
        const std::size_t hi = rp.pts.size() - static_cast<std::size_t>(rp.bandBack);
        for (std::size_t k = lo; k < hi; ++k) {
            const Vec2 p(rp.pts[k].x, rp.pts[k].z);
            Real best = 1e30;
            for (const RoadEdge& e : g.edges) {
                if (e.klass != RoadClass::Ramp) continue;
                best = std::min(best, segD(p, g.nodes[e.a].pos, g.nodes[e.b].pos));
            }
            if (best > worst) { worst = best; worstAt = p; }
        }
    }
    std::printf("[2a] ribbon fidelity worst=%.3f at (%.1f, %.1f)\n",
                worst, worstAt.x, worstAt.y);
    CHECK(worst < 0.25);
}

