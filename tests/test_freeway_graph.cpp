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
#include "../src/engine/procgen/city/corridor_plan.h"
#include "../src/engine/procgen/city/road_lattice.h"
#include "../src/engine/procgen/city/road_net.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <cstdio>
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
    // Streets: a straight Local road under the corridor's south side — long
    // enough to catch BOTH the exit's landing and the on-ramp's origin.
    f.net.nodes = { Vec2(60, -120),  Vec2(200, -120), Vec2(340, -120),
                    Vec2(460, -120), Vec2(600, -120), Vec2(740, -120) };
    f.net.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 } };
    f.net.width = 8.0;
    f.net.autoRoundabout = false;   // full-graph pass must not promote the gore
    f.streetNodeCount = static_cast<int>(f.net.nodes.size());

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

RoadClass edgeClass(const RoadNet& net, int ei) {
    return ei < static_cast<int>(net.edgeClasses.size()) ? net.edgeClasses[ei]
                                                         : RoadClass::Local;
}

}  // namespace

TEST_CASE(freeway_and_ramps_are_in_the_one_net) {
    Fixture f = makeFixture();
    CHECK(f.authored.rampPaths.size() == 2u);
    CHECK(!f.authored.rampPaths[0].pts.empty());   // the exit was NOT dropped
    CHECK(!f.authored.rampPaths[1].pts.empty());   // nor the on-ramp

    int nFreeway = 0, nRamp = 0, nLocal = 0;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei) {
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
    // EVERY ramp chain's far end must REUSE a pre-existing street node (index
    // < the street node count) carrying Local edges — the on-ramp too (its
    // authored points run street->merge; the old bake walked it backwards
    // and left its landing floating at the deck).
    std::vector<int> landings;
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp)
            for (int side = 0; side < 2; ++side)
                if (f.net.edges[ei][side] < f.streetNodeCount)
                    landings.push_back(f.net.edges[ei][side]);
    CHECK(landings.size() == 2u);   // one street landing per ramp, both grafted
    for (int landing : landings) {
        bool touchesLocal = false;
        for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
            if (edgeClass(f.net, ei) == RoadClass::Local &&
                (f.net.edges[ei][0] == landing || f.net.edges[ei][1] == landing))
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

TEST_CASE(baked_net_meshes_freeway_and_streets_once) {
    // 2e (inverts the S3b contract): the ONE mesher now BUILDS the baked
    // freeway — the mesh contains streets AND corridor, nav routes the
    // freeway natively from the graph. Only the terrain conform still
    // strips baked edges: corridorAuthor's engineered flatten owns that
    // carve, so nothing double-grades the interchange ground.
    Fixture f = makeFixture();
    RoadNet streetsOnly;                       // rebuild the pre-bake net
    streetsOnly.nodes = { Vec2(60, -120),  Vec2(200, -120), Vec2(340, -120),
                          Vec2(460, -120), Vec2(600, -120), Vec2(740, -120) };
    streetsOnly.edges = { { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 } };
    streetsOnly.width = 8.0;

    const RenderMesh a = buildRoadNetMesh(f.net);
    const RenderMesh b = buildRoadNetMesh(streetsOnly);
    CHECK(a.indices.size() > b.indices.size());   // the freeway IS in the mesh

    const RoadGraph na = navRoadGraph(f.net);
    const RoadGraph nb = navRoadGraph(streetsOnly);
    int fwyEdges = 0;
    for (const RoadEdge& e : na.edges)
        if (e.klass == RoadClass::Freeway || e.klass == RoadClass::Ramp)
            ++fwyEdges;
    CHECK(fwyEdges > 0);                          // nav routes it natively
    CHECK(na.edges.size() > nb.edges.size());

    const std::size_t ca = roadNetConformRegions(f.net).size();
    const std::size_t cb = roadNetConformRegions(streetsOnly).size();
    CHECK(ca == cb);                              // still exactly one carve
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
    RoadNet net;
    net.width = 8.0;
    net.heightAt = [](double, double) { return 0.0; };
    nlohmann::json gen = {{"kind", "metro"},          {"radius", 700},
                          {"seed", 5},                {"freeways", true},
                          {"corridor_freeways", true},   // plans, not fat streets
                          {"interchange_spacing", 520}};
    applyGenerateRecipe(net, gen);
    CHECK(!net.freewayPlans.empty());
    const std::size_t plans1 = net.freewayPlans.size();

    const int baked1 = rebakeNetCorridors(net, 520.0);
    CHECK(baked1 > 0);
    auto countClass = [&](RoadClass k) {
        int n = 0;
        for (std::size_t ei = 0; ei < net.edges.size(); ++ei)
            if (ei < net.edgeClasses.size() && net.edgeClasses[ei] == k) ++n;
        return n;
    };
    CHECK(countClass(RoadClass::Freeway) > 0);
    CHECK(countClass(RoadClass::Ramp) > 0);
    const std::size_t nodes1 = net.nodes.size(), edges1 = net.edges.size();

    // Regenerate AGAIN from the same recipe — the editor's actual flow.
    applyGenerateRecipe(net, gen);
    CHECK(net.freewayPlans.size() == plans1);     // plans don't ACCUMULATE
    for (uint8_t b : net.edgeBaked) CHECK(b == 0);  // no stale baked flags
    CHECK(countClass(RoadClass::Freeway) == 0);   // clean street-only net...
    const int baked2 = rebakeNetCorridors(net, 520.0);
    CHECK(baked2 == baked1);                      // ...until the re-bake
    CHECK(net.nodes.size() == nodes1);            // same graph, bit for bit
    CHECK(net.edges.size() == edges1);
    CHECK(countClass(RoadClass::Freeway) > 0);
    CHECK(countClass(RoadClass::Ramp) > 0);

    // And the re-baked ramps still LAND: every Ramp chain touches a street.
    std::vector<char> onStreet(net.nodes.size(), 0);
    for (std::size_t ei = 0; ei < net.edges.size(); ++ei)
        if (ei < net.edgeClasses.size() &&
            (net.edgeClasses[ei] == RoadClass::Local ||
             net.edgeClasses[ei] == RoadClass::Arterial ||
             net.edgeClasses[ei] == RoadClass::Collector)) {
            onStreet[net.edges[ei][0]] = 1;
            onStreet[net.edges[ei][1]] = 1;
        }
    int rampTouchesStreet = 0;
    for (std::size_t ei = 0; ei < net.edges.size(); ++ei)
        if (ei < net.edgeClasses.size() && net.edgeClasses[ei] == RoadClass::Ramp &&
            (onStreet[net.edges[ei][0]] || onStreet[net.edges[ei][1]]))
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
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp) ++rampEdges;
    CHECK(rampEdges >= 8);              // two real chains...
    CHECK(rampEdges <= 26);             // ...both editable (<= 14 handles each)

    // Tangents are stored for the baked nodes (parallel array padded).
    CHECK(f.net.tangents.size() == f.net.nodes.size());
    int tangentless = 0, rampNodes = 0;
    std::vector<char> onRamp(f.net.nodes.size(), 0);
    for (int ei = 0; ei < static_cast<int>(f.net.edges.size()); ++ei)
        if (edgeClass(f.net, ei) == RoadClass::Ramp) {
            onRamp[f.net.edges[ei][0]] = 1;
            onRamp[f.net.edges[ei][1]] = 1;
        }
    for (std::size_t ni = f.streetNodeCount; ni < f.net.nodes.size(); ++ni)
        if (onRamp[ni]) {
            ++rampNodes;
            const Vec2 t = f.net.tangents[ni];
            if (t.x == 0.0 && t.y == 0.0) ++tangentless;
        }
    CHECK(rampNodes > 0);
    CHECK(tangentless == 0);            // every baked ramp node carries one

    // Fidelity: sample the FULL graph (baked edges included) and check every
    // authored ramp point sits within 0.25 m of the sampled ramp polyline —
    // the Hermite through sparse nodes+tangents IS the clothoid, near enough.
    RoadGraph g = roadNetFullGraph(f.net);
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

TEST_CASE(unified_mesh_drives_freeway_ramps_and_landings) {
    // Roads-v2.1 R1 step 2b: the ONE mesher builds the baked freeway. The
    // full graph (baked edges included) through buildRoadNetLattice must
    // yield a mesh a car can drive: the elevated mainline, BOTH ramps from
    // gore to landing, and across each landing junction into the street —
    // with viaduct structure present (parapets along deck edges, gapped only
    // near gores; underside beneath every elevated metre). This is the gate
    // the corridor renderer's replacement must pass before the flip.
    Fixture f = makeFixture();
    auto ground = [](Real, Real) { return Real(0); };
    RoadGraph g = roadNetFullGraph(f.net);
    RenderMesh m = buildRoadNetLattice(g, ground, nullptr, 1.8, 0.15, true);
    CHECK(!m.vertices.empty());

    // Drive the mainline over the unified mesh.
    std::vector<Vec3> mainPath;
    for (int ni : f.mainline)
        mainPath.push_back(Vec3(f.net.nodes[ni].x, f.net.nodeElev[ni] + 0.3,
                                f.net.nodes[ni].y));
    driveprobe::Report mrep;
    driveprobe::drivePath(m, mainPath, mrep);
    std::printf("[2b] mainline: samples=%d holes=%d steps=%d blocked=%d\n",
                mrep.samples, mrep.holes, mrep.steps, mrep.blocked);

    CHECK(mrep.samples > 40);
    CHECK(mrep.holes == 0);
    CHECK(mrep.steps == 0);
    CHECK(mrep.blocked == 0);

    // Drive each ramp's RIBBON (authored path past the gore band) plus the
    // last stretch onto the street through the landing junction.
    for (const RampPath& rp : f.authored.rampPaths) {
        const std::size_t lo = static_cast<std::size_t>(rp.bandFront);
        const std::size_t hi = rp.pts.size() - static_cast<std::size_t>(rp.bandBack);
        std::vector<Vec3> path;
        for (std::size_t k = lo; k < hi; ++k)
            path.push_back(Vec3(rp.pts[k].x, rp.pts[k].y + 0.3, rp.pts[k].z));
        driveprobe::Report rrep;
        driveprobe::drivePath(m, path, rrep);
        std::printf("[2b] ramp: samples=%d holes=%d steps=%d blocked=%d\n",
                    rrep.samples, rrep.holes, rrep.steps, rrep.blocked);

        CHECK(rrep.samples > 10);
        CHECK(rrep.holes == 0);
        CHECK(rrep.steps == 0);
        CHECK(rrep.blocked == 0);
    }

    // STRUCTURE: underside — beneath every elevated mainline node there is
    // DOWN-facing geometry within the deck thickness band (the viaduct has a
    // belly, not a floating carpet).
    int missingBelly = 0;
    for (int ni : f.mainline) {
        const double y = f.net.nodeElev[ni];
        if (!(y > 3.0)) continue;              // elevated spans only
        const Vec2 p = f.net.nodes[ni];
        bool belly = false;
        for (std::size_t t = 0; t < m.indices.size() && !belly; t += 3) {
            const Vec3& A = m.vertices[m.indices[t]].position;
            const Vec3& B = m.vertices[m.indices[t + 1]].position;
            const Vec3& C = m.vertices[m.indices[t + 2]].position;
            const Vec3 nrm = cross(C - A, B - A);   // engine winding: outward
            if (nrm.y >= -1e-6) continue;           // want DOWN-facing
            const Vec3 cen = (A + B + C) * (1.0 / 3.0);
            if (std::fabs(cen.x - p.x) < 6.0 && std::fabs(cen.z - p.y) < 6.0 &&
                cen.y > y - 2.5 && cen.y < y + 0.2)
                belly = true;
        }
        if (!belly) ++missingBelly;
    }
    std::printf("[2b] elevated nodes missing underside: %d\n", missingBelly);
    CHECK(missingBelly == 0);

    // STRUCTURE: parapets — along the mainline, vertical geometry rises above
    // the deck near both edges for most of the run (gaps allowed only near
    // the gore nodes). Sample mid-edge stations between mainline nodes.
    int stations = 0, railed = 0, gapsAwayFromGores = 0;
    for (std::size_t i = 0; i + 1 < f.mainline.size(); ++i) {
        const Vec2 a = f.net.nodes[f.mainline[i]];
        const Vec2 b = f.net.nodes[f.mainline[i + 1]];
        const Vec2 mid((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
        const double y =
            (f.net.nodeElev[f.mainline[i]] + f.net.nodeElev[f.mainline[i + 1]]) * 0.5;
        if (!(y > 3.0)) continue;
        ++stations;
        bool rail = false;
        for (std::size_t t = 0; t < m.vertices.size() && !rail; ++t) {
            const Vec3& v = m.vertices[t].position;
            if (v.y > y + 0.5 && v.y < y + 1.6 &&
                std::fabs(v.x - mid.x) < 22.0 && std::fabs(v.z - mid.y) < 22.0)
                rail = true;
        }
        if (rail) ++railed;
        else {
            // A gap is legal only near a gore (deg>=3 mainline node).
            bool nearGore = false;
            for (int ni : f.mainline) {
                int deg = 0;
                for (const auto& e : f.net.edges)
                    if (e[0] == ni || e[1] == ni) ++deg;
                if (deg >= 3 && (f.net.nodes[ni] - mid).length() < 60.0)
                    nearGore = true;
            }
            if (!nearGore) ++gapsAwayFromGores;
        }
    }
    std::printf("[2b] parapet stations=%d railed=%d badGaps=%d\n", stations,
                railed, gapsAwayFromGores);
    CHECK(stations > 10);
    CHECK(gapsAwayFromGores == 0);
    CHECK(railed * 10 >= stations * 9);   // >= 90%% coverage
}

TEST_CASE(dragging_a_gore_node_moves_the_built_freeway) {
    // THE B5 GATE, end to end (asked six times, then found broken on
    // device: "moving them affects nothing"): drag a baked freeway node
    // through the editor's entry point, rebuild through the ONE mesher's
    // standard entry point, and the DECK VISIBLY MOVES — and still drives.
    // No corridor renderer, no stale load-time mesh: the graph is the road.
    Fixture f = makeFixture();
    const RenderMesh before = buildRoadNetMesh(f.net);
    CHECK(!before.vertices.empty());

    const int ni = f.mainline[f.mainline.size() / 2];
    const Vec2 from = f.net.nodes[ni];
    const Vec2 to = from + Vec2(0, 20);
    CHECK(roadNetMoveNode(f.net, ni, to));
    const RenderMesh after = buildRoadNetMesh(f.net);

    // The mesh CHANGED where the drag happened: deck-height vertices now sit
    // near the moved node, and none did before.
    auto deckVertsNear = [&](const RenderMesh& m, const Vec2& p) {
        int n = 0;
        for (const Vertex& v : m.vertices)
            if (std::fabs(v.position.x - p.x) < 4.0 &&
                std::fabs(v.position.z - p.y) < 4.0 && v.position.y > 8.0)
                ++n;
        return n;
    };
    CHECK(deckVertsNear(before, to) == 0);
    CHECK(deckVertsNear(after, to) > 0);

    // And the dragged mainline still drives clean over the rebuilt mesh.
    // The path follows the SAMPLED graph curve (what the mesh sweeps), not
    // straight node chords — after a 20 m jog the Hermite bows metres off
    // the chord, and a chord-driving probe would clip the parapet of a deck
    // that is in fact fine.
    std::vector<Vec3> path;
    {
        RoadGraph g = roadNetFullGraph(f.net);
        std::vector<std::vector<std::pair<int, int>>> adj(g.nodes.size());
        for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei)
            if (g.edges[ei].klass == RoadClass::Freeway) {
                adj[g.edges[ei].a].push_back({ g.edges[ei].b, ei });
                adj[g.edges[ei].b].push_back({ g.edges[ei].a, ei });
            }
        int start = -1;
        for (std::size_t v = 0; v < adj.size(); ++v)
            if (adj[v].size() == 1) { start = static_cast<int>(v); break; }
        CHECK(start >= 0);
        std::vector<char> used(g.edges.size(), 0);
        int cur = start;
        path.push_back(Vec3(g.nodes[cur].pos.x, g.nodes[cur].elev + 0.3,
                            g.nodes[cur].pos.y));
        for (;;) {
            int nxt = -1;
            for (const auto& [w, ei] : adj[cur])
                if (!used[ei]) { used[ei] = 1; nxt = w; break; }
            if (nxt < 0) break;
            path.push_back(Vec3(g.nodes[nxt].pos.x, g.nodes[nxt].elev + 0.3,
                                g.nodes[nxt].pos.y));
            cur = nxt;
        }
        CHECK(path.size() > 40u);   // the sampled curve, not 20 chords
    }
    driveprobe::Report rep;
    driveprobe::drivePath(after, path, rep);
    CHECK(rep.samples > 40);
    CHECK(rep.holes == 0);
    CHECK(rep.steps == 0);
    CHECK(rep.blocked == 0);
}

TEST_CASE(merge_probe_drives_ramp_onto_mainline_on_a_real_metro) {
    // THE MERGE PROBE (drive feedback B1: "You can't merge from the on
    // ramps... there's no way to merge on"). On a REAL generated metro net
    // (recipe -> plan -> resolve -> author -> bake -> ONE mesher), drive
    // from each ramp, THROUGH its gore, onto the mainline — the lane change
    // the old probes never attempted (they drove ramp and mainline
    // separately, which is exactly why the un-mergeable freeway shipped).
    RoadNet net;
    net.width = 11.0;
    net.sidewalk = 2.5;
    net.autoRoundabout = false;
    net.heightAt = [](double, double) { return 0.0; };
    nlohmann::json gen = { { "kind", "metro" },     { "radius", 700 },
                           { "hotspots", 5 },       { "block_size", 120 },
                           { "seed", 9 },           { "freeways", true },
                           { "corridor_freeways", true },
                           { "min_road_len", 24 },  { "terrain_aware", false } };
    applyGenerateRecipe(net, gen);
    CHECK(!net.freewayPlans.empty());
    const int baked = rebakeNetCorridors(net, 520.0);
    std::printf("[merge] corridors baked=%d\n", baked);
    CHECK(baked > 0);

    RenderMesh m = buildRoadNetMesh(net);
    CHECK(!m.vertices.empty());
    RoadGraph g = roadNetFullGraph(net);

    // For every gore (node with both Freeway and Ramp arms): drive the ramp's
    // sampled curve INTO the gore, then continue along the mainline's sampled
    // curve — one continuous path across the merge surface.
    std::vector<std::vector<std::pair<int, int>>> adj(g.nodes.size());
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        adj[g.edges[ei].a].push_back({ g.edges[ei].b, ei });
        adj[g.edges[ei].b].push_back({ g.edges[ei].a, ei });
    }
    auto walkCurve = [&](int from, RoadClass k, int maxN) {
        // Sampled polyline leaving `from` along class-k edges (BFS-free walk).
        std::vector<Vec3> pts;
        std::vector<char> used(g.edges.size(), 0);
        int cur = from;
        pts.push_back(Vec3(g.nodes[cur].pos.x, g.nodes[cur].elev + 0.3,
                           g.nodes[cur].pos.y));
        for (int n = 0; n < maxN; ++n) {
            int nxt = -1;
            for (const auto& [w, ei] : adj[cur])
                if (!used[ei] && g.edges[ei].klass == k) {
                    used[ei] = 1; nxt = w; break;
                }
            if (nxt < 0) break;
            pts.push_back(Vec3(g.nodes[nxt].pos.x, g.nodes[nxt].elev + 0.3,
                               g.nodes[nxt].pos.y));
            cur = nxt;
        }
        return pts;
    };
    int gores = 0, cleanMerges = 0, totalDefects = 0;
    for (std::size_t v = 0; v < adj.size(); ++v) {
        bool hasF = false, hasR = false;
        for (const auto& [w, ei] : adj[v]) {
            if (g.edges[ei].klass == RoadClass::Freeway) hasF = true;
            if (g.edges[ei].klass == RoadClass::Ramp) hasR = true;
        }
        if (!hasF || !hasR) continue;
        ++gores;
        // ramp approach (up to ~10 samples in) + mainline continuation.
        std::vector<Vec3> ramp = walkCurve(static_cast<int>(v), RoadClass::Ramp, 10);
        std::vector<Vec3> main = walkCurve(static_cast<int>(v), RoadClass::Freeway, 10);
        if (ramp.size() < 3 || main.size() < 3) continue;
        std::vector<Vec3> path(ramp.rbegin(), ramp.rend());   // toward the gore
        path.insert(path.end(), main.begin() + 1, main.end()); // onto the deck
        driveprobe::Report rep;
        driveprobe::drivePath(m, path, rep);
        if (rep.holes == 0 && rep.steps == 0 && rep.blocked == 0) ++cleanMerges;
        else {
            totalDefects += rep.holes + rep.steps + rep.blocked;
            std::printf("[merge] gore %zu: holes=%d steps=%d blocked=%d\n", v,
                        rep.holes, rep.steps, rep.blocked);
        }
    }
    std::printf("[merge] gores=%d clean=%d totalDefects=%d\n", gores,
                cleanMerges, totalDefects);
    CHECK(gores >= 2);              // the metro really built interchanges
    CHECK(cleanMerges >= 1);        // the merge mechanism is real
    // RATCHET -> cleanMerges == gores in R2: the residuals are gore-WEDGE
    // PAD quality (the long sliver fill between nose and gore node —
    // measured 1-4 holes / <=2 steps / <=1 blocked per defective gore on
    // seed 9). R2's junction-zoo pad work owns exactly this surface.
    CHECK(totalDefects <= 12);
}
