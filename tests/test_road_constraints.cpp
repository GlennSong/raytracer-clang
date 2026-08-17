#include "test_framework.h"

#include "../src/engine/procgen/city/road_constraints.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/procgen/city/district.h"
#include <cmath>

using namespace engine;

namespace {

// A hub: one centre node with `spokes` arms radiating out evenly to radius `len`.
RoadGraph radialHub(int spokes, double len = 60.0, double width = 12.0) {
    RoadGraph g;
    g.nodes.push_back({Vec2(0, 0)});                 // node 0 = the hub
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        int n = static_cast<int>(g.nodes.size());
        g.nodes.push_back({Vec2(std::cos(a) * len, std::sin(a) * len)});
        g.addEdge(0, n, width, RoadClass::Local);
    }
    return g;
}

int degreeOf(const RoadGraph& g, int v) {
    int d = 0;
    for (const RoadEdge& e : g.edges) if (e.a == v || e.b == v) ++d;
    return d;
}

int maxDegree(const RoadGraph& g) {
    int mx = 0;
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) mx = std::max(mx, degreeOf(g, v));
    return mx;
}

bool everyNodeReferenced(const RoadGraph& g) {
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v)
        if (degreeOf(g, v) == 0) return false;
    return true;
}

}  // namespace

// A clean 4-way crossing is a flat patch, not a roundabout: nothing should change.
TEST_CASE(constraints_leave_a_four_way_alone) {
    RoadGraph g = radialHub(4);
    CHECK(!nodeNeedsRoundabout(g, 0, {}));
    RoadGraph out = applyConstraints(g);
    CHECK(out.nodes.size() == g.nodes.size());
    CHECK(out.edges.size() == g.edges.size());
    CHECK(maxDegree(out) == 4);
}

// A 3-way T with healthy angles also stays flat.
TEST_CASE(constraints_leave_a_tee_alone) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(-30, 0)}, {Vec2(30, 0)}, {Vec2(0, 30)} };
    g.addEdge(0, 1); g.addEdge(0, 2); g.addEdge(0, 3);
    CHECK(!nodeNeedsRoundabout(g, 0, {}));
    CHECK(applyConstraints(g).edges.size() == g.edges.size());
}

// Many spokes -> the hub promotes to a roundabout: the degree-N super-node is gone and
// every surviving node is degree <= 3 (the case the analytic junction pad handles).
TEST_CASE(constraints_promote_a_busy_hub) {
    RoadGraph g = radialHub(8);
    CHECK(nodeNeedsRoundabout(g, 0, {}));
    RoadGraph out = applyConstraints(g);
    CHECK(maxDegree(out) <= 3);
    CHECK(out.edges.size() > g.edges.size());     // ring arcs were added
    CHECK(everyNodeReferenced(out));              // the super-node was compacted away
    // All 8 spokes survive: 8 nodes still sit at the original spoke radius (~60 m).
    int rim = 0;
    for (const RoadNode& n : out.nodes)
        if (std::fabs(n.pos.length() - 60.0) < 1e-3) ++rim;
    CHECK(rim == 8);
}

// Two arms too acute to share a flat junction promote even below the degree cap.
TEST_CASE(constraints_promote_an_acute_fork) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(60, 0)}, {Vec2(60, 6)}, {Vec2(-40, 30)} };
    g.addEdge(0, 1); g.addEdge(0, 2); g.addEdge(0, 3);   // arms 1 and 2 ~6 deg apart
    CHECK(degreeOf(g, 0) == 3);
    CHECK(degreeOf(g, 0) <= RoadRules{}.maxDegree);       // not promoted for being busy
    CHECK(nodeNeedsRoundabout(g, 0, {}));                 // promoted for being acute
    CHECK(maxDegree(applyConstraints(g)) <= 3);
}

// The auto radius answers the spokes: more arms (tighter packing) -> a bigger ring.
TEST_CASE(constraints_radius_grows_with_spokes) {
    auto ringExtent = [](const RoadGraph& g) {
        RoadGraph out = applyConstraints(g);
        double mx = 0;                              // largest non-spoke radius ~ the ring radius
        for (const RoadNode& n : out.nodes) {
            double r = n.pos.length();
            if (r < 55.0) mx = std::max(mx, r);
        }
        return mx;
    };
    double r8 = ringExtent(radialHub(8));
    double r16 = ringExtent(radialHub(16));
    CHECK(r16 > r8);
}

// End to end: the editable road with a busy hub now meshes without blowing up.
TEST_CASE(constraints_editable_hub_meshes) {
    RoadEntity net;
    net.look.defaultWidth = 12.0;
    net.graph.nodes.push_back(RoadNode{Vec2(0, 0)});
    const int spokes = 9;
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        net.graph.nodes.push_back(RoadNode{Vec2(std::cos(a) * 70.0, std::sin(a) * 70.0)});
        net.graph.addEdge(0, static_cast<int>(net.graph.nodes.size()) - 1, 12.0);
    }
    RenderMesh m = buildRoadNetMesh(net, nullptr);
    CHECK(!m.vertices.empty());
    CHECK(m.indices.size() % 3 == 0);
    // The roundabout opens a real island: no road geometry sits at the hub centre.
    int nearOrigin = 0;
    for (const Vertex& v : m.vertices)
        if (std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z) < 5.0) ++nearOrigin;
    CHECK(nearOrigin == 0);
}

// Coherence: terrain conform grades to the SAME roundabout the mesh shows, so the hub
// centre (the island) is left untouched — no conform footprint runs through the origin,
// where the raw spokes would otherwise all converge.
TEST_CASE(constraints_conform_clears_the_island) {
    RoadEntity net;
    net.look.defaultWidth = 12.0;
    net.graph.nodes.push_back(RoadNode{Vec2(0, 0)});
    const int spokes = 8;
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        net.graph.nodes.push_back(RoadNode{Vec2(std::cos(a) * 70.0, std::sin(a) * 70.0)});
        net.graph.addEdge(0, static_cast<int>(net.graph.nodes.size()) - 1, 12.0);
    }
    const RoadGroundFn slope = [](double x, double z) { return 0.05 * x + 0.02 * z; };
    std::vector<TerrainFlatten> regions = roadNetConformRegions(net, slope);
    CHECK(!regions.empty());
    double nearest = 1e30;
    for (const TerrainFlatten& f : regions)
        for (const Vec3& p : f.polygon)
            nearest = std::min(nearest, std::sqrt(p.x * p.x + p.z * p.z));
    CHECK(nearest > 4.0);     // the island is clear; roads grade around it, not through it
}

// With auto-roundabouts OFF (the city generator's policy), a busy hub is left untouched — no ring
// is added (capDegree handles the degree instead), and nodeNeedsRoundabout reports false.
TEST_CASE(constraints_skip_auto_roundabout_when_disabled) {
    RoadGraph g = radialHub(8);
    RoadRules rules; rules.autoRoundabout = false;
    CHECK(!nodeNeedsRoundabout(g, 0, rules));
    RoadGraph out = applyConstraints(g, rules);
    CHECK(out.nodes.size() == g.nodes.size());     // unchanged: no ring
    CHECK(out.edges.size() == g.edges.size());
}

// capDegree splits an over-busy hub into staggered junctions, each within the degree cap, without
// dropping a single arm.
TEST_CASE(cap_degree_splits_a_busy_hub) {
    RoadGraph g = radialHub(8);                     // hub = degree 8
    RoadRules rules;                                // maxDegree = 4
    RoadGraph out = capDegree(g, rules);
    CHECK(maxDegree(out) <= rules.maxDegree);       // no crossing exceeds the cap
    CHECK(everyNodeReferenced(out));                // no orphan nodes
    int rim = 0;                                    // all 8 spoke endpoints survive at ~60 m
    for (const RoadNode& n : out.nodes)
        if (std::fabs(n.pos.length() - 60.0) < 1e-3) ++rim;
    CHECK(rim == 8);
}

// A graph already within the cap is returned unchanged.
TEST_CASE(cap_degree_leaves_a_four_way_alone) {
    RoadGraph g = radialHub(4);
    RoadGraph out = capDegree(g, {});
    CHECK(out.nodes.size() == g.nodes.size());
    CHECK(out.edges.size() == g.edges.size());
}

// Block subdivision honours the [min,max] size bracket: a tighter max packs in more blocks, and no
// block's long edge runs past the max unless splitting would breach the min floor (or the footprint
// piece is itself thinner than min).
TEST_CASE(district_blocks_respect_size_bracket) {
    auto blockCount = [](double mn, double mx) {
        DistrictParams p; p.radius = 120; p.seed = 7;
        p.blockSizeMin = mn; p.blockSizeMax = mx;
        return buildDistrict(p).blocks.size();
    };
    CHECK(blockCount(14, 28) > blockCount(40, 80));   // smaller blocks -> more of them

    DistrictParams p; p.radius = 120; p.seed = 7; p.blockSizeMin = 14; p.blockSizeMax = 28;
    int violations = 0;
    for (const Poly2& b : buildDistrict(p).blocks) {
        OBB2 obb = orientedBoundingBox(b);
        double lo = std::max(obb.half[0], obb.half[1]) * 2.0;
        double sh = std::min(obb.half[0], obb.half[1]) * 2.0;
        if (!(lo <= p.blockSizeMax + 1e-6 || lo < 2.0 * p.blockSizeMin || sh < p.blockSizeMin))
            ++violations;
    }
    CHECK(violations == 0);
}

// applyGenerateRecipe turns a "generate" block into the editable net's graph: a connected,
// degree-capped network (no auto-roundabout) that meshes. The loader and the editor's regenerate
// (the tuning panel) share this one path. (road-network-v2-plan T2.1)
TEST_CASE(generate_recipe_builds_capped_net) {
    RoadEntity net;
    nlohmann::json g = {{"radius", 120}, {"arterials", 3}, {"block_size_max", 30},
                        {"block_size_min", 16}, {"seed", 4}};
    applyGenerateRecipe(net, g, nullptr);
    CHECK(!net.graph.nodes.empty());
    // Widths are RESOLVED at construction now: every generated edge carries one.
    for (const RoadEdge& e : net.graph.edges) CHECK(e.width > 0.0);
    std::vector<int> deg(net.graph.nodes.size(), 0);
    for (const RoadEdge& e : net.graph.edges) { ++deg[e.a]; ++deg[e.b]; }
    int mx = 0; for (int d : deg) mx = std::max(mx, d);
    CHECK(mx <= RoadRules{}.maxDegree);          // capped: no junction over 4 arms, even post-planarize
    RenderMesh m = buildRoadNetMesh(net, nullptr);
    CHECK(!m.vertices.empty());                  // and it meshes
}

// A generated road's recipe survives an edit: roadRecipeForSave keeps the "generate" block (refreshes
// only the look) instead of baking the nodes — the grown.json "save should be the same" fix. (T2.1)
TEST_CASE(generated_road_recipe_survives_edit) {
    RoadEntity net;
    nlohmann::json gen = {{"radius", 110}, {"arterials", 2}, {"seed", 3}};
    applyGenerateRecipe(net, gen, nullptr);             // net now holds the baked graph
    CHECK(!net.graph.nodes.empty());
    std::string recipe = nlohmann::json{{"generate", gen}, {"width", 7}, {"curved", true}}.dump();
    net.look.defaultWidth = 9.0;                        // simulate a Width edit in the inspector
    nlohmann::json saved = roadRecipeForSave(recipe, net);
    CHECK(saved.contains("generate"));                 // recipe preserved...
    CHECK(!saved.contains("nodes"));                   // ...NOT baked into geometry
    CHECK(saved["generate"]["radius"] == 110);         // the generate block is intact
    CHECK(saved["width"] == 9.0);                       // the look edit is captured

    RoadEntity hand;                                    // a hand-authored road still bakes (the graph IS source)
    hand.graph.nodes = {RoadNode{Vec2(0, 0)}, RoadNode{Vec2(10, 0)}};
    hand.graph.addEdge(0, 1, hand.look.defaultWidth);
    nlohmann::json handSaved = roadRecipeForSave("{}", hand);
    CHECK(handSaved.contains("nodes"));
}

// Minimum road length (device: "really short roads ... should be merged"). Two
// crossings that landed a few metres apart fold into ONE junction when the
// united node stays within the degree cap; the absorbed node is compacted away.
TEST_CASE(short_edge_merges_adjacent_crossings) {
    // Two T-junctions 5 m apart on a through road: |  5m  |, each with one side
    // arm. Merged: one 4-way (2 through + 2 side arms) — within the cap.
    RoadGraph g;
    g.nodes = { {Vec2(-40, 0)}, {Vec2(0, 0)}, {Vec2(5, 0)}, {Vec2(45, 0)},
                {Vec2(0, 30)}, {Vec2(5, -30)} };
    g.edges.push_back({0, 1, 7, RoadClass::Local, 0});
    g.edges.push_back({1, 2, 7, RoadClass::Local, 0});   // the 5 m stub
    g.edges.push_back({2, 3, 7, RoadClass::Local, 0});
    g.edges.push_back({1, 4, 7, RoadClass::Local, 0});   // side arm north
    g.edges.push_back({2, 5, 7, RoadClass::Local, 0});   // side arm south
    RoadGraph m = mergeShortEdges(g, 14.0, 4);
    CHECK(m.nodes.size() == 5);                          // one node absorbed
    CHECK(m.edges.size() == 4);                          // the stub is gone
    for (const RoadEdge& e : m.edges) {                  // no edge under the floor
        Real len = (m.nodes[e.a].pos - m.nodes[e.b].pos).length();
        CHECK(len >= 13.0);
    }
    int mx = 0;                                          // merged junction is a 4-way
    for (int v = 0; v < static_cast<int>(m.nodes.size()); ++v) {
        int d = 0;
        for (const RoadEdge& e : m.edges) if (e.a == v || e.b == v) ++d;
        mx = std::max(mx, d);
    }
    CHECK(mx == 4);
}

// When merging would over-crowd the junction (> maxDegree arms), the short road
// is LENGTHENED instead: endpoints pushed apart along the edge to minLen — the
// capDegree stagger link becomes a drivable block face, not a stub.
TEST_CASE(short_edge_lengthens_when_merge_would_overcrowd) {
    // Two 3-arm junctions joined by a 6 m link: merging would make a 6-way.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(6, 0)},
                {Vec2(-30, 20)}, {Vec2(-30, 0)}, {Vec2(-30, -20)},
                {Vec2(36, 20)}, {Vec2(36, 0)}, {Vec2(36, -20)} };
    g.edges.push_back({0, 1, 7, RoadClass::Local, 0});   // the 6 m link
    g.edges.push_back({0, 2, 7, RoadClass::Local, 0});
    g.edges.push_back({0, 3, 7, RoadClass::Local, 0});
    g.edges.push_back({0, 4, 7, RoadClass::Local, 0});
    g.edges.push_back({1, 5, 7, RoadClass::Local, 0});
    g.edges.push_back({1, 6, 7, RoadClass::Local, 0});
    g.edges.push_back({1, 7, 7, RoadClass::Local, 0});
    RoadGraph m = mergeShortEdges(g, 14.0, 4);
    CHECK(m.nodes.size() == 8);                          // nothing merged
    CHECK(m.edges.size() == 7);
    Real len = (m.nodes[0].pos - m.nodes[1].pos).length();
    CHECK(std::fabs(len - 14.0) < 1e-6);                 // pushed apart to minLen
    // Symmetric push: the midpoint stayed put.
    Vec2 mid = (m.nodes[0].pos + m.nodes[1].pos) * 0.5;
    CHECK(std::fabs(mid.x - 3.0) < 1e-6);
    CHECK(std::fabs(mid.y - 0.0) < 1e-6);
}

// The generate recipe applies the floor: no junction-to-junction road in a
// generated district comes out shorter than min_road_len (default 14 m), and the
// degree cap still holds afterwards.
TEST_CASE(generate_recipe_enforces_min_road_length) {
    RoadEntity net;
    net.look.defaultWidth = 7;
    nlohmann::json gen = {{"radius", 170.0}, {"arterials", 3}, {"artery_width", 13.0},
                          {"street_width", 7.0}, {"block_size", 82.0}, {"seed", 5}};
    applyGenerateRecipe(net, gen, nullptr);              // straight version (no warp samples)
    CHECK(!net.graph.edges.empty());
    int n = static_cast<int>(net.graph.nodes.size());
    std::vector<int> deg(n, 0);
    for (const RoadEdge& e : net.graph.edges) { ++deg[e.a]; ++deg[e.b]; }
    for (const RoadEdge& e : net.graph.edges) {
        Real len = (net.graph.nodes[e.a].pos - net.graph.nodes[e.b].pos).length();
        // Only junction-to-junction stubs matter (curve/warp samples are degree-2).
        if (deg[e.a] >= 3 && deg[e.b] >= 3) CHECK(len >= 13.0);
    }
    for (int v = 0; v < n; ++v) CHECK(deg[v] <= 4);
}

// No hairpins mid-block (device: "sharp bends in the road ... creating some
// really bad overlap"): a degree-2 corner sharper than the limit is relaxed
// toward its chord until the road can actually be stroked without folding.
// Junction and dead-end nodes never move.
TEST_CASE(sharp_through_bends_are_relaxed) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(40, 0)}, {Vec2(42, 8)}, {Vec2(4, 12)} };
    g.edges.push_back({0, 1, 7, RoadClass::Local, 0});
    g.edges.push_back({1, 2, 7, RoadClass::Local, 0});   // ~150 deg hairpin at node 1
    g.edges.push_back({2, 3, 7, RoadClass::Local, 0});
    RoadGraph r = relaxSharpBends(g, 0.9, 48);
    CHECK((r.nodes[0].pos - Vec2(0, 0)).length() < 1e-9);   // dead ends pinned
    CHECK((r.nodes[3].pos - Vec2(4, 12)).length() < 1e-9);
    for (int v = 1; v <= 2; ++v) {                          // through-nodes now gentle
        Vec2 a = r.nodes[v - 1].pos, m = r.nodes[v].pos, b = r.nodes[v + 1].pos;
        Vec2 d0 = normalize(m - a), d1 = normalize(b - m);
        CHECK(dot(d0, d1) >= std::cos(0.9) - 1e-6);
    }
}

// The generate recipe applies the bend limit: no degree-2 node in a generated
// district turns sharper than ~52 degrees, so the welded carriageway never folds.
TEST_CASE(generate_recipe_has_no_hairpin_bends) {
    RoadEntity net;
    net.look.defaultWidth = 7;
    nlohmann::json gen = {{"radius", 170.0}, {"arterials", 3}, {"artery_width", 13.0},
                          {"street_width", 7.0}, {"block_size", 82.0},
                          {"curviness", 0.22}, {"seed", 5}};
    applyGenerateRecipe(net, gen, nullptr);
    const int n = static_cast<int>(net.graph.nodes.size());
    std::vector<std::vector<int>> adj(n);
    for (const RoadEdge& e : net.graph.edges) { adj[e.a].push_back(e.b); adj[e.b].push_back(e.a); }
    for (int v = 0; v < n; ++v) {
        if (adj[v].size() != 2) continue;
        Vec2 a = net.graph.nodes[adj[v][0]].pos, m = net.graph.nodes[v].pos,
             b = net.graph.nodes[adj[v][1]].pos;
        if ((m - a).length() < 1e-6 || (b - m).length() < 1e-6) continue;
        Vec2 d0 = normalize(m - a), d1 = normalize(b - m);
        CHECK(dot(d0, d1) >= std::cos(0.95));   // limit + slack
    }
}

// Field-carry (semantic-layer S0): the promotion rebuild used to re-init
// edges positionally, silently resetting every field after `layer`
// (walkable, spec, oneWay, provenance — and the access bits the semantic
// layer adds). A promoted hub must keep its surviving spokes' payload.
TEST_CASE(constraints_promotion_preserves_edge_payload) {
    RoadGraph g = radialHub(7);   // over maxDegree: promotes to a roundabout
    for (RoadEdge& e : g.edges) {
        e.walkable = false;       // non-default on every spoke
        e.spec = 3;
        e.oneWay = true;
        e.provenance = RoadProvenance::CorridorRamp;
    }
    RoadGraph out = applyConstraints(g);
    CHECK(out.nodes.size() > g.nodes.size());   // it really promoted
    // Ring arcs are synthesized (default payload, and they inherit the
    // spokes' width — width cannot tell them apart). The payload itself
    // identifies the surviving spokes: exactly the original 7 edges carry
    // it, and each carries ALL of it.
    int spokes = 0;
    for (const RoadEdge& e : out.edges) {
        if (e.spec != 3) continue;      // a synthesized ring arc
        ++spokes;
        CHECK(!e.walkable);
        CHECK(e.oneWay);
        CHECK(e.provenance == RoadProvenance::CorridorRamp);
    }
    CHECK(spokes == 7);   // every spoke survived with its payload
}
