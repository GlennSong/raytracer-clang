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
    RoadNet net;
    net.nodes.push_back(Vec2(0, 0));
    const int spokes = 9;
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        net.nodes.push_back(Vec2(std::cos(a) * 70.0, std::sin(a) * 70.0));
        net.edges.push_back({0, static_cast<int>(net.nodes.size()) - 1});
    }
    net.width = 12.0;
    RenderMesh m = buildRoadNetMesh(net);
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
    RoadNet net;
    net.nodes.push_back(Vec2(0, 0));
    const int spokes = 8;
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        net.nodes.push_back(Vec2(std::cos(a) * 70.0, std::sin(a) * 70.0));
        net.edges.push_back({0, static_cast<int>(net.nodes.size()) - 1});
    }
    net.width = 12.0;
    net.heightAt = [](double x, double z) { return 0.05 * x + 0.02 * z; };   // gentle slope
    std::vector<TerrainFlatten> regions = roadNetConformRegions(net);
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
    RoadNet net;
    nlohmann::json g = {{"radius", 120}, {"arterials", 3}, {"block_size_max", 30},
                        {"block_size_min", 16}, {"seed", 4}};
    applyGenerateRecipe(net, g);
    CHECK(!net.nodes.empty());
    CHECK(net.edges.size() == net.edgeWidths.size());
    std::vector<int> deg(net.nodes.size(), 0);
    for (const std::array<int, 2>& e : net.edges) { ++deg[e[0]]; ++deg[e[1]]; }
    int mx = 0; for (int d : deg) mx = std::max(mx, d);
    CHECK(mx <= RoadRules{}.maxDegree);          // capped: no junction over 4 arms, even post-planarize
    RenderMesh m = buildRoadNetMesh(net);
    CHECK(!m.vertices.empty());                  // and it meshes
}

// A generated road's recipe survives an edit: roadRecipeForSave keeps the "generate" block (refreshes
// only the look) instead of baking the nodes — the grown.json "save should be the same" fix. (T2.1)
TEST_CASE(generated_road_recipe_survives_edit) {
    RoadNet net;
    nlohmann::json gen = {{"radius", 110}, {"arterials", 2}, {"seed", 3}};
    applyGenerateRecipe(net, gen);                      // net now holds the baked nodes/edges
    CHECK(!net.nodes.empty());
    std::string recipe = nlohmann::json{{"generate", gen}, {"width", 7}, {"curved", true}}.dump();
    net.width = 9.0;                                    // simulate a Width edit in the inspector
    nlohmann::json saved = roadRecipeForSave(recipe, net);
    CHECK(saved.contains("generate"));                 // recipe preserved...
    CHECK(!saved.contains("nodes"));                   // ...NOT baked into geometry
    CHECK(saved["generate"]["radius"] == 110);         // the generate block is intact
    CHECK(saved["width"] == 9.0);                       // the look edit is captured

    RoadNet hand;                                      // a hand-authored road still bakes (the net IS source)
    hand.nodes = {Vec2(0, 0), Vec2(10, 0)}; hand.edges = {{0, 1}};
    nlohmann::json handSaved = roadRecipeForSave("{}", hand);
    CHECK(handSaved.contains("nodes"));
}
