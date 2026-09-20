// What a road BUILDER hands the rest of the engine, for a real level (ADR-0089).
//
//   road_products_probe <level.json> [builder]
//
// Runs the level's own pre-pass (the `generate` recipe against the natural
// ground — exactly what the loader does before it builds anything), then asks
// the road builder for its three products and prints what came back. Naming a
// builder on the command line overrides the level's `"builder"` key, so the
// same level can be measured both ways: this is the A/B instrument for moving
// a level from the lattice to the lanes builder.
#include "engine/level_params.h"
#include "engine/procgen/city/roads/road_builder.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

using namespace engine;

namespace {

double polyArea(const std::vector<Vec3>& p) {   // shoelace in XZ
    double a = 0.0;
    for (std::size_t i = 0, n = p.size(); i < n; ++i) {
        const Vec3& u = p[i];
        const Vec3& v = p[(i + 1) % n];
        a += u.x * v.z - v.x * u.z;
    }
    return std::fabs(a) * 0.5;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: road_products_probe <level.json> [lattice|lanes]\n");
        return 2;
    }
    std::ifstream f(argv[1]);
    if (!f) { std::fprintf(stderr, "cannot open %s\n", argv[1]); return 1; }
    nlohmann::json root;
    try { f >> root; } catch (const std::exception& e) {
        std::fprintf(stderr, "%s: %s\n", argv[1], e.what()); return 1;
    }
    // What the loader does before it builds anything: the natural ground, then
    // each road entity's `generate` recipe run against it. (Deliberately not
    // cityPrePassForLevel — that one refuses a level whose freeway corridors
    // are solved in the loader, which is every metro.)
    propagateWaterSeaLevel(root);
    HeightField ground;
    if (root.contains("terrain")) {
        auto tp = std::make_shared<TerrainParams>(readTerrainParams(root["terrain"]));
        tp->erodedBase = readErodedBase(root);
        auto nz = std::make_shared<Noise>(root["terrain"].value("seed", 0u));
        ground = [tp, nz](double x, double z) { return terrainHeight(*tp, *nz, x, z); };
    }
    std::vector<nlohmann::json> blocks;
    std::vector<RoadEntity> nets;
    for (const nlohmann::json& e : root.value("entities", nlohmann::json::array())) {
        if (!e.is_object() || e.value("shape", std::string()) != "road") continue;
        const nlohmann::json block = e.contains("road") ? e["road"] : nlohmann::json::object();
        RoadEntity net = roadNetFromJson(block);
        if (block.contains("generate")) applyGenerateRecipe(net, block["generate"], ground);
        blocks.push_back(block);
        nets.push_back(std::move(net));
    }
    if (nets.empty()) { std::fprintf(stderr, "%s: no shape:\"road\" entity\n", argv[1]); return 1; }

    std::printf("%s: %zu road entit%s%s\n", argv[1], nets.size(), nets.size() == 1 ? "y" : "ies",
                ground ? "" : " (flat: no terrain block)");
    for (std::size_t i = 0; i < nets.size(); ++i) {
        nlohmann::json block = blocks[i];
        if (argc > 2) block["builder"] = argv[2];
        const RoadEntity& net = nets[i];
        roads::RoadBuilder& b = roads::roadBuilderFor(block);
        roads::RoadBuildInput in;
        in.road = &net;
        in.ground = ground;
        in.options = block;

        std::printf("\nroad %zu — builder \"%s\"\n", i, b.name());
        std::printf("  plan     : %zu control nodes, %zu edges (the recipe's own graph)\n",
                    net.graph.nodes.size(), net.graph.edges.size());

        const roads::GroundPlan gp = b.ground(in);
        if (gp.replaces()) {
            const roads::GroundGrid& g = *gp.replace;
            std::printf("  ground   : a baked grid, %d x %d @ %.1f m (%.0f x %.0f m), "
                        "cut and fill already in it\n",
                        g.nx, g.ny, g.res, g.res * (g.nx - 1), g.res * (g.ny - 1));
        } else {
            double flatArea = 0.0;
            for (const TerrainFlatten& t : gp.carve) flatArea += polyArea(t.polygon);
            std::printf("  ground   : %zu carve patches, %.0f m2 of terrain graded to the road\n",
                        gp.carve.size(), flatArea);
        }

        const roads::RoadProducts p = b.build(in);
        std::size_t deckPts = 0;
        for (const UnionSpine& s : p.deck.spines) deckPts += s.points.size();
        std::size_t bandPts = 0;
        for (const Poly2& l : p.bands.loops) bandPts += l.size();
        std::printf("  mesh     : %zu vertices, %zu triangles\n",
                    p.mesh.vertices.size(), p.mesh.indices.size() / 3);
        std::printf("  deck     : %zu spines / %zu profile points, %zu junction pad triangles\n",
                    p.deck.spines.size(), deckPts, p.deck.pads.size());
        std::printf("  bands    : %zu kerb loops / %zu points, sidewalk %.2f m, curb %.2f m\n",
                    p.bands.loops.size(), bandPts, p.bands.sidewalkWidth, p.bands.curbHeight);

        const RoadGraph nav = b.navGraph(in);
        std::vector<int> deg(nav.nodes.size(), 0);
        double metres = 0.0;
        for (const RoadEdge& e : nav.edges) {
            ++deg[static_cast<std::size_t>(e.a)];
            ++deg[static_cast<std::size_t>(e.b)];
            metres += (nav.nodes[static_cast<std::size_t>(e.b)].pos -
                       nav.nodes[static_cast<std::size_t>(e.a)].pos).length();
        }
        int junctions = 0;
        for (int d : deg) if (d >= 3) ++junctions;
        std::printf("  nav      : %zu nodes, %zu edges, %d junctions, %.0f m of centreline\n",
                    nav.nodes.size(), nav.edges.size(), junctions, metres);
    }
    return 0;
}
