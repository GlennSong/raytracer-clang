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
#include "engine/procgen/city/corridor_plan.h"   // --corridors: the freeway the recipe PLANNED
#include "engine/procgen/city/roads/road_builder.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain.h"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
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
        std::fprintf(stderr, "usage: road_products_probe <level.json> [lattice|lanes] [--corridors]\n");
        return 2;
    }
    bool corridors = false;
    std::string svgPath;
    std::vector<std::string> rest;
    for (int i = 2; i < argc; ++i) {
        const std::string a2 = argv[i];
        if (a2 == "--corridors") corridors = true;
        else if (a2 == "--svg" && i + 1 < argc) svgPath = argv[++i];
        else rest.push_back(a2);
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
        // The recipe PLANS a freeway (an anchor polyline); it only becomes road when the
        // corridor pipeline runs — which the loader does and the lane importer does not.
        // --corridors runs it here, so the two can be compared side by side.
        if (corridors && block.contains("generate")) {
            const double spacing = block["generate"].value("interchange_spacing", 700.0);
            const int n = rebakeNetCorridors(net, static_cast<Real>(spacing), ground);
            std::printf("  [corridors] %d baked from %zu planned route(s), spacing %.0f m\n",
                        n, net.plan.freewayPlans.size(), spacing);
        }
        blocks.push_back(block);
        nets.push_back(std::move(net));
    }
    if (nets.empty()) { std::fprintf(stderr, "%s: no shape:\"road\" entity\n", argv[1]); return 1; }

    std::printf("%s: %zu road entit%s%s\n", argv[1], nets.size(), nets.size() == 1 ? "y" : "ies",
                ground ? "" : " (flat: no terrain block)");
    for (std::size_t i = 0; i < nets.size(); ++i) {
        nlohmann::json block = blocks[i];
        if (!rest.empty()) block["builder"] = rest[0];
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

        // What the recipe actually planned, by class — the question the lane
        // builder has been answering for itself by tracing the city's outline.
        static const char* kClass[] = {"freeway", "arterial", "collector", "local", "ramp", "alley"};
        double byClass[6] = {0, 0, 0, 0, 0, 0}, elevated = 0;
        for (const RoadEdge& e : nav.edges) {
            const double len = (nav.nodes[static_cast<std::size_t>(e.b)].pos -
                                nav.nodes[static_cast<std::size_t>(e.a)].pos).length();
            byClass[static_cast<int>(e.klass)] += len;
            if (e.layer > 0) elevated += len;
        }
        std::printf("  classes  :");
        for (int c = 0; c < 6; ++c) if (byClass[c] > 0) std::printf(" %s %.0f m", kClass[c], byClass[c]);
        std::printf("\n  carried  : %.0f m elevated%s\n", elevated,
                    elevated > 0 ? "" : "   (nothing flies: every road is at grade)");

        // --svg: the graph this probe just measured, in plan, coloured by class.
        // SVG is XML; std::ofstream is the whole dependency (the house rule from
        // road_map_svg). An elevated edge is drawn with a pale casing under it so
        // a viaduct reads as carried rather than merely thick.
        if (!svgPath.empty()) {
            double x0 = 1e300, z0 = 1e300, x1 = -1e300, z1 = -1e300;
            for (const RoadNode& n : nav.nodes) {
                x0 = std::min(x0, n.pos.x); x1 = std::max(x1, n.pos.x);
                z0 = std::min(z0, n.pos.y); z1 = std::max(z1, n.pos.y);
            }
            const double pad = 40.0, step = (z1 - z0) * 0.022;
            const double legendH = step * 6.5;   // the legend lives UNDER the plan
            const double vw = (x1 - x0) + 2 * pad, vh = (z1 - z0) + 2 * pad + legendH;
            std::ofstream f(svgPath);
            f << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='" << x0 - pad << " " << z0 - pad
              << " " << vw << " " << vh << "'>\n"
              << "<rect x='" << x0 - pad << "' y='" << z0 - pad << "' width='" << vw
              << "' height='" << vh << "' fill='#f6f5f1'/>\n";
            auto draw = [&](RoadClass want, const char* colour, double w, bool casing) {
                // Casings for the WHOLE class first: drawn per edge, each one
                // overpainted the previous edge's centre line and the road came
                // out as a blank ribbon.
                if (casing) {
                    f << "<g stroke='#f0c9c9' stroke-width='" << w * 2.6
                      << "' stroke-linecap='round' fill='none'>\n";
                    for (const RoadEdge& e : nav.edges) {
                        if (e.klass != want || e.layer <= 0) continue;
                        const Vec2 a2 = nav.nodes[static_cast<std::size_t>(e.a)].pos;
                        const Vec2 b2 = nav.nodes[static_cast<std::size_t>(e.b)].pos;
                        f << "<line x1='" << a2.x << "' y1='" << a2.y << "' x2='" << b2.x
                          << "' y2='" << b2.y << "'/>\n";
                    }
                    f << "</g>\n";
                }
                f << "<g stroke='" << colour << "' stroke-width='" << w
                  << "' stroke-linecap='round' fill='none'>\n";
                for (const RoadEdge& e : nav.edges) {
                    if (e.klass != want) continue;
                    const Vec2 a2 = nav.nodes[static_cast<std::size_t>(e.a)].pos;
                    const Vec2 b2 = nav.nodes[static_cast<std::size_t>(e.b)].pos;
                    f << "<line x1='" << a2.x << "' y1='" << a2.y << "' x2='" << b2.x << "' y2='"
                      << b2.y << "'/>\n";
                }
                f << "</g>\n";
            };
            draw(RoadClass::Local, "#c9c9c2", 2.5, false);
            draw(RoadClass::Collector, "#9a9a90", 3.5, false);
            draw(RoadClass::Arterial, "#6b6b7d", 5.0, false);
            draw(RoadClass::Ramp, "#e8862a", 6.0, true);
            draw(RoadClass::Freeway, "#c0392b", 9.0, true);
            {   // a legend, so a picture can be read without the caption — only
                // the classes this level actually has, or v2 would advertise a
                // freeway it does not own
                const double lx = x0, ly = z1 + pad * 0.9;
                const char* names[5] = {"freeway (elevated)", "ramp", "arterial", "collector", "local"};
                const char* cols[5] = {"#c0392b", "#e8862a", "#6b6b7d", "#9a9a90", "#c9c9c2"};
                const RoadClass order[5] = {RoadClass::Freeway, RoadClass::Ramp, RoadClass::Arterial,
                                            RoadClass::Collector, RoadClass::Local};
                int row = 0;
                for (int i = 0; i < 5; ++i) {
                    if (byClass[static_cast<int>(order[i])] <= 0) continue;
                    const double y = ly + row++ * step;
                    f << "<line x1='" << lx << "' y1='" << y << "' x2='" << lx + step * 1.6 << "' y2='"
                      << y << "' stroke='" << cols[i] << "' stroke-width='" << (i == 0 ? 9 : 5) << "'/>\n"
                      << "<text x='" << lx + step * 2.0 << "' y='" << y + step * 0.3 << "' font-size='"
                      << step * 0.85 << "' font-family='sans-serif' fill='#333'>" << names[i] << "</text>\n";
                }
            }
            f << "</svg>\n";
            std::printf("  svg      : %s\n", svgPath.c_str());
        }
    }
    return 0;
}
