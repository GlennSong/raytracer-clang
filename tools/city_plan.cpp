// city_plan — design a city as data and look at it before anything is built.
//
//   city_plan generate BRIEF.json OUT_DIR [--variants N]
//       N candidate layouts (variant 0 is the brief itself): for each, the plan
//       (OUT_DIR/<name>.json: road graph + blocks with district, use and scores),
//       a map (<name>.svg, and <name>.png when Inkscape is installed) and a
//       scorecard line; with several variants, OUT_DIR/contact.png side by side.
//   city_plan brief
//       prints the default brief, to copy and edit.
//
// The scores come from the engine's own parcel walk and router, so a plan that
// scores well here builds the way it scored.

#include "engine/procgen/city/plan/city_plan.h"
#include "engine/procgen/city/plan/plan_scene.h"
#include "engine/procgen/world/island_world.h"
#include "engine/procgen/world/road_signs.h"
#include "engine/procgen/city/street_names.h"

#include <nlohmann/json.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace engine::plan;

static int usage() {
    std::fprintf(stderr,
                 "usage: city_plan generate BRIEF.json OUT_DIR [--variants N]\n"
                 "       city_plan scene BRIEF.json OUT_SCENE.json [--no-ramps]\n"
                 "       city_plan brief\n"
                 "       city_plan level-world BRIEF.json      the level's terrain + water blocks that match the brief's world, and its towns' hubs\n"
                 "       city_plan island SEED OUT_DIR [--variants N]   island worlds on the map: terrain, sites, ring freeway (ADR-0105)\n"
                 "       city_plan island-cities SEED OUT_DIR [--png] [--view X Z HALF NAME]...   that island's cities and towns planned on its land, mapped, and extra close-ups (ADR-0106)\n"
                 "       city_plan rivers BRIEF.json [STEP]         the rivers its world's hydrology makes, plan coordinates\n"
                 "       city_plan heights BRIEF.json [HALF STEP]   the brief's ground on a grid, plan coordinates (for placing mountain roads)\n");
    return 2;
}

static bool haveCommand(const char* cmd) {
    const std::string probe = std::string("command -v ") + cmd + " >/dev/null 2>&1";
    return std::system(probe.c_str()) == 0;
}

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string verb = argv[1];
    if (verb == "brief") {
        std::cout << briefToJson(Brief{}).dump(2) << "\n";
        return 0;
    }
    if (verb == "island" && argc >= 4) {
        // AN ISLAND WORLD ON THE MAP (ADR-0105): the terrain first, then where the cities and towns
        // can go, then the ring freeway, the pass and the mountain road -- a PNG and a report per
        // variant, before anything is built in 3D.
        const uint32_t seed = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
        const std::string outDir = argv[3];
        int variants = 1;
        for (int k = 4; k + 1 < argc; ++k) if (std::string(argv[k]) == "--variants") variants = std::atoi(argv[k + 1]);
        std::filesystem::create_directories(outDir);
        for (int v = 0; v < variants; ++v) {
            const uint32_t s = seed + static_cast<uint32_t>(v);
            const nlohmann::json block = engine::islandTerrainBlock(s);
            const engine::IslandWorld w = engine::planIsland(block);
            const std::string base = outDir + "/island_" + std::to_string(s);
            engine::writeIslandMap(w, base + ".png");
            std::ofstream(base + ".json") << nlohmann::json{{"seed", s}, {"terrain", block}, {"report", w.report}}.dump(2) << "\n";
            int cities = 0, towns = 0, unrouted = 0;
            for (const auto& st : w.sites) (st.kind == "city" ? cities : towns)++;
            for (const auto& rd : w.roads) if (rd.points.empty()) ++unrouted;
            std::printf("island %u: %.0f km2 of land, peak %.0f m, %zu rivers, %d cities, %d towns, ring freeway %.1f km%s (%.1f s) -> %s.png\n",
                        s, w.report["landKm2"].get<double>(), w.report["peak"]["height"].get<double>(), w.hydro ? w.hydro->rivers().size() : 0,
                        cities, towns, w.report["ringFreewayKm"].get<double>(),
                        unrouted ? (", " + std::to_string(unrouted) + " roads UNROUTED").c_str() : "", w.report["seconds"].get<double>(), base.c_str());
        }
        return 0;
    }
    if (verb == "island-cities" && argc >= 4) {
        // THE ISLAND'S CITIES ON THE MAP (ADR-0106): each site planned by the city planner inside its
        // own land -- a brief scaled to the site (islandSiteBrief), the plan cut at the sea and the
        // steep ground, a city's ring a C against the coast -- and the island freeway run onto each
        // city's ring. The whole island with every plan on it, a close-up per site, and the briefs.
        const uint32_t seed = static_cast<uint32_t>(std::strtoul(argv[2], nullptr, 10));
        const std::string outDir = argv[3];
        std::filesystem::create_directories(outDir);
        const nlohmann::json block = engine::islandTerrainBlock(seed);
        engine::IslandWorld w = engine::planIsland(block);
        struct Drawn { std::string name; engine::Vec2 at; double half; };
        std::vector<Drawn> sites;
        std::vector<std::pair<int, std::vector<std::vector<engine::Vec2>>>> cityLimits;
        std::vector<std::pair<int, std::vector<engine::Vec2>>> arterialNodes;
        std::map<int, std::vector<std::string>> arterialNames;
        engine::IslandMapLayer locals, collectors, arterials, freeways;
        locals.name = "streets-local"; collectors.name = "streets-collector"; arterials.name = "streets-arterial"; freeways.name = "city-freeways";
        locals.rgb[0] = 0.96f; locals.rgb[1] = 0.95f; locals.rgb[2] = 0.92f; locals.widthM = 12;
        collectors.rgb[0] = 1.0f; collectors.rgb[1] = 0.93f; collectors.rgb[2] = 0.62f; collectors.widthM = 16;
        arterials.rgb[0] = 1.0f; arterials.rgb[1] = 0.84f; arterials.rgb[2] = 0.35f; arterials.widthM = 22; arterials.minPx = 1.4;
        engine::IslandMapLayer limits;
        limits.name = "city-limits";
        limits.rgb[0] = 0.55f; limits.rgb[1] = 0.12f; limits.rgb[2] = 0.35f; limits.widthM = 4; limits.minPx = 1.2;
        freeways.rgb[0] = 0.98f; freeways.rgb[1] = 0.60f; freeways.rgb[2] = 0.10f; freeways.widthM = 30; freeways.minPx = 2.5; freeways.casing = true;
        for (std::size_t k = 0; k < w.sites.size(); ++k) {
            const auto t0 = std::chrono::steady_clock::now();
            const nlohmann::json bj = engine::islandSiteBrief(w, static_cast<int>(k));
            std::ofstream(outDir + "/" + bj["name"].get<std::string>() + ".brief.json") << bj.dump(2) << "\n";
            const Brief b = briefFromJson(bj);
            CityPlan plan = generatePlan(b);
            {
                // what its town-limit sign says: people, from its planned buildings (a rough 6 a building)
                const PlanScore sc = evaluatePlan(plan);
                const int pop = sc.predictedBuildings * 6;
                const int unit = pop > 10000 ? 1000 : pop > 1000 ? 100 : 10;
                w.sites[k].population = (pop + unit / 2) / unit * unit;
            }
            double km[4] = {0, 0, 0, 0};
            for (const auto& e : plan.streets.edges) {
                const engine::Vec2 a = plan.streets.nodes[static_cast<std::size_t>(e.a)].pos, c = plan.streets.nodes[static_cast<std::size_t>(e.b)].pos;
                engine::IslandMapLayer& L = e.klass == engine::RoadClass::Arterial ? arterials : e.klass == engine::RoadClass::Collector ? collectors : locals;
                L.lines.push_back({a, c});
                km[e.klass == engine::RoadClass::Arterial ? 2 : e.klass == engine::RoadClass::Collector ? 1 : 0] += (c - a).length() / 1000.0;
            }
            for (const auto& e : plan.freeway.edges) {
                const engine::Vec2 a = plan.freeway.nodes[static_cast<std::size_t>(e.a)].pos, c = plan.freeway.nodes[static_cast<std::size_t>(e.b)].pos;
                freeways.lines.push_back({a, c});
                km[3] += (c - a).length() / 1000.0;
            }
            for (const auto& l : plan.limits) limits.lines.push_back(l);
            if (!plan.limits.empty()) cityLimits.push_back({static_cast<int>(k), plan.limits});
            else if (!plan.ringArc.empty()) engine::joinFreewayToRing(w, static_cast<int>(k), plan.ringArc, false);
            else if (!plan.ring.empty()) engine::joinFreewayToRing(w, static_cast<int>(k), plan.ring, true);
            {
                // the arterials' junctions, each with its street's name (street_names.h), for the links
                engine::StreetNamingParams np;
                np.seed = b.seed;
                const engine::StreetNaming naming = engine::nameStreets(plan.streets, np);
                std::vector<engine::Vec2> art;
                std::vector<std::string> names;
                for (std::size_t ei = 0; ei < plan.streets.edges.size(); ++ei) {
                    const auto& e = plan.streets.edges[ei];
                    if (e.klass != engine::RoadClass::Arterial) continue;
                    const int st = naming.streetOf(static_cast<int>(ei));
                    const std::string nm = st >= 0 ? engine::abbreviateStreetName(naming.streets[static_cast<std::size_t>(st)].name) : std::string();
                    for (int nd : {e.a, e.b}) { art.push_back(plan.streets.nodes[static_cast<std::size_t>(nd)].pos); names.push_back(nm); }
                }
                arterialNodes.push_back({static_cast<int>(k), art});
                arterialNames[static_cast<int>(k)] = names;
            }
            sites.push_back({b.name, b.center, b.size * 0.5 + 250.0});
            std::printf("%-18s %-16s %-13s at (%6.0f, %6.0f), %4.0f m across: %4zu blocks, streets %.1f km local / %.1f collector / %.1f arterial, freeway %.1f km%s (%.1f s)\n",
                        w.sites[k].name.c_str(), b.name.c_str(), w.sites[k].kind.c_str(), b.center.x, b.center.y, b.size, plan.blocks.size(), km[0], km[1], km[2], km[3],
                        !plan.ringArc.empty() ? ", ring a C on the coast" : !plan.ring.empty() ? ", closed ring" : "",
                        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
        }
        if (!cityLimits.empty()) {
            engine::routeFreewayRoundCities(w, cityLimits);
            // and every city onto it: link roads from its arterials (a city three or four, a town one or two)
            for (const auto& [k, art] : arterialNodes) {
                const bool city = w.sites[static_cast<std::size_t>(k)].kind == "city";
                if (w.sites[static_cast<std::size_t>(k)].kind == "mountain town") continue;   // its road is the mountain road
                engine::linkCityToFreeway(w, k, art, city ? 4 : 2, city ? 1000.0 : 700.0, 700.0, arterialNames[k]);
            }
            std::vector<engine::Vec2> allArterial;
            for (const auto& [k, art] : arterialNodes) allArterial.insert(allArterial.end(), art.begin(), art.end());
            engine::islandInterchanges(w, allArterial);
            engine::planIslandSigns(w);
            engine::writeSignSheetSvg(w, outDir + "/signs.svg");
            std::map<std::string, int> kinds;
            for (const auto& sg : w.signs) ++kinds[sg.kind];
            std::printf("signs: %zu (", w.signs.size());
            for (const auto& [kd, n] : kinds) std::printf(" %s %d", kd.c_str(), n);
            std::printf(" ) -> %s/signs.svg\n", outDir.c_str());
            const auto& ic = w.report["interchanges"];
            std::printf("interchanges: %d diamonds at %d crossings (%zu ramps); refused: %s\n", ic["built"].get<int>(), ic["crossings"].get<int>(),
                        w.ramps.size(), ic["refused"].dump().c_str());
            std::printf("island freeway round the cities: %.1f km, %d legs unrouted\n", w.report["ringFreewayKm"].get<double>(),
                        w.report["freewayRoundCities"]["unrouted"].get<int>());
        }
        engine::IslandMapView whole;
        whole.sites = false;
        whole.layers = {limits, locals, collectors, arterials, freeways};
        const std::string base = outDir + "/island_" + std::to_string(seed);
        engine::writeIslandSvg(w, base + ".svg", whole);
        engine::writeIslandMap(w, base + "_cities.png", 2000, whole);   // a quick look; the SVG is the map
        std::printf("-> %s.svg (the map: vector, zoom into it; layers toggle) and %s_cities.png (a quick look)\n", base.c_str(), base.c_str());
        bool png = false;
        for (int a = 4; a < argc; ++a) if (std::string(argv[a]) == "--png") png = true;
        // extra close-ups: --view X Z HALF NAME (an interchange, a pass's foot)
        for (int a = 4; a + 4 < argc; ++a)
            if (std::string(argv[a]) == "--view")
                sites.push_back({argv[a + 4], engine::Vec2(std::atof(argv[a + 1]), std::atof(argv[a + 2])), std::atof(argv[a + 3])});
        for (const Drawn& d : sites) {
            if (!png && d.name.rfind("zoom", 0) != 0) continue;   // per-site close-ups only with --png: zoom into the SVG instead
            engine::IslandMapView z = whole;
            z.centre = d.at;
            z.half = d.half;
            engine::writeIslandMap(w, outDir + "/" + d.name + ".png", 1400, z);
        }
        return 0;
    }
    if (verb == "rivers" && argc >= 3) {
        // THE RIVERS A BRIEF'S WORLD MAKES: its base terrain's hydrology (a "rivers" block in
        // world.base, ADR-0099), each river's course in PLAN coordinates, width and water level.
        std::ifstream in(argv[2]);
        if (!in) { std::fprintf(stderr, "city_plan: cannot read %s\n", argv[2]); return 1; }
        nlohmann::json bj;
        in >> bj;
        const Brief b = briefFromJson(bj);
        const std::shared_ptr<const engine::Hydrology> hy = worldHydrology(b);
        if (!hy) { std::fprintf(stderr, "city_plan: %s's world has no rivers block\n", argv[2]); return 1; }
        const double step = argc > 3 ? std::atof(argv[3]) : 200.0;
        std::printf("%zu rivers, %zu lakes\n", hy->rivers().size(), hy->lakes().size());
        for (std::size_t i = 0; i < hy->rivers().size(); ++i) {
            const auto& r = hy->rivers()[i];
            double acc = 0, next = 0;
            std::printf("river %zu: %zu nodes, width %.0f..%.0f m, %s\n", i, r.nodes.size(), r.nodes.front().width,
                        r.nodes.back().width, r.mouth ? (r.intoLake >= 0 ? "into a lake" : "to the sea") : "ends inland");
            for (std::size_t k = 0; k < r.nodes.size(); ++k) {
                if (k) acc += (r.nodes[k].p - r.nodes[k - 1].p).length();
                if (acc >= next || k + 1 == r.nodes.size()) {
                    std::printf("   (%7.0f, %7.0f)  level %6.1f  width %4.0f\n", r.nodes[k].p.x, r.nodes[k].p.y, r.nodes[k].level, r.nodes[k].width);
                    next += step;
                }
            }
        }
        return 0;
    }
    if (verb == "heights" && argc >= 3) {
        // THE GROUND A BRIEF BUILDS ON, as numbers: its scene ground (the world base plus the city's
        // relief) on a coarse grid in PLAN coordinates -- what to read before placing a mountain road.
        std::ifstream in(argv[2]);
        if (!in) { std::fprintf(stderr, "city_plan: cannot read %s\n", argv[2]); return 1; }
        nlohmann::json bj;
        in >> bj;
        const Brief b = briefFromJson(bj);
        const double half = argc > 3 ? std::atof(argv[3]) : 3000.0, step = argc > 4 ? std::atof(argv[4]) : 250.0;
        const engine::HeightField g = sceneGround(b);
        std::printf("%8s", "y\\x");
        for (double x = -half; x <= half + 1e-6; x += step) std::printf("%6.0f", x);
        std::printf("\n");
        for (double y = half; y >= -half - 1e-6; y -= step) {
            std::printf("%8.0f", y);
            for (double x = -half; x <= half + 1e-6; x += step) std::printf("%6.0f", g(x, y));
            std::printf("\n");
        }
        return 0;
    }
    if (verb == "level-world" && argc == 3) {
        // THE LEVEL'S SIDE OF THE WORLD: the city's ground grid is the brief's world plus its hills,
        // and beyond the grid the level renders its own terrain block — which must be the same
        // world, or the seam shows. This prints the blocks to put in the level.
        std::ifstream in(argv[2]);
        if (!in) { std::fprintf(stderr, "city_plan: cannot read %s\n", argv[2]); return 1; }
        nlohmann::json bj;
        in >> bj;
        const Brief b = briefFromJson(bj);
        if (b.world.is_null()) { std::fprintf(stderr, "city_plan: %s has no world\n", argv[2]); return 1; }
        nlohmann::json terrain = b.world.at("base");
        terrain["_comment"] = "Written by `city_plan level-world " + std::string(argv[2]) + "`: the brief's world, which the city's ground grid is built on. Edit the brief, not this.";
        // baked: the ground is drawn from a height pyramid at 1 m where it needs it (ADR-0095)
        terrain["cdlod"] = {{"worldHalf", terrain.value("size", 3400.0) / 2}, {"baked", true}};
        terrain["material"] = {{"albedo", {1.0, 1.0, 1.0}}, {"roughness", 1.0}};
        nlohmann::json out = {{"terrain", terrain}};
        if (b.world.contains("seaLevel")) {
            nlohmann::json water = b.world.value("water", nlohmann::json::object());
            water["seaLevel"] = b.world["seaLevel"];
            out["water"] = water;
        }
        // and a district hub for each town the plan builds, so its lots grow as a small old town
        // rather than as the city's outskirts (citysim.districts.hubs; authored hubs win)
        const CityPlan plan = generatePlan(b);
        nlohmann::json hubs = nlohmann::json::array();
        for (const CityPlan::Town& t : plan.towns)
            if (t.built) hubs.push_back({{"at", {std::round(t.centre.x), std::round(t.centre.y)}}, {"kind", "oldtown"}});
        for (const CityPlan::Place& p : plan.places)   // and the hubs of each place on the outer loop
            for (const auto& h : p.hubs)
                hubs.push_back({{"at", {std::round(h.first.x), std::round(h.first.y)}}, {"kind", h.second}, {"_place", p.name}});
        out["townHubs"] = hubs;
        std::cout << out.dump(1) << "\n";
        return 0;
    }
    if ((verb != "generate" && verb != "scene") || argc < 4) return usage();
    const std::string briefPath = argv[2], outDir = argv[3];
    int variants = 1;
    for (int i = 4; i + 1 < argc; ++i)
        if (std::string(argv[i]) == "--variants") variants = std::max(1, std::atoi(argv[i + 1]));
    std::ifstream in(briefPath);
    if (!in) { std::fprintf(stderr, "city_plan: cannot read %s\n", briefPath.c_str()); return 1; }
    nlohmann::json bj;
    try { in >> bj; } catch (const std::exception& e) { std::fprintf(stderr, "city_plan: %s: %s\n", briefPath.c_str(), e.what()); return 1; }
    const Brief base = briefFromJson(bj);
    if (verb == "scene") {
        // THE PLAN, BUILT: the same plan the scorecard scored, as the lanes builder's scene.
        SceneOptions so;
        for (int i = 4; i < argc; ++i)
            if (std::string(argv[i]) == "--no-ramps") so.ramps = false;
        CityPlan plan = generatePlan(base);
        const PlanScore s = evaluatePlan(plan);
        const nlohmann::json scene = planToLanesScene(plan, so);
        std::ofstream o(outDir);   // the scene's path, in this verb
        if (!o) { std::fprintf(stderr, "city_plan: cannot write %s\n", outDir.c_str()); return 1; }
        o << scene.dump(1) << "\n";
        std::printf("%s: %zu scene edges from %zu street + %zu freeway plan edges; %d blocks, ~%d buildings scored\n",
                    outDir.c_str(), scene["edges"].size(), plan.streets.edges.size(), plan.freeway.edges.size(),
                    s.blocks, s.predictedBuildings);
        return 0;
    }
    fs::create_directories(outDir);
    const bool inkscape = haveCommand("inkscape");
    std::vector<std::string> pngs;
    for (int k = 0; k < variants; ++k) {
        const Brief b = variantOf(base, k);
        CityPlan plan = generatePlan(b);
        const PlanScore s = evaluatePlan(plan);
        const std::string stem = (fs::path(outDir) / b.name).string();
        { std::ofstream o(stem + ".json"); o << planToJson(plan, s).dump(1) << "\n"; }
        { std::ofstream o(stem + ".svg"); o << planToSvg(plan, s); }
        std::printf("%-20s blocks %4d (lots %4d, landmark %3d, park %3d, freeway %3d)  buildings ~%5d  "
                    "rectilinear %3.0f%% (core+mid %3.0f%%, grid %3.0f%%)  freeway commutes %3.0f%%  streets %.1f km  pieces %d  overlaps %d\n",
                    b.name.c_str(), s.blocks, s.lotBlocks, s.landmarkBlocks, s.parkBlocks, s.rowBlocks, s.predictedBuildings,
                    100 * s.rectilinearShare, 100 * s.coreRectilinearShare, 100 * s.gridRectilinearShare,
                    100 * s.freewayCommuteShare, s.streetKm, s.streetComponents, s.corridorOverlaps);
        if (inkscape) {
            const std::string cmd = "inkscape '" + stem + ".svg' --export-type=png --export-filename='" + stem +
                                    ".png' --export-width=1800 >/dev/null 2>&1";
            if (std::system(cmd.c_str()) == 0) pngs.push_back(stem + ".png");
        }
    }
    if (pngs.size() > 1 && haveCommand("montage")) {
        std::string cmd = "montage";
        for (const std::string& p : pngs) cmd += " '" + p + "'";
        cmd += " -tile 3x -geometry 1200x+12+12 -background '#f4f1ea' '" + (fs::path(outDir) / "contact.png").string() + "' >/dev/null 2>&1";
        if (std::system(cmd.c_str()) == 0) std::printf("contact sheet: %s\n", (fs::path(outDir) / "contact.png").string().c_str());
    }
    if (!inkscape) std::printf("(Inkscape not found: SVG maps only)\n");
    return 0;
}
