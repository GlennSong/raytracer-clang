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

#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace engine::plan;

static int usage() {
    std::fprintf(stderr,
                 "usage: city_plan generate BRIEF.json OUT_DIR [--variants N]\n"
                 "       city_plan scene BRIEF.json OUT_SCENE.json [--no-ramps]\n"
                 "       city_plan brief\n"
                 "       city_plan level-world BRIEF.json      the level's terrain + water blocks that match the brief's world, and its towns' hubs\n");
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
        terrain["cdlod"] = {{"worldHalf", terrain.value("size", 3400.0) / 2}};
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
