// THE SHIPPED LEVELS' WALKS AND THE SIM (#306 task 4, piece 6; ADR-0480): every lanes level hands the city sim its road
// model, and what its walks are.
//
// (DELETED in piece 6 W3: `level_walk_join_the_street_maps_lookups_beside_the_one_query`, ADR-0468's comparison of the
// street map's three look-ups -- placing a place, starting a walk, finishing one -- with the road model's one answer,
// RoadModel::walkJoinOn / walkJoinNear. The sim's walks now start and end at that one answer, and the street map's
// look-ups it compared against -- CitySim::walkStartNode, kerbSideLink, kerbLinkNear, the stop-short -- are deleted.)
//
//   RT_TEST_FILTER=level_every_lanes_level ./build-viewer/level_tests

#include "test_framework.h"

#include "../src/engine/procgen/city/roads/road_model.h"
#include "../src/engine/procgen/city/roads/lanes/city_producer.h"      // cityEntities: a level's lane-built cities
#include "../src/engine/procgen/city/roads/lanes/road_model_cache.h"   // obtainLevelRoadModel: the level's one road model
#include "../src/engine/bundle/bake.h"                                  // loadLevelInputs
#include "../src/engine/components.h"                                   // LevelRoadModel::builtCity
#include "../src/apps/citysim/city_sim.h"

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace engine;

// EVERY LANES LEVEL HANDS THE SIM ITS ROAD MODEL (ADR-0480; #306 piece 6, W1), whether or not it drives: the built model
// (its lane level, with the walks people will walk by) goes to the sim by the same two calls the city host makes
// (CityRenderSystem::build: LevelRoadModel::builtCity, then CitySim::setRoads), on the model the loader publishes
// (obtainLevelRoadModel, from its cache or built). Every shipped level with a lane-built city and a city sim is asked,
// and the ones that have no built model to hand are named.
TEST_CASE(level_every_lanes_level_hands_the_sim_its_road_model) {
    // Glenn's levels in progress (untracked in the repository): not shipped, and not read here
    const std::set<std::string> inProgress = {"island_8_beach.json", "metro_rivers.json", "zz_mtn_window.json"};
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::directory_entry& f : std::filesystem::directory_iterator(std::string(RT_SOURCE_DIR) + "/assets/levels"))
        if (f.path().extension() == ".json" && !inProgress.count(f.path().filename().string())) files.push_back(f.path());
    std::sort(files.begin(), files.end());
    int lanesLevels = 0, handed = 0, withWalks = 0;
    std::vector<std::string> notHanded;
    std::map<std::string, std::size_t> walksOf;   // per level handed: its walks
    for (const std::filesystem::path& p : files) {
        bundle::LevelInputs in;
        std::string err;
        if (!bundle::loadLevelInputs(p.string(), in, &err) || !in.level.contains("citysim")) continue;
        if (roads::lanes::cityEntities(in.level).empty()) continue;
        ++lanesLevels;
        const roads::lanes::LevelBuiltRoads built = roads::lanes::obtainLevelRoadModel(in, roads::lanes::levelRoadModelCacheDir());
        LevelRoadModel published;
        published.cities = built.cities;
        published.laneLevelReport = roads::lanes::describeLevelRoadModel(built);
        citysim::CitySim sim;
        sim.setRoads(published.builtCity(), published.laneLevelReport);
        const RoadModel* m = sim.roads();
        const std::size_t walks = m ? m->laneLevel.walks.size() : 0;
        handed += m ? 1 : 0;
        withWalks += walks > 0 ? 1 : 0;
        if (!m) notHanded.push_back(p.filename().string() + " (" + (built.error.empty() ? "no built model" : built.error) + ")");
        std::printf("    [lanes levels] %-24s %s; %zu walks; %s\n", p.filename().string().c_str(),
                    m ? "the sim holds its road model" : "NO ROAD MODEL", walks, built.fromCache ? "from the cache" : "built");
        for (const std::string& n : built.notes) std::printf("        note: %s\n", n.c_str());
        if (!m) continue;
        walksOf[p.filename().string()] = walks;
        // what the walks are, and how much of the sidewalks one walker can get round (the parts on foot, ADR-0466)
        const LaneGraph& G = m->laneLevel;
        std::map<std::string, uint32_t> kinds;
        for (uint32_t w = 0; w < G.walks.size(); ++w) ++kinds[m->walkKind(WalkId(w))];
        const ReachableParts parts = reachableWalks(G, m->openTo("walker").walks);
        uint32_t sidewalks = 0, inLargest = 0;
        for (uint32_t w = 0; w < G.walks.size(); ++w) {
            if (m->walkKind(WalkId(w)) != "sidewalk") continue;
            ++sidewalks;
            inLargest += parts.partOf[w] == parts.largest ? 1u : 0u;
        }
        std::printf("        ");
        for (const std::pair<const std::string, uint32_t>& k : kinds) std::printf("%u %s, ", k.second, k.first.c_str());
        std::printf("%zu lane paths; %zu parts on foot, %u of %u sidewalks (%.1f%%) in the largest\n", G.paths.size(),
                    parts.sizes.size(), inLargest, sidewalks, sidewalks ? 100.0 * inLargest / sidewalks : 0.0);
    }
    std::printf("    [lanes levels] %d levels with a lane-built city and a city sim; %d hand the sim their road model, %d with walks\n",
                lanesLevels, handed, withWalks);
    for (const std::string& n : notHanded) std::printf("        not handed: %s\n", n.c_str());
    // Every level whose first city IS built hands it over, walks and all. The rest have no built model to hand: their
    // road files do not yet state what the lane level needs (lane rules, junction arms in ring order, and on rail_lab every
    // junction's control; the line above names the tool run for each), so the loader publishes only the planned layer and
    // there are no walks to walk. Measured 2026-11-21: 13 levels, 6 handed. The 7 left stay without walks by Glenn's
    // decision (metro_planned is his work in progress). island_8_nature's road file was given its lane facts once (ADR-0480
    // W1b) and its j396 stated as the two junctions it is (ADR-0480 "W1b, j396"): 14,573 walks. When a level gains its
    // lane level, it moves from the list below to the handed ones: take it out of the list.
    const std::set<std::string> roadFileStatesNoLaneLevel = {"light_rail_lab.json", "metro_mountain.json", "metro_planned.json",
                                                             "metro_v2_test.json",  "piedmont.json",       "rail_lab.json",
                                                             "wheel_lab.json"};
    std::set<std::string> notHandedNames;
    for (const std::string& n : notHanded) notHandedNames.insert(n.substr(0, n.find(' ')));
    CHECK(lanesLevels == 13);
    CHECK(handed == 6 && withWalks == handed);
    CHECK(notHandedNames == roadFileStatesNoLaneLevel);
    CHECK(walksOf.count("island_8_nature.json") == 1u && walksOf["island_8_nature.json"] == 14573u);
}
