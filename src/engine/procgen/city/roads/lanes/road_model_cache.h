#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_MODEL_CACHE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_MODEL_CACHE_H

// A LEVEL'S ROAD MODEL, MADE IN ONE PLACE (ADR-0435), ITS BUILT LAYER SAVED AND READ BACK (ADR-0429).
// `obtainLevelRoadModel` is the one function that makes the road model of a level. The loader calls it once a load and
// publishes what it returns (components.h LevelRoadModel); the lot pass and the driving read that object. The bake's
// lot producer and the offline tracer, which run without the loader, call the same function.
//
// The built model (road_model.h: lanes, connections, turns and the lane level the driving drives on) needs the whole
// road build, which a level loading from its bake does not have: the traffic lab's is about 12 s of road build and 4 s
// of lane level. So it is built once and saved, and every later load reads the file instead. This is the roads' one
// cache: nothing else about a level's roads is saved outside its bake.
//
// Where it lives: `cache/roadmodel/<key>.roadmodel`, beside `cache/levels` (the road bake) and `cache/population`, and
// like them a cache -- deleting it is always safe. The file is a bundle (bundle.h) holding the model as its RMDL bytes
// (the lane level inside them, road_model.h putRoadModel) and a short summary.
//
// THE VEHICLES the turns are checked against are the level's own fleet, read here from the vehicle script the level
// names (`citysim.vehicles`, `citysim.fleet`; levelVehicles below), so the key needs no host: the loader, rt_bake and a
// tool make the same key and the same model.
//
// THE ONE KEY says everything the built model is made from:
//   - the ROADS: the city producer's own key for the level (city_producer.h), the same number that names the road
//     bake: the lanes tag (kLanesBuildTag), the road graph file's bytes, its terrain file, the level's sidewalk
//     override;
//   - the road file's LAYOUT AND LANE FACTS, which the city's bake key does not see (road_layout.h laneLevelKey): its
//     junctions (arms, through pairs, controls, signs), streets, divided pairs and "lane_rules" (ADR-0426);
//   - the TURN TABLE's own hash (assets/data/lane_turns.json, lane_turns.h);
//   - the VEHICLES the turns were checked against, in order: each tag, its tightest turning radius, its width and its
//     length;
//   - the builder's version, kLaneGraphBuildVersion (lane_graph_build.h), bumped by hand when what is built can change,
//     and the layout of the model's bytes, kRoadModelFormat.
// A file whose name, recorded key, section hashes or model do not check out is refused with the reason, rebuilt and
// written again; it never crashes the load. Two builds of one key write the same bytes.
//
// Each read and write is recorded beside the file (`<file>.uses`, cache_use.h), so `rt_bake --prune` clears the files
// no level uses any more.

#include "engine/bundle/bake.h"
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"
#include "engine/procgen/city/roads/road_model.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// Where the files go (relative to the folder the game runs from, like cache/levels and cache/population) ...
constexpr const char* kRoadModelCacheDir = "cache/roadmodel";
// ... and the kind their use records name (cache_use.h).
constexpr const char* kRoadModelCacheKind = "roadmodel";

// WHAT A LANE LEVEL HOLDS, counted: the load's report line and the `lanegraph?` answer. Saved with the model, since the
// refused turns are the builder's to know (a refused turn leaves no path behind).
struct LaneGraphSummary {
    uint32_t paths = 0;          // lanes and connectors
    uint32_t lanes = 0;
    uint32_t connectors = 0;
    uint32_t sections = 0;
    uint32_t junctions = 0;
    uint32_t refusedTurns = 0;   // turns a junction offers that no vehicle could make (lane_graph_connect.h)
    uint32_t problems = 0;       // the builder's problem lines (refusals among them)
    double reachableShare = 0;   // the largest part a car can get round, as a share of the lane paths (0..1)
};
bool operator==(const LaneGraphSummary& a, const LaneGraphSummary& b);
// The counts of a lane level just built, and what building it reported.
LaneGraphSummary summarizeLaneGraph(const LaneGraph& graph, const LaneGraphBuild& built);
// In words, one line: "4,000 paths (1,927 lanes, 1,997 connectors), 900 sections, 168 junctions, 50 turns refused, a
// car can get round 97.7% of the lane paths".
std::string describeLaneGraphSummary(const LaneGraphSummary& s);

// THE KEY of the model built from roads `roadsKey`, the road file's layout and lane facts `laneFacts` (road_layout.h
// laneLevelKey), the data tables the model is made by `tables` (roadModelTablesHash) and `opts` (its vehicles in their
// order), by builder version `builderVersion` (a parameter so a test can show a bump changes it). Any of them changing
// makes another key.
uint64_t roadModelCacheKey(uint64_t roadsKey, uint64_t laneFacts, uint64_t tables, const LaneGraphOptions& opts,
                           uint32_t builderVersion = kLaneGraphBuildVersion);
// THE DATA TABLES A SAVED MODEL IS MADE BY, as one number for its key: the turn table's hash (laneTurnRules().hash:
// which lane takes which turn, ADR-0426) and the kerb parking rules' (kerbParkingRules().hash: how a parking band is cut
// into spaces, ADR-0439) and the path users' (pathUsers().hash: who may use which kind of path, and how a dead end is
// joined, ADR-0464), each the hash of its file's bytes. Editing any of them makes every saved model again.
uint64_t roadModelTablesHash(uint64_t turnTable, uint64_t kerbParking, uint64_t pathUsers);
// The file for a key in `dir`: <dir>/<key as 16 hex digits>.roadmodel
std::string roadModelCachePath(const std::string& dir, uint64_t key);

// SAVE: the model and its summary as the file at `path`, written beside it under a temporary name and renamed into
// place, so a reader never sees half a file. False (and why in `err`) when it cannot be written; nothing is left.
bool writeRoadModelCache(const std::string& path, uint64_t key, const RoadModel& model, const LaneGraphSummary& summary,
                         std::string* err);
// LOAD: the model and summary from `path`, refused (false, and why in `err`; the outputs emptied) when the file is
// missing, is not a bundle, is cut short, records another key, has a section whose bytes no longer match their hash,
// holds a model its reader refuses (the lane level is checked: validate), or a summary that does not agree with it.
bool readRoadModelCache(const std::string& path, uint64_t key, RoadModel& model, LaneGraphSummary& summary,
                        std::string* err);

// THE ROADS' KEY for a level: the city producer's key (city_producer.h), or 0 when the level has no lane-built city.
uint64_t levelRoadsKey(const bundle::LevelInputs& in);

// THE VEHICLES A LEVEL'S ROADS ARE CHECKED AGAINST: each vehicle tag's most demanding body over the level's fleet
// (engine/scripting/fleet_lua.h fleetVehicleSizes), read from the vehicle script the level names (`citysim.vehicles`,
// its fleet `citysim.fleet`). Empty when the level names no script, the script fails or the build has no scripting;
// `notes` says which, and names every slot left out.
LaneGraphOptions levelVehicles(const bundle::LevelInputs& in, std::vector<std::string>* notes = nullptr);

// Where the loader, the bake and the tools keep the files: kRoadModelCacheDir, or empty (build, never read or write)
// when RT_NOCACHE is set.
std::string levelRoadModelCacheDir();

// A LEVEL'S ROAD MODEL, from the cache or built.
struct LevelBuiltRoads {
    // THE LEVEL'S ROAD MODEL: one per lane-built city, in the order the level lists them. The first city's is the BUILT
    // model (`model` below) when its road file states what the lane level needs; otherwise, and for every later city,
    // the planned layer read from the road file. An entry is null only when its road file could not be read.
    std::vector<std::shared_ptr<RoadModel>> cities;
    std::shared_ptr<RoadModel> model;         // the first city's, when it is built (its lane level in it); else null (`error` says why)
    LaneGraphOptions vehicles;                // what its turns were checked against
    LaneGraphSummary summary;                 // of its lane level
    uint64_t key = 0;
    std::string path;                         // the cache file; empty when caching is off
    bool fromCache = false;                   // read from the file (no road build); else built here
    bool saved = false;                       // built and written to the file
    double seconds = 0;                       // the whole of obtainLevelRoadModel
    double roadSeconds = 0;                   // when built: the road build ...
    double graphSeconds = 0;                  // ... and the model with its lane level from it
    std::string error;
    std::vector<std::string> notes;           // a file refused and rebuilt, a city not driven, a lane level that fails its check
};
// THE LEVEL'S ROAD MODEL (ADR-0435): the one function that makes it. The first lane-built city's built model is read
// from `cacheDir` when a file of this key is there and sound; otherwise the roads are built (the same graph and rule the
// road bake uses, city_producer.h loadCityGraph), the model is assembled from them (assembleRoadModel) and the file is
// written. `cacheDir` empty: built, never read or written (RT_NOCACHE). A road file lacking a fact the lane level needs
// is told so before any road is built (`error` names what to run) and gets the planned layer. A level with more than
// one city drives on the first; the rest get their planned layer and are named in the notes.
// `vehicles`: null, the level's own fleet (levelVehicles); a test names its own.
LevelBuiltRoads obtainLevelRoadModel(const bundle::LevelInputs& in, const std::string& cacheDir,
                                     const LaneGraphOptions* vehicles = nullptr);
// THE LEVEL'S PLANNED ROAD MODEL: each lane-built city's planned layer, read from its road file alone (no road build, no
// cache), in the order the level lists them; null where a file could not be read (`notes` says which). WHAT THE LOT PASS
// READS (ADR-0476): the lots are grown before the built model is assembled, and their paths are stated on it after.
std::vector<std::shared_ptr<RoadModel>> levelPlannedRoadModels(const bundle::LevelInputs& in, std::vector<std::string>* notes = nullptr);
// One line for the load's log and for `lanegraph?`: the summary, then where it came from and how long it took.
std::string describeLevelRoadModel(const LevelBuiltRoads& lg);

}  // namespace roads::lanes
}  // namespace engine

#endif
