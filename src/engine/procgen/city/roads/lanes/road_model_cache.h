#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_MODEL_CACHE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_ROAD_MODEL_CACHE_H

// A LEVEL'S BUILT ROAD MODEL, SAVED AND READ BACK (ADR-0429). The built model (road_model.h: lanes, connections, turns
// and the lane level the driving drives on) needs the whole road build, which a level loading from its bake does not
// have: the traffic lab's is about 12 s of road build and 4 s of lane level. So it is built once and saved, and every
// later load reads the file instead. This is the roads' one cache: nothing else about a level's roads is saved outside
// its bake.
//
// Where it lives: `cache/roadmodel/<key>.roadmodel`, beside `cache/levels` (the road bake) and `cache/population`, and
// like them a cache -- deleting it is always safe. The file is a bundle (bundle.h) holding the model as its RMDL bytes
// (the lane level inside them, road_model.h putRoadModel) and a short summary. It is not a section of the level's own
// bundle because its key needs the vehicles the turns were checked against, which come from the vehicle data the host
// reads (the fleet in vehicles.lua, through the driving library), and a bundle producer's key is worked out from the
// level file alone, before any host runs, by the loader and by rt_bake alike. For the same reason the loader publishes
// the planned layer, and the host that knows the fleet asks for the built model here.
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
// laneLevelKey), the turn table `turnTable` (laneTurnRules().hash) and `opts` (its vehicles in their order), by builder
// version `builderVersion` (a parameter so a test can show a bump changes it). Any of them changing makes another key.
uint64_t roadModelCacheKey(uint64_t roadsKey, uint64_t laneFacts, uint64_t turnTable, const LaneGraphOptions& opts,
                           uint32_t builderVersion = kLaneGraphBuildVersion);
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

// A LEVEL'S BUILT ROAD MODEL, from the cache or built.
struct LevelBuiltRoads {
    std::shared_ptr<RoadModel> model;         // null when there is none (`error` says why)
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
// THE BUILT ROAD MODEL of the level's first lane-built city: read from `cacheDir` when a file of this key is there and
// sound; otherwise the roads are built (the same graph and rule the road bake uses, city_producer.h loadCityGraph), the
// model is assembled from them with `opts` (assembleRoadModel), and the file is written. `cacheDir` empty: built, never
// read or written (RT_NOCACHE). A road file lacking a fact the lane level needs gives no model (`error` names what to
// run). A level with more than one city drives on the first; the rest are named in the notes.
LevelBuiltRoads obtainLevelRoadModel(const bundle::LevelInputs& in, const LaneGraphOptions& opts,
                                     const std::string& cacheDir);
// One line for the load's log and for `lanegraph?`: the summary, then where it came from and how long it took.
std::string describeLevelRoadModel(const LevelBuiltRoads& lg);

}  // namespace roads::lanes
}  // namespace engine

#endif
