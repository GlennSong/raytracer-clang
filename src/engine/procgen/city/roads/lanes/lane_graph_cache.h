#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CACHE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_CACHE_H

// A LEVEL'S LANE GRAPH, SAVED AND READ BACK (driving model, the step after #238). Building the lane graph needs the
// whole road build (lane footprints, deck heights), which a level loading from its bake does not have: the traffic
// lab's is about 12 s of road build and 4 s of graph. So the graph is built once and saved, and every later load reads
// the file instead.
//
// Where it lives, and why there: `cache/lanegraph/<key>.lanegraph`, beside `cache/levels` (the road bake) and
// `cache/population`, and like them a cache -- deleting it is always safe. The file is a bundle (bundle.h: a header,
// hashed sections, a table of contents) holding the graph as its LGRF section and a short summary. It is not a section
// of the level's own bundle because its key needs the vehicles the connectors were checked against, which come from
// the vehicle data the host reads (the fleet in vehicles.lua, through the driving library), and a bundle producer's key
// is worked out from the level file alone, before any host runs, by the loader and by rt_bake alike.
//
// THE KEY says everything the graph is made from:
//   - the ROADS: the city producer's own key for the level (city_producer.h), the same number that names the road
//     bake: the lanes tag (kLanesBuildTag), the road graph file's bytes, its terrain file, the level's sidewalk
//     override. New roads, new key -- and the lanes tag is never bumped for this. The control plan (each junction's
//     control, the signs, the posted limits: issue #240) is part of the road graph file, so a new plan is a new key;
//   - the VEHICLES the connectors were checked against, in order: each tag, its tightest turning radius and its width
//     (they decide which vehicles each connector is open to), and the control every junction carries when the road
//     graph plans none;
//   - the builder's version, kLaneGraphBuildVersion (lane_graph_build.h), bumped by hand when the builder's output
//     can change, and this file's own layout version.
// A file whose name, recorded key, section hashes or graph do not check out is refused with the reason, rebuilt and
// written again; it never crashes the load. Two builds of one key write the same bytes.
//
// Each read and write is recorded beside the file (`<file>.uses`, cache_use.h), so `rt_bake --prune` clears the files
// no level uses any more.

#include "engine/bundle/bake.h"
#include "engine/procgen/city/roads/lane_graph.h"
#include "engine/procgen/city/roads/lanes/lane_graph_build.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// Where the files go (relative to the folder the game runs from, like cache/levels and cache/population) ...
constexpr const char* kLaneGraphCacheDir = "cache/lanegraph";
// ... the kind their use records name (cache_use.h) ...
constexpr const char* kLaneGraphCacheKind = "lanegraph";
// ... and the layout of the file itself: which sections, which fields. Bump when that changes.
constexpr uint32_t kLaneGraphCacheFormat = 1;

// WHAT A LANE GRAPH HOLDS, counted: the load's report line and the `lanegraph?` answer. Saved with the graph, since the
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
// The counts of a graph just built.
LaneGraphSummary summarizeLaneGraph(const LaneGraphBuild& built);
// In words, one line: "4,000 paths (1,927 lanes, 1,997 connectors), 900 sections, 168 junctions, 50 turns refused, a
// car can get round 97.7% of the lane paths".
std::string describeLaneGraphSummary(const LaneGraphSummary& s);

// THE CACHE KEY of the lane graph built from roads `roadsKey` with `opts` (its vehicles in their order, and its
// control), by builder version `builderVersion` (a parameter so a test can show a bump changes it).
uint64_t laneGraphCacheKey(uint64_t roadsKey, const LaneGraphOptions& opts,
                           uint32_t builderVersion = kLaneGraphBuildVersion);
// The file for a key in `dir`: <dir>/<key as 16 hex digits>.lanegraph
std::string laneGraphCachePath(const std::string& dir, uint64_t key);

// SAVE: the graph and its summary as the file at `path`, written beside it under a temporary name and renamed into
// place, so a reader never sees half a file. False (and why in `err`) when it cannot be written; nothing is left.
bool writeLaneGraphCache(const std::string& path, uint64_t key, const LaneGraph& graph, const LaneGraphSummary& summary,
                         std::string* err);
// LOAD: the graph and summary from `path`, refused (false, and why in `err`; the outputs emptied) when the file is
// missing, is not a bundle, is cut short, records another key or layout, has a section whose bytes no longer match
// their hash, holds a graph LGRF's reader refuses (it validates), or a summary that does not agree with the graph.
bool readLaneGraphCache(const std::string& path, uint64_t key, LaneGraph& graph, LaneGraphSummary& summary,
                        std::string* err);

// THE ROADS' KEY for a level: the city producer's key (city_producer.h), or 0 when the level has no lane-built city.
uint64_t levelRoadsKey(const bundle::LevelInputs& in);

// A LEVEL'S LANE GRAPH, from the cache or built.
struct LevelLaneGraph {
    std::shared_ptr<const LaneGraph> graph;   // null when there is none (`error` says why)
    LaneGraphSummary summary;
    uint64_t key = 0;
    std::string path;                         // the cache file; empty when caching is off
    bool fromCache = false;                   // read from the file (no road build); else built here
    bool saved = false;                       // built and written to the file
    double seconds = 0;                       // the whole of obtainLevelLaneGraph
    double roadSeconds = 0;                   // when built: the road build ...
    double graphSeconds = 0;                  // ... and the lane graph from it
    std::string error;
    std::vector<std::string> notes;           // a file refused and rebuilt, a city not driven, a graph that fails its check
};
// THE LEVEL'S LANE GRAPH for its first lane-built city: read from `cacheDir` when a file of this key is there and
// sound; otherwise the roads are built (the same graph and rule the road bake uses, city_producer.h loadCityGraph), the
// lane graph is built from them with `opts`, and the file is written. `cacheDir` empty: built, never read or written
// (RT_NOCACHE). A level with more than one city drives on the first; the rest are named in the notes.
LevelLaneGraph obtainLevelLaneGraph(const bundle::LevelInputs& in, const LaneGraphOptions& opts,
                                    const std::string& cacheDir);
// One line for the load's log and for `lanegraph?`: the summary, then where it came from and how long it took.
std::string describeLevelLaneGraph(const LevelLaneGraph& lg);

}  // namespace roads::lanes
}  // namespace engine

#endif
