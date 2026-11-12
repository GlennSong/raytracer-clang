#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_BUILD_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_BUILD_H

// THE LANE GRAPH FROM THE BUILT ROADS (issue #237, driving-model-design.md 5.2): reads the lanes the road builder laid
// and cuts them into the lane graph's lanes and sections, and records its junctions. It reads; it never changes what the
// builder made.
//
// - A CORRIDOR is one road's lanes running one way, plus the stretch of a ramp's lane that runs beside it (the ramp's
//   deceleration lane before the gore where it leaves, or its merge lane after the gore where it joins: the builder's
//   ramp anchors say which host lane). Distances along a corridor are measured along its road's spine, in the way the
//   traffic runs.
// - A corridor is CUT wherever a lane in it begins or ends (a turn pocket opening, a lane dropping, a ramp's lane
//   arriving or leaving), at every junction, and where the approach to a stop line begins (the lines there are solid).
//   Between two cuts every lane runs the whole way: one SECTION, its lanes listed from the offside to the kerb.
// - JUNCTIONS are the paint's junction boxes (deck_mesh.h junctionSpans): each street's lanes stop at the stop line
//   kStopLineBack before the box and start again as far beyond it; what lies between is the junction's, left for the
//   connectors (the next task). A junction's outline is the convex hull of its stop lines. A lane of a road with no box
//   of its own there (a ramp ending on a street, a freeway crossing at grade) is cut where it crosses that outline.
//   Each junction lists the lanes arriving at it and leaving it.
// - What follows a lane is the lane it carries on as in the next section, or the lane of the next road that begins
//   exactly where it ends (within kJoinDistance and kJoinDegrees). Nothing follows a lane arriving at a junction yet.
// - Not turned into lane paths: the builder's hand-declared connector lanes, and the lanes of track and footway classes
//   (no car drives them).
//
// It needs the whole build (the junction boxes come from the lane footprints and deck heights): seconds on a city, so
// it is not run at level load (see the report on #237).

#include "engine/procgen/city/roads/lane_graph.h"
#include "engine/procgen/city/roads/lanes/lanes.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// The control every junction carries until the control plan names one (a later task). Not a kind of control: the
// absence of a plan, so whatever reads it can say so.
constexpr const char* kControlNotPlanned = "not_planned";
// How far before a junction's stop line the lines between lanes are solid (metres). To become rule-set data.
constexpr double kSolidBeforeStopLine = 20.0;
// A section shorter than this is reported with its place (metres).
constexpr double kShortestSection = 2.0;
// Bends gentler than this radius count as straight when a path's tightest radius is measured (metres).
constexpr double kStraightEnoughRadius = 1000.0;
// A lane point within this height of a junction's lane ends is at the junction's level (metres): a lane passing under
// or over a junction is not in it.
constexpr double kJunctionLevelReach = 2.0;
// One lane leads into another where its end is within this of the other's start (metres) ...
constexpr double kJoinDistance = 0.01;
// ... and the two run the same way within this (degrees).
constexpr double kJoinDegrees = 1.0;

struct LaneGraphOptions {
    // Every lane is open to every one of these vehicle tags, until lane kinds carry who may use them (a later task).
    std::vector<std::string> vehicleTags;
    // The control every junction carries.
    std::string control = kControlNotPlanned;
};

// Which builder lane a lane path was cut from, and the stretch of it: metres along the builder lane in the way cars
// drive it.
struct PathSource {
    int lane = -1;
    double from = 0, to = 0;
};

// Where a section lies: the road (graph edge index) whose spine it is measured along, which way its traffic runs (+1
// along the spine), and its start and end in metres along the spine the way the traffic runs.
struct SectionSource {
    int edge = -1, dir = 1;
    double from = 0, to = 0;
};

struct LaneGraphBuild {
    LaneGraph graph;
    std::vector<std::string> problems;     // in plain words, each with its place; empty = nothing to look at
    std::vector<PathSource> sources;       // per lane path
    std::vector<SectionSource> sectionSources;   // per section
    std::vector<double> insideJunctions;   // per builder lane: metres of it inside junction outlines (the connectors')
    std::vector<char> leftOut;             // per builder lane: 1 = made into no lane path (a connector, a track, a footway)
    uint32_t connectorsLeftOut = 0;        // builder connector lanes left out
    uint32_t otherLanesLeftOut = 0;        // track and footway lanes left out
    uint32_t mergedJunctions = 0;          // junction boxes that overlapped another junction's, made one
    uint32_t endsAtOutline = 0;            // arriving and leaving lanes cut at an outline: no stop line of their own
    uint32_t roadJoins = 0;                // lanes leading into the next road's lane where two roads meet end to end
    uint32_t crossingsOutside = 0;         // places two roads' lanes cross at one level outside every junction
    double milliseconds = 0;               // the whole of buildLaneGraph
    double boxMilliseconds = 0;            // ... of which finding the paint's junction boxes
};

// The lane graph of a built road network, its problems, and where each path came from.
LaneGraphBuild buildLaneGraph(const Result& r, const LaneGraphOptions& opts);

}  // namespace roads::lanes
}  // namespace engine

#endif
