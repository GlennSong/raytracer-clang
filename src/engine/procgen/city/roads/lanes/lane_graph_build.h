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
// - JUNCTIONS are the paint's junction boxes (deck_mesh.h junctionSpans), found with the lane graph's own, looser
//   parallel test (kJunctionParallelForGraph: streets meeting at 35 degrees are a junction, not two roads side by
//   side; the paint keeps its own number): each street's lanes stop at the stop line kStopLineBack before the box and
//   start again as far beyond it; what lies between is the junction's, for the connectors. A junction's outline runs
//   round its stop lines. A lane of a road with no box of its own there (a ramp ending on a street, a freeway crossing
//   at grade) is cut where it crosses that outline; a ramp that ends on a street (no gore on a host lane) is the
//   junction's from that outline to its end, within kRampEndReach. An outline that crosses itself is REFUSED: reported,
//   and no connectors are made there. Each junction lists the lanes arriving at it and leaving it.
// - Where one road CARRIES ON AS ANOTHER with no junction (its end meets exactly one other road's end) and their lanes
//   just miss (more than kJoinDistance or kJoinDegrees apart, less than kNearMiss and kNearMissDegrees), the two meet at
//   a junction of their own, a CARRY-ON: each road gives up the stretch by its end it needs for a connector a vehicle
//   can drive (kCarryOnTrims), and the connectors there all go straight on. Where three or more road ends meet so (a
//   ramp ending where a street begins, beside the ramp that leaves it), they are a NODE junction, its turns stated as
//   at any junction. Where lanes meet exactly, a lane simply leads into the next road's lane.
// - What follows a lane is the lane it carries on as in the next section, the lane of the next road that begins exactly
//   where it ends, or (a lane arriving at a junction) the connectors leaving it (lane_graph_connect.h).
// - TIDYING, in the lane graph only (the builder's lanes are not touched): a lane on the inside of a corner of its road
//   sharper than its offset folds back on itself there; the fold is cut out so its distance along always moves forward
//   (reported, "tidied"). A section shorter than kShortestSection where a lane begins or ends (a turn pocket ending a
//   metre before the stop line) is merged into its neighbour, the lanes that begin or end there carried straight on
//   to the neighbour's cut.
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
// A section shorter than this is merged into its neighbour where a lane begins or ends at one of its cuts, and
// reported with its place where not (metres).
constexpr double kShortestSection = 2.0;
// The lane graph's parallel test (deck_mesh.h junctionCrossRank): a crossing lane within this (|cos| of the angle) of
// a street's direction runs beside it. About 18 degrees -- looser than the paint's 46, so streets meeting at 35 degrees
// and a ramp's end crossing a street are junctions, while a gore or an aux lane beside its host is not.
constexpr double kJunctionParallelForGraph = 0.95;
// A ramp that ends on a street (no gore on a host lane) belongs to the junction its lanes run into within this of its
// end (metres along the ramp): the stretch between the junction and the ramp's own end lies inside the street.
constexpr double kRampEndReach = 40.0;
// How far back from where they meet two roads carrying on as one give up to a carry-on junction (metres, each): the
// first that lets every connector there be as wide a bend as the widest-turning vehicle needs, else the last.
constexpr double kCarryOnTrims[] = {1.0, 2.0, 4.0, 8.0};
// ... but never more than this share of either road's length.
constexpr double kCarryOnTrimShare = 0.25;
// Bends gentler than this radius count as straight when a path's tightest radius is measured (metres).
constexpr double kStraightEnoughRadius = 1000.0;
// A lane point within this height of a junction's lane ends is at the junction's level (metres): a lane passing under
// or over a junction is not in it.
constexpr double kJunctionLevelReach = 2.0;
// One lane leads into another where its end is within this of the other's start (metres) ...
constexpr double kJoinDistance = 0.01;
// ... and the two run the same way within this (degrees).
constexpr double kJoinDegrees = 1.0;
// A lane ending nearer than this to where another begins, but not within the join tolerances, just misses it (metres)
// -- joined by a carry-on junction where its road carries on as one other road, else reported ...
constexpr double kNearMiss = 0.5;
// ... when the two run within this of the same way (degrees): a lane beside, not across. A carry-on's connectors are
// all straight on (turned through less than this).
constexpr double kNearMissDegrees = 45.0;

// WHICH WAY IS THE OFFSIDE, for a lane `normalOffset` from its road's spine (along the spine's left normal, perp())
// running `dir` (+1 along the spine, -1 against it): the larger the result, the nearer the middle of the road. The
// builder lays a road's forward lanes on the negative side of the spine's left normal and its backward lanes on the
// positive side, and names each direction's lanes from the median outwards (laneSlots: f0 and b0 are the innermost; a
// pocket's "inner" lane is the one with the largest offset x dir), so offset x dir grows toward the middle. In the
// world the plan is mirrored (plan y is world z), so a forward lane on the plan's right of its spine is on the driver's
// left: that is how this engine's roads keep left. Nothing else says which side is which.
inline double offsideness(double normalOffset, int dir) { return normalOffset * static_cast<double>(dir); }
// Which side of a lane its kerb lies, as a sign on perp(the lane's direction): a lane's direction is its spine's times
// dir, so a point along perp of it lies offside where offsideness says a positive offset does; the kerb is the other way.
inline double kerbPerpSign() { return offsideness(1.0, 1) > 0 ? -1.0 : 1.0; }

// THE LANE GRAPH BUILDER'S VERSION. A saved lane graph (lane_graph_cache.h) is keyed by the roads it was built from,
// the vehicles its connectors were checked against and this number, so BUMP IT whenever what buildLaneGraph,
// connectJunctions (lane_graph_connect.h) or findCrossingPoints (lane_graph_crossings.h) makes can change -- a new rule
// for which turns exist, a connector's shape, a tolerance above or there -- or every level in v2 keeps loading the graph
// the old code built. (A change to the roads themselves
// is the lanes tag's, kLanesBuildTag, which keys the roads; a change to the file format is LGRF's own version.)
// History: git log -G kLaneGraphBuildVersion.
constexpr uint32_t kLaneGraphBuildVersion = 2;   // 1: the graph of #237 and the connectors of #238 (ADR-0398)
                                                 // 2: crossing points (#239, ADR-0400)

// A VEHICLE TAG and how it turns: the tightest radius its path may have, and its width (metres). Passed in by the
// caller (the engine does not know the driving model's vehicles); a connector tighter than the radius is closed to it.
struct VehicleSize {
    std::string tag;
    double turningRadius = 0;
    double width = 0;
};

struct LaneGraphOptions {
    // Every lane is open to every one of these vehicle tags, until lane kinds carry who may use them (a later task).
    std::vector<VehicleSize> vehicles;
    // The control every junction carries.
    std::string control = kControlNotPlanned;
};

// What a junction was made from, and how it fared, by junction id.
enum class JunctionOrigin : uint8_t {
    Box,       // the paint's junction boxes, one or more merged
    CarryOn,   // where one road carries on as another and their lanes just miss
    Node,      // where three or more roads meet end to end with no box, and lanes just miss
};
struct JunctionNote {
    JunctionOrigin origin = JunctionOrigin::Box;
    Vec2 at;                    // the middle of its outline
    std::string roads;          // the roads that meet there, by id
    bool refused = false;       // no connectors: its outline is unusable (the problems say why)
    uint32_t turnsStated = 0;   // the turns junctionTurns states there
    uint32_t turnsBuilt = 0;    // ... made into connectors (some may be closed to some vehicles)
    uint32_t turnsRefused = 0;  // ... refused (each with a line in the problems; all of them when the junction is)
};
// A turn not built, and why, where.
struct Refusal {
    uint32_t junction = 0;
    PathId from, to;            // not valid when the whole junction is refused
    Turn turn = Turn::None;
    Vec2 at;
};

// HOW THE CROSSING POINTS CAME OUT (lane_graph_crossings.h findCrossingPoints), counted.
struct CrossingReport {
    uint32_t byMeeting[kMeetingCount] = {};   // records between two paths, by Meeting (cross, join, split)
    uint32_t withCrosswalks = 0;              // records of a path over a crosswalk
    uint32_t betweenJunctions = 0;            // ... of the path records: connectors of two junctions
    uint32_t withLanes = 0;                   // ... a connector and a lane
    uint32_t busiestJunction = kNoIndex;      // the junction with the most records filed under it (the lowest of equals)
    uint32_t busiestCount = 0;
    uint32_t junctionsWithout = 0;            // junctions with two or more connectors and no record
    std::vector<std::string> problems;        // anything that looks wrong, in plain words with its place
    double milliseconds = 0;
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
    std::vector<PathSource> sources;       // per path (a connector's: lane -1, no builder lane)
    std::vector<SectionSource> sectionSources;   // per section
    std::vector<double> insideJunctions;   // per builder lane: metres of it inside junction outlines (the connectors')
    std::vector<double> laneLengths;       // per builder lane: its length as the graph reads it (a fold tidied away)
    std::vector<char> leftOut;             // per builder lane: 1 = made into no lane path (a connector, a track, a footway)
    std::vector<JunctionNote> junctionNotes;   // per junction
    std::vector<Refusal> refusals;         // every turn not built, by junction then turn
    std::vector<std::pair<uint32_t, uint32_t>> unjoined;   // lane ends not joined: (the lane, the lane it just misses)
    uint32_t connectorsLeftOut = 0;        // builder connector lanes left out
    uint32_t otherLanesLeftOut = 0;        // track and footway lanes left out
    uint32_t mergedJunctions = 0;          // junction boxes that overlapped another junction's, made one
    uint32_t endsAtOutline = 0;            // arriving and leaving lanes cut at an outline: no stop line of their own
    uint32_t roadJoins = 0;                // lanes leading into the next road's lane where two roads meet end to end
    uint32_t crossingsOutside = 0;         // places two roads' lanes cross at one level outside every junction
    uint32_t tidiedLanes = 0;              // lanes whose fold at a sharp corner was cut out
    uint32_t mergedSections = 0;           // short sections merged into a neighbour
    uint32_t rampEnds = 0;                 // ramp ends on a street taken into the junction they run into
    uint32_t connectorsByTurn[kTurnCount] = {};   // connectors made, by Turn
    std::vector<uint32_t> closedTo;        // per vehicle tag name: connectors too tight for it (closed to it)
    // How much of the graph a car can get round (lane_graph_connect.h reachReport): the parts, the largest's share of
    // the LANE paths, and the lanes outside it grouped by why (why, lane ids in id order), in a fixed order.
    ReachableParts reach;
    double reachableShare = 0;
    std::vector<std::pair<std::string, std::vector<uint32_t>>> outside;
    CrossingReport crossings;              // the crossing points (their problems are in `problems` too)
    double milliseconds = 0;               // the whole of buildLaneGraph
    double boxMilliseconds = 0;            // ... of which finding the paint's junction boxes
    double connectMilliseconds = 0;        // ... of which the connectors (the crossing points: crossings.milliseconds)
};

// The lane graph of a built road network, its problems, and where each path came from.
LaneGraphBuild buildLaneGraph(const Result& r, const LaneGraphOptions& opts);

}  // namespace roads::lanes
}  // namespace engine

#endif
