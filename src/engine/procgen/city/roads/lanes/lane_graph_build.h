#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_BUILD_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_LANE_GRAPH_BUILD_H

// THE LANE GRAPH'S BUILD: WHAT IS MEASURED, AND WHAT A BUILD REPORTS (issue #237, driving-model-design.md 5.2). Since
// ADR-0426 the lane graph is a table built from the road model (lane_level.h buildLaneLevel: paths, sections, junctions,
// next links, controls and signs by the model's ids). What stays here is what the design keeps as measurement on top of
// it, and the shapes they share:
//   - how tightly each path bends, as a vehicle feels it (measureBends), and the kinks a driver would feel (findKinks);
//   - the tidying of a lane on the inside of a corner sharper than its offset, which folds back on itself (cutFolds);
//   - which side of a lane its kerb is (offsideness, kerbPerpSign): the one statement of it;
//   - the vehicles a build is checked against (VehicleSize), and what a build reports (LaneGraphBuild).
// The paths through junctions (connector shapes, the corner-fit test) are lane_graph_connect.h; where two paths' bodies
// could touch, lane_graph_crossings.h.

#include "engine/procgen/city/roads/lane_graph.h"
#include "engine/procgen/city/roads/lanes/lanes.h"
#include "engine/procgen/city/roads/road_problem.h"   // RoadProblem: a problem as a record (ADR-0438)
#include "engine/procgen/city/roads/vehicle_size.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {
namespace roads::lanes {

// The ramp-landing check's parallel test (lanes.cpp checkRampLandings, its one reader since ADR-0426: the lane graph
// worked out from the built roads used it too, and is gone): a ramp's bare end within this (|cos| of the angle) of a
// street's direction runs along it, and is judged; one at a wider angle is an arm of a junction there. About 18 degrees,
// looser than the paint's 46 (deck_mesh.h kJunctionParallel). Its name is from that old reader.
constexpr double kJunctionParallelForGraph = 0.95;
// How far a ramp that ends on a street (no gore on a host lane) may run on inside the street's lanes (metres along the
// ramp): what the control planner reads to find where such a ramp comes out of the street's lanes, the junction's place
// (control_plan.cpp findJunctions, its one reader since ADR-0426).
constexpr double kRampEndReach = 40.0;
// Bends gentler than this radius count as straight when a path's tightest radius is measured (metres).
constexpr double kStraightEnoughRadius = 1000.0;

// HOW A BEND IS MEASURED: THE BEND A VEHICLE FEELS (issue #281). A lane line is a chain of points, and the heading turns
// only where two pieces of the chain meet. Read from neighbouring points, a turn of a fraction of a degree between two
// points a few centimetres apart is a hairpin; a driver never feels it, because a driver does not follow the drawn line
// to the centimetre. So a bend is measured as a driver takes it:
//   1. Each point's turn is spread evenly over its share of the line (halfway back to the point before, halfway on to
//      the point after, each at most kBendSpread). Round a circle drawn with points up to a metre apart, at any
//      spacing, this gives exactly one over its radius everywhere; a long straight segment carries no bend, so a sharp
//      turn at its end stays sharp.
//   2. The bend at a place is that turning averaged over a stretch centred there (shifted to fit inside the path, the
//      whole path when it is shorter): the turn over the stretch, divided by its length, is one over the radius.
//   3. How long the stretch is follows from the room a driver has: a driver whose line strays no more than kBendRoom
//      from the drawn one takes a turn spread over any stretch with a sagitta that small as one smooth curve. A circle
//      of radius R strays from its chord by kBendRoom across 2 * sqrt(2 * kBendRoom * R) of it, so the stretch used is
//      the shortest (of kBendMeasureLengths) at least that long for the radius read over it: 2 m round a 5 m corner, 16
//      m round a 300 m curve, and up to 28 m, where the radius read is kStraightEnoughRadius. A kink of angle a read
//      this way is a curve of at least 8 * kBendRoom / a^2 (a little more, as the stretches come in steps): a quarter
//      of a degree reads as straight, 2 degrees as about 800 m, 3 degrees about 300 m, 5 degrees about 130 m.
//   4. Every radius is laid over the part of the stretch it was read over where the line turns, the tightest wherever
//      readings overlap, so a bend is reached at its own radius from where it begins, and a straight is not slowed by
//      a bend at the far end of a long stretch.
// Why not one fixed length: a stretch long enough to read the lab's 0.4-degree jitter as straight (10 m) reads its 5 m
// corners (a third of its connectors, under 8 m of arc) up to a quarter gentler than they are; one short enough for the
// corners reads the jitter as a bend.
//
// The room (metres): a hand's width, well inside the room a lane leaves beside a car (a 3.7 m lane, a 1.9 m car). It is
// the shape's, so it lives with the builder, not in a rule set: the bend profile is baked with the lane graph (design
// 5.1, the shape layer) and every rule set reads the same one.
constexpr double kBendRoom = 0.1;
// How far either side of a point its turn is spread, at most (metres): half the builder's point spacing of about a
// metre. Further apart, the segments between points are straight and carry no bend of their own.
constexpr double kBendSpread = 0.5;
// The stretches a bend is read over, shortest first, each sqrt(2) longer (metres): from a metre (a turn inside a metre is
// taken over a metre) to the length that reads kStraightEnoughRadius (2 * sqrt(2 * kBendRoom * kStraightEnoughRadius),
// 28.3 m): a turn that reads gentler than that over it is straight.
constexpr double kBendMeasureLengths[] = {1.0, 1.414, 2.0, 2.828, 4.0, 5.657, 8.0, 11.31, 16.0, 22.63, 28.29};
// Where along a path the bend is read: every this many metres (and at both ends). Each reading covers a stretch at least
// a metre long, so nothing between two readings is missed.
constexpr double kBendSampleStep = 1.0;
// Neighbouring stretches whose radii differ by no more than this share are kept as one piece, at the tighter radius
// (so a car slows at most 5% more than it need on a curve whose radius changes along it, and a path keeps a handful
// of pieces).
constexpr double kBendPieceShare = 0.1;
// THE BEND PROFILE OF A CENTRE LINE (points and their distances along it, rising; at least two points): pieces from 0,
// in order, each with the tightest radius read anywhere over it (kNoBend where it runs straight), neighbours of
// different radii. Points closer together than kMinBendChord to the one kept before are passed over. A line with no
// bend at all is one straight piece.
std::vector<BendPiece> measureBends(const Vec2* xy, const double* along, std::size_t count);

// A KINK: a place where a line's heading jumps by more than kKinkDegrees at one point, beyond what a smooth line turns
// there (the bend just before the point over its share of the line before it, plus the bend just after it over its
// share after, each read at the neighbouring point: round a circle that is exactly the point's turn; where a side has
// no such point -- the line's end, or a neighbour that is itself a kink -- the smaller jump of that side running
// straight or carrying on the other side's bend). The
// builder reports each with its place and never smooths it away; the bend profile slows for it. Three degrees: read as
// a bend (step 3 above) a 3-degree jump is a curve of about 300 m, which at a comfortable 2 m/s2 sideways holds a car to
// about 55 mph, under a freeway's 65; a jump under about 2.6 degrees slows nobody below 65 mph.
constexpr double kKinkDegrees = 3.0;
struct Kink {
    double along = 0;      // metres along the line
    double degrees = 0;    // how far the heading jumps there, beyond the bend around it (a turn either way, unsigned)
    Vec2 at;               // where, in plan
};
// The kinks of a centre line (as measureBends reads its points), in order along it.
std::vector<Kink> findKinks(const Vec2* xy, const double* along, std::size_t count);

// CUTS THE FOLD OUT OF A LANE that runs backwards for a stretch (on the inside of a corner of its road sharper than its
// offset from the road's spine, the builder's offset points loop back on themselves): `forward[k]` is the way its road
// runs at point k. Where the lane before a backward run crosses the lane after it, the lane runs to that crossing and
// on from it; where it does not, every point that would take it backwards is dropped. `u` (each point's distance
// along its road) is kept in step. Returns whether anything was cut. In the lane graph's copy only: the builder's lanes
// are never touched (the lane graph reports each lane it tidies).
bool cutFolds(std::vector<Vec2>& xy, std::vector<double>& u, const std::vector<Vec2>& forward);

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

// THE LANE GRAPH BUILDER'S VERSION. A saved road model (road_model_cache.h) is keyed by the roads it was built from,
// the vehicles its connectors were checked against and this number, so BUMP IT whenever what buildLaneLevel
// (lane_level.h), fitConnector (lane_graph_connect.h) or findCrossingPoints (lane_graph_crossings.h) makes can change --
// a connector's shape, a tolerance there -- or every level in v2 keeps loading the lane level the old code built. (A change to
// the roads themselves is the lanes tag's, kLanesBuildTag, which keys the roads; to the road file's layout, lane rules
// or the turns' table, the cache key's own; a change to the file format is LGRF's own version.)
// History: git log -G kLaneGraphBuildVersion.
constexpr uint32_t kLaneGraphBuildVersion = 16;  // 16: a crosswalk's level is its junction's whole span of lane-end
                                                 //     heights, so a path falling across a sloping junction crosses it
                                                 //     once (ADR-0458)
                                                 // 14, 15: the road model's crosswalks laid in, and the paths crossing
                                                 //     each found (road_model.h layLaneCrosswalks, ADR-0452); 15: one
                                                 //     whose kerb end lies in another road is not one people use
                                                 // 13: a lane the kerb parking band overlaps is driven along the middle
                                                 //     of the rest of it, that much narrower (road_graph_spec.h
                                                 //     drivenLine, ADR-0450, #302)
                                                 // 12: one tolerance for stations written to the centimetre (rt_math.h
                                                 //     kWrittenStation, 7.5 mm where the lane level had 1 cm); a ramp's
                                                 //     merge is where the builder laid it (ADR-0433)
                                                 // 11: a far turn waits inside facing the approach its junction STATES as
                                                 //     its through partner; one road half width (ADR-0432)
                                                 // 1: the graph of #237 and the connectors of #238 (ADR-0398)
                                                 // 2: crossing points (#239, ADR-0400)
                                                 // 3: the control plan carried through: controls, signs, limits (#240)
                                                 // 4: each path's bend profile, measured as a vehicle feels it (#281)
                                                 // 5: far turns' places to wait inside the junction (#261, ADR-0406)
                                                 // 6: bodies swept at their length: the paved-road test and the
                                                 //    crossing points' reach (#284 item 9, ADR-0412)
                                                 // 7: a table built from the road model (lane_level.h, ADR-0426)
                                                 // 8: ADR-0426's review: a street's end at a junction with no stop line
                                                 //    for its arm is joined to nothing across it; an arm whose lanes' ways
                                                 //    across cancel is left out of the outline; turns placed by junction
                                                 //    (no shipped level changes)
                                                 // 9: each stated merge marked on its Join (CrossingPoint::merging)
                                                 // 10: turns and outlines read the junction's stated ring (ADR-0427)

// (A vehicle tag and its body, VehicleSize: roads/vehicle_size.h. A level's are read from its fleet by
// road_model_cache.h levelVehicles.)
struct LaneGraphOptions {
    // Every lane is open to every one of these vehicle tags, until lane kinds carry who may use them (a later task).
    std::vector<VehicleSize> vehicles;
};

// THE STATED CONTROLS AND SIGNS, counted as the lane level carries them (every junction's control is the road file's).
struct ControlPlanReport {
    uint32_t planned = 0;                        // stated junctions
    std::vector<std::pair<std::string, uint32_t>> byControl;   // lane-level junctions per control, by name
    std::vector<std::pair<std::string, uint32_t>> regulations; // regulations stood, per kind, by name
    uint32_t lanesPosted = 0;                    // lane paths with a posted limit
    uint32_t connectorsPosted = 0;               // connectors with a posted limit (the lane they join's)
};

// How a junction fared, by lane-level junction id.
struct JunctionNote {
    Vec2 at;                    // the stated junction's place
    std::string roads;          // the roads of its arms, by id
    uint32_t turnsStated = 0;   // the turns the road model states there
    uint32_t turnsBuilt = 0;    // ... made into connectors (some may be closed to some vehicles)
    uint32_t turnsRefused = 0;  // ... with no connector: fitting no vehicle (a corner fault), or with no lane path to take them
};
// A stated turn with no connector, why, and where: its corner fits no vehicle (a corner fault for every class), or there
// is no lane path to start it from or end it on (the lanes stop short of the junction: not the corner's fault). `from` and
// `to` are the lane paths it would join; with no lane path, the missing one is invalid.
enum class RefusalWhy : uint8_t { FitsNoVehicle, NoLanePath };
struct Refusal {
    uint32_t junction = 0;
    PathId from, to;
    Turn turn = Turn::None;
    RefusalWhy why = RefusalWhy::FitsNoVehicle;
    uint32_t modelTurn = kNoIndex;   // the road model's turn (RoadModel::turns)
    Vec2 at;
};
// A STRETCH OF A LANE CARS DRIVE THAT NO LANE PATH COVERS, in metres along the lane the way cars drive it (PathSource's
// measure), and its own cause, recorded where the stretch was left out: inside a stated junction (`junction`, the model's:
// between its street's end there and the arm's stop line, or on a stretch of road the file states inside it), or said in
// `why` -- a street with no room between its stop lines (a problem line), a hole of a centimetre or two between two of
// the lane's streets' stated stations (the file's rounding: listed, no problem line), or nothing recorded (a problem
// line). Every such lane is covered end to end by its lane paths and these.
struct LaneGap {
    uint32_t lane = 0;            // the builder's lane
    double from = 0, to = 0;
    uint32_t junction = kNoIndex; // the model junction it lies inside; kNoIndex: see `why`
    std::string why;
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
    uint32_t waitInsidePoints = 0;            // far turns given a place to wait inside (placeWaitInsidePoints)
    // LONG BODIES (ADR-0412): records of two paths found only because a long body cuts inside a bend, and records made
    // longer by it (counted before records of one pair are merged)
    uint32_t onlyForLength = 0, widenedForLength = 0;
    uint32_t forSomeVehicles = 0;             // records that concern only some vehicle tags on a side (the rest: all)
    std::vector<RoadProblem> problems;        // anything that looks wrong, each a record with its kind and place (road_problem.h)
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

// A CORNER FAULT (ADR-0426): a stated turn whose drawn corner does not fit a vehicle class -- its tightest bend tighter
// than the class turns in, or its body swept along it leaving the paved road, or no arc joining the two lane ends at all
// -- or that no vehicle may use (no vehicle open on both its lanes). The turn stays stated in the road model; the lane
// level closes it to that class. The work list for design step 9 (the builder drawing every corner to fit its stated
// turns). (A turn with no lane path to start or end on is no corner's fault: a Refusal of its own, RefusalWhy::NoLanePath.)
struct CornerFault {
    uint32_t turn = kNoIndex;     // the road model's turn (RoadModel::turns)
    std::string tag;              // the vehicle tag it does not fit; empty: every one (no arc, no vehicle on both lanes)
    std::string why;              // in words
    Vec2 at;
};

// WHAT BUILDING THE LANE LEVEL REPORTS. The lane level itself is the road model's (RoadModel::laneLevel): this is what
// was found on the way, for a tool or a test to read. It is not saved.
struct LaneGraphBuild {
    // A fact the lane level cannot be built without, missing from the road file, in words naming what to run; empty when
    // it was built (lane_level.h). The model's lane level is empty then.
    std::string error;
    std::vector<CornerFault> cornerFaults;   // the stated turns whose corners do not fit, per class (lane_level.h)
    // where the lane level joined two lanes the model states lead one into the other (lane_level.h): how many at places a
    // street carries on as another, where a lane leaves its host and where one merges into its host; of them, how many
    // whose lines did not meet (each also a line in the problems, with the gap)
    uint32_t carryOnJoins = 0, divergeJoins = 0, mergeJoins = 0, joinsWithGap = 0;
    // (where the paint and the stated junctions disagree is the road model's to say: RoadModel::problems)
    // WHAT LOOKS WRONG, each a record (road_problem.h RoadProblem: its kind, the ids it concerns, its place and numbers);
    // its sentence is problemLine(record). Empty: nothing to look at.
    std::vector<RoadProblem> problems;
    std::vector<PathSource> sources;       // per path (a connector's: lane -1, no builder lane)
    std::vector<SectionSource> sectionSources;   // per section
    std::vector<double> laneLengths;       // per builder lane: its length as the graph reads it (a fold tidied away)
    // per builder lane: 1 = made into no lane path (a connector, a track, a footway; or a lane cars drive that could not
    // be, counted in drivenLanesLeftOut, each a line in the problems)
    std::vector<char> leftOut;
    std::vector<LaneGap> laneGaps;         // every stretch of a kept lane no lane path covers, and why
    std::vector<JunctionNote> junctionNotes;   // per junction
    std::vector<Refusal> refusals;         // every stated turn with no connector, by junction then turn
    uint32_t connectorsLeftOut = 0;        // builder connector lanes left out
    uint32_t otherLanesLeftOut = 0;        // track and footway lanes left out
    uint32_t drivenLanesLeftOut = 0;       // lanes cars drive made into no lane path (each a line in the problems)
    uint32_t tidiedLanes = 0;              // lanes whose fold at a sharp corner was cut out
    // KINKS (issue #281, findKinks), each also a line in the problems with its place: in the builder's lane lines, in
    // the connectors' lines, and where one path leads into the next with its heading jumping by more than
    // kKinkDegrees. The largest jump of each, in degrees.
    uint32_t laneKinks = 0, connectorKinks = 0, joinKinks = 0;
    double worstLaneKink = 0, worstConnectorKink = 0, worstJoinKink = 0;
    std::size_t bendPieceBytes = 0;        // the bend profiles' memory (the pieces and each path's range)
    uint32_t connectorsByTurn[kTurnCount] = {};   // connectors made, by Turn
    std::vector<uint32_t> closedTo;        // per vehicle tag name: connectors too tight for it (closed to it)
    // How much of the graph a car can get round (lane_graph_connect.h reachReport): the parts, the largest's share of
    // the LANE paths, and the lanes outside it grouped by why (why, lane ids in id order), in a fixed order.
    ReachableParts reach;
    double reachableShare = 0;
    std::vector<std::pair<std::string, std::vector<uint32_t>>> outside;
    CrossingReport crossings;              // the crossing points (their problems are in `problems` too)
    ControlPlanReport controlPlan;         // the stated controls and signs, carried through
    double milliseconds = 0;               // the whole of buildLaneLevel
    double connectMilliseconds = 0;        // ... of which the connectors (the crossing points: crossings.milliseconds)
};

}  // namespace roads::lanes
}  // namespace engine

#endif
