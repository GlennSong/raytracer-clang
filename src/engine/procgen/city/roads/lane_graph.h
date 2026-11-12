#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANE_GRAPH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANE_GRAPH_H

// THE LANE GRAPH (driving-model-design.md 5.2, issue #236): the road as the driving model reads it. Every curve a car
// can be on (a PATH: a lane, or a CONNECTOR, the path through a junction from an arriving lane to a leaving lane),
// how lanes lie side by side (SECTIONS), the junctions, where two paths or a path and a crosswalk or rail track meet
// (CROSSING POINTS), the rules standing along the paths (REGULATIONS) and the places to leave a car (CAR SPACES).
//
// This file holds the types, the save and load (one bundle section), a structural check and a text dump. The lane
// graph builder (lanes/lane_graph_build.h) fills it from the built roads, with each junction's control and the
// regulations the control plan wrote into the road graph (lanes/control_plan.h); the driving model reads it.
//
// How it is stored. The graph is whole-world and fixed after load, so it is a few big arrays: a path does not own a
// vector of points or of next-paths, it holds a RANGE (first, count) into one shared array. An island with tens of
// thousands of lanes is then a handful of allocations.
//
// Kinds are data. Lane tags (what a lane is for), vehicle tags (who may use a path), regulation kinds, junction
// controls and road classes are NAMES the graph carries in small tables, filled from the data files; a path or a
// regulation refers to a name by its index, and a set of tags is a bit mask (bit i = name i). The only fixed
// vocabularies here are the geometric ones: a path is a lane or a connector; a connector goes straight, turns near
// (stays by the kerb), turns far (crosses oncoming traffic) or turns back; two things meet by crossing, joining or
// splitting; and the other side of a crossing is a path, a crosswalk or a rail track.
//
// Units: metres, seconds, metres per second.

#include "engine/bundle/binary_stream.h"
#include "engine/procgen/city/polygon.h"   // Vec2, Real

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace engine {

namespace bundle {
class BundleWriter;
class Bundle;
}  // namespace bundle

// AN INDEX INTO ONE OF THE GRAPH'S ARRAYS, typed by what it points at, so a path's index cannot be passed where a
// junction's is wanted. kNone is "no such thing" (a lane's junction, a connector's section). (handle.h's Handle is
// for slots that are freed and reused, and carries a generation; the graph never frees, so it would only double the
// size of every reference.)
constexpr uint32_t kNoIndex = 0xFFFFFFFFu;

template <typename Tag>
struct GraphIndex {
    static constexpr uint32_t kNone = kNoIndex;
    uint32_t index = kNone;

    GraphIndex() = default;
    explicit GraphIndex(uint32_t i) : index(i) {}
    bool valid() const { return index != kNone; }
    bool operator==(const GraphIndex& o) const { return index == o.index; }
    bool operator!=(const GraphIndex& o) const { return index != o.index; }
    bool operator<(const GraphIndex& o) const { return index < o.index; }
};

struct PathTag {};
struct SectionTag {};
struct JunctionTag {};
struct CrossingTag {};
struct CrosswalkTag {};
struct TrackTag {};
using PathId = GraphIndex<PathTag>;
using SectionId = GraphIndex<SectionTag>;
using JunctionId = GraphIndex<JunctionTag>;
using CrossingId = GraphIndex<CrossingTag>;
using CrosswalkId = GraphIndex<CrosswalkTag>;
using TrackId = GraphIndex<TrackTag>;

// An index into one of the graph's NAME tables (laneTagNames, vehicleTagNames, ...): which regulation kind, which
// control, which road class. kNone where the table has nothing to say.
using NameIndex = uint32_t;
constexpr NameIndex kNoName = 0xFFFFFFFFu;
// A set of names from one table: bit i set = name i is in the set. So a table holds at most kMaxTagNames names.
using TagMask = uint32_t;
constexpr uint32_t kMaxTagNames = 32;

// A stretch of one of the graph's shared arrays: `count` entries from `first`.
struct Range {
    uint32_t first = 0;
    uint32_t count = 0;
    uint32_t end() const { return first + count; }
    bool operator==(const Range& o) const { return first == o.first && count == o.count; }
    bool operator!=(const Range& o) const { return !(*this == o); }
};

// Whether a path runs along a road or through a junction.
enum class PathKind : uint8_t { Lane, Connector };
constexpr uint8_t kPathKindCount = 2;

// Which way a connector leaves the junction, read from its shape. A lane has none.
enum class Turn : uint8_t { None, Straight, NearTurn, FarTurn, UTurn };
constexpr uint8_t kTurnCount = 5;

// How two things meet: their paths pass through each other (Cross), they end in the same lane (Join), or they start
// from the same lane (Split).
enum class Meeting : uint8_t { Cross, Join, Split };
constexpr uint8_t kMeetingCount = 3;

// What the second party of a crossing point is.
enum class OtherSide : uint8_t { Path, Crosswalk, Track };
constexpr uint8_t kOtherSideCount = 3;

// A connector with no place to wait inside its junction.
constexpr Real kNoWaitInside = -1;
// TWO APPROACHES TO A JUNCTION FACE EACH OTHER (their traffic meets head on: "oncoming") when the dot of their unit
// directions of travel, at the ends of their arriving lanes, is below this: within about 37 degrees of opposite, the
// figure the old signal controller uses for opposing approaches (ADR-0109). The road builder reads it to place a far
// turn's waiting point inside the junction (lanes/lane_graph_crossings.h placeWaitInsidePoints); the driving model's
// right-of-way table reads the same one.
constexpr Real kOncomingDot = -0.8;
// The tightest radius of a path with no bend at all.
constexpr Real kNoBend = std::numeric_limits<Real>::infinity();

// ONE PIECE OF A PATH'S BEND PROFILE (issue #281): from `from` metres along the path to where the next piece begins (or
// the path's end), the path bends no tighter than `radius` (kNoBend: it runs straight there). A path's pieces start at
// 0 and rise. The road builder measures them once, from the path's shape over a stretch the way a vehicle feels it
// (lanes/lane_graph_build.h measureBends), so a car reads the bend AHEAD of it rather than the tightest anywhere on its
// path.
struct BendPiece {
    Real from = 0;
    Real radius = kNoBend;
};

// A STRETCH OF A PATH AND HOW TIGHT IT BENDS, as the readers hand it out: from `from` to `to` metres along, no tighter
// than `radius`.
struct BendSpan {
    Real from = 0;
    Real to = 0;
    Real radius = kNoBend;
};

// ONE CURVE A CAR CAN BE ON: a lane, or a connector through a junction.
struct Path {
    PathKind kind = PathKind::Lane;
    Turn turn = Turn::None;              // connectors only
    TagMask laneTags = 0;                // what the lane is for (bits of LaneGraph::laneTagNames)
    TagMask vehicleTags = 0;             // who may use it (bits of LaneGraph::vehicleTagNames)
    Range curve;                         // its centre line: into LaneGraph::points / along / heights
    Real length = 0;                     // metres along the centre line
    Real width = 0;
    Real tightestRadius = kNoBend;       // the radius of its tightest bend (a path with no bend: kNoBend, the default).
                                         // A lane's is its bend profile's tightest piece; a connector's is the tightest
                                         // arc it was designed with (what closes it to a vehicle that turns wider).
    Range bends;                         // into LaneGraph::bendPieces: its bend profile, pieces in order along it. None:
                                         // the whole path bends at tightestRadius (a straight path, or a hand-made one)
    Real speedLimit = 0;                 // metres per second
    SectionId section;                   // lanes: the section it lies in
    uint32_t indexInSection = 0;         // lanes: 0 = offside ... last = kerb
    JunctionId junction;                 // connectors: the junction it runs through
    PathId from;                         // connectors: the lane it leaves
    Real waitInsideAt = kNoWaitInside;   // connectors: where along it a car may wait inside the junction
    Range next;                          // into LaneGraph::nextPaths: lanes, what leaves its end; connectors, the lane
    Range regulations;                   // into LaneGraph::regulations, sorted by distance along
    Range crossings;                     // into LaneGraph::pathCrossings: the crossing points it is part of (as either
                                         // side), by where their stretch on it begins, then by id (indexCrossings)
};

// The line between two neighbouring lanes of a section: from which lane a car may cross it. Neither = a solid line.
struct LaneLine {
    bool crossFromOffside = false;     // from the lane nearer the middle of the road into the one nearer the kerb
    bool crossFromKerbside = false;    // the other way
    bool operator==(const LaneLine& o) const {
        return crossFromOffside == o.crossFromOffside && crossFromKerbside == o.crossFromKerbside;
    }
};

// LANES SIDE BY SIDE with the same start and end, so "the lane to my kerb side" is one lane all the way along.
struct Section {
    Range lanes;                   // into LaneGraph::sectionLanes, offside to kerb
    Range lines;                   // into LaneGraph::laneLines: one fewer than lanes, line i between lane i and i + 1
    NameIndex roadClass = kNoName; // a LaneGraph::roadClassNames entry
};

// TWO THINGS THAT MEET, and the stretch of each where bodies could touch. `a` is always a path; the other side is a
// path (then a < b, so each pair is stored once), a crosswalk or a rail track. For a crosswalk the other stretch runs
// across the walk from where it starts; for a track, along its line.
//
// Two paths meet in one of three ways: their paths pass through each other (Cross: the stretch on each from where the
// bodies could first touch to where they no longer can), they end in the same lane (Join: from where they first come
// within touching distance to the end of both), or they start from the same lane (Split: from the start of both to
// where they are clear). Two paths can meet more than once: each meeting is its own record.
//
// Where it is filed. Two connectors of one junction: `junction` is that junction. Two paths that do not run through one
// junction -- connectors of two junctions, or a connector and a lane, crowded together where a large merged junction or
// a carry-on brings them within touching distance -- have no junction: that is the mark that they reach across
// junctions, and they are stored after every junction's own, found from their paths' lists (Path::crossings). A
// crosswalk's or a track's record is filed under the crosswalk's junction (none for one out on a road).
struct CrossingPoint {
    PathId a;
    OtherSide otherSide = OtherSide::Path;
    Meeting meeting = Meeting::Cross;
    uint32_t other = kNoIndex;                     // a path, crosswalk or track index, by otherSide
    JunctionId junction;                           // the junction it lies in; none for one out on a road, or across two
    Real aFrom = 0, aTo = 0;                       // along a
    Real otherFrom = 0, otherTo = 0;               // along the other

    PathId otherPath() const { return otherSide == OtherSide::Path ? PathId(other) : PathId(); }
    CrosswalkId otherCrosswalk() const { return otherSide == OtherSide::Crosswalk ? CrosswalkId(other) : CrosswalkId(); }
    TrackId otherTrack() const { return otherSide == OtherSide::Track ? TrackId(other) : TrackId(); }
};

// A PLACE WHERE PEOPLE WALK ACROSS the road: its area, and the link of the walkers' network it carries.
struct Crosswalk {
    Range outline;                                   // into LaneGraph::planPoints, a closed ring
    JunctionId junction;                             // none for one out on a road
    uint32_t walkLink = kNoIndex;                    // the walkers' network link it is (that network's index)
};

// A RAIL TRACK at street level, where it lies on the road.
struct Track {
    Range line;                                      // into LaneGraph::planPoints, its centre line
    uint32_t railTrack = kNoIndex;                   // the rail network's own index of the track
};

// How far from its junction's outline a lane arriving at the junction may end, or a lane leaving it begin (metres, in
// plan): the lanes the connectors through a junction will join.
constexpr Real kJunctionEndReach = 1.5;

struct Junction {
    Range connectors;          // into LaneGraph::junctionConnectors, in id order
    Range crossings;           // into LaneGraph::crossings directly (crossings are stored junction by junction)
    Range crosswalks;          // into LaneGraph::crosswalks directly (likewise)
    NameIndex control = kNoName;   // a LaneGraph::controlNames entry, from the control plan
    Range box;                 // into LaneGraph::planPoints: the outline of the area that must not be blocked
    Range arriving;            // into LaneGraph::junctionLanes: the lanes ending at its outline (at a stop line), id order
    Range leaving;             // into LaneGraph::junctionLanes: the lanes beginning at its outline, in id order
};

// A RULE STANDING ON A PATH: a kind (a LaneGraph::regulationKindNames entry), the stretch of the path where it applies
// (from `from` to `to` metres along it; a sign that stands at one place, a stop sign at the end of an arriving lane,
// has from == to), and a value whose meaning the kind's rule says ("stop here": unused; "25 from here": the speed in
// metres per second). Who wrote it: the control plan (issue #240), carried through by the lane graph builder.
struct Regulation {
    NameIndex kind = kNoName;
    PathId path;
    Real from = 0;
    Real to = 0;
    Real value = 0;
};

// A PLACE TO LEAVE A CAR: a stretch of a lane tagged for it, or of a bay.
struct CarSpace {
    PathId lane;
    Real from = 0, to = 0;
};

struct LaneGraph {
    // The name tables (filled from the data files; the graph only carries the names it refers to by index).
    std::vector<std::string> laneTagNames;
    std::vector<std::string> vehicleTagNames;
    std::vector<std::string> regulationKindNames;
    std::vector<std::string> controlNames;
    std::vector<std::string> roadClassNames;

    std::vector<Path> paths;
    std::vector<Section> sections;
    std::vector<Junction> junctions;
    std::vector<CrossingPoint> crossings;   // junction by junction (those on no junction last), then by (a, other side,
                                            // other, aFrom)
    std::vector<Crosswalk> crosswalks;      // junction by junction (those on no junction last)
    std::vector<Track> tracks;
    std::vector<Regulation> regulations;    // path by path, then by where each begins (from), then where it ends (to)
    std::vector<CarSpace> carSpaces;        // by (lane, from)

    // The shared arrays the ranges point into.
    std::vector<Vec2> points;           // path centre lines, plan position
    std::vector<Real> along;            // ... each point's distance along its path (0 at the first)
    std::vector<Real> heights;          // ... each point's height, off the road deck
    std::vector<PathId> nextPaths;
    std::vector<PathId> sectionLanes;
    std::vector<LaneLine> laneLines;
    std::vector<PathId> junctionConnectors;
    std::vector<PathId> junctionLanes;  // the lanes arriving at and leaving each junction
    std::vector<CrossingId> pathCrossings;   // each path's crossing points (Path::crossings)
    std::vector<Vec2> planPoints;       // junction boxes, crosswalk outlines, track lines
    std::vector<BendPiece> bendPieces;  // each path's bend profile (Path::bends)

    // A name's index in a table, or kNoName.
    static NameIndex nameIndex(const std::vector<std::string>& table, const std::string& name);
    // Adds a centre line (plan points and their heights, the same count) to the shared arrays, with each point's
    // distance along it (polyline.h stations), and returns its range for a Path::curve.
    Range addCurve(const std::vector<Vec2>& xy, const std::vector<Real>& z);
    // Adds points to planPoints and returns their range.
    Range addPlanPoints(const std::vector<Vec2>& pts);
    // Adds a bend profile to bendPieces and returns its range for a Path::bends.
    Range addBends(const std::vector<BendPiece>& pieces);

    // READING A PATH'S BEND PROFILE (issue #281). Its stretches in order along it: a path with no pieces of its own has
    // one stretch, the whole path at its tightestRadius; an unknown path has none.
    uint32_t bendCount(PathId path) const;
    // Stretch i of bendCount (an i out of range reads as an empty stretch at the path's end, straight).
    BendSpan bendSpan(PathId path, uint32_t i) const;
    // How tight the path bends at a distance along it: the radius of the stretch holding it (clamped to the path's
    // ends; kNoBend for an unknown path).
    Real radiusAt(PathId path, Real distanceAlong) const;
    // Its tightest stretch (the first of equals): where the bend a car must slow most for lies.
    BendSpan tightestBend(PathId path) const;

    // READING A PATH'S CENTRE LINE at a distance along it (metres from its start; clamped to its ends), each by a
    // binary search over the curve's stations (polyline.h's station readers, over the path's range). An unknown path
    // reads as the origin, height 0, heading +x, grade 0.
    // Where it is, in plan.
    Vec2 pointAt(PathId path, Real distanceAlong) const;
    // How high it is.
    Real heightAt(PathId path, Real distanceAlong) const;
    // Which way it runs, in plan, as a unit vector: the chord from polyline.h's kTangentHalfSpan before to as far
    // after (chordAtStation), so it turns smoothly through a vertex instead of jumping at it. A path with no length
    // there reads as the chord from its first point to its last, and failing that as +x.
    Vec2 directionAt(PathId path, Real distanceAlong) const;
    // How steeply it climbs there: metres up per metre along the plan, over the same chord as directionAt (0 where
    // the chord has no plan length).
    Real gradeAt(PathId path, Real distanceAlong) const;
};

bool operator==(const Path& a, const Path& b);
bool operator==(const Section& a, const Section& b);
bool operator==(const CrossingPoint& a, const CrossingPoint& b);
bool operator==(const Crosswalk& a, const Crosswalk& b);
bool operator==(const Track& a, const Track& b);
bool operator==(const Junction& a, const Junction& b);
bool operator==(const Regulation& a, const Regulation& b);
bool operator==(const CarSpace& a, const CarSpace& b);
bool operator==(const LaneGraph& a, const LaneGraph& b);   // field for field, exactly
inline bool operator!=(const LaneGraph& a, const LaneGraph& b) { return !(a == b); }

// FILES THE CROSSING POINTS once g.crossings holds them in any order: sorts them junction by junction (those on no
// junction last), then by (a, other side, other, aFrom); sets each junction's crossings range; and lists each path's
// crossing points (Path::crossings into pathCrossings: as `a` or as the other path), by where the stretch on that path
// begins, then by id. Whoever fills g.crossings calls it: the road builder, a test's hand-made graph.
void indexCrossings(LaneGraph& g);

// THE STRUCTURAL CHECKS (design 5.3; the geometric ones come with the road builder): every index and range in
// bounds, every enum value known, every tag inside its name table, lanes and their sections agreeing both ways,
// connectors and their junctions agreeing both ways, a connector's from-lane listing it as next, lists sorted,
// stretches running forwards, a junction's arriving lanes ending and its leaving lanes beginning within
// kJunctionEndReach of its outline; each path's bend profile starting at 0, rising, on the path. Crossing points: their stretches on their paths, two paths filed under a junction
// both its connectors (and two filed under none not), a split starting both paths from a lane they both leave, a join
// reaching the end of both into a lane they both enter, no two records of one pair overlapping on both paths, and each
// path's list holding exactly the records that name it, in order. Returns the problems in plain words, each naming
// what and where; empty = sound.
std::vector<std::string> validate(const LaneGraph& g);

// THE PARTS OF THE GRAPH A CAR CAN GO ROUND (issue #238): sets of paths that can all reach one another, following each
// path's next list and the lane changes its section's lines allow (from a lane to its neighbour where the line between
// them may be crossed from that lane's side). A car on any path of a part can get to every other path of that part and
// back; a path in a part of its own is a dead end, or a road nothing leads into, or both. (Graph theory calls the parts
// strongly connected components.) Parts are numbered by their lowest path id, so the answer is the same every run.
struct ReachableParts {
    std::vector<uint32_t> partOf;   // per path: its part
    std::vector<uint32_t> sizes;    // per part: how many paths it holds
    uint32_t largest = 0;           // the part with the most paths (the lowest numbered of equals); 0 when there are none
};
// WHICH MOVES COUNT when working out the parts. The default follows every next list and every lane change the lines
// allow, over every path. A caller asking for one kind of vehicle, or for cars that cannot change lanes yet, narrows it.
struct ReachRules {
    bool laneChanges = true;        // follow the lane changes the lines allow (false: next lists only)
    std::vector<uint8_t> open;      // per path, non-zero: it may be driven; empty: every path may. A closed path is never
                                    // stepped onto and steps nowhere, so it is a part of its own.
};
ReachableParts reachableParts(const LaneGraph& g);
ReachableParts reachableParts(const LaneGraph& g, const ReachRules& rules);
// What `from` leads to in one step: its next paths, then the lanes beside it it may change into (both in id order).
// What reachableParts follows; an index out of range leads nowhere. With rules: only the moves they allow, onto open
// paths (and none from a closed one).
std::vector<PathId> stepsFrom(const LaneGraph& g, PathId from);
std::vector<PathId> stepsFrom(const LaneGraph& g, PathId from, const ReachRules& rules);

// THE WHOLE GRAPH AS TEXT, in id order, with the names from the name tables: for a person checking a junction by
// hand, and for tests to compare. Same graph, same text, every run.
std::string dumpText(const LaneGraph& g);
// The text of one thing, as dumpText writes it (its header line and the lines under it).
std::string dumpSection(const LaneGraph& g, SectionId id);
std::string dumpPath(const LaneGraph& g, PathId id);
std::string dumpCrossing(const LaneGraph& g, CrossingId id);

// SAVE AND LOAD: the graph as one bundle section (magic "LGRF"), written field by field so the bytes are the same
// every run. getLaneGraph refuses a section it cannot read, one that ends early, or a graph validate() rejects, and
// says what and where in `err`; `g` is then empty.
void putLaneGraph(bundle::BinWriter& w, const LaneGraph& g);
bool getLaneGraph(bundle::BinReader& r, LaneGraph& g, std::string* err);
// The same, as the named section of a bundle.
bool writeLaneGraph(bundle::BundleWriter& out, const std::string& section, const LaneGraph& g);
bool readLaneGraph(const bundle::Bundle& in, const std::string& section, LaneGraph& g, std::string* err);

}  // namespace engine

#endif
