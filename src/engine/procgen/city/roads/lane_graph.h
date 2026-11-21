#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANE_GRAPH_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANE_GRAPH_H

// THE LANE GRAPH (driving-model-design.md 5.2, issue #236): the road as the driving model reads it. Every curve a car
// can be on (a PATH: a lane, or a CONNECTOR, the path through a junction from an arriving lane to a leaving lane),
// how lanes lie side by side (SECTIONS), the junctions, and where two paths or a path and a crosswalk or rail track
// meet (CROSSING POINTS). Beside the paths, in a list of their own so nothing that reads the car paths meets them, the
// WALKS people on foot take and where their ends meet (ADR-0466; see Walk below).
//
// This file holds the types, the save and load (one bundle section), a structural check and a text dump. The lane
// level (lanes/lane_level.h) fills it from the road model's built layer; the driving model reads it through the model
// that holds it.
//
// How it is stored. The graph is whole-world and fixed after load, so it is a few big arrays: a path does not own a
// vector of points or of next-paths, it holds a RANGE (first, count) into one shared array. An island with tens of
// thousands of lanes is then a handful of allocations.
//
// WHAT IT DOES NOT HOLD (ADR-0436, issue #291): what the road IS where a path runs. A junction's control, a path's
// posted limit, its road's class, the turn a connector takes and which path joins by a stated merge are the road
// model's facts (road_model.h), reached by the ids a path, junction or crossing point carries (RoadModel::controlAt,
// postedLimit, roadClassOf, turnOf, mergingAt). The table keeps no copy, so a reader holds the model and asks it.
// Nor THE RULES STANDING ALONG A PATH (a stop sign at its end, a top speed from 30 m along; ADR-0437): the model states
// them, on a junction's arm or on a stretch of a street, and answers for a path (RoadModel::regulationsAlong) from the
// street and the stretch of spine the path carries (Path::street, stationFrom, stationTo, arm).
//
// Kinds are data. Vehicle tags (who may use a path) are NAMES
// the graph carries in a small table, filled from the data files (what a lane is for is its kind in the road model,
// reached by Path::modelLane: not copied here since ADR-0429); a path
// refers to a name by its index, and a set of tags is a bit mask (bit i = name i). The only fixed
// vocabularies here are the geometric ones: a path is a lane or a connector; a connector goes straight, turns near
// (stays by the kerb), turns far (crosses oncoming traffic) or turns back; two things meet by crossing, joining or
// splitting; and the other side of a crossing is a path, a crosswalk or a rail track.
//
// Units: metres, seconds, metres per second.

#include "engine/bundle/binary_stream.h"
#include "engine/procgen/city/polygon.h"   // Vec2, Real
#include "engine/procgen/city/roads/graph_index.h"   // GraphIndex, Range, kNoIndex

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace engine {

struct PathTag {};
struct SectionTag {};
struct JunctionTag {};
struct CrossingTag {};
using PathId = GraphIndex<PathTag>;
using SectionId = GraphIndex<SectionTag>;
using JunctionId = GraphIndex<JunctionTag>;
using CrossingId = GraphIndex<CrossingTag>;

// An index into a table of NAMES (the graph's vehicleTagNames; a reader's own list of names). kNoName where the table
// has nothing to say.
using NameIndex = uint32_t;
constexpr NameIndex kNoName = 0xFFFFFFFFu;
// A set of names from one table: bit i set = name i is in the set. So a table holds at most kMaxTagNames names.
using TagMask = uint32_t;
constexpr uint32_t kMaxTagNames = 32;
// Every vehicle, whatever its tags (a crossing point's CrossingPoint::aTags / otherTags).
constexpr TagMask kEveryVehicle = ~TagMask(0);

// Whether a path runs along a road or through a junction.
enum class PathKind : uint8_t { Lane, Connector };
constexpr uint8_t kPathKindCount = 2;

// Which way a turn through a junction goes, as the road model states it (ModelTurn::turn; a connector's is asked of
// the model: RoadModel::turnOf). A lane has none.
enum class Turn : uint8_t { None, Straight, NearTurn, FarTurn, UTurn };
constexpr uint8_t kTurnCount = 5;

// How two things meet: their paths pass through each other (Cross), they end in the same lane (Join), or they start
// from the same lane (Split).
enum class Meeting : uint8_t { Cross, Join, Split };
constexpr uint8_t kMeetingCount = 3;

// What the second party of a crossing point is.
enum class OtherSide : uint8_t { Path, Crosswalk, Track };

// A connector with no place to wait inside its junction.
constexpr Real kNoWaitInside = -1;
// (Which two approaches to a junction face each other is the road model's answer, from the junction's stated through
// pairs: road_model.h RoadModel::facing. An arriving lane carries only the arm it arrives by, Path::arm.)
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
    TagMask vehicleTags = 0;            // who may use it (bits of LaneGraph::vehicleTagNames)
    Range curve;                         // its centre line: into LaneGraph::points / along / heights
    Real length = 0;                     // metres along the centre line
    Real width = 0;
    Real tightestRadius = kNoBend;       // the radius of its tightest bend (a path with no bend: kNoBend, the default).
                                         // A lane's is its bend profile's tightest piece; a connector's is the tightest
                                         // arc it was designed with (what closes it to a vehicle that turns wider).
    Range bends;                         // into LaneGraph::bendPieces: its bend profile, pieces in order along it. None:
                                         // the whole path bends at tightestRadius (a straight path, or a hand-made one)
    SectionId section;                  // lanes: the section it lies in
    uint32_t indexInSection = 0;         // lanes: 0 = offside ... last = kerb
    JunctionId junction;                 // connectors: the junction it runs through
    PathId from;                         // connectors: the lane it leaves
    Real waitInsideAt = kNoWaitInside;   // connectors: where along it a car may wait inside the junction
    Range next;                          // into LaneGraph::nextPaths: lanes, what leaves its end; connectors, the lane
    Range crossings;                   // into LaneGraph::pathCrossings: the crossing points it is part of (as either
                                         // side), by where their stretch on it begins, then by id (indexCrossings)
    // WHERE IT IS ON THE ROAD MODEL (road_model.h; ADR-0426), by the model's ids: a lane its street and its lane (the
    // builder's lane it was cut from); a connector the stated turn it follows. kNoIndex: none (a hand-made graph).
    uint32_t street = kNoIndex;
    uint32_t modelLane = kNoIndex;
    uint32_t modelTurn = kNoIndex;
    // A LANE'S STRETCH OF ITS ROAD'S SPINE (ADR-0437): the stations (metres along the spine of its street's road) at
    // its start and at its end, the way it is driven -- so stationTo is below stationFrom on a lane running against
    // the spine. `onSpine` false: none stated (a connector, or a hand-made table that states none). Where the lane
    // level cut the path: its own data. What the road model places a street's regulations on the path by
    // (RoadModel::regulationsAlong), and how it knows which end of its street the path runs to.
    bool onSpine = false;
    Real stationFrom = 0, stationTo = 0;
    // Runs the way its road's spine does (true) or against it. Asked only of a path that is onSpine.
    bool alongSpine() const { return stationTo > stationFrom; }
    // A LANE ARRIVING AT A JUNCTION: the arm it arrives by (its place in the junction's stated list of arms). kNoIndex:
    // it arrives at no junction. The id the road model is asked by for what that arm states: its signs
    // (RoadModel::regulationsAlong) and which arriving lanes face each other (RoadModel::facing, from the junction's
    // stated through pairs; the lane table keeps no copy of the pair, ADR-0438).
    uint32_t arm = kNoIndex;
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
    // WHICH VEHICLES IT CONCERNS, on each side (issue #284 item 9, ADR-0412): a vehicle on `a` is concerned when it
    // carries one of aTags, one on the other path when it carries one of otherTags (bits of vehicleTagNames);
    // kEveryVehicle concerns every vehicle. Where long bodies cut inside a bend the road builder records a meeting once
    // for each pair of body sizes that meet differently, so two vehicles meet only where their own bodies could.
    TagMask aTags = kEveryVehicle, otherTags = kEveryVehicle;
    // A STATED MERGE (ADR-0426): on a Join of two lane paths where one ends into the other's lane by a merge the road
    // model states -- a ramp's lane into its host's, a pocket or dropped lane tapering into its neighbour -- the model's
    // connection that states it (RoadModel::connections). Which of the two paths joins, and so gives way, is asked of
    // the model (RoadModel::mergingAt), never kept here. kNoIndex: no stated merge here.
    uint32_t modelConnection = kNoIndex;

    PathId otherPath() const { return otherSide == OtherSide::Path ? PathId(other) : PathId(); }
    // Whether it concerns a vehicle carrying `vehicleTags` on `path` (one of its two paths; a path it does not name: no).
    bool concerns(PathId path, TagMask vehicleTags) const {
        const TagMask mine = path == a ? aTags : (otherSide == OtherSide::Path && path.index == other) ? otherTags : 0;
        return mine == kEveryVehicle || (mine & vehicleTags) != 0;
    }
};

// A PLACE WHERE PEOPLE WALK ACROSS the road: its area, and the road model's crosswalk it is. A COPY for the crossing
// finder, laid from the model (road_model.h layLaneCrosswalks, ADR-0452): its ring starts with the edge at the model
// crosswalk's kerb end 0 and runs to the edge at end 1, so a crossing point's otherFrom/otherTo are metres from end 0.
struct Crosswalk {
    Range outline;                                   // into LaneGraph::planPoints, a closed ring
    JunctionId junction;                             // none for one out on a road
    uint32_t modelCrosswalk = kNoIndex;              // the road model's crosswalk (RoadModel::crosswalks); kNoIndex: none
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
    Range box;                // into LaneGraph::planPoints: the outline of the area that must not be blocked
    Range arriving;            // into LaneGraph::junctionLanes: the lanes ending at its outline (at a stop line), id order
    Range leaving;             // into LaneGraph::junctionLanes: the lanes beginning at its outline, in id order
    uint32_t modelJunction = kNoIndex;   // the road model's junction it is (road_model.h; ADR-0426); kNoIndex: none
};

// ---- WHERE PEOPLE WALK (ADR-0466, #306 task 4 piece 3) ---------------------------------------------------------------
// A WALK is a path a person on foot can take, either way: a stretch of a sidewalk along one kerb of one street, a corner
// walk round a junction's kerb, a crossing over a crosswalk, a turn-round across a dead end's road end, or a stretch of a
// footway. Walks are a list of their own beside `paths` (the curves a car can be on), sharing the curve arrays, so
// nothing that reads the car paths meets one. Who may use a walk is its kind, the road model's (RoadModel::walkKind), and
// the path users' table (RoadModel::openTo): no kind is copied here.
//
// HOW WALKS MEET: at their ends. Every walk end lists the other walk ends that meet it (`atStart`, `atEnd` into
// walkJoins), always both ways: a person reaching that end of one may go on along any of the others. Where they meet is
// stated, never measured: the kerb end of a junction's arm (its sidewalk, its corner walks and its crossing on that
// kerb), a street carrying on as another, a dead end's turn-round.
struct WalkTag {};
using WalkId = GraphIndex<WalkTag>;
// ONE END OF A WALK: the walk and which end (0 its start, 1 its end).
struct WalkEnd {
    WalkId walk;
    uint8_t end = 0;
};
struct Walk {
    Range curve;                         // its centre line: into LaneGraph::points / along / heights
    Real length = 0;
    Real width = 0;
    Range atStart, atEnd;                // into LaneGraph::walkJoins: the walk ends that meet its start, its end
    // WHAT IT IS ON THE ROAD MODEL (road_model.h), by the model's ids; one of the five is set. A sidewalk or a corner
    // walk: its walk (RoadModel::walks). A footway: its lane (RoadModel::lanes). A crossing: its crosswalk
    // (RoadModel::crosswalks; the walk starts at the crosswalk's kerb end 0). A turn-round: its dead end
    // (RoadModel::deadEnds; it starts on the +1 kerb's sidewalk). A walkway's stretch: its walkway
    // (RoadModel::walkways, ADR-0475; it runs the way the walkway's line does).
    uint32_t modelWalk = kNoIndex;
    uint32_t modelLane = kNoIndex;
    uint32_t modelCrosswalk = kNoIndex;
    uint32_t modelDeadEnd = kNoIndex;
    uint32_t modelWalkway = kNoIndex;
    // A sidewalk's or a footway's stretch: its street, and the stations along that street's road's spine at its start
    // and its end (kNoIndex: none, a corner, crossing, turn-round or walkway). A walkway's stretch: no street, and the
    // metres along the walkway's line at its start and its end.
    uint32_t street = kNoIndex;
    Real stationFrom = 0, stationTo = 0;
    Range joins(int end) const { return end == 0 ? atStart : atEnd; }
};

// A PLACE ON THE LANE LEVEL: a path (a lane or a connector) and a distance along it, in metres from its start. A car's
// front, a trip's destination and a hold are each one (the driving's records); the car router routes between two.
struct PathPoint {
    PathId path;
    Real along = 0;

    bool operator==(const PathPoint& o) const { return path == o.path && along == o.along; }
    bool operator!=(const PathPoint& o) const { return !(*this == o); }
};
// A PLACE ON A WALK: the walk and a distance along it, in metres from its start: where a walker's route starts and ends
// (route.h findWalkRoute, ADR-0467).
struct WalkPoint {
    WalkId walk;
    Real along = 0;

    bool operator==(const WalkPoint& o) const { return walk == o.walk && along == o.along; }
    bool operator!=(const WalkPoint& o) const { return !(*this == o); }
};

// (PLACES TO LEAVE A CAR are not here. The kerb parking spaces are the road model's, stated on its streets
// (road_model.h RoadModel::kerbRuns, ADR-0439); the kerb lane path beside a space is asked of the model:
// RoadModel::kerbLaneBeside. The `CarSpace` list this table once declared was never filled and is deleted.)

struct LaneGraph {
    // The name tables (filled from the data files; the graph only carries the names it refers to by index).
    std::vector<std::string> vehicleTagNames;

    std::vector<Path> paths;
    std::vector<Section> sections;
    std::vector<Junction> junctions;
    std::vector<CrossingPoint> crossings;   // junction by junction (those on no junction last), then by (a, other side,
                                            // other, aFrom)
    std::vector<Crosswalk> crosswalks;      // junction by junction (those on no junction last)
    std::vector<Track> tracks;
    std::vector<Walk> walks;            // where people walk (ADR-0466), after every path's curve in the shared arrays

    // The shared arrays the ranges point into.
    std::vector<Vec2> points;           // path and walk centre lines, plan position
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
    std::vector<WalkEnd> walkJoins;     // the walk ends each walk end meets (Walk::atStart, atEnd), in walk then end order

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
    // ... the same on a walk's centre line (an unknown walk: the origin).
    Vec2 pointAt(WalkId walk, Real distanceAlong) const;
    // How high it is.
    Real heightAt(PathId path, Real distanceAlong) const;
    // ... the same on a walk's centre line: the height of the pavement it runs on, as the lane level laid it (a sidewalk
    // over a bridge is as high as the bridge's sidewalk). An unknown walk: 0.
    Real heightAt(WalkId walk, Real distanceAlong) const;
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
bool operator==(const Walk& a, const Walk& b);
bool operator==(const WalkEnd& a, const WalkEnd& b);
bool operator==(const LaneGraph& a, const LaneGraph& b);   // field for field, exactly
inline bool operator!=(const LaneGraph& a, const LaneGraph& b) { return !(a == b); }

// A VEHICLE'S BODY ON THE PLAN (issue #284, review item 9, ADR-0412): the ONE rule for where a body that follows a
// path lies, read by the road builder (does a turn fit on the paved road, which paths' bodies can touch) and by the
// driving model (where a car is drawn and measured, driving/traffic_state.h carPose, driving/measure.h planBody), so the
// graph and the measurements agree. The middle of the front bumper is on the path; the middle of the rear bumper is on
// the path `length` metres back along it (and along the paths that led to it); the body is a rectangle `length` long
// and `width` wide, centred halfway between the two bumpers and pointing from the rear one to the front one. Through a
// bend the two bumpers are on the curve and the body lies along the chord between them, so its middle cuts inside the
// bend: a 12 m bus on a 10 m radius turn lies about 1.7 m inside the line its front follows (the SWING; traffic
// engineers call it off-tracking).
struct PlanRectangle {
    Vec2 middle;
    Vec2 forward = Vec2(1, 0);   // unit, from the rear end to the front end
    Real halfLength = 0, halfWidth = 0;
    // The point on its middle line `share` of the way from its rear end (0) to its front end (1).
    Vec2 alongMiddle(Real share) const { return middle + forward * ((share - Real(0.5)) * 2 * halfLength); }
};
// Bumpers closer together than this are one place: the body then points the way the front travels (metres).
constexpr Real kShortestBodySpan = 1e-9;
// The body whose front bumper's middle is at `front` and rear bumper's middle at `rear` (plan points); `travel` (the
// way the front is going) where the two are one place, and failing that +x.
PlanRectangle bodyOnPlan(const Vec2& front, const Vec2& rear, const Vec2& travel, Real length, Real width);

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
// THE PARTS OF THE WALKS A PERSON ON FOOT CAN GO ROUND (ADR-0466): sets of walks joined end to end, each walked either
// way, over the walks `open` names (per walk, non-zero: open; RoadModel::openTo gives a mover's). A closed walk is a part
// of its own. Numbered by their lowest walk, as reachableParts numbers paths.
ReachableParts reachableWalks(const LaneGraph& g, const std::vector<uint8_t>& open);

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

}  // namespace engine

#endif
