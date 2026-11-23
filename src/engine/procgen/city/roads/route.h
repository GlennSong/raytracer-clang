#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROUTE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_ROUTE_H

// THE ONE ROUTE-FINDER (ADR-0467, #306 task 4 piece 4): routes over the road model's one lane level, for whoever moves
// along it. A car is routed over the lane paths (findRoute); a person on foot over the walks (findWalkRoute). Both are
// the one search (route.cpp `Search`: A* over one binary heap, ties to the lower node number, a straight-line lower
// bound); what differs is only the mover's STEPS: which paths it may take and what each costs. Which mover may use which
// kind of path is data (assets/data/path_users.json, RoadModel::openTo); a vehicle class's tags, bends and closed turns
// are the driving's, bound by it (apps/driving/route.h bindRouteClass) and passed in as a RouteClass.
//
// CARS (driving-model-design.md 10 "Routes"; issue #254): given where a car is and where it is going, which roads it
// takes, and at every point along the way which lanes lead on to its next turn.
//
// The search runs over PATHS (lanes and connectors). From a lane a car may go on to anything in its `next` list, or
// sideways into the neighbouring lane of its section when the line between may be crossed from its side; from a
// connector it goes on to the lane it joins. It starts from the path the car's front is on, at its distance along,
// facing the way the path runs, and never goes backwards: a trip never begins by turning round, and a U-turn happens
// only where the road data has a connector for one. The router never invents a step the graph does not have.
//
// The answer is NOT a list of lanes to follow blindly. It is the ordered list of SECTIONS the car drives along (its
// LEGS) and the junction turn between each pair, ending at the destination; and for every lane of every leg, the
// fewest lane changes still needed from that lane to finish the route (or "this lane cannot finish it"). That table
// answers "am I in a lane that leads on, and if not, how many lanes over and which way" (the facts lanes_to_cross and
// distance_to_last_chance), and the questions below read it.
//
// Cost is time: each path's length over its speed limit (capped by the class's top speed), or the seconds a learned
// table gives for a section; each lane change adds kLaneChangeCostSeconds. The search is A* over a binary heap, with a
// straight-line lower bound that provably never over-estimates (see RouteBounds); ties go to the lower path id, so the
// same question gives the same route every run.
//
// Same result every run: the search's heap is ordered by (estimate, node number), every list here is a vector built
// in a fixed order, and nothing walks an unordered container.
//
// Units: metres, seconds, metres per second.

#include "engine/procgen/city/roads/lane_graph.h"   // PathPoint, WalkPoint: places on the lane level
#include "engine/procgen/city/roads/road_model.h"   // the road it routes on: the lane level, and the model's limits and turns

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace engine::roads {

// WHAT ONE LANE CHANGE COSTS a route, in seconds, on top of the time spent driving. A route that needs fewer changes
// wins over one a little quicker. (A placeholder until the rule set carries it.)
constexpr Real kLaneChangeCostSeconds = 4.0;

// "This lane cannot finish the route": no number of lane changes from it gets the car to its destination.
constexpr uint32_t kCannotFinish = 0xFFFFFFFFu;

// WAYS ON CLOSED TO A CLASS (issue #284; room.h chainsClosedForRoom): chains of turns, each taken straight after the one
// before through junctions too close together for the class to stop between, that the class could never (or too
// seldom) be granted room through -- the lights on the way never line up for a car arriving from rest at the first line.
// The first turn alone may be fine (another way on from it is), so it is the CHAIN that is closed, not the turn. Kept as
// a tree over the chains' turns: node 0 is the root (no chain under way); each node's children are the turns that can
// follow it, sorted, and a node is CLOSED when the chain to it is. The router walks it as it takes turns (step).
constexpr uint32_t kChainClosed = 0xFFFFFFFFu;
struct ClosedChains {
    struct Node {
        engine::PathId turn;          // the turn taken to reach this node (none for the root)
        uint32_t firstChild = 0;      // its children are nodes[firstChild, firstChild + childCount), sorted by turn
        uint32_t childCount = 0;
        bool closed = false;          // the chain ending here is closed to the class
        bool never = false;           // ... and its lights never line up for it at all (not merely too seldom)
    };
    std::vector<Node> nodes;          // empty: nothing closed
    uint32_t chains = 0;              // how many closed chains it holds

    bool empty() const { return chains == 0; }
    // THE STATE AFTER TAKING TURN `turn` in state `state`: the child for that turn (kChainClosed when the chain it
    // completes is closed); 0 when no closed chain goes on that way (a chain under way that turns elsewhere is over).
    uint32_t step(uint32_t state, engine::PathId turn) const;
    // Whether the chain of turns `turns` (`count` of them, from its first) is closed, and whether it never lines up.
    bool closedChain(const engine::PathId* turns, std::size_t count, bool* never = nullptr) const;
};
// BUILDS THE TREE from closed chains (each two turns or more; `never` per chain: its lights never line up at all).
ClosedChains buildClosedChains(std::vector<std::vector<engine::PathId>> chains, const std::vector<uint8_t>& never);

// A VEHICLE CLASS AS THE ROUTER SEES IT ON ONE GRAPH: its tag names turned into the graph's tag bits, its tightest
// turn and its top speed. Built once per graph and class by the driving, which owns vehicle classes
// (apps/driving/route.h bindRouteClass).
struct RouteClass {
    engine::TagMask tags = 0;          // the graph's vehicle-tag bits for the class's tag names
    Real tightestTurnRadius = 0;       // a path bending tighter than this is refused
    Real topSpeed = 0;                 // m/s; caps every speed limit
    // THE LIMIT EACH PATH IS DRIVEN AT, by path index (the driving's rules: RoadRules::limitOfPath -- the posted limit,
    // else the rule set's default for the road's class), so a route is timed as its car will drive it. Null, or a
    // path with no entry or 0: the limit the road model posts.
    const std::vector<Real>* limits = nullptr;
    // Routes may change lanes. Off, a route keeps to the lane the car is in and the lanes that lane leads into, so a car
    // that cannot change lanes yet can drive every route it is given (Driving sets it from DrivingSettings::laneChanges).
    bool laneChanges = true;
    // ...and only out of a lane at least this long (m; 0: any): a move across needs room, the shortest its turning circle
    // allows and its own length (the driving's lane_change.h laneMoveShortest), so a short stretch is not changed in.
    Real laneChangeLeast = 0;
    // ...but never out of the stretch it starts in: the car keeps its lane to the stretch's end, and is free to change
    // past it (a driver that missed its lane, routed again from where it is: the driving's missed turn, sitting 5).
    bool keepLaneAtStart = false;
    // Connectors CLOSED to the class, sorted: turns it could never (or too seldom) be granted room beyond (room.h
    // turnsClosedForRoom: into a stretch it cannot stop in, with every way on through the next junction barred to it).
    // Driving fills it once it has the rules; bindRouteClass leaves it empty.
    std::vector<engine::PathId> closed;
    // WAYS ON CLOSED to the class (above): a route never takes the turns of one in a row. Filled with `closed`.
    ClosedChains closedChains;
};

// WHETHER A CLASS MAY DRIVE A PATH: the path's vehicle tags include at least one of the class's, and its tightest bend
// is no tighter than the class can turn. A path whose tags are empty is open to nobody. An unknown path: false.
// (The path users' table opens every kind of lane path the lane level lays to the vehicles, and the lane level lays no
// lane path for a footway, so the car search does not yet ask RoadModel::openTo: ADR-0467.)
bool mayUse(const engine::LaneGraph& roads, const RouteClass& cls, engine::PathId path);

// The speed a class drives a path at with nothing in the way: the limit it is driven at (RouteClass::limits, else the
// limit the road model posts on the path, RoadModel::postedLimit), capped by the class's top speed. A path with no limit
// (0) is driven at the top speed. Zero for an unknown path.
Real freeFlowSpeed(const engine::RoadModel& model, const RouteClass& cls, engine::PathId path);

// WHAT THE LOWER BOUND NEEDS TO KNOW ABOUT A GRAPH, measured once per graph (studyRoads). The bound is the straight
// distance to the destination over the fastest speed any step can go; for it never to over-estimate, it must also
// allow for the two places a car moves without driving along a path: a lane change (which shifts the car sideways for
// kLaneChangeCostSeconds) and a gap between the end of a path and the start of the one it leads into.
struct RouteBounds {
    Real fastestLimit = 0;       // the highest speed limit on any path
    bool anyUnlimited = false;   // some path has no posted limit (it is driven at the class's top speed)
    Real widestSidestep = 0;     // the furthest apart, in plan, the starts of two neighbouring lanes of a section are
    Real widestGap = 0;          // the furthest a path's start lies from the end of a path that leads into it
    Real shortestPath = 0;       // the shortest path's length
    bool usable = false;         // false (an empty graph): the search uses no bound (plain Dijkstra)
};
RouteBounds studyRoads(const engine::RoadModel& model);

// LEARNED TIMES: the seconds driving the length of a section takes, by section id. An entry at or below zero, or past
// the end, means "use the free-flow time". (What simulated drivers actually took comes later; the router only reads it.)
using SectionSeconds = std::vector<Real>;

// WHY THERE IS NO ROUTE.
enum class NoRouteWhy : uint8_t {
    None,                  // there is one
    StartNotOnRoad,        // the car's place is not a place on the road
    DestinationNotOnRoad,  // the destination is not a place on the road
    DestinationClosed,     // the destination's path is not one this class may use
    NothingLeadsThere,     // nothing this class may use leads from here to there
};
constexpr uint8_t kNoRouteWhyCount = 5;

// ONE LEG: a stretch of the route along one section.
struct RouteLeg {
    engine::SectionId section;   // not valid for a lane that lies in no section (then the leg has that lane alone)
    engine::PathId lane;         // the lane the route leaves the leg by (or the one the destination is on)
    Real enterFraction = 0;      // where the route enters it, as a share of each lane's length: 0, except the first
    Real leaveFraction = 1;      // where it leaves: 1, except the last (the destination)
    uint32_t firstLane = 0;      // into Route::lanes: this section's lanes, offside to kerb
    uint32_t laneCount = 0;
    Real metres = 0;             // along `lane`, from enterFraction to leaveFraction
    Real seconds = 0;            // ... at the free-flow speed
    Real metresAfter = 0;        // from the end of this leg to the destination (turns and later legs)
    Real secondsAfter = 0;       // ... at the free-flow speed
};

// THE MOVE FROM ONE LEG TO THE NEXT: the junction turn (the connector's kind) and the pair of sections it joins.
struct RouteTurn {
    engine::Turn turn = engine::Turn::None;   // None: the lanes run straight on into the next section, no junction
    engine::JunctionId junction;              // not valid when there is no junction
    engine::SectionId fromSection;
    engine::SectionId toSection;
    engine::PathId connector;                 // the connector the search took (not valid when there is none)
    Real metres = 0;                          // along that connector
    Real seconds = 0;                         // ... at the free-flow speed
};

// ONE LANE OF ONE LEG, and what it takes to finish the route from it.
struct RouteLane {
    engine::PathId lane;
    Real length = 0;
    // The fewest lane changes still needed from this lane (anywhere along this leg) to finish; kCannotFinish when no
    // number of changes will do.
    uint32_t changes = kCannotFinish;
    // The same, if the car does NOT change lanes in this leg: the changes still needed after it leads on into the next
    // leg (or 0 when the destination is on it). kCannotFinish: this lane does not lead on.
    uint32_t changesIfStaying = kCannotFinish;
};

// A ROUTE. When `found` is false only `why`, `whyWords`, `from` and `to` mean anything.
struct Route {
    bool found = false;
    NoRouteWhy why = NoRouteWhy::None;
    std::string whyWords;              // why there is no route, in plain words
    PathPoint from;                    // where the car's front was
    PathPoint to;                      // the destination
    RouteClass vehicle;                // the class it was found for
    // When the car starts on a connector: that connector (it leads into the first leg).
    engine::PathId startConnector;
    // When the destination lies on a connector: that connector (it leaves the last leg). When the car starts on the
    // same connector, ahead of the destination, both are it and there are no legs.
    engine::PathId endConnector;
    std::vector<RouteLeg> legs;        // in order
    std::vector<RouteTurn> turns;      // turns[i] joins legs[i] to legs[i + 1]
    std::vector<RouteLane> lanes;      // each leg's lanes, leg by leg (RouteLeg::firstLane)
    Real metres = 0;                   // from the car to the destination, along the route's own paths
    Real seconds = 0;                  // ... at the free-flow speed
    Real cost = 0;                     // what the search minimised: seconds driving (learned where given) plus lane changes
};

// FINDS THE ROUTE for a car of class `cls` from `from` (its front: a path and a distance along it) to `to`.
// `sectionSeconds`, when given, replaces the free-flow time of the sections it has entries for. `bounds` is
// studyRoads(model); passing one measured for another graph makes the bound wrong (still a route, maybe not the best).
// The path the car is on may be driven on whether or not the class may use it (the car is there); every other path the
// route uses must be open to the class. `model`: the road model whose lane level is routed on.
Route findRoute(const engine::RoadModel& model, const RouteBounds& bounds, const RouteClass& cls, const PathPoint& from,
                const PathPoint& to, const SectionSeconds* sectionSeconds = nullptr);

// ---- Questions asked of a route --------------------------------------------------------------------------------------
//
// A section can appear twice in one route (a car whose destination is behind it goes round the block and back along
// its own road). So each question takes `fromLeg`: the first leg to look in (the car's progress along the route, which
// the car keeps); 0 reads the route from its start.

// WHERE A PLACE IS ON A ROUTE.
enum class RoutePart : uint8_t {
    Off,              // not on the route
    Leg,              // on a lane of legs[index] (any lane of its section, inside the leg's stretch)
    Turn,             // on a connector joining legs[index] to legs[index + 1] (any such connector)
    StartConnector,   // on the start connector, at or past the car's start
    EndConnector,     // on the end connector, at or before the destination
};
constexpr uint8_t kRoutePartCount = 5;
struct RoutePlace {
    RoutePart part = RoutePart::Off;
    uint32_t index = 0;   // the leg (Leg) or turn (Turn); 0 otherwise
    bool onRoute() const { return part != RoutePart::Off; }
};
RoutePlace placeOnRoute(const Route& route, const engine::LaneGraph& roads, const PathPoint& at, uint32_t fromLeg = 0);

// Which way across the road.
enum class Across : uint8_t { None, TowardKerb, TowardOffside };
constexpr uint8_t kAcrossCount = 3;

// THE LANE BESIDE `lane` in its section, toward the kerb or toward the offside, when the line between may be crossed from
// this lane's side; none otherwise (no such lane, a solid line, no sound section). The router's own sidestep: the one
// answer to "which lane runs beside mine here, and may I cross into it" (the driving's lane changes ask it too).
engine::PathId laneBeside(const engine::LaneGraph& g, engine::PathId lane, bool towardKerb);

// HOW MANY LANES OVER the nearest lane with strictly fewer changes still needed is, and which way. Zero lanes (and
// None) when this lane is already as good as any in its section, or when the lane is in no leg at or after `fromLeg`
// (onRoute false). Two lanes as near on both sides: the one needing fewer changes, then the one toward the kerb.
struct LanesToCross {
    bool onRoute = false;
    uint32_t lanes = 0;
    Across toward = Across::None;
};
LanesToCross lanesToCross(const Route& route, engine::PathId lane, uint32_t fromLeg = 0);

// HOW FAR THE CAR MAY STAY IN THIS LANE: metres from `along` to where it must be out of it -- the end of its leg, or in
// the last leg the destination (at the same share of the lane's length) -- when the lane does not lead on. `needed`
// false ("no need") when the lane leads on to the next leg (or holds the destination), or is in no leg from `fromLeg`
// on. Never below zero.
struct LastChance {
    bool needed = false;
    Real metres = 0;
};
LastChance lastChance(const Route& route, engine::PathId lane, Real along, uint32_t fromLeg = 0);

// THE PATHS AHEAD OF A CAR, in order: the first is the path its front bumper is on, each next one leads on from the one
// before (the driving's facts read it, apps/driving/notice.h).
struct WayAhead {
    std::vector<engine::PathId> paths;
};
// The most paths a way ahead holds (only a loop of empty paths reaches it).
constexpr std::size_t kMaxFollowedPaths = 256;

// THE CONCRETE PATHS AHEAD of a car on the route, as the facts take them: its own path; then at each junction the
// connector out of its lane that stays on the route with the fewest changes still needed after it (the lower path id
// on a tie), then that connector's lane, and so on, until the paths reach `lookAhead` metres past `along` or the
// destination's path is reached. Where the car's lane does not lead on, the way stops at the end of that lane (to the
// driver, a lane that ends). A car not on the route gets its own path alone.
WayAhead wayAhead(const Route& route, const engine::LaneGraph& roads, engine::PathId path, Real along, Real lookAhead,
                  uint32_t fromLeg = 0);

// HOW FAR REMAINS from a place on the route to the destination, along the route, and how long that takes at the
// free-flow speed. `onRoute` false (and zeros) for a place not on it.
struct Remaining {
    bool onRoute = false;
    Real metres = 0;
    Real seconds = 0;
};
Remaining remainingFrom(const Route& route, const engine::RoadModel& model, const PathPoint& at, uint32_t fromLeg = 0);

// ONE NUMBER OVER EVERYTHING A ROUTE SAYS, folded in a fixed order (for the driving model's state hash).
uint64_t routeHash(const Route& route);

// ---- People on foot: routes over the walks (ADR-0467) ----------------------------------------------------------------
//
// A walker's route runs over the lane level's WALKS (lane_graph.h Walk: sidewalks, corners, crossings, turn-rounds,
// footways), each walked either way: no one-way or turn rule of the cars applies. It steps from one walk to another only
// where the model states their ends meet (LaneGraph::walkJoins), across the straight gap between the two ends. Cost is
// time: every metre walked (along a walk or across a gap) at the walker's pace, plus the mover's stated wait for each walk
// it steps onto by kind (path_users.json `waits`: a crossing's). The search is the cars' one search; its lower bound is the
// straight distance to the destination at the walker's pace, which never over-estimates, since every metre of the way is
// walked at that pace and a wait only adds.

// A MOVER ON FOOT AS THE ROUTER SEES IT ON ONE LEVEL: the walks it may use and the wait it adds on each, by the walk's
// kind and the path users' table, and its pace. Built once per level and mover (bindWalker).
struct RouteWalker {
    std::string mover;               // its name in the path users' table
    std::vector<uint8_t> open;       // per walk, 1: it may use it (RoadModel::openTo(mover).walks)
    std::vector<Real> waits;         // per walk: the seconds stepping onto it adds (PathMover::waitOn its kind)
    Real pace = 0;                   // m/s
};
// BINDS A MOVER to a level: its open walks and their waits, from the model's walk kinds and the table `users`; `pace`
// the caller's (the person's walking speed). A mover the table does not name may use no walk.
RouteWalker bindWalker(const engine::RoadModel& model, const std::string& mover, Real pace,
                       const engine::PathUsers& users = engine::pathUsers());

// ONE WALK OF A WALKER'S ROUTE: the stretch of it walked, from `from` to `to` metres along it. `forward`: walked from its
// start toward its end (`to` > `from`, or equal on a walk of no length entered at its start).
struct WalkStep {
    engine::WalkId walk;
    Real from = 0, to = 0;
    bool forward = true;
    bool operator==(const WalkStep& o) const { return walk == o.walk && from == o.from && to == o.to && forward == o.forward; }
};
// A CROSSWALK THE ROUTE CROSSES: the road model's crosswalk (RoadModel::crosswalks; the walker waits at it by its id:
// apps/driving walkAtCrosswalk), the kerb end stepped off from (0 or 1, ModelCrosswalk::ends), and which step crosses it.
struct WalkCrossing {
    uint32_t crosswalk = engine::kNoIndex;
    uint8_t fromEnd = 0;
    uint32_t step = 0;
    bool operator==(const WalkCrossing& o) const { return crosswalk == o.crosswalk && fromEnd == o.fromEnd && step == o.step; }
};
// A WALKER'S ROUTE. When `found` is false only `why`, `whyWords`, `from` and `to` mean anything.
struct WalkRoute {
    bool found = false;
    NoRouteWhy why = NoRouteWhy::None;
    std::string whyWords;
    engine::WalkPoint from, to;
    std::vector<WalkStep> steps;           // in order; consecutive steps are joined walk ends
    std::vector<WalkCrossing> crossings;   // in the order crossed
    Real metres = 0;                       // along the steps and across the gaps between them
    Real seconds = 0;                      // metres at the pace, plus the waits (what the search minimised)
    Real waitSeconds = 0;                  // ... of which waits
};

// FINDS A WALKER'S ROUTE from `from` to `to`. The walk the walker is on may be walked whether or not it is open to them
// (they are there); every other walk the route uses must be. NoRouteWhy reads as for a car: StartNotOnRoad /
// DestinationNotOnRoad (not a place on a walk), DestinationClosed (its walk is not open to the mover), NothingLeadsThere.
WalkRoute findWalkRoute(const engine::RoadModel& model, const RouteWalker& walker, const engine::WalkPoint& from,
                        const engine::WalkPoint& to);

}  // namespace engine::roads

#endif
