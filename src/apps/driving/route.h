#ifndef RAYTRACER_APPS_DRIVING_ROUTE_H
#define RAYTRACER_APPS_DRIVING_ROUTE_H

// THE ROUTER (driving-model-design.md 10 "Routes"; issue #254): given where a car is and where it is going, which roads
// it takes, and at every point along the way which lanes lead on to its next turn.
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

#include "engine/procgen/city/roads/lane_graph.h"
#include "apps/driving/notice.h"          // WayAhead
#include "apps/driving/traffic_state.h"   // PathPoint
#include "apps/driving/vehicle_class.h"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace driving {

using engine::Real;

// WHAT ONE LANE CHANGE COSTS a route, in seconds, on top of the time spent driving. A route that needs fewer changes
// wins over one a little quicker. (A placeholder until the rule set carries it.)
constexpr Real kLaneChangeCostSeconds = 4.0;

// "This lane cannot finish the route": no number of lane changes from it gets the car to its destination.
constexpr uint32_t kCannotFinish = 0xFFFFFFFFu;

// A VEHICLE CLASS AS THE ROUTER SEES IT ON ONE GRAPH: its tag names turned into the graph's tag bits, its tightest
// turn and its top speed. Built once per graph and class (bindRouteClass).
struct RouteClass {
    engine::TagMask tags = 0;          // the graph's vehicle-tag bits for the class's tag names
    Real tightestTurnRadius = 0;       // a path bending tighter than this is refused
    Real topSpeed = 0;                 // m/s; caps every speed limit
    // Routes may change lanes. Off, a route keeps to the lane the car is in and the lanes that lane leads into, so a car
    // that cannot change lanes yet can drive every route it is given (Driving sets it from DrivingSettings::laneChanges).
    bool laneChanges = true;
    // Connectors CLOSED to the class, sorted: turns it could never (or too seldom) be granted room beyond (room.h
    // turnsClosedForRoom: into a stretch it cannot stop in, with every way on through the next junction barred to it).
    // Driving fills it once it has the rules; bindRouteClass leaves it empty.
    std::vector<engine::PathId> closed;
};

// BINDS A CLASS TO A GRAPH: each of its tag names to the graph's bit for it. A tag the graph does not know is not an
// error -- the class simply matches no path by that tag -- and is reported as a plain line in `notes` when given.
RouteClass bindRouteClass(const engine::LaneGraph& roads, const VehicleClass& cls, std::vector<std::string>* notes = nullptr);

// WHETHER A CLASS MAY DRIVE A PATH: the path's vehicle tags include at least one of the class's, and its tightest bend
// is no tighter than the class can turn. A path whose tags are empty is open to nobody. An unknown path: false.
bool mayUse(const engine::LaneGraph& roads, const RouteClass& cls, engine::PathId path);

// The speed a class drives a path at with nothing in the way: the path's limit, capped by the class's top speed. A path
// with no posted limit (0) is driven at the top speed. Zero for an unknown path.
Real freeFlowSpeed(const engine::LaneGraph& roads, const RouteClass& cls, engine::PathId path);

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
RouteBounds studyRoads(const engine::LaneGraph& roads);

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
// studyRoads(roads); passing one measured for another graph makes the bound wrong (still a route, maybe not the best).
// The path the car is on may be driven on whether or not the class may use it (the car is there); every other path the
// route uses must be open to the class.
Route findRoute(const engine::LaneGraph& roads, const RouteBounds& bounds, const RouteClass& cls, const PathPoint& from,
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
Remaining remainingFrom(const Route& route, const engine::LaneGraph& roads, const PathPoint& at, uint32_t fromLeg = 0);

// ONE NUMBER OVER EVERYTHING A ROUTE SAYS, folded in a fixed order (for the driving model's state hash).
uint64_t routeHash(const Route& route);

}  // namespace driving

#endif
