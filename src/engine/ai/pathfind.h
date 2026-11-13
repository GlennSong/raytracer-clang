#ifndef RAYTRACER_ENGINE_AI_PATHFIND_H
#define RAYTRACER_ENGINE_AI_PATHFIND_H

#include "nav_graph.h"
#include <vector>

namespace engine {

// A route through the NavGraph: the ordered list of directed links to traverse
// from a start node to a goal node. Empty when the start IS the goal, or when no
// path exists — callers distinguish via the goal node, not the route.
struct Route {
    std::vector<int> links;                 // link indices in traversal order
    bool valid() const { return !links.empty(); }
    Real length(const NavGraph& g) const;   // total metres along the route
};

// Free-travel speed (m/s) per road class — used both as the A* cost basis and as
// the agent's target speed, so the planner and the sim agree on "fast roads".
Real classSpeed(RoadClass klass);

// A* over the directed lane graph (ADR-0059). Cost is TRAVEL TIME
// (length / classSpeed), so routes prefer arterials over local streets like real
// driving; the heuristic is straight-line time at the fastest class speed
// (admissible -> optimal). ON FOOT every link is walked at one pace: the cost is
// the walk's length, the shortest walk wins. Deterministic: ties broken by link index. Returns an
// empty Route if the goal is unreachable (or equals the start).
// `onFoot`: skip Freeway/Ramp-class links — a pedestrian must never route
// along a carriageway (§10.5; the unified graph made those links visible to
// every consumer, including walkers).
//
// `linkCostScale` (optional, one entry per link, each >= 1): multiplies a
// link's travel time. A planner uses it to steer a route OFF streets it wants
// to avoid without forbidding them -- the bus network charges extra for a
// street another route already drives, so routes spread across the city
// instead of piling onto the same arterial. Scales >= 1 keep the heuristic
// admissible.
Route findRoute(const NavGraph& graph, int startNode, int goalNode,
                bool onFoot = false,
                const std::vector<Real>* linkCostScale = nullptr);

// WHAT findRoute CHARGES FOR ONE LINK (seconds), the one rule for the router and for anyone pricing a route it found
// (the trip planner's minutes, ADR-0416): the link's length at its class speed (on foot, one pace), plus a street
// junction at its end (not at `goalNode`, not on a freeway or a ramp), times `costScale` (>= 1), plus a U-turn from
// `inLink` (the link it is entered from; -1 at the route's start) priced as a minute.
Real linkTravelSeconds(const NavGraph& graph, int inLink, int link, int goalNode, bool onFoot, Real costScale = 1);
// A route's travel time: the sum of linkTravelSeconds along it, the time findRoute minimised (no cost scale).
Real routeSeconds(const NavGraph& graph, const Route& route, bool onFoot = false);

// HOW MUCH A ROUTE THROUGH SOME LINKS COSTS (by car): the router's time from `startNode` to every node, twice -- by
// the fastest route that uses none of the links with `through[link]` set (`without`), and by the fastest that uses at
// least one (`via`; the freeway, ADR-0416) -- one Dijkstra over the graph doubled. linkTravelSeconds' rule, the street
// junction at the end of the last link included (no goal is known). Unreachable: infinity.
void travelSecondsVia(const NavGraph& graph, int startNode, const std::vector<char>& through,
                      std::vector<Real>& without, std::vector<Real>& via);

// Running totals of every findRoute on this thread (diagnostics: what a spike spent routing).
struct RouteStats {
    long calls = 0; long expanded = 0; double ms = 0.0;
    std::vector<std::pair<const void*, long>> callers;   // return address -> calls (diagnostics)
};
RouteStats& routeStats();

// Every node findRoute can reach from `startNode` (same link rules, on foot or not): 1 = reachable.
// One flood of the graph -- what a caller about to try many goals from one node runs once instead
// of paying for each unreachable goal with an exhaustive failed search.
std::vector<char> reachableFrom(const NavGraph& graph, int startNode, bool onFoot = false);

// The graph's STRONGLY CONNECTED components under findRoute's link rules (on foot or not): comp[a] ==
// comp[b] exactly when findRoute(a, b) and findRoute(b, a) both find a route (a != b). One linear pass --
// what a caller asking "there and back?" of thousands of pairs runs once instead of two searches a pair.
std::vector<int> stronglyConnected(const NavGraph& graph, bool onFoot = false);

// Convenience: snap world points to their nearest nodes, then route.
Route findRouteBetween(const NavGraph& graph, const Vec2& start, const Vec2& goal);
// Same, on foot (skips Freeway/Ramp links) — walkers snap and route too.
Route findRouteBetweenOnFoot(const NavGraph& graph, const Vec2& start,
                             const Vec2& goal);

// Sample a route into a followable polyline (LaneFollower::setPath input):
// lane centres for a car, the sidewalk offset for a walker. Points are at most
// `step` metres apart along each link, and consecutive links share their
// junction point once (no duplicates — LaneFollower drops them anyway, but the
// arc lengths stay honest). This is the sampler CitySim::lanePath performs
// agent-keyed and test_piedmont_drive open-codes; possession needed it pure.
std::vector<Vec2> routePolyline(const NavGraph& graph, const Route& route,
                                Real step = 3.0, bool sidewalk = false,
                                int lane = 0);

// The same polyline plus a parallel per-point cruise speed — classSpeed of the
// link each point lies on (walk callers pass their own pace instead). The two
// vectors are index-aligned so a follower can look up the local limit at its
// current segment.
std::vector<Vec2> routePolylineWithSpeeds(const NavGraph& graph,
                                          const Route& route,
                                          std::vector<Real>& outSpeeds,
                                          Real step = 3.0,
                                          bool sidewalk = false, int lane = 0);

}  // namespace engine

#endif
