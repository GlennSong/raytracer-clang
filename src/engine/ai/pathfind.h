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
// (admissible -> optimal). Deterministic: ties broken by link index. Returns an
// empty Route if the goal is unreachable (or equals the start).
// `onFoot`: skip Freeway/Ramp-class links — a pedestrian must never route
// along a carriageway (§10.5; the unified graph made those links visible to
// every consumer, including walkers).
Route findRoute(const NavGraph& graph, int startNode, int goalNode,
                bool onFoot = false);

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
