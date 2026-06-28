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

// A* over the directed lane graph (ADR-0057). Cost is TRAVEL TIME
// (length / classSpeed), so routes prefer arterials over local streets like real
// driving; the heuristic is straight-line time at the fastest class speed
// (admissible -> optimal). Deterministic: ties broken by link index. Returns an
// empty Route if the goal is unreachable (or equals the start).
Route findRoute(const NavGraph& graph, int startNode, int goalNode);

// Convenience: snap world points to their nearest nodes, then route.
Route findRouteBetween(const NavGraph& graph, const Vec2& start, const Vec2& goal);

}  // namespace engine

#endif
