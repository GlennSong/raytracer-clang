#include "pathfind.h"

#include <algorithm>
#include <limits>
#include <queue>

namespace engine {

Real classSpeed(RoadClass klass) {
    switch (klass) {
        case RoadClass::Freeway:   return 28.0;   // ~100 km/h
        case RoadClass::Arterial:  return 16.0;   // ~58 km/h
        case RoadClass::Collector: return 12.0;
        case RoadClass::Local:     return 8.0;
        case RoadClass::Ramp:      return 10.0;
    }
    return 8.0;
}

namespace {
// The fastest class speed bounds the A* heuristic so it never overestimates the
// remaining travel time (admissibility -> optimal routes).
constexpr Real kMaxSpeed = 28.0;
}  // namespace

Real Route::length(const NavGraph& g) const {
    Real total = 0;
    for (int li : links) total += g.links[li].length;
    return total;
}

Route findRoute(const NavGraph& graph, int startNode, int goalNode) {
    Route route;
    const int n = graph.nodeCount();
    if (startNode < 0 || goalNode < 0 || startNode >= n || goalNode >= n) return route;
    if (startNode == goalNode) return route;

    const Real INF = std::numeric_limits<Real>::infinity();
    std::vector<Real> g(n, INF);          // best known travel time to a node
    std::vector<int> cameLink(n, -1);     // link used to reach a node on the best path

    auto heuristic = [&](int node) {
        return distance(graph.nodes[node], graph.nodes[goalNode]) / kMaxSpeed;
    };

    // Frontier ordered by f = g + h, tie-broken by node index for determinism.
    struct QEntry {
        Real f;
        int node;
    };
    struct Cmp {
        bool operator()(const QEntry& a, const QEntry& b) const {
            if (a.f != b.f) return a.f > b.f;
            return a.node > b.node;
        }
    };
    std::priority_queue<QEntry, std::vector<QEntry>, Cmp> open;

    g[startNode] = 0;
    open.push({heuristic(startNode), startNode});

    while (!open.empty()) {
        QEntry cur = open.top();
        open.pop();
        int u = cur.node;
        // Stale entry (a better g was found after this was queued).
        if (cur.f > g[u] + heuristic(u) + 1e-9) continue;
        if (u == goalNode) break;

        for (int li : graph.outLinks[u]) {
            const NavLink& link = graph.links[li];
            Real step = link.length / classSpeed(link.klass);
            Real tentative = g[u] + step;
            if (tentative + 1e-12 < g[link.to]) {
                g[link.to] = tentative;
                cameLink[link.to] = li;
                open.push({tentative + heuristic(link.to), link.to});
            }
        }
    }

    if (g[goalNode] == INF) return route;   // unreachable

    // Reconstruct the link sequence by walking cameLink back to the start.
    for (int node = goalNode; node != startNode;) {
        int li = cameLink[node];
        if (li < 0) { route.links.clear(); return route; }   // defensive
        route.links.push_back(li);
        node = graph.links[li].from;
    }
    std::reverse(route.links.begin(), route.links.end());
    return route;
}

Route findRouteBetween(const NavGraph& graph, const Vec2& start, const Vec2& goal) {
    return findRoute(graph, graph.nearestNode(start), graph.nearestNode(goal));
}

}  // namespace engine
