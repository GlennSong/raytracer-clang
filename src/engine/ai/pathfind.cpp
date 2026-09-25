#include "pathfind.h"

#include <algorithm>
#include <chrono>
#include <cmath>
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
// THE TIME A CAR LOSES AT A STREET INTERSECTION — signals, stop signs, yielding
// (Glenn, 2026-09-22: "I'd like more traffic on the freeway"). Travel time was
// length / class speed and nothing else, so a straight arterial through town cost
// the same as a free-flowing freeway at 28 m/s against 16 m/s — and in a city 1.4 x
// 2.4 km the detour out to the ring and back never paid. Real freeways win because
// an arterial stops every block; charging each street junction a few seconds is the
// same fact in the router. Freeway and ramp merges are free-flowing and cost nothing,
// and a walker (onFoot) is not charged.
constexpr Real kStreetJunctionDelay = 6.0;   // seconds
}  // namespace

Real Route::length(const NavGraph& g) const {
    Real total = 0;
    for (int li : links) total += g.links[li].length;
    return total;
}

RouteStats& routeStats() {
    thread_local RouteStats s;
    return s;
}

namespace {
Route findRouteImpl(const NavGraph& graph, int startNode, int goalNode, bool onFoot,
                    const std::vector<Real>* linkCostScale, long& expanded);
}

Route findRoute(const NavGraph& graph, int startNode, int goalNode,
                bool onFoot, const std::vector<Real>* linkCostScale) {
    const auto t0 = std::chrono::steady_clock::now();
    long expanded = 0;
    Route r = findRouteImpl(graph, startNode, goalNode, onFoot, linkCostScale, expanded);
    RouteStats& s = routeStats();
    ++s.calls;
    s.expanded += expanded;
    const void* caller = __builtin_return_address(0);
    auto it = std::find_if(s.callers.begin(), s.callers.end(), [&](const auto& c) { return c.first == caller; });
    if (it == s.callers.end()) s.callers.push_back({caller, 1}); else ++it->second;
    s.ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

namespace {
Route findRouteImpl(const NavGraph& graph, int startNode, int goalNode, bool onFoot,
                    const std::vector<Real>* linkCostScale, long& expanded) {
    Route route;
    const int n = graph.nodeCount();
    if (startNode < 0 || goalNode < 0 || startNode >= n || goalNode >= n) return route;
    if (startNode == goalNode) return route;

    const Real INF = std::numeric_limits<Real>::infinity();
    // Per-thread scratch, reused across searches: a node's g and cameLink count only when its
    // stamp is this search's. Filling two graph-sized arrays per call was most of a short
    // search's cost -- and a morning rush runs thousands of them in a step.
    struct Scratch {
        std::vector<Real> g;
        std::vector<int> came;
        std::vector<uint32_t> stamp;
        uint32_t gen = 0;
    };
    thread_local Scratch sc;
    if (sc.stamp.size() != static_cast<std::size_t>(n)) {
        sc.g.assign(static_cast<std::size_t>(n), INF);
        sc.came.assign(static_cast<std::size_t>(n), -1);
        sc.stamp.assign(static_cast<std::size_t>(n), 0);
        sc.gen = 0;
    }
    if (++sc.gen == 0) {   // wrapped: clear the stamps once
        std::fill(sc.stamp.begin(), sc.stamp.end(), 0u);
        sc.gen = 1;
    }
    const uint32_t gen = sc.gen;
    auto gOf = [&](int u) { return sc.stamp[static_cast<std::size_t>(u)] == gen ? sc.g[static_cast<std::size_t>(u)] : INF; };
    auto setG = [&](int u, Real v, int link) {
        sc.stamp[static_cast<std::size_t>(u)] = gen;
        sc.g[static_cast<std::size_t>(u)] = v;
        sc.came[static_cast<std::size_t>(u)] = link;
    };

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
    // the frontier: a reused vector heap, exactly priority_queue's push_heap / pop_heap ordering
    thread_local std::vector<QEntry> open;
    open.clear();
    const Cmp cmp;

    setG(startNode, 0, -1);
    open.push_back({heuristic(startNode), startNode});

    while (!open.empty()) {
        std::pop_heap(open.begin(), open.end(), cmp);
        const QEntry cur = open.back();
        open.pop_back();
        int u = cur.node;
        // Stale entry (a better g was found after this was queued).
        if (cur.f > gOf(u) + heuristic(u) + 1e-9) continue;
        if (u == goalNode) break;
        ++expanded;

        for (int li : graph.outLinks[u]) {
            const NavLink& link = graph.links[li];
            if (onFoot && (link.klass == RoadClass::Freeway ||
                       link.klass == RoadClass::Ramp || !link.walkable))
                continue;
            Real step = link.length / classSpeed(link.klass);
            if (!onFoot && link.to != goalNode && link.klass != RoadClass::Freeway &&
                link.klass != RoadClass::Ramp && graph.isJunction(link.to))
                step += kStreetJunctionDelay;
            if (linkCostScale && static_cast<std::size_t>(li) < linkCostScale->size())
                step *= std::max<Real>(1.0, (*linkCostScale)[static_cast<std::size_t>(li)]);
            Real tentative = gOf(u) + step;
            if (tentative + 1e-12 < gOf(link.to)) {
                setG(link.to, tentative, li);
                open.push_back({tentative + heuristic(link.to), link.to});
                std::push_heap(open.begin(), open.end(), cmp);
            }
        }
    }

    if (gOf(goalNode) == INF) return route;   // unreachable

    // Reconstruct the link sequence by walking cameLink back to the start.
    for (int node = goalNode; node != startNode;) {
        int li = sc.stamp[static_cast<std::size_t>(node)] == gen ? sc.came[static_cast<std::size_t>(node)] : -1;
        if (li < 0) { route.links.clear(); return route; }   // defensive
        route.links.push_back(li);
        node = graph.links[li].from;
    }
    std::reverse(route.links.begin(), route.links.end());
    return route;
}
}  // namespace

std::vector<char> reachableFrom(const NavGraph& graph, int startNode, bool onFoot) {
    const int n = graph.nodeCount();
    std::vector<char> seen(static_cast<std::size_t>(n), 0);
    if (startNode < 0 || startNode >= n) return seen;
    std::vector<int> stack{startNode};
    seen[static_cast<std::size_t>(startNode)] = 1;
    while (!stack.empty()) {
        const int u = stack.back();
        stack.pop_back();
        for (int li : graph.outLinks[static_cast<std::size_t>(u)]) {
            const NavLink& link = graph.links[static_cast<std::size_t>(li)];
            if (onFoot && (link.klass == RoadClass::Freeway || link.klass == RoadClass::Ramp || !link.walkable))
                continue;
            if (!seen[static_cast<std::size_t>(link.to)]) {
                seen[static_cast<std::size_t>(link.to)] = 1;
                stack.push_back(link.to);
            }
        }
    }
    return seen;
}

Route findRouteBetween(const NavGraph& graph, const Vec2& start, const Vec2& goal) {
    return findRoute(graph, graph.nearestNode(start), graph.nearestNode(goal));
}

Route findRouteBetweenOnFoot(const NavGraph& graph, const Vec2& start,
                             const Vec2& goal) {
    return findRoute(graph, graph.nearestNode(start), graph.nearestNode(goal),
                     /*onFoot=*/true);
}

std::vector<Vec2> routePolyline(const NavGraph& graph, const Route& route,
                                Real step, bool sidewalk, int lane) {
    std::vector<Real> speeds;
    return routePolylineWithSpeeds(graph, route, speeds, step, sidewalk, lane);
}

std::vector<Vec2> routePolylineWithSpeeds(const NavGraph& graph,
                                          const Route& route,
                                          std::vector<Real>& outSpeeds,
                                          Real step, bool sidewalk, int lane) {
    std::vector<Vec2> pts;
    outSpeeds.clear();
    if (step <= 0) step = 3.0;
    for (std::size_t leg = 0; leg < route.links.size(); ++leg) {
        const int li = route.links[leg];
        if (li < 0 || li >= static_cast<int>(graph.links.size())) continue;
        const NavLink& link = graph.links[li];
        const Real speed = classSpeed(link.klass);
        const int segs =
            std::max(1, static_cast<int>(std::ceil(link.length / step)));
        // The first link contributes its start point; later links skip t=0 —
        // it IS the previous link's end (the shared junction node).
        const int first = (leg == 0) ? 0 : 1;
        for (int s = first; s <= segs; ++s) {
            const Real t = static_cast<Real>(s) / segs;
            pts.push_back(sidewalk ? graph.sidewalkPoint(li, t)
                                   : graph.laneCenter(li, lane, t));
            outSpeeds.push_back(speed);
        }
    }
    return pts;
}

}  // namespace engine
