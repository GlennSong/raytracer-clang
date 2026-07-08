#include "ped_graph.h"

#include <algorithm>
#include <cmath>
#include <queue>

namespace citysim {

namespace {
Real dist(Vec2 a, Vec2 b) {
    const Real dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}
}  // namespace

PedGraph buildPedGraph(const engine::NavGraph& roads, const PlaceMap& places) {
    PedGraph g;
    // One corner node per road node (same position — sidewalks hug the road, and
    // a shared corner is where a junction's sidewalks meet, i.e. an implicit
    // crossing). Index-parallel to roads.nodes so link endpoints map directly.
    g.nodes = roads.nodes;
    g.adj.assign(g.nodes.size(), {});

    auto addEdge = [&](int a, int b, PedEdge::Kind kind) {
        if (a == b) return;
        PedEdge e;
        e.a = a;
        e.b = b;
        e.length = dist(g.nodes[a], g.nodes[b]);
        e.kind = kind;
        const int ei = static_cast<int>(g.edges.size());
        g.edges.push_back(e);
        g.adj[a].push_back(ei);
        g.adj[b].push_back(ei);
    };

    // A sidewalk edge per road link. The road graph is directed (two NavLinks per
    // two-way road); dedup to one undirected sidewalk edge per node pair so a
    // corner isn't wired to its neighbour twice.
    std::vector<std::pair<int, int>> seen;
    for (const engine::NavLink& link : roads.links) {
        int a = link.from, b = link.to;
        if (a > b) std::swap(a, b);
        const std::pair<int, int> key{a, b};
        if (std::find(seen.begin(), seen.end(), key) != seen.end()) continue;
        seen.push_back(key);
        addEdge(a, b, PedEdge::Kind::Sidewalk);
    }

    // An entrance node + connector per place: the door snaps to its nearest road
    // corner (walkers reach the door via that corner's sidewalk).
    g.entranceNode.assign(places.size(), -1);
    for (const Place& p : places.places()) {
        const int corner = roads.nearestNode(p.entrance);
        if (corner < 0) continue;
        const int door = static_cast<int>(g.nodes.size());
        g.nodes.push_back(p.entrance);
        g.adj.push_back({});
        addEdge(door, corner, PedEdge::Kind::Entrance);
        g.entranceNode[p.id] = door;
    }
    return g;
}

std::vector<int> pedRoute(const PedGraph& g, int from, int to) {
    const int n = g.nodeCount();
    if (from < 0 || to < 0 || from >= n || to >= n) return {};
    if (from == to) return {from};

    const Real kInf = std::numeric_limits<Real>::infinity();
    std::vector<Real> gScore(n, kInf);
    std::vector<int> came(n, -1);
    std::vector<uint8_t> closed(n, 0);
    const Vec2 goal = g.nodes[to];
    auto heur = [&](int v) { return dist(g.nodes[v], goal); };

    // (fScore, node) min-heap.
    using QItem = std::pair<Real, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> open;
    gScore[from] = 0;
    open.push({heur(from), from});

    while (!open.empty()) {
        const int u = open.top().second;
        open.pop();
        if (closed[u]) continue;
        closed[u] = 1;
        if (u == to) break;
        for (int ei : g.adj[u]) {
            const PedEdge& e = g.edges[ei];
            const int v = e.a == u ? e.b : e.a;   // undirected
            if (closed[v]) continue;
            const Real cand = gScore[u] + e.length;
            if (cand < gScore[v]) {
                gScore[v] = cand;
                came[v] = u;
                open.push({cand + heur(v), v});
            }
        }
    }

    if (gScore[to] == kInf) return {};
    std::vector<int> path;
    for (int v = to; v != -1; v = came[v]) path.push_back(v);
    std::reverse(path.begin(), path.end());
    return path;
}

}  // namespace citysim
