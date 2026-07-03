#include "nav_graph.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {
// Right-hand normal of a (unit) travel direction: rotate -90 deg. With Vec2
// mapping to world XZ and +Y up, perp() = {-y, x} is the LEFT (+90) normal, so
// the right side is its negation: {y, -x}.
inline Vec2 rightOf(const Vec2& dir) { return {dir.y, -dir.x}; }

// Distance from point p to segment [a,b], plus the clamped parameter.
Real pointSegDist(const Vec2& p, const Vec2& a, const Vec2& b) {
    Vec2 ab = b - a;
    Real len2 = ab.lengthSquared();
    Real t = len2 > 1e-12 ? dot(p - a, ab) / len2 : 0.0;
    t = t < 0 ? 0 : (t > 1 ? 1 : t);
    return (p - (a + ab * t)).length();
}
}  // namespace

Vec2 NavGraph::direction(int link) const {
    const NavLink& l = links[link];
    Vec2 d = nodes[l.to] - nodes[l.from];
    Real len = d.length();
    return len > 1e-9 ? d * (1.0 / len) : Vec2(1, 0);
}

Vec2 NavGraph::pointOnLink(int link, Real t) const {
    const NavLink& l = links[link];
    return lerp(nodes[l.from], nodes[l.to], t);
}

Vec2 NavGraph::laneCenter(int link, int lane, Real t, Real laneWidth) const {
    Vec2 c = pointOnLink(link, t);
    Vec2 r = rightOf(direction(link));
    return c + r * ((0.5 + lane) * laneWidth);
}

Vec2 NavGraph::sidewalkPoint(int link, Real t, Real verge) const {
    Vec2 c = pointOnLink(link, t);
    Vec2 r = rightOf(direction(link));
    return c + r * (links[link].width * 0.5 + verge);
}

int NavGraph::nearestNode(const Vec2& p) const {
    int best = -1;
    Real bestD2 = 0;
    for (int i = 0; i < nodeCount(); ++i) {
        Real d2 = (nodes[i] - p).lengthSquared();
        if (best < 0 || d2 < bestD2) { best = i; bestD2 = d2; }
    }
    return best;
}

int NavGraph::nearestLink(const Vec2& p) const {
    int best = -1;
    Real bestD = 0;
    for (int i = 0; i < linkCount(); ++i) {
        Real d = pointSegDist(p, nodes[links[i].from], nodes[links[i].to]);
        if (best < 0 || d < bestD) { best = i; bestD = d; }
    }
    return best;
}

NavGraph buildNavGraph(const RoadGraph& roads, const NavBuildParams& params) {
    NavGraph g;
    g.nodes.reserve(roads.nodes.size());
    for (const RoadNode& n : roads.nodes) g.nodes.push_back(n.pos);
    g.outLinks.resize(g.nodes.size());

    const int n = static_cast<int>(g.nodes.size());
    auto addLink = [&](int a, int b, const RoadEdge& e) {
        if (a < 0 || b < 0 || a >= n || b >= n || a == b) return;
        NavLink l;
        l.from = a;
        l.to = b;
        l.length = (g.nodes[b] - g.nodes[a]).length();
        l.width = e.width;
        l.klass = e.klass;
        // Lanes per direction from carriageway width (both directions share it).
        int lanes = static_cast<int>(std::lround(e.width / (params.laneWidth * 2.0)));
        l.lanes = lanes < 1 ? 1 : lanes;
        l.layer = e.layer;
        int idx = static_cast<int>(g.links.size());
        g.links.push_back(l);
        g.outLinks[a].push_back(idx);
    };

    for (const RoadEdge& e : roads.edges) {
        addLink(e.a, e.b, e);                                  // forward
        if (!(params.oneWayRamps && e.klass == RoadClass::Ramp))
            addLink(e.b, e.a, e);                              // reverse (two-way)
    }

    // Flag intersections: a node with three or more distinct neighbours (counting
    // both edge endpoints, so one-way ramps still register). Agents slow here.
    std::vector<std::vector<int>> nbr(g.nodes.size());
    for (const NavLink& l : g.links) {
        nbr[l.from].push_back(l.to);
        nbr[l.to].push_back(l.from);
    }
    g.junction.assign(g.nodes.size(), 0);
    for (int i = 0; i < n; ++i) {
        std::vector<int>& v = nbr[i];
        std::sort(v.begin(), v.end());
        v.erase(std::unique(v.begin(), v.end()), v.end());
        if (v.size() >= 3) g.junction[i] = 1;
    }
    return g;
}

}  // namespace engine
