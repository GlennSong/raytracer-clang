#include "nav_graph.h"

#include <algorithm>
#include <cmath>
#include <functional>

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
    // Knot merge (see NavBuildParams::junctionMergeRadius): collapse clusters of
    // nearby JUNCTION nodes into one intersection before building links, so a
    // generated crossing is one junction with real approach lengths instead of a
    // gridlock-prone smear of car-length stubs. Pure index remapping — degree-2
    // curve-sample nodes are untouched, so drawn road geometry is preserved.
    std::vector<Vec2> pos;
    std::vector<Real> elev;
    pos.reserve(roads.nodes.size());
    elev.reserve(roads.nodes.size());
    for (const RoadNode& rn : roads.nodes) {
        pos.push_back(rn.pos);
        elev.push_back(rn.elev);
    }
    std::vector<int> remap(pos.size());
    for (std::size_t i = 0; i < remap.size(); ++i) remap[i] = static_cast<int>(i);

    struct Edge { int a, b; RoadEdge e; };
    std::vector<Edge> edges;
    edges.reserve(roads.edges.size());
    for (const RoadEdge& e : roads.edges) edges.push_back({e.a, e.b, e});
    std::vector<Real> spread(pos.size(), 0.0);   // per-node knot spread (see header)

    if (params.junctionMergeRadius > 0 && !pos.empty()) {
        // Degree per node (distinct neighbours), on the raw graph.
        std::vector<std::vector<int>> nb(pos.size());
        for (const Edge& e : edges) {
            if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(pos.size()) ||
                e.b >= static_cast<int>(pos.size()))
                continue;
            nb[e.a].push_back(e.b);
            nb[e.b].push_back(e.a);
        }
        std::vector<int> deg(pos.size(), 0);
        for (std::size_t i = 0; i < pos.size(); ++i) {
            std::sort(nb[i].begin(), nb[i].end());
            nb[i].erase(std::unique(nb[i].begin(), nb[i].end()), nb[i].end());
            deg[i] = static_cast<int>(nb[i].size());
        }
        // Union-find over junction nodes within the merge radius.
        std::vector<int> parent(pos.size());
        for (std::size_t i = 0; i < parent.size(); ++i) parent[i] = static_cast<int>(i);
        std::function<int(int)> find = [&](int x) {
            while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
            return x;
        };
        const Real r2 = params.junctionMergeRadius * params.junctionMergeRadius;
        for (std::size_t i = 0; i < pos.size(); ++i) {
            if (deg[i] < 3) continue;
            for (std::size_t j = i + 1; j < pos.size(); ++j) {
                if (deg[j] < 3) continue;
                if ((pos[i] - pos[j]).lengthSquared() > r2) continue;
                parent[find(static_cast<int>(i))] = find(static_cast<int>(j));
            }
        }
        // Each cluster's representative moves to the member centroid.
        std::vector<Vec2> sum(pos.size(), Vec2(0, 0));
        std::vector<int> cnt(pos.size(), 0);
        for (std::size_t i = 0; i < pos.size(); ++i) {
            int r = find(static_cast<int>(i));
            sum[r] = sum[r] + pos[i];
            cnt[r]++;
        }
        for (std::size_t i = 0; i < pos.size(); ++i) {
            remap[i] = find(static_cast<int>(i));
            if (cnt[remap[i]] > 1)
                pos[remap[i]] = sum[remap[i]] / static_cast<Real>(cnt[remap[i]]);
        }
        // The knot's SPREAD: how far members sat from their cluster centroid.
        // Recorded per surviving node so placement (signal poles) can clear the
        // whole drawn crossing, which spans the knot around the merged node.
        std::vector<Real> spreadOf(pos.size(), 0.0);
        for (std::size_t i = 0; i < pos.size(); ++i) {
            const int r = find(static_cast<int>(i));
            if (cnt[r] < 2) continue;
            spreadOf[r] = std::max(spreadOf[r], (roads.nodes[i].pos - pos[r]).length());
        }
        // Compact away the merged-off members (a stale orphan node would still be
        // returned by nearestNode and strand trips), then remap edges through the
        // compaction; drop intra-knot stubs and duplicate pairs (keep the first —
        // insertion order keeps this deterministic).
        std::vector<int> compact(pos.size(), -1);
        std::vector<Vec2> packed;
        std::vector<Real> packedElev;
        std::vector<Real> packedSpread;
        packed.reserve(pos.size());
        for (std::size_t i = 0; i < pos.size(); ++i) {
            if (remap[i] != static_cast<int>(i)) continue;   // merged away
            compact[i] = static_cast<int>(packed.size());
            packed.push_back(pos[i]);
            packedElev.push_back(elev[i]);
            packedSpread.push_back(spreadOf[i]);
        }
        std::vector<Edge> kept;
        kept.reserve(edges.size());
        std::vector<std::pair<int, int>> seen;
        for (Edge e : edges) {
            e.a = compact[remap[e.a]];
            e.b = compact[remap[e.b]];
            if (e.a < 0 || e.b < 0 || e.a == e.b) continue;
            std::pair<int, int> key{std::min(e.a, e.b), std::max(e.a, e.b)};
            if (std::find(seen.begin(), seen.end(), key) != seen.end()) continue;
            seen.push_back(key);
            kept.push_back(e);
        }
        edges.swap(kept);
        pos.swap(packed);
        elev.swap(packedElev);
        spread.swap(packedSpread);
    }

    NavGraph g;
    g.nodes = pos;
    g.nodeSpread = spread;
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
        l.elevA = elev[a];
        l.elevB = elev[b];
        int idx = static_cast<int>(g.links.size());
        g.links.push_back(l);
        g.outLinks[a].push_back(idx);
    };

    for (const Edge& e : edges) {
        addLink(e.a, e.b, e.e);                                // forward
        if (!(params.oneWayRamps && e.e.klass == RoadClass::Ramp))
            addLink(e.b, e.a, e.e);                            // reverse (two-way)
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
