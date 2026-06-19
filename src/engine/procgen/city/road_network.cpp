#include "road_network.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

struct Rng {
    uint32_t s;
    explicit Rng(uint32_t seed) : s(seed ? seed : 0x6c078965u) {}
    uint32_t next() { s ^= s << 13; s ^= s >> 17; s ^= s << 5; return s; }
    Real unit() { return (next() >> 8) * (1.0 / 16777216.0); }
    Real range(Real a, Real b) { return a + (b - a) * unit(); }
};

// Proper interior intersection of segments p1p2 and p3p4. Excludes shared/near
// endpoints (eps), so only true crossings split an edge. Returns t along p1p2.
bool segCross(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4,
              Vec2& out, Real& t) {
    Vec2 d1 = p2 - p1, d2 = p4 - p3;
    Real denom = cross(d1, d2);
    if (std::abs(denom) < 1e-9) return false;        // parallel/collinear
    Vec2 d13 = p3 - p1;
    t = cross(d13, d2) / denom;
    Real u = cross(d13, d1) / denom;
    const Real eps = 1e-6;
    if (t <= eps || t >= 1 - eps || u <= eps || u >= 1 - eps) return false;
    out = p1 + d1 * t;
    return true;
}

}  // namespace

int RoadGraph::addNode(const Vec2& p, Real tol) {
    Real tol2 = tol * tol;
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if ((nodes[i].pos - p).lengthSquared() <= tol2) return static_cast<int>(i);
    nodes.push_back({p});
    return static_cast<int>(nodes.size() - 1);
}

void RoadGraph::addEdge(int a, int b, Real width, RoadClass klass) {
    if (a == b) return;
    for (const RoadEdge& e : edges)                  // no duplicate undirected edge
        if ((e.a == a && e.b == b) || (e.a == b && e.b == a)) return;
    edges.push_back({a, b, width, klass});
}

RoadGraph gridRoads(const GridRoadParams& p) {
    RoadGraph g;
    Rng rng(p.seed);
    int n = std::max(1, static_cast<int>(std::lround(p.extent * 2 / p.cellSize)));
    int dim = n + 1;
    Real step = p.extent * 2 / n;
    Vec2 o = p.center - Vec2(p.extent, p.extent);

    // Node grid with interior jitter.
    std::vector<std::vector<int>> id(dim, std::vector<int>(dim, -1));
    for (int j = 0; j < dim; ++j)
        for (int i = 0; i < dim; ++i) {
            Vec2 pos = o + Vec2(i * step, j * step);
            bool interior = (i > 0 && i < dim - 1 && j > 0 && j < dim - 1);
            if (interior) {
                Real jr = p.jitter * step;
                pos += Vec2(rng.range(-jr, jr), rng.range(-jr, jr));
            }
            id[j][i] = g.addNode(pos, 0.01);
        }

    auto classOf = [&](int line) -> RoadClass {
        if (line == 0 || line == dim - 1) return RoadClass::Arterial;
        return (line % 3 == 0) ? RoadClass::Collector : RoadClass::Local;
    };
    auto widthOf = [&](RoadClass c) {
        return c == RoadClass::Arterial ? p.arterialWidth
             : c == RoadClass::Collector ? p.collectorWidth : p.localWidth;
    };

    // Horizontal + vertical segments. Local streets may drop out, but border
    // arterials always stay (so the region stays enclosed).
    for (int j = 0; j < dim; ++j)
        for (int i = 0; i < dim; ++i) {
            if (i + 1 < dim) {
                RoadClass c = classOf(j);
                if (!(c == RoadClass::Local && rng.unit() < p.dropout))
                    g.addEdge(id[j][i], id[j][i + 1], widthOf(c), c);
            }
            if (j + 1 < dim) {
                RoadClass c = classOf(i);
                if (!(c == RoadClass::Local && rng.unit() < p.dropout))
                    g.addEdge(id[j][i], id[j + 1][i], widthOf(c), c);
            }
        }
    return g;
}

RoadGraph planarize(const RoadGraph& in, Real tol) {
    RoadGraph out;
    // Re-key nodes into the output (dedup by tol).
    std::vector<int> remap(in.nodes.size());
    for (std::size_t i = 0; i < in.nodes.size(); ++i)
        remap[i] = out.addNode(in.nodes[i].pos, tol);

    for (const RoadEdge& e : in.edges) {
        Vec2 a = in.nodes[e.a].pos, b = in.nodes[e.b].pos;
        // Collect crossing points with every other edge.
        std::vector<std::pair<Real, Vec2>> cuts;   // (t, point)
        for (const RoadEdge& o : in.edges) {
            if (&o == &e) continue;
            Vec2 c = in.nodes[o.a].pos, d = in.nodes[o.b].pos;
            Vec2 x; Real t;
            if (segCross(a, b, c, d, x, t)) cuts.emplace_back(t, x);
        }
        std::sort(cuts.begin(), cuts.end(),
                  [](const auto& l, const auto& r) { return l.first < r.first; });
        // Chain: a -> cut0 -> cut1 -> ... -> b.
        int prev = remap[e.a];
        for (const auto& cut : cuts) {
            int mid = out.addNode(cut.second, tol);
            out.addEdge(prev, mid, e.width, e.klass);
            prev = mid;
        }
        out.addEdge(prev, remap[e.b], e.width, e.klass);
    }
    return out;
}

std::vector<Poly2> extractBlocks(const RoadGraph& g, Real minArea) {
    std::vector<Poly2> blocks;
    const int nNodes = static_cast<int>(g.nodes.size());
    if (nNodes < 3 || g.edges.empty()) return blocks;

    // Directed half-edges: each undirected edge -> two, twins adjacent (2k, 2k+1).
    struct Half { int tail, head, twin; Real angle; };
    std::vector<Half> he;
    he.reserve(g.edges.size() * 2);
    for (const RoadEdge& e : g.edges) {
        int i = static_cast<int>(he.size());
        Vec2 va = g.nodes[e.a].pos, vb = g.nodes[e.b].pos;
        he.push_back({e.a, e.b, i + 1, std::atan2(vb.y - va.y, vb.x - va.x)});
        he.push_back({e.b, e.a, i,     std::atan2(va.y - vb.y, va.x - vb.x)});
    }

    // Outgoing half-edges per node, sorted CCW by angle.
    std::vector<std::vector<int>> outAt(nNodes);
    for (int i = 0; i < static_cast<int>(he.size()); ++i) outAt[he[i].tail].push_back(i);
    for (auto& list : outAt)
        std::sort(list.begin(), list.end(),
                  [&](int x, int y) { return he[x].angle < he[y].angle; });

    // next(h): at head(h), take the half-edge clockwise-adjacent to the twin —
    // the previous entry in the CCW-sorted outgoing list. Traces interior faces
    // CCW (positive area); the unbounded outer face comes out CW (negative).
    auto nextHalf = [&](int h) {
        int v = he[h].head;
        int twin = he[h].twin;
        const std::vector<int>& list = outAt[v];
        int idx = 0;
        for (int k = 0; k < static_cast<int>(list.size()); ++k)
            if (list[k] == twin) { idx = k; break; }
        int deg = static_cast<int>(list.size());
        return list[(idx + deg - 1) % deg];
    };

    std::vector<char> used(he.size(), 0);
    for (int start = 0; start < static_cast<int>(he.size()); ++start) {
        if (used[start]) continue;
        Poly2 face;
        int h = start;
        int guard = 0;
        do {
            used[h] = 1;
            face.push_back(g.nodes[he[h].tail].pos);
            h = nextHalf(h);
            if (++guard > nNodes * 4 + 8) break;     // malformed: bail
        } while (h != start && !used[h]);

        if (face.size() < 3) continue;
        Real sa = signedArea(face);
        if (sa > 0 && sa >= minArea) blocks.push_back(std::move(face));
    }
    return blocks;
}

}  // namespace engine
