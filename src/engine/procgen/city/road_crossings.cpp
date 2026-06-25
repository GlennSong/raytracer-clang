#include "road_crossings.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

constexpr double kEps = 1e-6;

// Intersection of segments p1p2 and p3p4, endpoints INCLUDED (so T-junctions and shared termini
// count). On a hit, ta/tb are the params along each segment (clamped to [0,1]) and `pt` the point.
bool segInter(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4,
              double& ta, double& tb, Vec2& pt) {
    Vec2 r = p2 - p1, s = p4 - p3;
    double denom = cross(r, s);
    if (std::fabs(denom) < 1e-12) return false;          // parallel / collinear: not a crossing
    Vec2 qp = p3 - p1;
    ta = cross(qp, s) / denom;
    tb = cross(qp, r) / denom;
    if (ta < -kEps || ta > 1.0 + kEps || tb < -kEps || tb > 1.0 + kEps) return false;
    ta = std::clamp(ta, 0.0, 1.0);
    tb = std::clamp(tb, 0.0, 1.0);
    pt = p1 + r * ta;
    return true;
}

// A crossing point landing on one spine: which segment, how far along it, and where.
struct Split { int seg; double t; Vec2 pt; };

// Rebuild a polyline with the split points spliced in (in order along each segment), dropping any
// that coincide with an existing vertex so the welds share an exact vertex without duplicates.
std::vector<Vec2> spliceSplits(const std::vector<Vec2>& pts, std::vector<Split> splits) {
    std::sort(splits.begin(), splits.end(),
              [](const Split& a, const Split& b) { return a.seg != b.seg ? a.seg < b.seg : a.t < b.t; });
    std::vector<Vec2> out;
    const int n = static_cast<int>(pts.size());
    for (int seg = 0; seg + 1 < n; ++seg) {
        out.push_back(pts[seg]);
        for (const Split& sp : splits) {
            if (sp.seg != seg) continue;
            if ((sp.pt - pts[seg]).length() < kEps || (sp.pt - pts[seg + 1]).length() < kEps) continue;
            if (!out.empty() && (sp.pt - out.back()).length() < kEps) continue;
            out.push_back(sp.pt);
        }
    }
    out.push_back(pts[n - 1]);
    return out;
}

}  // namespace

ResolvedGraph resolveCrossings(const std::vector<GraphSpine>& in, double weldTol) {
    const int ns = static_cast<int>(in.size());
    std::vector<std::vector<Split>> splits(ns);          // per-spine split points to splice
    struct Hit { Vec2 pt; int a, b; };                   // a crossing of spines a and b
    std::vector<Hit> hits;

    for (int i = 0; i < ns; ++i)
        for (int j = i + 1; j < ns; ++j) {
            const std::vector<Vec2>& A = in[i].points;
            const std::vector<Vec2>& B = in[j].points;
            for (int si = 0; si + 1 < static_cast<int>(A.size()); ++si)
                for (int sj = 0; sj + 1 < static_cast<int>(B.size()); ++sj) {
                    double ta, tb; Vec2 pt;
                    if (!segInter(A[si], A[si + 1], B[sj], B[sj + 1], ta, tb, pt)) continue;
                    splits[i].push_back({si, ta, pt});
                    splits[j].push_back({sj, tb, pt});
                    hits.push_back({pt, i, j});
                }
        }

    ResolvedGraph g;
    g.spines.reserve(ns);
    for (int i = 0; i < ns; ++i) {
        GraphSpine s = in[i];
        if (!splits[i].empty()) s.points = spliceSplits(in[i].points, splits[i]);
        g.spines.push_back(std::move(s));
    }

    // Cluster crossing points within weldTol into shared nodes; a node is grade-separated when its
    // incident spines don't all share one layer.
    std::vector<char> used(hits.size(), 0);
    for (std::size_t h = 0; h < hits.size(); ++h) {
        if (used[h]) continue;
        CrossNode node;
        node.pos = hits[h].pt;
        std::vector<int> sp;
        int loLayer = 1 << 30, hiLayer = -(1 << 30);
        for (std::size_t k = h; k < hits.size(); ++k) {
            if (used[k] || (hits[k].pt - node.pos).length() > weldTol) continue;
            used[k] = 1;
            for (int idx : {hits[k].a, hits[k].b}) {
                if (std::find(sp.begin(), sp.end(), idx) == sp.end()) sp.push_back(idx);
                loLayer = std::min(loLayer, in[idx].layer);
                hiLayer = std::max(hiLayer, in[idx].layer);
            }
        }
        std::sort(sp.begin(), sp.end());
        node.spines = std::move(sp);
        node.gradeSeparated = (hiLayer != loLayer);
        g.nodes.push_back(std::move(node));
    }
    return g;
}

}  // namespace engine
