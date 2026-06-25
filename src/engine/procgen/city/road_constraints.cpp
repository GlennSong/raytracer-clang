#include "road_constraints.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace engine {
namespace {

constexpr double kTwoPi = 6.283185307179586;

// Incident edge indices per node.
std::vector<std::vector<int>> incidence(const RoadGraph& g) {
    std::vector<std::vector<int>> inc(g.nodes.size());
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
        inc[g.edges[e].a].push_back(e);
        inc[g.edges[e].b].push_back(e);
    }
    return inc;
}

struct Arm {
    int   edge;        // incident edge index
    bool  vIsA;        // is node v the `a` endpoint of that edge?
    double angle;      // direction from v toward the other end (radians)
    double halfWidth;
};

// Arms of node v, sorted CCW by angle. Empty for an isolated node.
std::vector<Arm> armsOf(const RoadGraph& g, int v, const std::vector<int>& inc) {
    std::vector<Arm> arms;
    for (int e : inc) {
        const RoadEdge& ed = g.edges[e];
        bool vIsA = (ed.a == v);
        int other = vIsA ? ed.b : ed.a;
        Vec2 d = g.nodes[other].pos - g.nodes[v].pos;
        if (d.lengthSquared() < 1e-12) continue;        // zero-length: skip
        arms.push_back({e, vIsA, std::atan2(d.y, d.x), g.edgeWidth(e) * 0.5});
    }
    std::sort(arms.begin(), arms.end(),
              [](const Arm& a, const Arm& b) { return a.angle < b.angle; });
    return arms;
}

// CCW gap from arm k to its neighbour (k+1, wrapping). Always in (0, 2*pi].
double gapAfter(const std::vector<Arm>& arms, int k) {
    int m = static_cast<int>(arms.size());
    double g = arms[(k + 1) % m].angle - arms[k].angle;
    if (g <= 0) g += kTwoPi;
    return g;
}

double tightestGap(const std::vector<Arm>& arms) {
    double mn = kTwoPi;
    for (int k = 0; k < static_cast<int>(arms.size()); ++k)
        mn = std::min(mn, gapAfter(arms, k));
    return mn;
}

// Ring radius: the tightest adjacent pair drives it, so adjacent attach points clear each
// other (chord 2*R*sin(gap/2) >= the two half-widths + margin). This is the SAME
// w/sin(theta) geometry the analytic trim diverges on — used here to size the ring.
double chooseRadius(const std::vector<Arm>& arms, const RoadRules& r) {
    if (r.roundaboutRadius > 0.0)
        return std::clamp(r.roundaboutRadius, r.islandRadius, r.maxRadius);
    // Floor the radius at a multiple of the widest arm so the ring reads as a real roundabout
    // (a drivable annulus around a visible island), not just a hole where spokes merge.
    double maxHalf = 0.0;
    for (const Arm& a : arms) maxHalf = std::max(maxHalf, a.halfWidth);
    double need = std::max(r.islandRadius, maxHalf * r.ringWidthFactor);
    for (int k = 0; k < static_cast<int>(arms.size()); ++k) {
        double half = gapAfter(arms, k) * 0.5;
        double s = std::sin(std::max(half, 1e-3));
        const Arm& a = arms[k];
        const Arm& b = arms[(k + 1) % arms.size()];
        need = std::max(need, (a.halfWidth + b.halfWidth + r.clearMargin) / (2.0 * s));
    }
    return std::clamp(need, r.islandRadius, r.maxRadius);
}

}  // namespace

bool nodeNeedsRoundabout(const RoadGraph& g, int v, const RoadRules& rules) {
    if (v < 0 || v >= static_cast<int>(g.nodes.size())) return false;
    auto inc = incidence(g);
    std::vector<Arm> arms = armsOf(g, v, inc[v]);
    if (static_cast<int>(arms.size()) < 3) return false;        // never a through/dead-end
    if (static_cast<int>(arms.size()) > rules.maxDegree) return true;
    return tightestGap(arms) < rules.minArmAngle;
}

RoadGraph applyConstraints(const RoadGraph& in, const RoadRules& rules) {
    // Decide promotions up front from the ORIGINAL degrees, so re-pointing one node's arms
    // can't change another's classification mid-pass.
    auto inc0 = incidence(in);
    std::vector<int> promote;
    for (int v = 0; v < static_cast<int>(in.nodes.size()); ++v) {
        std::vector<Arm> arms = armsOf(in, v, inc0[v]);
        if (static_cast<int>(arms.size()) < 3) continue;
        bool busy = static_cast<int>(arms.size()) > rules.maxDegree;
        bool acute = tightestGap(arms) < rules.minArmAngle;
        if (busy || acute) promote.push_back(v);
    }
    if (promote.empty()) return in;     // nothing to do: pass through unchanged

    RoadGraph g = in;     // work on a copy; push_back never invalidates existing indices,
                          // and we only re-point edges incident to the node being promoted.
    for (int v : promote) {
        std::vector<Arm> arms = armsOf(g, v, incidence(g)[v]);
        if (static_cast<int>(arms.size()) < 3) continue;        // defensive
        double R = chooseRadius(arms, rules);
        Vec2 V = g.nodes[v].pos;

        // The ring inherits the widest arm and the most-major class of the node.
        Real ringW = 0;
        RoadClass ringK = RoadClass::Local;
        for (const Arm& a : arms) {
            ringW = std::max(ringW, static_cast<Real>(a.halfWidth * 2.0));
            ringK = std::min(ringK, g.edges[a.edge].klass);     // enum: Arterial=0 most major
        }

        // One attach node per arm, on the ring at the arm's bearing; re-point the spoke
        // from the super-node onto it (the spoke now meets the ring, not a point).
        int m = static_cast<int>(arms.size());
        std::vector<int> attach(m);
        for (int k = 0; k < m; ++k) {
            Vec2 p = V + Vec2(std::cos(arms[k].angle), std::sin(arms[k].angle)) * R;
            attach[k] = static_cast<int>(g.nodes.size());
            g.nodes.push_back(RoadNode{p});
            RoadEdge& spoke = g.edges[arms[k].edge];
            (arms[k].vIsA ? spoke.a : spoke.b) = attach[k];
        }
        // Close the ring: a sampled arc between each pair of adjacent attach points, so the
        // carriageway reads as a true circle and every attach node ends up degree 3.
        for (int k = 0; k < m; ++k) {
            double a0 = arms[k].angle;
            double a1 = a0 + gapAfter(arms, k);
            RoadArc arc{V, R, a0, a1};
            sampleArc(g, attach[k], attach[(k + 1) % m], arc, ringW, ringK,
                      rules.arcChordError, 2);
        }
    }

    // Drop the now-orphaned super-nodes and compact the node list.
    std::vector<int> remap(g.nodes.size(), -1);
    for (const RoadEdge& e : g.edges) { remap[e.a] = 0; remap[e.b] = 0; }
    RoadGraph out;
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i)
        if (remap[i] == 0) { remap[i] = static_cast<int>(out.nodes.size()); out.nodes.push_back(g.nodes[i]); }
    out.edges.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges)
        out.edges.push_back(RoadEdge{remap[e.a], remap[e.b], e.width, e.klass});
    return out;
}

}  // namespace engine
