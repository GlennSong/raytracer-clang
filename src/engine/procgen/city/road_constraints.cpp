#include "road_constraints.h"

#include <algorithm>
#include <array>
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

// Split an over-connected node into two nodes joined by a short link, each within the degree cap.
// Arms are partitioned into two angularly-contiguous groups at the widest gap; each new node sits
// offset from the original along its group's mean bearing, so the link reads as a short staggered
// street. The peeled node B carries up to maxDegree-1 arms; if the A side is still over the cap,
// capDegree's loop splits it again.
void splitOverNode(RoadGraph& g, int v, const RoadRules& rules) {
    std::vector<Arm> arms = armsOf(g, v, incidence(g)[v]);
    int d = static_cast<int>(arms.size());
    if (d <= rules.maxDegree) return;

    int cap = std::max(1, rules.maxDegree - 1);       // each new node also carries the link edge
    int kB = std::min(cap, d - 1);                    // peel up to `cap` arms onto the new node

    // Start the peeled block just after the widest angular gap, so the split follows a natural seam.
    int start = 0; double widest = -1.0;
    for (int k = 0; k < d; ++k) {
        double gp = gapAfter(arms, k);
        if (gp > widest) { widest = gp; start = (k + 1) % d; }
    }
    std::vector<char> toB(d, 0);
    for (int j = 0; j < kB; ++j) toB[(start + j) % d] = 1;

    // Mean bearing of each group -> the two offset directions; widest arm -> the link width/offset.
    Vec2 dirA(0, 0), dirB(0, 0);
    double maxHalf = 0.0;
    Real linkW = 0;
    RoadClass linkK = RoadClass::Local;
    for (int k = 0; k < d; ++k) {
        Vec2 u(std::cos(arms[k].angle), std::sin(arms[k].angle));
        if (toB[k]) dirB = dirB + u; else dirA = dirA + u;
        maxHalf = std::max(maxHalf, arms[k].halfWidth);
        linkW = std::max(linkW, static_cast<Real>(arms[k].halfWidth * 2.0));
        linkK = std::min(linkK, g.edges[arms[k].edge].klass);    // enum: Arterial=0 most major
    }
    auto unit = [](Vec2 u) { double l = u.length(); return l > 1e-9 ? u * (1.0 / l) : Vec2(1, 0); };
    double offset = std::max(maxHalf * 1.5, 4.0);     // short staggered link

    Vec2 V = g.nodes[v].pos;
    int B = static_cast<int>(g.nodes.size());
    g.nodes.push_back(RoadNode{V + unit(dirB) * offset});
    g.nodes[v].pos = V + unit(dirA) * offset;          // node A reuses v, nudged onto its side
    for (int k = 0; k < d; ++k)
        if (toB[k]) {
            RoadEdge& e = g.edges[arms[k].edge];
            (arms[k].vIsA ? e.a : e.b) = B;            // re-point the peeled arms onto the new node
        }
    g.addEdge(v, B, linkW, linkK);                     // the short link between the two halves
}

}  // namespace

bool nodeNeedsRoundabout(const RoadGraph& g, int v, const RoadRules& rules) {
    if (!rules.autoRoundabout) return false;
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
    if (rules.autoRoundabout)
        for (int v = 0; v < static_cast<int>(in.nodes.size()); ++v) {
            std::vector<Arm> arms = armsOf(in, v, inc0[v]);
            if (static_cast<int>(arms.size()) < 3) continue;
            bool busy = static_cast<int>(arms.size()) > rules.maxDegree;
            bool acute = tightestGap(arms) < rules.minArmAngle;
            if (busy || acute) promote.push_back(v);
        }
    if (promote.empty()) return in;     // nothing to do (or auto-roundabouts off): pass through

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
        out.edges.push_back(RoadEdge{remap[e.a], remap[e.b], e.width, e.klass, e.layer});
    return out;
}

RoadGraph capDegree(const RoadGraph& in, const RoadRules& rules) {
    RoadGraph g = in;
    // Each split lowers one node's degree but may leave the other side still over the cap (a very
    // busy hub needs several), so loop until none exceeds it. Recompute incidence each pass — a
    // split adds a node + edge and re-points arms. The guard is a runaway backstop, never reached
    // for real graphs (degree falls by >=1 each split).
    for (int guard = 0; guard < 256; ++guard) {
        auto inc = incidence(g);
        int over = -1;
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v)
            if (static_cast<int>(inc[v].size()) > rules.maxDegree) { over = v; break; }
        if (over < 0) break;
        splitOverNode(g, over, rules);
    }
    return g;
}

RoadGraph relaxSharpBends(const RoadGraph& in, Real maxTurn, int iterations) {
    RoadGraph g = in;
    const int n = static_cast<int>(g.nodes.size());
    // Degree-2 adjacency (through-nodes only; junctions/dead-ends are pinned).
    std::vector<std::array<int, 2>> nbr(n, {-1, -1});
    std::vector<int> deg(n, 0);
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        if (deg[e.a] < 2) nbr[e.a][deg[e.a]] = e.b;
        if (deg[e.b] < 2) nbr[e.b][deg[e.b]] = e.a;
        ++deg[e.a]; ++deg[e.b];
    }
    const Real cosLimit = std::cos(maxTurn);
    for (int it = 0; it < iterations; ++it) {
        bool any = false;
        for (int v = 0; v < n; ++v) {
            if (deg[v] != 2) continue;
            const Vec2& a = g.nodes[nbr[v][0]].pos;
            const Vec2& b = g.nodes[nbr[v][1]].pos;
            Vec2 m = g.nodes[v].pos;
            Vec2 d0 = m - a, d1 = b - m;
            Real l0 = d0.length(), l1 = d1.length();
            if (l0 < 1e-6 || l1 < 1e-6) continue;
            if (dot(d0, d1) / (l0 * l1) >= cosLimit) continue;   // gentle enough
            // Ease the corner toward the chord midpoint — each pass halves the
            // deviation, so even a hairpin converges to a drivable bend.
            g.nodes[v].pos = m * 0.5 + (a + b) * 0.25;
            any = true;
        }
        if (!any) break;
    }
    return g;
}

RoadGraph mergeShortEdges(const RoadGraph& in, Real minLen, int maxDegree) {
    RoadGraph g = in;
    auto degreeOf = [&](int v) {
        int d = 0;
        for (const RoadEdge& e : g.edges)
            if (e.a == v || e.b == v) ++d;
        return d;
    };
    // Take the globally shortest offender each round (deterministic: ties fall to
    // the lower edge index) so nearby short edges resolve in a stable order. Each
    // round either removes an edge (merge) or brings one up to minLen (lengthen),
    // so the loop terminates; the guard is a backstop for pathological graphs.
    for (int guard = 0; guard < 512; ++guard) {
        int shortest = -1;
        Real shortestLen = minLen;
        for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
            const RoadEdge& e = g.edges[ei];
            if (e.a == e.b) continue;
            Real len = (g.nodes[e.a].pos - g.nodes[e.b].pos).length();
            if (len < shortestLen - 1e-9) { shortestLen = len; shortest = ei; }
        }
        if (shortest < 0) break;
        const int a = g.edges[shortest].a, b = g.edges[shortest].b;

        // Unique arms of the united node (neighbours of a or b, minus themselves).
        std::vector<int> arms;
        for (const RoadEdge& e : g.edges) {
            for (int v : {e.a, e.b}) {
                int other = (v == e.a) ? e.b : e.a;
                if ((v == a || v == b) && other != a && other != b) {
                    bool seen = false;
                    for (int u : arms) if (u == other) seen = true;
                    if (!seen) arms.push_back(other);
                }
            }
        }

        if (static_cast<int>(arms.size()) <= maxDegree) {
            // MERGE b into a at the degree-weighted midpoint (the busier crossing
            // moves less, so a T folding into a 4-way barely disturbs the through
            // road). Then rewire, and drop self-loops + parallel duplicates.
            const Real da = std::max(1, degreeOf(a)), db = std::max(1, degreeOf(b));
            g.nodes[a].pos = (g.nodes[a].pos * da + g.nodes[b].pos * db) * (1.0 / (da + db));
            for (RoadEdge& e : g.edges) {
                if (e.a == b) e.a = a;
                if (e.b == b) e.b = a;
            }
            std::vector<RoadEdge> kept;
            kept.reserve(g.edges.size());
            for (const RoadEdge& e : g.edges) {
                if (e.a == e.b) continue;                       // the merged edge itself
                bool dup = false;
                for (RoadEdge& k : kept)
                    if ((k.a == e.a && k.b == e.b) || (k.a == e.b && k.b == e.a)) {
                        k.width = std::max(k.width, e.width);   // widest parallel wins
                        dup = true;
                        break;
                    }
                if (!dup) kept.push_back(e);
            }
            g.edges = std::move(kept);
        } else {
            // Can't merge without over-crowding the junction (e.g. the staggered
            // link a capDegree split leaves): LENGTHEN instead — push the two
            // junctions apart along the edge axis until the road is minLen.
            Vec2 axis = g.nodes[b].pos - g.nodes[a].pos;
            const Real len = axis.length();
            axis = len > 1e-9 ? axis * (1.0 / len) : Vec2(1, 0);
            const Real push = (minLen - len) * 0.5;
            g.nodes[a].pos = g.nodes[a].pos - axis * push;
            g.nodes[b].pos = g.nodes[b].pos + axis * push;
        }
    }
    // Compact orphaned nodes (a merge leaves its absorbed node unreferenced).
    std::vector<int> remap(g.nodes.size(), -1);
    for (const RoadEdge& e : g.edges) { remap[e.a] = 0; remap[e.b] = 0; }
    RoadGraph out;
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i)
        if (remap[i] == 0) { remap[i] = static_cast<int>(out.nodes.size()); out.nodes.push_back(g.nodes[i]); }
    out.edges.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges)
        out.edges.push_back(RoadEdge{remap[e.a], remap[e.b], e.width, e.klass, e.layer});
    return out;
}

}  // namespace engine
