#include "road_constraints.h"

#include "../../../log.h"

#include <algorithm>
#include <set>
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
    for (const RoadEdge& e : g.edges) {
        // Copy-then-remap: a positional re-init here silently dropped every
        // field after `layer` (walkable/spec/oneWay/provenance — and the
        // semantic access bits) whenever a node promoted.
        RoadEdge o = e;
        o.a = remap[e.a];
        o.b = remap[e.b];
        out.edges.push_back(o);
    }
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
    // An 8km multi-site metro with a big minLen legitimately has more offenders
    // than the flat 512, so the backstop scales with the graph.
    const int maxRounds =
        std::max(512, static_cast<int>(in.edges.size()) / 3);
    int rounds = 0;
    for (int guard = 0; guard < maxRounds; ++guard, ++rounds) {
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
    // Loud when the cleanup was really doing generation's job: surviving
    // offenders mean the guard bound (growth params are wrong for this minLen).
    int survivors = 0;
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        if ((g.nodes[e.a].pos - g.nodes[e.b].pos).length() < minLen - 1e-9) ++survivors;
    }
    if (survivors > 0)
        LOG_WARN << "mergeShortEdges: " << survivors << " edges still under "
                 << minLen << " m after " << rounds
                 << " rounds — growth spacing is too tight for this min_road_len";

    // Compact orphaned nodes (a merge leaves its absorbed node unreferenced).
    std::vector<int> remap(g.nodes.size(), -1);
    for (const RoadEdge& e : g.edges) { remap[e.a] = 0; remap[e.b] = 0; }
    RoadGraph out;
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i)
        if (remap[i] == 0) { remap[i] = static_cast<int>(out.nodes.size()); out.nodes.push_back(g.nodes[i]); }
    out.edges.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges) {
        // Copy-then-remap: a positional re-init here silently dropped every
        // field after `layer` (walkable/spec/oneWay/provenance — and the
        // semantic access bits) whenever a node promoted.
        RoadEdge o = e;
        o.a = remap[e.a];
        o.b = remap[e.b];
        out.edges.push_back(o);
    }
    return out;
}

namespace {
// Do segments (a,b) and (c,d) PROPERLY cross? Shared endpoints do not count —
// two arms leaving one node touch there by construction.
bool segsCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d) {
    auto side = [](const Vec2& p, const Vec2& q, const Vec2& r) {
        const double v = cross(q - p, r - p);
        return v > 1e-9 ? 1 : (v < -1e-9 ? -1 : 0);
    };
    const int d1 = side(a, b, c), d2 = side(a, b, d);
    const int d3 = side(c, d, a), d4 = side(c, d, b);
    return d1 * d2 < 0 && d3 * d4 < 0;
}
}  // namespace

RoadGraph realignAcuteJunctions(const RoadGraph& in, Real minAngle, Real runIn, int passes) {
    const double kTwoPi = 6.283185307179586;
    RoadGraph g = in;
    if (minAngle <= 0.0) return g;
    int dbgTight = 0, dbgApplied = 0, dbgBlocked = 0, dbgInfeasible = 0, dbgBaked = 0;
    // Bend nodes this pass inserted, so a later pass ROTATES the one it already
    // made instead of stacking a second one behind it (which would kink the
    // approach and never converge).
    std::vector<char> isBend(g.nodes.size(), 0);

    for (int pass = 0; pass < passes; ++pass) {
        const int N = static_cast<int>(g.nodes.size());
        isBend.resize(N, 0);
        std::vector<std::vector<std::pair<int, int>>> nbr(N);   // (far, edge)
        for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
            const RoadEdge& e = g.edges[ei];
            if (e.a == e.b) continue;
            nbr[e.a].push_back({e.b, ei});
            nbr[e.b].push_back({e.a, ei});
        }
        bool changed = false;

        for (int v = 0; v < N; ++v) {
            const int d = static_cast<int>(nbr[v].size());
            if (d < 3) continue;
            if (g.nodes[v].elevAbsolute) continue;          // a deck node: not ours to move
            // Arithmetically impossible to open: leave it for the next rung.
            if (d * static_cast<double>(minAngle) > kTwoPi) { ++dbgInfeasible; continue; }

            struct Arm { double ang; int far; int edge; };
            std::vector<Arm> arms;
            arms.reserve(d);
            bool baked = false;
            for (const auto& fe : nbr[v]) {
                if (g.edges[fe.second].baked) { baked = true; break; }
                const Vec2 dv = g.nodes[fe.first].pos - g.nodes[v].pos;
                if (dv.lengthSquared() < 1e-12) continue;
                arms.push_back({std::atan2((double)dv.y, (double)dv.x), fe.first, fe.second});
            }
            if (baked || static_cast<int>(arms.size()) < 3) { ++dbgBaked; continue; }
            std::sort(arms.begin(), arms.end(),
                      [](const Arm& a, const Arm& b) { return a.ang < b.ang; });

            const int n = static_cast<int>(arms.size());
            std::vector<double> want(n);
            for (int k = 0; k < n; ++k) want[k] = arms[k].ang;
            // Push neighbouring bearings apart until every gap clears minAngle.
            // Local and iterative (the file's contract: adjust, never solve).
            bool tight = false;
            for (int it = 0; it < 16; ++it) {
                bool any = false;
                for (int k = 0; k < n; ++k) {
                    const int k1 = (k + 1) % n;
                    double gap = want[k1] - want[k];
                    if (k1 == 0) gap += kTwoPi;
                    // Aim a hair PAST the target: relaxing to exactly minAngle
                    // leaves a junction sitting on the line, and the dead-band
                    // below then skips the 1-degree correction it needs — which
                    // is why the first cut stalled with 52 junctions parked
                    // between 56 and 60 degrees.
                    const double aim = minAngle * 1.03;
                    if (gap >= aim) continue;
                    const double push = (aim - gap) * 0.5;
                    want[k] -= push;
                    want[k1] += push;
                    any = true;
                    tight = true;
                }
                if (!any) break;
            }
            if (!tight) continue;
            ++dbgTight;

            for (int k = 0; k < n; ++k) {
                double delta = want[k] - arms[k].ang;
                while (delta > kTwoPi * 0.5) delta -= kTwoPi;
                while (delta < -kTwoPi * 0.5) delta += kTwoPi;
                if (std::fabs(delta) < 0.006) continue;      // under ~0.35 deg: noise
                const Vec2 dir(std::cos(want[k]), std::sin(want[k]));
                const Vec2 V = g.nodes[v].pos;
                const int far = arms[k].far;
                // STALE INCIDENCE GUARD. `nbr` is built once per pass, but the
                // graph is mutated as we walk it: if a junction earlier in this
                // pass already bent the arm we share with it, this edge no
                // longer joins (v, far) and rewiring it again welds two bend
                // stubs together — measured as 7 pairs of roads lying on top of
                // each other. Re-check the edge before touching it; the next
                // pass will see the fresh topology and finish the job.
                {
                    const RoadEdge& cur = g.edges[arms[k].edge];
                    if (!((cur.a == v && cur.b == far) || (cur.a == far && cur.b == v)))
                        continue;
                }
                const double elen = (g.nodes[far].pos - V).length();
                if (elen < 1e-6) continue;

                // VALIDATE BEFORE APPLYING. A bend that swings across a
                // neighbouring street is worse than the angle it fixes: the
                // planarize behind this pass would node the crossing into a
                // brand-new junction at whatever angle the accident made — the
                // first attempt at this manufactured junctions at 15 deg, below
                // anything the generator had produced. Construct, test, and
                // leave the junction for the next rung if the bend will not fit.
                // A bend splits the arm's first edge, so keep both pieces real
                // roads rather than slivers — but no more than that. An earlier
                // version refused any arm under ~67 m on the theory that short
                // pieces were what killed kerbside parking; the real cause was
                // curvature shattering a chain into sub-40 m nav links, fixed
                // where it belonged (bays ride the chain now). The over-strict
                // guard was left behind, and it cost real compliance: 45
                // junctions under 60 degrees instead of 7.
                const double L = std::max(4.0, std::min<double>(runIn, elen * 0.4));
                if (L < 6.0 || elen - L < 10.0) continue;
                const Vec2 B = V + dir * L;
                const Vec2 F = g.nodes[far].pos;
                bool blocked = false;
                for (int ei = 0; ei < static_cast<int>(g.edges.size()) && !blocked; ++ei) {
                    const RoadEdge& e = g.edges[ei];
                    if (ei == arms[k].edge || e.a == e.b) continue;
                    if (e.a == v || e.b == v || e.a == far || e.b == far) continue;
                    const Vec2 p = g.nodes[e.a].pos, q = g.nodes[e.b].pos;
                    const double lo = std::min(V.x, F.x) - 2.0, hi = std::max(V.x, F.x) + 2.0;
                    const double lo2 = std::min(V.y, F.y) - 2.0, hi2 = std::max(V.y, F.y) + 2.0;
                    if (std::max(p.x, q.x) < lo || std::min(p.x, q.x) > hi ||
                        std::max(p.y, q.y) < lo2 || std::min(p.y, q.y) > hi2) continue;
                    if (segsCross(V, B, p, q) || segsCross(B, F, p, q)) blocked = true;
                }
                if (blocked) { ++dbgBlocked; continue; }

                if (isBend[far] && static_cast<int>(nbr[far].size()) == 2) {
                    // Our own bend node from an earlier pass: swing it, keeping
                    // its distance, so the approach re-aims without new nodes.
                    g.nodes[far].pos = V + dir * elen;
                } else {
                    // A bend must not overshoot the arm it is bending: cap it
                    // well inside the first edge, and keep a real run-in.
                    const int b = static_cast<int>(g.nodes.size());
                    RoadNode bn;
                    bn.pos = B;
                    bn.elev = g.nodes[v].elev;
                    // AUTHOR THE TANGENT so the run-in is genuinely STRAIGHT.
                    // With an auto (Catmull-Rom) knot the spline starts bending
                    // the moment it leaves the junction, which is both wrong for
                    // a road — real practice is a straight approach, then the
                    // curve — and expensive here: the sampler collapses a
                    // straight edge to ONE segment but shatters a curved one,
                    // and the nav layer makes a link per segment. Curving the
                    // approach put every kerbside link under the 40 m parking
                    // threshold and silently deleted the city's parking (346
                    // banded links, 0 bays). Tangent along the approach keeps
                    // v->B straight; the bend then lives in B->far.
                    bn.tangent = dir;
                    g.nodes.push_back(bn);
                    isBend.push_back(1);
                    // Re-point the existing edge onto the bend, then carry the
                    // SAME edge attributes (width, class, spec, one-way, access)
                    // through to the far node — a fresh RoadEdge would drop them.
                    RoadEdge rest = g.edges[arms[k].edge];
                    RoadEdge& first = g.edges[arms[k].edge];
                    if (first.a == v) first.b = b; else first.a = b;
                    if (rest.a == v) { rest.a = b; rest.b = far; }
                    else             { rest.a = far; rest.b = b; }
                    g.edges.push_back(rest);
                }
                changed = true;
                ++dbgApplied;
            }
        }
        if (!changed) break;
    }
    if (std::getenv("RT_REALIGN_DEBUG"))
        std::fprintf(stderr,
                     "[realign] tight-junctions=%d arms-bent=%d blocked=%d "
                     "infeasible=%d baked/elev=%d\n",
                     dbgTight, dbgApplied, dbgBlocked, dbgInfeasible, dbgBaked);
    return g;
}

RoadGraph dissolveAcuteArms(const RoadGraph& in, Real minDot, int maxDetourSpans) {
    // Two near-parallel arms leaving one junction enclose a sliver wedge the
    // mesher's acute-pair trim can only partially cover — the piedmont drive
    // probe measured 24 hole samples riding the trimmed twin (two width-17
    // arterials 26 degrees apart at the city hub). Deletion-only: drop the
    // narrower/shorter arm's whole span IF its far junction stays reachable
    // within maxDetourSpans spans — planarity untouched, connectivity proven
    // before every cut. Deterministic: node order, then arm order.
    RoadGraph g = in;
    for (int guard = 0; guard < 256; ++guard) {
        // Span walk tables (junction-to-junction through degree-2 nodes).
        const int N = static_cast<int>(g.nodes.size());
        std::vector<int> deg(N, 0);
        std::vector<std::vector<std::pair<int, int>>> nbr(N);
        for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
            const RoadEdge& e = g.edges[ei];
            if (e.a == e.b) continue;
            ++deg[e.a]; ++deg[e.b];
            nbr[e.a].push_back({e.b, ei});
            nbr[e.b].push_back({e.a, ei});
        }
        struct Span {
            int far = -1;
            Real len = 0, width = 0;
            std::vector<int> edges;
            std::vector<Vec2> pts;   // node polyline, junction first
            Vec2 dir;   // unit, leaving the junction
        };
        auto walkSpan = [&](int from, int firstOther, int firstEdge) {
            Span s;
            s.width = g.edges[firstEdge].width;
            s.edges.push_back(firstEdge);
            s.pts.push_back(g.nodes[from].pos);
            s.pts.push_back(g.nodes[firstOther].pos);
            Vec2 d0 = g.nodes[firstOther].pos - g.nodes[from].pos;
            s.len = d0.length();
            s.dir = s.len > 1e-9 ? d0 * (1.0 / s.len) : Vec2(1, 0);
            int prev = from, cur = firstOther;
            while (deg[cur] == 2) {
                auto [a0, ea] = nbr[cur][0];
                auto [a1, eb] = nbr[cur][1];
                int nn = (a0 == prev) ? a1 : a0;
                int ne = (a0 == prev) ? eb : ea;
                s.edges.push_back(ne);
                s.pts.push_back(g.nodes[nn].pos);
                s.len += (g.nodes[cur].pos - g.nodes[nn].pos).length();
                prev = cur; cur = nn;
            }
            s.far = cur;
            return s;
        };
        // Find the first dissolvable pair.
        bool acted = false;
        for (int v = 0; v < N && !acted; ++v) {
            if (deg[v] < 3) continue;
            std::vector<Span> arms;
            for (auto [o, ei] : nbr[v]) arms.push_back(walkSpan(v, o, ei));
            for (std::size_t i = 0; i < arms.size() && !acted; ++i)
                for (std::size_t j = i + 1; j < arms.size() && !acted; ++j) {
                    if (dot(arms[i].dir, arms[j].dir) < minDot) continue;
                    if (arms[i].far == v || arms[j].far == v) continue;
                    // Victim: narrower, then shorter, then higher j (determinism).
                    const Span& victim =
                        arms[i].width != arms[j].width
                            ? (arms[i].width < arms[j].width ? arms[i] : arms[j])
                            : (arms[i].len <= arms[j].len ? arms[i] : arms[j]);
                    // Only SLIVER twins die: the un-meshable wedge is a
                    // local artifact (~tens of metres). Long near-parallel
                    // arms are structure — deleting one collapses blocks and
                    // strands frontage (measured 33 -> 18 blocks). A stricter
                    // ribbon-overlap criterion was tried and FALSIFIED: pairs
                    // that diverge past ribbon reach still hole the mesh.
                    if (victim.len > 250.0) continue;
                    // Redundancy proof: far junction reachable without the span,
                    // within maxDetourSpans span-hops.
                    std::vector<char> banned(g.edges.size(), 0);
                    for (int e : victim.edges) banned[e] = 1;
                    std::vector<int> frontier{v};
                    std::vector<char> seen(N, 0);
                    seen[v] = 1;
                    bool reach = false;
                    for (int hop = 0; hop < maxDetourSpans && !reach; ++hop) {
                        std::vector<int> next;
                        for (int u : frontier)
                            for (auto [o, ei] : nbr[u]) {
                                if (banned[ei]) continue;
                                Span sp = walkSpan(u, o, ei);
                                bool skip = false;
                                for (int e : sp.edges)
                                    if (banned[e]) { skip = true; break; }
                                if (skip || seen[sp.far]) continue;
                                if (sp.far == victim.far) { reach = true; break; }
                                seen[sp.far] = 1;
                                next.push_back(sp.far);
                            }
                        frontier = std::move(next);
                    }
                    if (!reach) continue;
                    std::vector<RoadEdge> kept;
                    kept.reserve(g.edges.size());
                    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei)
                        if (!banned[ei]) kept.push_back(g.edges[ei]);
                    g.edges = std::move(kept);
                    acted = true;
                }
        }
        if (!acted) break;
    }
    // Compact orphaned nodes.
    std::vector<int> remap(g.nodes.size(), -1);
    for (const RoadEdge& e : g.edges) { remap[e.a] = 0; remap[e.b] = 0; }
    RoadGraph out;
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i)
        if (remap[i] == 0) { remap[i] = static_cast<int>(out.nodes.size()); out.nodes.push_back(g.nodes[i]); }
    out.edges.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges) {
        RoadEdge o = e;
        o.a = remap[e.a];
        o.b = remap[e.b];
        out.edges.push_back(o);
    }
    return out;
}

RoadGraph cutSharpCorners(const RoadGraph& in, Real maxTurn) {
    // Corner-cut the bends relaxSharpBends cannot converge (easing one vertex
    // can sharpen its neighbour — ping-pong): DELETE a degree-2 vertex whose
    // deflection exceeds maxTurn and chord its neighbours. Removing a vertex
    // strictly reduces the chain's bend, so this terminates, preserves every
    // face, and never moves a junction. Sharpest first, deterministic.
    RoadGraph g = in;
    for (int guard = 0; guard < static_cast<int>(in.nodes.size()) + 16; ++guard) {
        std::vector<int> deg(g.nodes.size(), 0);
        std::vector<std::array<int, 2>> nbr(g.nodes.size(), {-1, -1});
        std::vector<std::array<int, 2>> nbrEdge(g.nodes.size(), {-1, -1});
        for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
            const RoadEdge& e = g.edges[ei];
            if (e.a == e.b) continue;
            if (deg[e.a] < 2) { nbr[e.a][deg[e.a]] = e.b; nbrEdge[e.a][deg[e.a]] = ei; }
            if (deg[e.b] < 2) { nbr[e.b][deg[e.b]] = e.a; nbrEdge[e.b][deg[e.b]] = ei; }
            ++deg[e.a]; ++deg[e.b];
        }
        int worstV = -1;
        Real worstCos = std::cos(maxTurn);   // sharper = smaller cosine
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
            if (deg[v] != 2) continue;
            if (nbr[v][0] == nbr[v][1]) continue;   // parallel pair: dropParallelEdges' job
            Vec2 d0 = g.nodes[v].pos - g.nodes[nbr[v][0]].pos;
            Vec2 d1 = g.nodes[nbr[v][1]].pos - g.nodes[v].pos;
            Real l0 = d0.length(), l1 = d1.length();
            if (l0 < 1e-6 || l1 < 1e-6) continue;
            Real c = dot(d0, d1) / (l0 * l1);
            if (c < worstCos - 1e-9) { worstCos = c; worstV = v; }
        }
        if (worstV < 0) break;
        // Chord: rewire the first edge to span both neighbours, drop the second.
        RoadEdge& keep = g.edges[nbrEdge[worstV][0]];
        const int other = nbr[worstV][1];
        if (keep.a == worstV) keep.a = other; else keep.b = other;
        const RoadEdge& gone = g.edges[nbrEdge[worstV][1]];
        keep.width = std::max(keep.width, gone.width);
        g.edges.erase(g.edges.begin() + nbrEdge[worstV][1]);
    }
    return g;
}

RoadGraph dropParallelEdges(const RoadGraph& in) {
    // Exact parallel duplicates (two edges joining the same node pair) are
    // degenerate two-edge loops: un-relaxable (the chord is a point), face
    // slivers, and 180-degree "folds" to any chain audit. Keep the widest.
    RoadGraph g = in;
    std::vector<RoadEdge> kept;
    kept.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges) {
        if (e.a == e.b) continue;
        bool dup = false;
        for (RoadEdge& k : kept)
            if ((k.a == e.a && k.b == e.b) || (k.a == e.b && k.b == e.a)) {
                k.width = std::max(k.width, e.width);
                dup = true;
                break;
            }
        if (!dup) kept.push_back(e);
    }
    g.edges = std::move(kept);
    return g;
}

RoadGraph consolidateJunctionSpans(const RoadGraph& in, Real minSpan,
                                   int maxDegree) {
    return consolidateJunctionSpans(
        in, [minSpan](const Vec2&) { return minSpan; }, maxDegree);
}

// REGION-AWARE overload (P7 density unlock): the span floor is a function of
// position, evaluated at the span midpoint — the city core keeps a tighter
// floor than the periphery, so dense downtown fabric is LEGAL while country
// arterials keep big-block spacing. Deterministic: the round's victim is the
// span most below ITS OWN floor (ties: shorter, then lower node pair).
RoadGraph consolidateJunctionSpans(
    const RoadGraph& in, const std::function<Real(const Vec2&)>& minSpanAt,
    int maxDegree) {
    RoadGraph g = in;
    // Iterate: each fuse changes degrees and span lengths, so recompute the
    // chain decomposition per round and take the globally shortest offender.
    const int maxRounds = std::max(256, static_cast<int>(in.edges.size()));
    int fused = 0;
    std::set<long long> unfixable;   // (min,max) junction pairs proven stuck
    for (int round = 0; round < maxRounds; ++round) {
        const int N = static_cast<int>(g.nodes.size());
        std::vector<int> deg(N, 0);
        std::vector<std::vector<std::pair<int, int>>> nbr(N);   // (other, edge)
        for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
            const RoadEdge& e = g.edges[ei];
            if (e.a == e.b) continue;
            ++deg[e.a]; ++deg[e.b];
            nbr[e.a].push_back({e.b, ei});
            nbr[e.b].push_back({e.a, ei});
        }
        // Shortest fixable junction-to-junction span below minSpan this round.
        // A span fixes by FUSE (fused arms fit maxDegree) or, when both ends
        // are already full, by DELETE — a short link between two busy
        // junctions is a redundant connection, removable when its endpoints
        // stay hop-connected without it. Unfixable spans are remembered and
        // skipped so the loop terminates.
        struct Span { int a = -1, b = -1; Real len = 0; Real floor = 0;
                      std::vector<int> path;
                      std::vector<int> spanEdges; bool fuse = true; };
        Span best;
        Real bestDeficit = -1e-9;   // most-below-floor wins
        std::vector<char> walked(g.edges.size(), 0);
        for (int n = 0; n < N; ++n) {
            if (deg[n] == 2 || deg[n] == 0) continue;
            for (auto [next, ei0] : nbr[n]) {
                if (walked[ei0]) continue;
                walked[ei0] = 1;
                Real len = (g.nodes[n].pos - g.nodes[next].pos).length();
                int prev = n, cur = next;
                std::vector<int> path;        // interior degree-2 nodes
                std::vector<int> spanEdges{ei0};
                while (deg[cur] == 2) {
                    path.push_back(cur);
                    auto [a0, ea] = nbr[cur][0];
                    auto [a1, eb] = nbr[cur][1];
                    int nn = (a0 == prev) ? a1 : a0;
                    int ne = (a0 == prev) ? eb : ea;
                    if (walked[ne]) break;
                    walked[ne] = 1;
                    spanEdges.push_back(ne);
                    len += (g.nodes[cur].pos - g.nodes[nn].pos).length();
                    prev = cur; cur = nn;
                }
                if (deg[cur] == 2) continue;             // hit a walked cycle
                if (cur == n && path.empty()) continue;  // degenerate loop
                const Vec2 mid =
                    (g.nodes[n].pos + g.nodes[cur].pos) * 0.5;
                const Real floor = minSpanAt(mid);
                const Real deficit = len - floor;        // negative = offender
                if (deficit >= bestDeficit) continue;
                const long long key =
                    static_cast<long long>(std::min(n, cur)) * N + std::max(n, cur);
                if (unfixable.count(key)) continue;
                int arms = deg[n] + deg[cur] - 2;
                if (cur == n) arms = deg[n] - 2;         // short loop back onto itself
                if (arms <= maxDegree) {
                    best = {n, cur, len, floor, path, spanEdges, /*fuse=*/true};
                    bestDeficit = deficit;
                    continue;
                }
                // FUSE would over-crowd: DELETE only a genuinely REDUNDANT
                // link — the detour between its endpoints must be at most 2
                // spans (a parallel link or a triangle sliver). A real block
                // side's detour is >=3 spans (around the block); deleting
                // those merges finished blocks. BFS counts SPAN hops: walking
                // a whole chain of degree-2 curve nodes costs 1.
                std::vector<char> spanEdge(g.edges.size(), 0);
                for (int se : spanEdges) spanEdge[se] = 1;
                std::vector<char> visited(N, 0);
                visited[n] = 1;
                bool redundant = false;
                std::vector<int> frontier{n};
                for (int hopJ = 0; hopJ < 2 && !redundant; ++hopJ) {
                    std::vector<int> nextFrontier;
                    for (int u : frontier) {
                        for (auto [v0, ve0] : nbr[u]) {
                            if (spanEdge[ve0] || visited[v0]) continue;
                            // Walk this chain to its far junction.
                            int prev2 = u, cur2 = v0;
                            visited[cur2] = 1;
                            bool dead = false;
                            while (deg[cur2] == 2) {
                                auto [b0, eb0] = nbr[cur2][0];
                                auto [b1, eb1] = nbr[cur2][1];
                                int nn = (b0 == prev2) ? b1 : b0;
                                int ne = (b0 == prev2) ? eb1 : eb0;
                                if (spanEdge[ne] || visited[nn]) { dead = true; break; }
                                visited[nn] = 1;
                                prev2 = cur2; cur2 = nn;
                            }
                            if (dead) continue;
                            if (cur2 == cur) { redundant = true; break; }
                            nextFrontier.push_back(cur2);
                        }
                        if (redundant) break;
                    }
                    frontier = std::move(nextFrontier);
                }
                if (redundant) {
                    best = {n, cur, len, floor, path, spanEdges, /*fuse=*/false};
                    bestDeficit = deficit;
                    continue;
                }
                // Not redundant either: LENGTHEN in place — push the two
                // junctions apart along the span chord to minSpan, curve nodes
                // sliding proportionally. Topology (and every face) survives;
                // the block deforms a little. Applied immediately; the pair is
                // then remembered so float residue can't re-trigger it.
                if (cur != n) {
                    Vec2 chord = g.nodes[cur].pos - g.nodes[n].pos;
                    const Real clen = chord.length();
                    chord = clen > 1e-9 ? chord * (1.0 / clen) : Vec2(1, 0);
                    const Real push = (floor - len) * 0.5;
                    for (int pn : path) {
                        const Real t = clen > 1e-9
                            ? dot(g.nodes[pn].pos - g.nodes[n].pos, chord) / clen
                            : 0.5;
                        g.nodes[pn].pos = g.nodes[pn].pos +
                                          chord * (push * (2.0 * std::clamp(t, 0.0, 1.0) - 1.0));
                    }
                    g.nodes[n].pos = g.nodes[n].pos - chord * push;
                    g.nodes[cur].pos = g.nodes[cur].pos + chord * push;
                    ++fused;
                }
                unfixable.insert(key);
            }
        }
        if (best.a < 0) break;
        if (!best.fuse) {
            // DELETE the redundant span: drop its edges + curve nodes.
            std::vector<char> dropE(g.edges.size(), 0);
            for (int se : best.spanEdges) dropE[se] = 1;
            std::vector<RoadEdge> kept;
            kept.reserve(g.edges.size());
            for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei)
                if (!dropE[ei]) kept.push_back(g.edges[ei]);
            g.edges = std::move(kept);
            ++fused;
            continue;
        }
        ++fused;
        const int a = best.a, b = best.b;
        // Fuse b (and the span's curve nodes) into a at the degree-weighted
        // midpoint — the busier junction moves less.
        if (a != b) {
            const Real da = std::max(1, deg[a]), db = std::max(1, deg[b]);
            g.nodes[a].pos = (g.nodes[a].pos * da + g.nodes[b].pos * db) * (1.0 / (da + db));
        }
        std::vector<char> dropNode(g.nodes.size(), 0);
        for (int pn : best.path) dropNode[pn] = 1;
        std::vector<RoadEdge> kept;
        kept.reserve(g.edges.size());
        for (const RoadEdge& e : g.edges) {
            RoadEdge o = e;
            if (o.a == b) o.a = a;
            if (o.b == b) o.b = a;
            if (o.a == o.b) continue;                    // the span's own stub(s)
            if (dropNode[o.a] || dropNode[o.b]) continue; // span curve segments
            bool dup = false;
            for (RoadEdge& k : kept)
                if ((k.a == o.a && k.b == o.b) || (k.a == o.b && k.b == o.a)) {
                    k.width = std::max(k.width, o.width);
                    dup = true;
                    break;
                }
            if (!dup) kept.push_back(o);
        }
        g.edges = std::move(kept);
    }
    if (fused > 0)
        LOG_INFO << "consolidateJunctionSpans: fused " << fused
                 << " junction spans under their regional floors";

    // Compact orphaned nodes.
    std::vector<int> remap(g.nodes.size(), -1);
    for (const RoadEdge& e : g.edges) { remap[e.a] = 0; remap[e.b] = 0; }
    RoadGraph out;
    for (int i = 0; i < static_cast<int>(g.nodes.size()); ++i)
        if (remap[i] == 0) { remap[i] = static_cast<int>(out.nodes.size()); out.nodes.push_back(g.nodes[i]); }
    out.edges.reserve(g.edges.size());
    for (const RoadEdge& e : g.edges) {
        RoadEdge o = e;
        o.a = remap[e.a];
        o.b = remap[e.b];
        out.edges.push_back(o);
    }
    return out;
}

}  // namespace engine
