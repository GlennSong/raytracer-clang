#include "road_network.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace engine {

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

Vec2 RoadArc::at(Real t) const {
    Real a = a0 + (a1 - a0) * t;
    return center + Vec2(std::cos(a), std::sin(a)) * radius;
}

void sampleArc(RoadGraph& g, int n0, int n1, const RoadArc& arc, Real width,
               RoadClass klass, Real maxErr, int minSegs) {
    Real sweep = std::abs(arc.a1 - arc.a0);
    int segs = std::max(1, minSegs);
    if (arc.radius > 1e-6 && maxErr > 0.0) {
        // Chord error e = R(1 - cos(dTheta/2)); pick the step so e <= maxErr.
        Real dThetaMax = 2.0 * std::acos(std::max(Real(0), Real(1) - maxErr / arc.radius));
        if (dThetaMax > 1e-6)
            segs = std::max(segs, static_cast<int>(std::ceil(sweep / dThetaMax)));
    }
    int prev = n0;
    for (int i = 1; i < segs; ++i) {
        int mid = g.addNode(arc.at(static_cast<Real>(i) / segs), 0.01);
        g.addEdge(prev, mid, width, klass);
        prev = mid;
    }
    g.addEdge(prev, n1, width, klass);
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

namespace engine {

std::vector<DanglingJoin> joinDanglingEnds(RoadGraph& g, Real sidewalk) {
    std::vector<DanglingJoin> joins;
    const int n = static_cast<int>(g.nodes.size());
    std::vector<std::vector<int>> inc(n);
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
        inc[g.edges[e].a].push_back(e);
        inc[g.edges[e].b].push_back(e);
    }
    // Snapshot the dead ends first: joining adds nodes and edges.
    std::vector<int> ends;
    for (int v = 0; v < n; ++v)
        if (inc[v].size() == 1) ends.push_back(v);
    const int nEdges0 = static_cast<int>(g.edges.size());
    std::vector<int> dead;   // edges a pull-back consumed; removed at the end
    for (int E : ends) {
        const int eE = inc[E][0];
        const RoadEdge endEdge = g.edges[eE];
        if (endEdge.baked || endEdge.klass == RoadClass::Freeway ||
            endEdge.klass == RoadClass::Ramp)
            continue;                                   // authored corridor ends
        if (endEdge.oneWay || endEdge.provenance != RoadProvenance::Street)
            continue;                                   // a carriageway of a divided road
        const int M = endEdge.a == E ? endEdge.b : endEdge.a;
        const Vec2 P = g.nodes[E].pos;
        Vec2 dirE = P - g.nodes[M].pos;                  // outward along the stub
        const Real lenE = dirE.length();
        if (lenE < 1e-6) continue;
        dirE = dirE * (1.0 / lenE);
        const Real hwE = endEdge.width * 0.5;
        // CORRIDOR TERRITORY is off limits (the merge probe on a freeway
        // metro: a street end joined INTO a ramp broke a gore): any freeway,
        // ramp or baked edge within 40 m of this end, and it is left alone.
        // And a road that ENDS TOGETHER with this one — another dead end
        // within 30 m, the twin carriageway of a divided arterial, a pair
        // of stubs at a city edge — is not a road this one stops inside.
        bool corridor = false, twinEnd = false;
        for (int o = 0; o < nEdges0 && !corridor; ++o) {
            const RoadEdge& oe = g.edges[o];
            if (oe.a == E || oe.b == E) continue;
            if (!(oe.baked || oe.klass == RoadClass::Freeway || oe.klass == RoadClass::Ramp ||
                  oe.provenance != RoadProvenance::Street))
                continue;
            const Vec2 A = g.nodes[oe.a].pos, B = g.nodes[oe.b].pos;
            const Vec2 AB = B - A;
            const Real l2 = AB.lengthSquared();
            Real t = l2 > 1e-9 ? dot(P - A, AB) / l2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            if ((A + AB * t - P).length() < 40.0) corridor = true;
        }
        if (corridor) continue;
        for (int v = 0; v < n && !twinEnd; ++v)
            if (v != E && inc[v].size() == 1 && (g.nodes[v].pos - P).length() < 30.0) twinEnd = true;
        if (twinEnd) continue;
        int best = -1;
        Real bestD = 0, bestT = 0, bestReach = 0, bestHeading = 0;
        Vec2 bestQ;
        for (int o = 0; o < nEdges0; ++o) {
            const RoadEdge& oe = g.edges[o];
            if (oe.a == E || oe.b == E || oe.a == M || oe.b == M) continue;
            if (oe.baked || oe.oneWay || oe.layer != endEdge.layer) continue;
            if (oe.klass == RoadClass::Freeway || oe.klass == RoadClass::Ramp ||
                oe.provenance != RoadProvenance::Street)
                continue;
            const Vec2 A = g.nodes[oe.a].pos, B = g.nodes[oe.b].pos;
            const Vec2 AB = B - A;
            const Real l2 = AB.lengthSquared();
            if (l2 < 1e-9) continue;
            const Real t = dot(P - A, AB) / l2;
            if (t <= 0.05 || t >= 0.95) continue;         // interior of the edge only
            const Vec2 Q = A + AB * t;
            const Real d = (Q - P).length();
            const Real reach = hwE + oe.width * 0.5 + sidewalk;
            if (d >= reach) continue;
            const Vec2 toQ = d > 1e-6 ? (Q - P) * (1.0 / d) : dirE;
            if (best < 0 || d < bestD) {
                best = o; bestD = d; bestT = t; bestQ = Q; bestReach = reach;
                bestHeading = dot(dirE, toQ);
            }
        }
        if (best < 0) continue;
        // Already inside the other carriageway, or heading into it: a T.
        // Otherwise the other road skirts past this cap obliquely — a T
        // there would meet at an absurd angle — so pull the end back along
        // its last edge until the cap clears the corridor and sidewalk.
        const bool inside = bestD < g.edges[best].width * 0.5;
        if (!inside && bestHeading < 0.5) {
            // Moving back by s along -dirE grows the distance by ~s * heading
            // (heading > 0: the end was leaning toward the road at all).
            const Real need = bestReach - bestD + 0.5;
            const Real want = bestHeading > 0.1 ? need / bestHeading : need * 2.0;
            // A pull-back is a few metres of a stray cap. One that would take
            // a road back 15-86 m (the freeway metro, before the guards
            // above) is not a stray cap — leave the road alone.
            if (want > 12.0) continue;
            // A sampled road has nodes every few metres, so the pull walks
            // back along the chain through degree-2 nodes, dropping every
            // edge it passes, and moves the node it lands in. Dropped edges
            // are removed at the end (their nodes stay, unconnected).
            Real pulled = 0.0;
            int cur = E, prevEdge = eE;
            while (true) {
                const RoadEdge& ce = g.edges[prevEdge];
                const int back = ce.a == cur ? ce.b : ce.a;
                const Vec2 seg = g.nodes[cur].pos - g.nodes[back].pos;
                const Real segLen = seg.length();
                const Real remaining = want - pulled;
                if (remaining < segLen * 0.9 || inc[back].size() != 2 || segLen < 1e-6) {
                    // Land inside this edge (never past 90 % of it).
                    const Real s = std::min(remaining, segLen * 0.9);
                    g.nodes[cur].pos = g.nodes[cur].pos - seg * (s / segLen);
                    pulled += s;
                    break;
                }
                // Consume the whole edge: `cur` becomes the dead end.
                dead.push_back(prevEdge);
                pulled += segLen;
                const int e0 = inc[back][0], e1 = inc[back][1];
                prevEdge = (e0 == prevEdge) ? e1 : e0;
                cur = back;
            }
            joins.push_back({P, g.nodes[cur].pos, bestD, endEdge.klass, g.edges[best].klass,
                             true, pulled});
            continue;
        }
        // The new junction node, on the other road, height from its ends.
        RoadNode nn;
        nn.pos = bestQ;
        const RoadNode& A = g.nodes[g.edges[best].a];
        const RoadNode& B = g.nodes[g.edges[best].b];
        nn.elev = A.elev + (B.elev - A.elev) * bestT;
        nn.elevAbsolute = A.elevAbsolute;
        const int N = static_cast<int>(g.nodes.size());
        g.nodes.push_back(nn);
        // Split the other edge in place: (a, b) -> (a, N) + (N, b), everything copied.
        RoadEdge second = g.edges[best];
        const int oldB = g.edges[best].b;
        g.edges[best].b = N;
        second.a = N;
        second.b = oldB;
        g.edges.push_back(second);
        // Bridge the gap with the stub's own class and width (no parking on the stub).
        RoadEdge bridge = endEdge;
        bridge.a = E;
        bridge.b = N;
        bridge.parkOffset = 0;
        bridge.parkWidth = 0;
        g.edges.push_back(bridge);
        joins.push_back({P, bestQ, bestD, endEdge.klass, second.klass});
    }
    if (!dead.empty()) {
        std::vector<char> gone(g.edges.size(), 0);
        for (int e : dead) gone[e] = 1;
        std::vector<RoadEdge> kept;
        kept.reserve(g.edges.size());
        for (std::size_t e = 0; e < g.edges.size(); ++e)
            if (!gone[e]) kept.push_back(g.edges[e]);
        g.edges.swap(kept);
    }
    return joins;
}

}  // namespace engine
