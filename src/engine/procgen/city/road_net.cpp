#include "road_net.h"
#include "../../../log.h"
#include "road_lattice.h"        // swept-lattice street mesher (stage 3)
#include "../../mesh_builder.h"  // MeshBuilder::append

#include "road_network.h"       // RoadGraph, RoadEdge
#include "road_constraints.h"   // applyConstraints, capDegree, RoadRules
#include "road_rules.h"         // DesignRules (clearance, deck thickness, ramp grade)
#include "district.h"           // DistrictParams, buildDistrict (generate recipe)
#include "metro.h"              // MetroParams, buildMetro ("kind":"metro" recipe)
#include <algorithm>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <cstdio>

namespace engine {

// The curvature cap must clear the WIDEST ribbon in the net, not the default
// width: the metro recipe writes 13 m arterials beside 7 m streets, and a bend
// with radius between the two folds the arterial's band over itself (the
// "side of the road raised up vertically" flap). Half-width + sidewalk + margin
// of the widest edge.
static double netMinTurnRadius(const RoadNet& net) {
    double maxW = net.width;
    for (double w : net.edgeWidths) if (w > 0) maxW = std::max(maxW, w);
    return maxW * 0.5 + net.sidewalk + 0.5;
}


using json = nlohmann::json;

namespace {

bool isZero(const Vec2& v) { return v.x == 0.0 && v.y == 0.0; }

// Valid, non-degenerate edges as index pairs.
std::vector<std::array<int, 2>> validEdges(const RoadNet& net) {
    std::vector<std::array<int, 2>> out;
    const int n = static_cast<int>(net.nodes.size());
    for (const std::array<int, 2>& e : net.edges)
        if (e[0] >= 0 && e[1] >= 0 && e[0] < n && e[1] < n && e[0] != e[1]) out.push_back(e);
    return out;
}

// The road's graph for the mesher: EVERY edge is a Catmull-Rom spline, sampled as a Hermite cubic
// through its endpoints' tangents (Catmull-Rom auto on a degree-2 through-road, a straight chord into
// a junction/dead-end). The sampler is ADAPTIVE — an edge that doesn't actually bend collapses back to
// a single segment, so the grid's straight runs don't densify and clog the junction meshes. The
// original nodes keep their indices (so junction degree is preserved); curve samples append.
RoadGraph netGraph(const RoadNet& net, double minTurnRadius = 0.0) {
    const int n = static_cast<int>(net.nodes.size());
    auto P = [&](int i) { return net.nodes[i]; };
    // Valid edge indices, and each edge's width (its override or the default).
    std::vector<int> ev;
    for (int ei = 0; ei < static_cast<int>(net.edges.size()); ++ei) {
        const std::array<int, 2>& e = net.edges[ei];
        if (e[0] >= 0 && e[1] >= 0 && e[0] < n && e[1] < n && e[0] != e[1]) ev.push_back(ei);
    }
    auto ewidth = [&](int ei) {
        return static_cast<Real>(
            (ei < static_cast<int>(net.edgeWidths.size()) && net.edgeWidths[ei] > 0.0)
                ? net.edgeWidths[ei] : net.width);
    };
    auto elayer = [&](int ei) {
        return (ei < static_cast<int>(net.edgeLayers.size())) ? net.edgeLayers[ei] : 0;
    };
    auto eclass = [&](int ei) {
        return (ei < static_cast<int>(net.edgeClasses.size())) ? net.edgeClasses[ei]
                                                               : RoadClass::Local;
    };
    // Optional per-node ABSOLUTE deck Y (authored elevated roads). Finite = the
    // node rides at that world Y; the weld carries it as UnionSpine.yAbs.
    auto nhasElev = [&](int i) {
        return i < static_cast<int>(net.nodeElev.size()) && std::isfinite(net.nodeElev[i]);
    };

    RoadGraph g;
    g.nodes.resize(n);
    for (int i = 0; i < n; ++i) {
        g.nodes[i].pos = net.nodes[i];
        if (nhasElev(i)) { g.nodes[i].elev = static_cast<Real>(net.nodeElev[i]);
                           g.nodes[i].elevAbsolute = true; }
    }

    // Per-node neighbours (for degree + the Catmull-Rom "other" neighbour).
    std::vector<std::vector<int>> nbr(n);
    for (int ei : ev) {
        const std::array<int, 2>& e = net.edges[ei];
        nbr[e[0]].push_back(e[1]); nbr[e[1]].push_back(e[0]);
    }
    auto stored = [&](int i) {
        return (i < static_cast<int>(net.tangents.size())) ? net.tangents[i] : Vec2(0, 0);
    };
    auto otherNbr = [&](int v, int notThis) {
        for (int u : nbr[v]) if (u != notThis) return u;
        return notThis;
    };
    // Derivative at `v` for an edge oriented from `v` toward `to` (the curve's start).
    auto outTan = [&](int v, int to) -> Vec2 {
        Vec2 travel = P(to) - P(v);
        if (!isZero(stored(v))) {
            Vec2 t = stored(v);
            return (dot(t, travel) < 0) ? Vec2(-t.x, -t.y) : t;
        }
        if (nbr[v].size() == 2) return (P(to) - P(otherNbr(v, to))) * 0.5;   // Catmull-Rom
        return travel;                                                       // straight
    };
    // Derivative at `v` for an edge arriving from `from` (the curve's end).
    auto inTan = [&](int v, int from) -> Vec2 {
        Vec2 travel = P(v) - P(from);
        if (!isZero(stored(v))) {
            Vec2 t = stored(v);
            return (dot(t, travel) < 0) ? Vec2(-t.x, -t.y) : t;
        }
        if (nbr[v].size() == 2) return (P(otherNbr(v, from)) - P(from)) * 0.5;
        return travel;
    };

    for (int ei : ev) {
        const std::array<int, 2>& e = net.edges[ei];
        int a = e[0], b = e[1];
        Real w = ewidth(ei);
        Vec2 m0 = outTan(a, b), m1 = inTan(b, a);
        double len = (P(b) - P(a)).length();
        int segs = std::max(4, static_cast<int>(std::ceil(len / 5.0)));
        // Sample the spline with its curvature capped so the half-width (plus sidewalk)
        // ribbon can't fold on an over-tight bend. Endpoints — the shared junction nodes —
        // are preserved, so the graph stays stitched.
        std::vector<Vec2> poly = fairHermite(P(a), m0, P(b), m1, segs, minTurnRadius);
        int lay = elayer(ei);
        // Adaptive tessellation: an edge that never leaves its chord (a straight run — the grid's
        // junction-to-junction edges, whose tangents ARE the chord) collapses back to ONE segment.
        // Densifying straight edges into len/5 collinear samples was the real cause of the overlap and
        // terrain gaps at junctions (road-network-v2-plan T3.2); a genuine bend keeps its samples.
        // EXCEPT a bridge (layer>0): it keeps its samples so the overpass elevation pre-pass has the
        // resolution to raise a flat clearing span over the roads it crosses (else a straight bridge
        // collapses to two endpoints and the crossing in the middle has no node to lift).
        if (lay == 0) {
            Vec2 ab = P(b) - P(a);
            double abl = ab.length();
            double maxDev = 0.0;
            if (abl > 1e-9) {
                Vec2 dir = ab * (1.0 / abl);
                for (std::size_t s = 1; s + 1 < poly.size(); ++s) {
                    Vec2 r = poly[s] - P(a);
                    maxDev = std::max(maxDev, (r - dir * dot(r, dir)).length());
                }
            }
            if (maxDev < 0.06) poly = {P(a), P(b)};
        }

        RoadClass kls = eclass(ei);
        // Elevated span: interior samples ride the authored deck, interpolated by
        // arc-length between the two authored endpoints. Only when BOTH ends are
        // authored — a span with an at-grade end drapes (a homogeneous chain is
        // what the weld's yAbs path needs; a mixed one falls back to the ground).
        const bool elevSpan = nhasElev(a) && nhasElev(b);
        double ea = 0.0, eb = 0.0;
        std::vector<double> arc;
        if (elevSpan) {
            ea = net.nodeElev[a]; eb = net.nodeElev[b];
            arc.assign(poly.size(), 0.0);
            for (std::size_t s = 1; s < poly.size(); ++s)
                arc[s] = arc[s - 1] + (poly[s] - poly[s - 1]).length();
        }
        int prev = a;
        for (std::size_t s = 1; s + 1 < poly.size(); ++s) {     // interior -> new nodes
            int idx = static_cast<int>(g.nodes.size());
            RoadNode nd{poly[s]};
            if (elevSpan) {
                const double f = arc.back() > 1e-9 ? arc[s] / arc.back() : 0.0;
                nd.elev = static_cast<Real>(ea + (eb - ea) * f);
                nd.elevAbsolute = true;
            }
            g.nodes.push_back(nd);
            g.edges.push_back(RoadEdge{prev, idx, w, kls, lay});
            prev = idx;
        }
        g.edges.push_back(RoadEdge{prev, b, w, kls, lay});   // last -> shared node b
    }
    return g;
}

// The graph the mesher AND the terrain-conform both build from: the sampled net graph put
// through the local-constraints pass (ADR-0052), so a promoted roundabout is reflected
// identically in the carriageway and in the ground it grades. One source keeps them in sync.
RoadGraph constrainedNetGraph(const RoadNet& net) {
    double minR = netMinTurnRadius(net);
    RoadRules rules;
    rules.autoRoundabout = net.autoRoundabout;   // honour the net's policy (ADR-0075 P0)
    return applyConstraints(netGraph(net, minR), rules);
}

}  // namespace

// Public accessor (ADR-0059): hand runtime consumers the same sampled+constrained
// graph the mesher uses, without exposing the file-local builder above.
RoadGraph navRoadGraph(const RoadNet& net) { return constrainedNetGraph(net); }

// One UnionSpine per CHAIN (a maximal degree-2 run between junctions/dead-ends), carrying that
// road's WIDTH — so the weld gets smooth per-road ribbons (no per-edge spikes) AND the right width
// (arterials stay wider than streets, which a width-less chain trace loses). A pure cycle with no
// junction (a bare roundabout ring) comes back marked closed, so weldSolid makes it an annulus.
static std::vector<UnionSpine> weldChainSpines(const RoadGraph& g) {
    const int n = static_cast<int>(g.nodes.size());
    std::vector<std::vector<int>> inc(n);
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
        inc[g.edges[e].a].push_back(e); inc[g.edges[e].b].push_back(e);
    }
    auto breaksChain = [&](int v) { return static_cast<int>(inc[v].size()) != 2; };
    std::vector<char> used(g.edges.size(), 0);
    std::vector<UnionSpine> spines;
    auto traceFrom = [&](int v, int e0, bool closed) {
        UnionSpine s;
        s.halfWidth = g.edges[e0].width * 0.5;
        s.klass = g.edges[e0].klass;             // carry class into the weld (P1)
        s.closed = closed;
        s.points.push_back(g.nodes[v].pos);
        // 3-D channel (welder-goes-3D): a chain whose nodes ALL carry an
        // ABSOLUTE deck Y (corridor decks / ramps, elevAbsolute) rides those
        // heights through weldChainProfiles instead of draping. Streets keep
        // elevAbsolute=false, so ys stays discarded and the drape path is
        // unchanged. Chains are homogeneous by construction: a ramp-to-street
        // transition is a degree change, which breaks the chain here.
        std::vector<double> ys{ g.nodes[v].elev };
        bool allAbs = g.nodes[v].elevAbsolute;
        int prev = v, e = e0, startNode = v;
        while (!used[e]) {
            used[e] = 1;
            int nx = (g.edges[e].a == prev) ? g.edges[e].b : g.edges[e].a;
            if (closed && nx == startNode) break;
            s.points.push_back(g.nodes[nx].pos);
            ys.push_back(g.nodes[nx].elev);
            allAbs = allAbs && g.nodes[nx].elevAbsolute;
            if (!closed && breaksChain(nx)) break;
            int ne = -1;
            for (int ee : inc[nx]) if (ee != e && !used[ee]) { ne = ee; break; }
            if (ne < 0) break;
            prev = nx; e = ne;
        }
        if (allAbs && ys.size() == s.points.size()) s.yAbs = std::move(ys);
        return s;
    };
    for (int v = 0; v < n; ++v) {                  // open chains: start at every junction/dead-end
        if (!breaksChain(v)) continue;
        for (int e0 : inc[v])
            if (!used[e0]) {
                UnionSpine s = traceFrom(v, e0, false);
                if (s.points.size() >= 2) spines.push_back(std::move(s));
            }
    }
    for (int e0 = 0; e0 < static_cast<int>(g.edges.size()); ++e0)   // leftover pure cycles (rings)
        if (!used[e0]) {
            UnionSpine s = traceFrom(g.edges[e0].a, e0, true);
            if (s.points.size() >= 3) spines.push_back(std::move(s));
        }
    return spines;
}

namespace {
// Trim a chain to start `rA` / end `rB` into it (arc length), so its ends stop at
// the junction boundary and its swept end rings become the arm mouths.
UnionSpine trimSpine(const UnionSpine& s, double rA, double rB) {
    const int n = static_cast<int>(s.points.size());
    if (n < 2) return s;
    std::vector<double> cum(n, 0.0);
    for (int i = 1; i < n; ++i) cum[i] = cum[i - 1] + (s.points[i] - s.points[i - 1]).length();
    const double L = cum.back();
    const double a = std::min(rA, L * 0.45);
    const double b = std::max(L - std::min(rB, L * 0.45), a + 0.5);
    const bool hasY = static_cast<int>(s.yAbs.size()) == n;
    auto at = [&](double d, Vec2& p, double& y) {
        int i = 0; while (i + 1 < n && cum[i + 1] < d) ++i;
        const double seg = cum[i + 1] - cum[i];
        const double t = seg > 1e-9 ? (d - cum[i]) / seg : 0.0;
        p = s.points[i] + (s.points[i + 1] - s.points[i]) * t;
        if (hasY) y = s.yAbs[i] + (s.yAbs[i + 1] - s.yAbs[i]) * t;
    };
    UnionSpine o; o.halfWidth = s.halfWidth; o.klass = s.klass;
    Vec2 p; double y = 0;
    at(a, p, y); o.points.push_back(p); if (hasY) o.yAbs.push_back(y);
    for (int i = 0; i < n; ++i)
        if (cum[i] > a + 1e-6 && cum[i] < b - 1e-6) {
            o.points.push_back(s.points[i]); if (hasY) o.yAbs.push_back(s.yAbs[i]);
        }
    at(b, p, y); o.points.push_back(p); if (hasY) o.yAbs.push_back(y);
    return o;
}
}  // namespace

RenderMesh buildRoadNetLattice(const RoadGraph& g,
                               const std::function<Real(Real, Real)>& heightAt,
                               std::vector<std::size_t>* chainTriEndsOut) {
    const int N = static_cast<int>(g.nodes.size());
    // Stage 1 of the junction re-architecture (docs/junction-weld-decision.md):
    // the junction owns the FULL cross-section. Trim each body by its full
    // half-width (carriageway + sidewalk), so an arm's raised sidewalk pulls
    // back out of the junction disc instead of sweeping into the pad and
    // double-covering it — the measured 85% of the 16% overlap.
    const double kSidewalkW = 3.0;   // matches the streetProfile(..) call below
    std::vector<int> deg(N, 0);
    std::vector<double> rad(N, 0.0);
    for (const RoadEdge& e : g.edges) {
        ++deg[e.a]; ++deg[e.b];
        rad[e.a] = std::max(rad[e.a], static_cast<double>(e.width) * 0.5 + kSidewalkW);
        rad[e.b] = std::max(rad[e.b], static_cast<double>(e.width) * 0.5 + kSidewalkW);
    }
    auto ground = [&](double x, double z) { return heightAt ? (double)heightAt(x, z) : 0.0; };
    // Position -> node, O(1): the chain endpoints ARE node positions (weldChainSpines
    // uses g.nodes[v].pos), so a 0.5 m-cell hash finds them without the O(N) scan
    // that would make this O(chains * N) on a city.
    auto key = [](const Vec2& p) {
        return (static_cast<long long>(std::llround(p.x * 2.0)) << 32) ^
               (static_cast<long long>(std::llround(p.y * 2.0)) & 0xffffffffLL);
    };
    std::unordered_map<long long, int> nodeIndex;
    nodeIndex.reserve(N * 2);
    for (int v = 0; v < N; ++v) nodeIndex.emplace(key(g.nodes[v].pos), v);
    auto nodeAt = [&](const Vec2& q) {
        auto it = nodeIndex.find(key(q));
        return it == nodeIndex.end() ? -1 : it->second;
    };

    // Dump junction WORLD positions so a camera can be aimed at each one (we know
    // where every intersection is — no coordinate-guessing). Format: "x z degree".
    if (const char* path = std::getenv("RT_DUMP_JUNCTIONS")) {
        if (std::FILE* f = std::fopen(path, "w")) {
            for (int v = 0; v < N; ++v)
                if (deg[v] >= 3)
                    std::fprintf(f, "%.3f %.3f %d\n", (double)g.nodes[v].pos.x,
                                 (double)g.nodes[v].pos.y, deg[v]);
            std::fclose(f);
        }
    }

    RenderMesh out;
    const std::function<double(double, double)> groundFn = ground;   // for strips
    std::vector<std::vector<JunctionArm>> arms(N);
    // ONE profile source (Glenn: "roads under the terrain... floating ribbons").
    // The terrain carve grades to weldChainProfiles' reconciled, grade-limited
    // heights — but the lattice was draping each ring on RAW ground, so wherever
    // smoothing changed the height the road sat under (or above) the carved
    // terrain. Ride the SAME profiles the carve uses: mesh and terrain now agree
    // by construction, exactly like the weld path (plan P3.2).
    std::vector<UnionSpine> chains = weldChainSpines(g);
    {
        std::vector<std::vector<double>> profiles = weldChainProfiles(
            chains, groundFn, 0.0, WeldSolidParams{}.maxGrade, 3.0 + 4.0);
        for (std::size_t si = 0; si < chains.size(); ++si)
            if (chains[si].yAbs.empty() &&
                profiles[si].size() == chains[si].points.size())
                chains[si].yAbs = profiles[si];
    }
    for (const UnionSpine& s : chains) {
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        const double rA = (a >= 0 && deg[a] >= 3) ? rad[a] : 0.0;
        const double rB = (b >= 0 && deg[b] >= 3) ? rad[b] : 0.0;
        UnionSpine t = trimSpine(s, rA, rB);
        if (t.points.size() < 2) continue;

        const int lanesPerSide = std::max(1, static_cast<int>(std::lround(s.halfWidth / 3.6)));
        const int cw = 2 * lanesPerSide + 1;
        std::vector<Vec3> ring0, ringN;
        MeshBuilder::append(out, sweepRoadLattice(t, streetProfile(lanesPerSide, 3.0, 0.15),
                                                  ground, 2.0, nullptr, &ring0, &ringN));
        // The mouth is the FULL profile ring (sidewalk|curb|lanes|curb|sidewalk),
        // reversed to run left -> right looking OUTWARD. Slicing only the
        // carriageway here was the boundary bug: each arm swept its raised
        // sidewalk into a pad that covered carriageway only, so sidewalks
        // double-covered at every corner. The pad now spans verge to verge and
        // its corner points sit at the sidewalk outer edge, where the kerb
        // returns will fillet (stage 2).
        // The pad is ASPHALT: slice the CARRIAGEWAY columns out of the full
        // profile ring (drop sidewalk+curb, 2 columns each side). Curb loops and
        // sidewalk bands own the rest (roads-v2 Part 2, stages 5-6). Reversed so
        // the mouth runs left->right looking OUTWARD along the arm.
        auto mouth = [&](const std::vector<Vec3>& ring) {
            std::vector<Vec3> rev(ring.rbegin(), ring.rend());
            if (static_cast<int>(rev.size()) >= cw + 4)
                return std::vector<Vec3>(rev.begin() + 2, rev.begin() + 2 + cw);
            return rev;
        };
        if (chainTriEndsOut) chainTriEndsOut->push_back(out.indices.size());
        const std::size_t tn = t.points.size();
        if (a >= 0 && deg[a] >= 3 && ring0.size() >= 4)
            arms[a].push_back({ normalize(t.points[1] - t.points[0]), mouth(ring0) });
        if (b >= 0 && deg[b] >= 3 && ringN.size() >= 4)
            arms[b].push_back({ normalize(t.points[tn - 2] - t.points[tn - 1]), mouth(ringN) });
    }

    int nCoons = 0, nT = 0, nFan = 0, nStub = 0, nDeg2 = 0, nMismatch = 0;
    std::vector<int> degHist(12, 0);
    for (int v = 0; v < N; ++v) {
        if (deg[v] < 12) ++degHist[deg[v]];
        if (deg[v] < 3) { if (deg[v] == 2) ++nDeg2; continue; }
        const int na = static_cast<int>(arms[v].size());
        if (na < 3) { ++nStub; continue; }          // arms not gathered (trim/lookup failed)
        if (na != deg[v]) ++nMismatch;               // gathered != incident: a real bug
        if (na == 3) ++nT;                           // Coons T patch
        else if (na == 4) ++nCoons;                  // Coons grid
        else ++nFan;                                 // N>=5: still the fan stopgap
        MeshBuilder::append(out, junctionPatch(arms[v]));
    }
    if (std::getenv("RT_LATTICE_DEBUG")) {
        int degen = 0;
        for (std::size_t t = 0; t + 2 < out.indices.size(); t += 3) {
            const Vec3& a = out.vertices[out.indices[t]].position;
            const Vec3& b = out.vertices[out.indices[t + 1]].position;
            const Vec3& c = out.vertices[out.indices[t + 2]].position;
            if (cross(b - a, c - a).length() < 1e-9) ++degen;
        }
        LOG_INFO << "[lattice] nodes=" << N << " deg2chains=" << nDeg2
                 << " coons4=" << nCoons << " coonsT=" << nT << " fan(5+)=" << nFan
                 << " stub(arms<3)=" << nStub << " armMismatch=" << nMismatch
                 << " | tris=" << (out.indices.size() / 3) << " degenerate=" << degen;
        std::string h;
        for (int d = 0; d < 12; ++d) if (degHist[d]) h += " d" + std::to_string(d) + "=" + std::to_string(degHist[d]);
        LOG_INFO << "[lattice] degree histogram:" << h;
    }
    return out;
}

RenderMesh buildRoadNetMesh(const RoadNet& net) {
    // Cap centerline curvature so neither the carriageway nor the sidewalk outer rail can
    // fold: keep the turn radius above the widest offset (half-width + sidewalk) + margin.
    double minR = netMinTurnRadius(net);
    RoadGraph raw = netGraph(net, minR);
    // Does the local-constraints pass (ADR-0052) promote any node to a roundabout?
    // Honour the net's junction policy (ADR-0075 P0): a generated net set
    // autoRoundabout=false, so we must NOT probe/promote with default rules here
    // (that re-promoted roundabouts the generator disabled).
    RoadRules rules;
    rules.autoRoundabout = net.autoRoundabout;
    bool roundabout = false;
    for (int v = 0; v < static_cast<int>(raw.nodes.size()); ++v)
        if (nodeNeedsRoundabout(raw, v, rules)) { roundabout = true; break; }
    RoadGraph g = applyConstraints(raw, rules);   // promoted graph (= constrainedNetGraph)

    // Grade separations (ADR-0051/0054): an edge on a higher layer is an overpass.
    // Instead of a separate bridge mesher, STAMP an absolute deck elevation onto
    // each higher-layer chain (a flat clearing span over the roads it crosses,
    // ramping down at rampGrade to grade) and let the ONE welder build it — deck,
    // piers, and the Δz grade-separation from the roads below — the same weld that
    // meshes the streets. (Retires buildLayeredRoadNetMesh; unifies the fork.)
    {
        int maxLayer = 0;
        for (const RoadEdge& e : g.edges) maxLayer = std::max(maxLayer, e.layer);
        if (maxLayer > 0) {
            const DesignRules& dr = defaultDesign();
            const double clearance = dr.clearance, deckThk = dr.deckThickness,
                         rampGrade = dr.rampGrade, groundSurf = net.lift;
            auto groundFn = [&](const Vec2& q) { return net.heightAt ? net.heightAt(q.x, q.y) : 0.0; };
            const int N = static_cast<int>(g.nodes.size());
            std::vector<std::vector<int>> incL(N);
            for (int L = 1; L <= maxLayer; ++L) {
                for (auto& v : incL) v.clear();
                for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei)
                    if (g.edges[ei].layer == L) {
                        incL[g.edges[ei].a].push_back(ei); incL[g.edges[ei].b].push_back(ei);
                    }
                auto degL = [&](int v) { return static_cast<int>(incL[v].size()); };
                auto other = [&](int e, int v) { return g.edges[e].a == v ? g.edges[e].b : g.edges[e].a; };
                std::vector<char> used(g.edges.size(), 0);
                auto walk = [&](int startV, int startE) {
                    std::vector<int> ns{startV};
                    int cur = startV, ce = startE;
                    for (;;) {
                        used[ce] = 1; int nx = other(ce, cur); ns.push_back(nx);
                        if (degL(nx) != 2) break;
                        int ne = -1;
                        for (int e2 : incL[nx]) if (e2 != ce && !used[e2]) { ne = e2; break; }
                        if (ne < 0) break; cur = nx; ce = ne;
                    }
                    return ns;
                };
                std::vector<std::vector<int>> chains;
                for (int v = 0; v < N; ++v)
                    if (degL(v) != 2 && degL(v) > 0)
                        for (int e0 : incL[v]) if (!used[e0]) chains.push_back(walk(v, e0));
                for (int e0 = 0; e0 < static_cast<int>(g.edges.size()); ++e0)   // pure rings
                    if (g.edges[e0].layer == L && !used[e0]) chains.push_back(walk(g.edges[e0].a, e0));
                for (const std::vector<int>& ns : chains) {
                    const int n = static_cast<int>(ns.size());
                    if (n < 2) continue;
                    std::vector<double> s(n, 0.0), minH(n);
                    for (int i = 1; i < n; ++i)
                        s[i] = s[i - 1] + (g.nodes[ns[i]].pos - g.nodes[ns[i - 1]].pos).length();
                    // Hold the deck FLAT over each LOWER road it passes above, clearing it by
                    // `clearance`. A chain node within (lower half-width + overhang) of a lower
                    // road's centreline is over the crossing — proximity, not a strict segment
                    // cross, because the sampler puts a shared vertex exactly at the crossing.
                    for (int i = 0; i < n; ++i) {
                        const Vec2 q = g.nodes[ns[i]].pos;
                        minH[i] = groundFn(q) + groundSurf;
                        for (const RoadEdge& e : g.edges) {
                            if (e.layer >= L) continue;                        // only clear roads below
                            const Vec2 a = g.nodes[e.a].pos, ab = g.nodes[e.b].pos - a;
                            const double L2 = ab.lengthSquared();
                            const double t = L2 < 1e-12 ? 0.0
                                : std::max(0.0, std::min(1.0, dot(q - a, ab) / L2));
                            if ((q - (a + ab * t)).length() > e.width * 0.5 + 5.0) continue;
                            minH[i] = std::max(minH[i], groundFn(q) + groundSurf + clearance + deckThk);
                        }
                    }
                    std::vector<double> deckY = clearanceProfile(s, minH, rampGrade);
                    for (int i = 0; i < n; ++i) {
                        g.nodes[ns[i]].elev = static_cast<Real>(deckY[i]);
                        g.nodes[ns[i]].elevAbsolute = true;                     // ride it through the weld
                    }
                }
            }
        }
    }

    // Swept-lattice streets (street-lattice-plan.md stage 3c): temporary env gate
    // so the whole-city switch can be DRIVEN before it becomes the default. Not a
    // standing flag — it comes out when the lattice reaches parity and weldSolid
    // is deleted.
    if (net.latticeStreets || std::getenv("RT_LATTICE_STREETS"))
        return buildRoadNetLattice(g, net.heightAt);

    // Three road meshers. The polygon-union WELD is the default — each road welded into ONE surface
    // (no double-coverage, so sidewalks can't overlap at any junction angle), crisp, fast, UV-native.
    // The SDF grid weld and the analytic per-junction pad are kept as opt-in alternatives (A/B +
    // fallback). A roundabout ring welds fine too: weldChainSpines hands the loop to weldSolid, whose
    // union opens the island as a hole.

    // Opt-in SDF grid weld: RT_SDF_ROADS, or a roundabout under RT_ROUNDABOUT_SDF.
    if (std::getenv("RT_SDF_ROADS") || (roundabout && std::getenv("RT_ROUNDABOUT_SDF"))) {
        RoadbedParams rp;
        rp.cell = 0.4;
        rp.sidewalkWidth = net.sidewalk;
        rp.curbHeight = net.curb;
        rp.lift = net.lift;
        rp.roadColor = net.color;
        rp.heightAt = net.heightAt;
        // Suppress lane markings inside each junction (degree >= 3): the crossing reads as plain
        // asphalt and the road-local UV is ambiguous there. Radius ~ the widest incident road.
        std::vector<int> deg(g.nodes.size(), 0);
        std::vector<double> jw(g.nodes.size(), 0.0);
        for (const RoadEdge& e : g.edges) {
            ++deg[e.a]; ++deg[e.b];
            jw[e.a] = std::max(jw[e.a], static_cast<double>(e.width));
            jw[e.b] = std::max(jw[e.b], static_cast<double>(e.width));
        }
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v)
            if (deg[v] >= 3) { rp.noPaintCenters.push_back(g.nodes[v].pos); rp.noPaintRadii.push_back(jw[v] * 0.85); }
        return unionRoadbed(g, rp);
    }
    // Opt-in analytic per-junction mesher (RT_ANALYTIC_ROADS) — the former default, kept as a fallback
    // and for anything the weld doesn't yet cover.
    if (std::getenv("RT_ANALYTIC_ROADS")) {
        RoadMeshParams p;
        p.lift = net.lift;
        p.color = net.color;
        p.sidewalkWidth = net.sidewalk;
        p.curbHeight = net.curb;
        p.cornerRadius = net.cornerRadius;
        p.laneMarkings = net.markings;
        p.shaderMarkings = net.markings;     // paint via the RoadMarkings surface, not geometry
        p.crosswalks = net.crosswalks;
        p.minSetback = net.width * 0.5 + 0.5;        // pad clears the curb corners
        p.heightAt = net.heightAt;
        return buildRoadMesh(g, p);
    }

    // DEFAULT: polygon-union weld. Smooth per-road ribbons (weldChainSpines) boolean-unioned into one
    // surface with raised sidewalk bands, rounded curb returns, and UV-native lane markings — the
    // welded boundary snap-rounded + de-spiked so sharp junctions stay clean.
    WeldSolidParams wp;
    wp.topY = net.lift;
    wp.thickness = 0.5;
    wp.cornerRadius = net.cornerRadius;
    wp.sidewalkWidth = net.sidewalk;
    wp.curbHeight = net.curb;
    wp.topColor = net.color;
    wp.heightAt = net.heightAt;
    wp.crosswalks = net.crosswalks;   // paint set-back zebra bands into the road texture
    wp.crosswalkMaxWidth = 18.0;      // ...but never across a freeway-width chain
    wp.clearance = defaultDesign().clearance;   // grade-separation Δz threshold (P4)
    wp.barriers = true;               // Freeway-class chains get parapets + median (one mesher)
    // Junction pads (device: mesh holes at skewed T-junctions): chains end square
    // to their own direction, so where a through-road BENDS at a junction the two
    // arm caps disagree by the bend angle and a wedge of ground shows through. A
    // disc per junction node, radius = the widest incident arm's half-width,
    // fills every such wedge regardless of the arm angles.
    {
        std::vector<int> deg(g.nodes.size(), 0);
        std::vector<double> jw(g.nodes.size(), 0.0);
        for (const RoadEdge& e : g.edges) {
            ++deg[e.a]; ++deg[e.b];
            jw[e.a] = std::max(jw[e.a], static_cast<double>(e.width));
            jw[e.b] = std::max(jw[e.b], static_cast<double>(e.width));
        }
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v)
            if (deg[v] >= 3) {
                wp.padCenters.push_back(g.nodes[v].pos);
                wp.padRadii.push_back(jw[v] * 0.5 * 1.02);
            }
    }
    return weldSolid(weldChainSpines(g), wp);
}

std::vector<TerrainFlatten> roadNetConformRegions(const RoadNet& net, double shoulder,
                                                  double falloff, double maxGrade) {
    std::vector<TerrainFlatten> out;
    if (!net.heightAt) return out;                       // flat road: nothing to carve
    // Mirror the DECK's own profile computation EXACTLY — the same constrained
    // graph, the same weldChainSpines decomposition (curve-sampled points,
    // per-chain widths), the same roadProfile smoothing at the weld's grade —
    // so the carve grades the ground to where the deck actually is. The old
    // independently-densified profile diverged by metres on slopes: the
    // grade-limited deck cut through hills the carve never lowered (device:
    // "the road is being buried by the terrain — it's not conforming").
    RoadGraph g = constrainedNetGraph(net);              // grade to the roundabout, not the raw spokes
    (void)maxGrade;   // superseded: the carve must use the deck's own grade
    std::vector<UnionSpine> spines = weldChainSpines(g);
    if (std::getenv("RT_POKE_SITE")) {
        double fp = 0;
        for (std::size_t si = 0; si < spines.size(); ++si)
            fp += spines[si].points.front().x * (si + 1) * 1e-3;
        LOG_INFO << "[conform-fingerprint] nodes=" << g.nodes.size()
                 << " edges=" << g.edges.size() << " spines=" << spines.size()
                 << " fp=" << fp;
    }
    // ONE profile source (plan P3.2): weldChainProfiles now reconciles mid-span
    // overlaps INSIDE the shared pass — the mesher rides the same reconciled
    // heights — so deck and carve cannot disagree at junctions. The old carve-
    // only minOverlapping left the higher deck floating up to 2.6 m above the
    // ground it never lowered (road_poke_probe metropolis, 8.4% verts >1 m).
    std::vector<std::vector<double>> profiles =
        weldChainProfiles(spines, net.heightAt, 0.0, WeldSolidParams{}.maxGrade,
                          net.sidewalk + 4.0);
    for (std::size_t si = 0; si < spines.size(); ++si) {
        const UnionSpine& sp = spines[si];
        if (profiles[si].size() < 2) continue;
        // AUTHORED elevated decks ride on piers — the ground must NOT be graded
        // up to them, or the terrain balloons into a ridge that buries the
        // flyover (weldChainProfiles returns the +Y deck for these). Skip their
        // conform; the piers span deck-to-ground and the surface roads they fly
        // over keep their own at-grade carve. (Ramp feet on sloped ground want a
        // partial carve — a later refinement; on flat ground none is needed.)
        if (!sp.yAbs.empty()) continue;
        std::vector<double> profile = profiles[si];
        // Carve a step BELOW the drivable profile, not exactly to it: the
        // terrain grid interpolates between its samples and can overshoot the
        // carve target past the road's small lift, patchily swallowing the
        // deck. 0.22 m keeps the deck proud; the curb skirt hides the step.
        for (double& hh : profile) hh -= 0.22;
        // Flatten out to the SIDEWALK's outer edge, not just the carriageway —
        // the sidewalk band rides the same smoothed profile and needs ground
        // graded under it too.
        // +2 m margin past the sidewalk's outer edge: corner bevels and the
        // curb skirt reach slightly beyond the band, and the flatten's full
        // strength must cover them before its falloff starts.
        std::vector<TerrainFlatten> r = roadConformRegions(
            sp.points, profile, sp.halfWidth + net.sidewalk + 2.0, shoulder,
            falloff);
        for (TerrainFlatten& f : r) f.owner = static_cast<int>(si);
        // NOTE (ADR-0075): the DaylightBatter earthwork was tried here and REGRESSED
        // the conform on the real CDLOD city — the batter rises too steeply near the
        // road, so coarse-LOD terrain samples poke through the deck (headless
        // road_poke_probe: 0.00% -> 0.14% device poke, worst 0.44 m -> 2.15 m). The
        // smoothstep feather (roadConformRegions' default) sits the road cleanly, so
        // we keep it. The batter mode stays available on TerrainFlatten for a future
        // attempt that samples the deck the way the LOD terrain does.
        out.insert(out.end(), r.begin(), r.end());
    }
    // JUNCTION PAD CARVE (plan P3.2 round 3): the chain footprints above are
    // RECTANGLES that stop at the chain ends, but the weld adds a DISC per
    // junction node (padCenters) whose lobes between the arms sit past every
    // rectangle — on sloped ground those lobes ride natural, uncarved terrain
    // and the deck stands metres proud (or the ground swallows the pad). Carve
    // each disc to the deck's own local plane, sampled from the SAME reconciled
    // profiles the mesher rides (nearest-spine, like weldSolid's heightOf).
    {
        auto deckAt = [&](const Vec2& q) {
            double best = 1e30, h = 0.0;
            for (std::size_t si = 0; si < spines.size(); ++si) {
                if (profiles[si].size() < 2) continue;
                const auto& pts = spines[si].points;
                for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
                    Vec2 ab = pts[i + 1] - pts[i];
                    double L2 = ab.lengthSquared();
                    double t = L2 < 1e-12
                                   ? 0.0
                                   : std::max(0.0, std::min(1.0, dot(q - pts[i], ab) / L2));
                    double d2 = (q - (pts[i] + ab * t)).lengthSquared();
                    if (d2 < best) {
                        best = d2;
                        h = profiles[si][i] + (profiles[si][i + 1] - profiles[si][i]) * t;
                    }
                }
            }
            return h;
        };
        std::vector<int> deg(g.nodes.size(), 0);
        std::vector<double> jw(g.nodes.size(), 0.0);
        for (const RoadEdge& e : g.edges) {
            ++deg[e.a]; ++deg[e.b];
            jw[e.a] = std::max(jw[e.a], static_cast<double>(e.width));
            jw[e.b] = std::max(jw[e.b], static_cast<double>(e.width));
        }
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
            if (deg[v] < 3) continue;
            const Vec2 C = g.nodes[v].pos;
            // Cover the weld disc + the sidewalk wrap + the same margin the
            // chain footprints use, so the pad's full apron sits on graded ground.
            const double r = jw[v] * 0.5 * 1.02 + net.sidewalk + 2.0;
            // The deck's local plane: sampled along the steepest deck direction
            // through the node (8-direction probe), carved 0.22 m under like the
            // chains so the deck stays proud of the interpolating terrain grid.
            Vec2 gdir(0, 0);
            const double hC = deckAt(C);
            for (int k = 0; k < 8; ++k) {
                const double a = 2.0 * 3.14159265358979323846 * k / 8.0;
                const Vec2 d(std::cos(a), std::sin(a));
                gdir = gdir + d * (deckAt(C + d * r) - hC);
            }
            Vec2 axis = gdir.length() > 1e-6 ? normalize(gdir) : Vec2(1, 0);
            const Vec2 A = C - axis * r, B = C + axis * r;
            out.push_back(makeFlattenRamp(Vec3(A.x, 0, A.y), Vec3(B.x, 0, B.y),
                                          deckAt(A) - 0.22, deckAt(B) - 0.22, r,
                                          falloff));
            out.back().owner = -2 - v;   // junction pad for node v
        }
    }
    // FIX A (frontage seams): road regions outrank lot/building pads wherever
    // both cover — the street owns its corridor and verge; pads own the block
    // interior. Priority 1 also lets the LOD bake identify "near a road" for
    // its corner clamp (roadPlaneNear).
    for (TerrainFlatten& f : out) f.priority = 1;
    return out;
}

StructureSet buildRoadWalls(const RoadNet& net, const StructureParams& p) {
    StructureSet set;
    if (!net.heightAt) return set;                      // flat road: no walls
    // SAME decomposition as roadNetConformRegions, so a wall lands exactly where
    // that pass's batter clamps at reach — the two read the identical deck profile.
    RoadGraph g = constrainedNetGraph(net);
    std::vector<UnionSpine> spines = weldChainSpines(g);
    std::vector<std::vector<double>> profiles =
        weldChainProfiles(spines, net.heightAt, 0.0, WeldSolidParams{}.maxGrade,
                          net.sidewalk + 4.0);

    for (std::size_t si = 0; si < spines.size(); ++si) {
        const UnionSpine& sp = spines[si];
        const int n = static_cast<int>(sp.points.size());
        if (static_cast<int>(profiles[si].size()) != n || n < 2) continue;
        const double flatHalf = sp.halfWidth + net.sidewalk + 2.0;   // graded-corridor half-width
        const double wallLat = flatHalf + p.reach;                    // where the batter clamps
        // The wall top sits at the batter-end height at lateral offset wallLat; its
        // drop is the residual to natural ground there. One short quad per station
        // pair, so the wall follows the road's undulation.
        auto stationWall = [&](int side, int k, Vec3& topOut, double& dropOut, bool& retainOut) {
            int seg = std::min(k, n - 2);
            Vec2 dir = normalize(sp.points[seg + 1] - sp.points[seg]);
            Vec2 nrm = perp(dir);
            Vec2 lat = sp.points[k] + nrm * (static_cast<double>(side) * wallLat);
            double deckY = profiles[si][k];
            double naturalEnd = net.heightAt(lat.x, lat.y);
            bool cut = naturalEnd > deckY;
            double batterEndY = cut ? deckY + p.reach * p.cutBatter
                                    : deckY - p.reach * p.fillBatter;
            double drop = cut ? (naturalEnd - batterEndY)   // hill still above the cut batter
                              : (batterEndY - naturalEnd);  // fill batter still above ground
            topOut = Vec3(lat.x, batterEndY, lat.y);
            dropOut = std::max(0.0, drop);
            retainOut = cut;
        };
        for (int side = -1; side <= 1; side += 2)
            for (int k = 0; k + 1 < n; ++k) {
                Vec3 topA, topB; double dropA, dropB; bool retainA, retainB;
                stationWall(side, k, topA, dropA, retainA);
                stationWall(side, k + 1, topB, dropB, retainB);
                if (dropA <= p.minWall && dropB <= p.minWall) continue;
                set.walls.push_back(WallSegment{topA, topB, dropA, dropB, retainA || retainB});
                set.colliderEdges.push_back({topA, topB});
            }
    }
    set.mesh = bakeWallMesh(set.walls, p.color);
    return set;
}

void roadNetSetWidth(RoadNet& net, double width) {
    net.width = std::max(0.5, width);
}

double roadNetEdgeWidth(const RoadNet& net, int ei) {
    if (ei >= 0 && ei < static_cast<int>(net.edgeWidths.size()) && net.edgeWidths[ei] > 0.0)
        return net.edgeWidths[ei];
    return net.width;
}

bool roadNetSetEdgeWidth(RoadNet& net, int ei, double w) {
    if (ei < 0 || ei >= static_cast<int>(net.edges.size())) return false;
    if (static_cast<int>(net.edgeWidths.size()) < static_cast<int>(net.edges.size()))
        net.edgeWidths.resize(net.edges.size(), 0.0);
    net.edgeWidths[ei] = (w > 0.0) ? w : 0.0;          // <= 0 reverts to the default width
    return true;
}

bool roadNetMoveNode(RoadNet& net, int i, const Vec2& pos) {
    if (i < 0 || i >= static_cast<int>(net.nodes.size())) return false;
    net.nodes[i] = pos;
    return true;
}

bool roadNetSetTangent(RoadNet& net, int i, const Vec2& tangent) {
    if (i < 0 || i >= static_cast<int>(net.nodes.size())) return false;
    if (static_cast<int>(net.tangents.size()) < static_cast<int>(net.nodes.size()))
        net.tangents.resize(net.nodes.size(), Vec2(0, 0));
    net.tangents[i] = tangent;
    return true;
}

Vec2 roadNetTangentAt(const RoadNet& net, int i) {
    const int n = static_cast<int>(net.nodes.size());
    if (i < 0 || i >= n) return Vec2(0, 0);
    if (i < static_cast<int>(net.tangents.size()) && !isZero(net.tangents[i]))
        return net.tangents[i];
    // Auto: Catmull-Rom on a degree-2 through-road, chord at an end, else zero.
    std::vector<int> nb;
    for (const std::array<int, 2>& e : validEdges(net)) {
        if (e[0] == i) nb.push_back(e[1]);
        if (e[1] == i) nb.push_back(e[0]);
    }
    if (nb.size() == 2) return (net.nodes[nb[1]] - net.nodes[nb[0]]) * 0.5;
    if (nb.size() == 1) return (net.nodes[nb[0]] - net.nodes[i]) * 0.5;
    return Vec2(0, 0);
}

int roadNetAddNode(RoadNet& net, const Vec2& pos) {
    net.nodes.push_back(pos);                 // tangents stays shorter -> the new node is auto
    return static_cast<int>(net.nodes.size()) - 1;
}

bool roadNetAddEdge(RoadNet& net, int a, int b) {
    const int n = static_cast<int>(net.nodes.size());
    if (a < 0 || b < 0 || a >= n || b >= n || a == b) return false;
    for (const std::array<int, 2>& e : net.edges)
        if ((e[0] == a && e[1] == b) || (e[0] == b && e[1] == a)) return false;   // already joined
    net.edges.push_back({a, b});
    if (!net.edgeWidths.empty()) net.edgeWidths.push_back(0.0);   // default width
    if (!net.edgeClasses.empty()) net.edgeClasses.push_back(RoadClass::Local);
    return true;
}

int roadNetExtend(RoadNet& net, int from, const Vec2& pos) {
    if (from < 0 || from >= static_cast<int>(net.nodes.size())) return -1;
    int ni = roadNetAddNode(net, pos);
    roadNetAddEdge(net, from, ni);
    return ni;
}

int roadNetSplitEdge(RoadNet& net, int edgeIndex, const Vec2& pos) {
    if (edgeIndex < 0 || edgeIndex >= static_cast<int>(net.edges.size())) return -1;
    int a = net.edges[edgeIndex][0], b = net.edges[edgeIndex][1];
    int ni = roadNetAddNode(net, pos);
    net.edges[edgeIndex] = {a, ni};           // a -> new
    net.edges.push_back({ni, b});             // new -> b
    if (!net.edgeWidths.empty())              // both halves inherit the split edge's width
        net.edgeWidths.push_back(edgeIndex < static_cast<int>(net.edgeWidths.size())
                                     ? net.edgeWidths[edgeIndex] : 0.0);
    if (!net.edgeClasses.empty())             // ...and its class
        net.edgeClasses.push_back(edgeIndex < static_cast<int>(net.edgeClasses.size())
                                      ? net.edgeClasses[edgeIndex] : RoadClass::Local);
    return ni;
}

bool roadNetDeleteNode(RoadNet& net, int i) {
    const int n = static_cast<int>(net.nodes.size());
    if (i < 0 || i >= n) return false;
    std::vector<std::array<int, 2>> kept;
    std::vector<double> keptW;
    std::vector<RoadClass> keptC;
    const bool hasW = !net.edgeWidths.empty();
    const bool hasC = !net.edgeClasses.empty();
    for (int ei = 0; ei < static_cast<int>(net.edges.size()); ++ei) {
        const std::array<int, 2>& e = net.edges[ei];
        if (e[0] == i || e[1] == i) continue;                 // drop incident edges
        kept.push_back({ e[0] > i ? e[0] - 1 : e[0], e[1] > i ? e[1] - 1 : e[1] });
        if (hasW) keptW.push_back(ei < static_cast<int>(net.edgeWidths.size())
                                      ? net.edgeWidths[ei] : 0.0);
        if (hasC) keptC.push_back(ei < static_cast<int>(net.edgeClasses.size())
                                      ? net.edgeClasses[ei] : RoadClass::Local);
    }
    net.edges = std::move(kept);
    if (hasW) net.edgeWidths = std::move(keptW);              // stay parallel to edges
    if (hasC) net.edgeClasses = std::move(keptC);
    net.nodes.erase(net.nodes.begin() + i);
    if (i < static_cast<int>(net.tangents.size()))
        net.tangents.erase(net.tangents.begin() + i);          // keep tangents parallel
    return true;
}

int roadNetNearestEdge(const RoadNet& net, const Vec2& p, double maxDist) {
    int best = -1;
    double bestD2 = maxDist * maxDist;
    for (int ei = 0; ei < static_cast<int>(net.edges.size()); ++ei) {
        int a = net.edges[ei][0], b = net.edges[ei][1];
        if (a < 0 || b < 0 || a >= static_cast<int>(net.nodes.size())
            || b >= static_cast<int>(net.nodes.size())) continue;
        Vec2 A = net.nodes[a], B = net.nodes[b], ab = B - A;
        double len2 = ab.lengthSquared();
        double t = len2 > 1e-9 ? std::max(0.0, std::min(1.0, dot(p - A, ab) / len2)) : 0.0;
        double d2 = (p - (A + ab * t)).lengthSquared();
        if (d2 < bestD2) { bestD2 = d2; best = ei; }
    }
    return best;
}

RoadNet roadNetFromJson(const json& j) {
    RoadNet net;
    if (j.contains("nodes") && j["nodes"].is_array())
        for (const json& p : j["nodes"])
            net.nodes.push_back(Vec2(p.value("x", 0.0), p.value("z", 0.0)));
    if (j.contains("edges") && j["edges"].is_array())
        for (const json& e : j["edges"]) {
            double w = 0.0;
            if (e.is_array() && e.size() >= 2) {
                net.edges.push_back({e[0].get<int>(), e[1].get<int>()});
                if (e.size() >= 3) w = e[2].get<double>();        // [a, b, width]
            } else if (e.is_object()) {
                net.edges.push_back({e.value("a", 0), e.value("b", 0)});
                w = e.value("width", 0.0);
            } else {
                continue;
            }
            if (w > 0.0) {                                        // a per-edge override
                if (net.edgeWidths.size() < net.edges.size())
                    net.edgeWidths.resize(net.edges.size(), 0.0);
                net.edgeWidths.back() = w;
            }
        }
    if (j.contains("edge_layers") && j["edge_layers"].is_array()) {
        net.edgeLayers.assign(net.edges.size(), 0);
        const json& el = j["edge_layers"];
        for (std::size_t i = 0; i < el.size() && i < net.edgeLayers.size(); ++i)
            net.edgeLayers[i] = el[i].get<int>();
    }
    if (j.contains("edge_classes") && j["edge_classes"].is_array()) {
        net.edgeClasses.assign(net.edges.size(), RoadClass::Local);
        const json& ec = j["edge_classes"];
        auto parseClass = [](const std::string& s) {
            if (s == "freeway")   return RoadClass::Freeway;
            if (s == "arterial")  return RoadClass::Arterial;
            if (s == "collector") return RoadClass::Collector;
            if (s == "ramp")      return RoadClass::Ramp;
            return RoadClass::Local;
        };
        for (std::size_t i = 0; i < ec.size() && i < net.edgeClasses.size(); ++i)
            net.edgeClasses[i] = parseClass(ec[i].get<std::string>());
    }
    if (j.contains("tangents") && j["tangents"].is_array())
        for (const json& t : j["tangents"]) {
            if (t.is_array() && t.size() >= 2)
                net.tangents.push_back(Vec2(t[0].get<double>(), t[1].get<double>()));
            else if (t.is_object())
                net.tangents.push_back(Vec2(t.value("x", 0.0), t.value("z", 0.0)));
        }
    // Per-node absolute deck Y (elevated roads). Parallel to nodes; a null (or a
    // non-number) entry means at-grade, stored as NaN so netGraph drapes it.
    if (j.contains("node_elev") && j["node_elev"].is_array()) {
        const json& ne = j["node_elev"];
        net.nodeElev.assign(net.nodes.size(),
                            std::numeric_limits<double>::quiet_NaN());
        for (std::size_t i = 0; i < ne.size() && i < net.nodeElev.size(); ++i)
            if (ne[i].is_number()) net.nodeElev[i] = ne[i].get<double>();
    }
    net.width = j.value("width", net.width);
    net.sidewalk = j.value("sidewalk", net.sidewalk);
    net.curb = j.value("curb", net.curb);
    net.cornerRadius = j.value("corner_radius", net.cornerRadius);
    net.lift = j.value("lift", net.lift);
    net.markings = j.value("markings", net.markings);
    net.crosswalks = j.value("crosswalks", net.crosswalks);
    net.autoRoundabout = j.value("auto_roundabout", net.autoRoundabout);
    net.latticeStreets = j.value("lattice", net.latticeStreets);
    if (j.contains("color") && j["color"].is_array() && j["color"].size() == 3)
        net.color = Vec3(j["color"][0].get<double>(), j["color"][1].get<double>(),
                         j["color"][2].get<double>());
    return net;
}

json roadNetToJson(const RoadNet& net) {
    json j;
    json nodes = json::array();
    for (const Vec2& p : net.nodes) nodes.push_back({{"x", p.x}, {"z", p.y}});
    j["nodes"] = std::move(nodes);
    json edges = json::array();
    for (int i = 0; i < static_cast<int>(net.edges.size()); ++i) {
        const std::array<int, 2>& e = net.edges[i];
        double w = (i < static_cast<int>(net.edgeWidths.size())) ? net.edgeWidths[i] : 0.0;
        if (w > 0.0) edges.push_back(json::array({e[0], e[1], w}));    // [a, b, width]
        else         edges.push_back(json::array({e[0], e[1]}));
    }
    j["edges"] = std::move(edges);
    bool anyLayer = false;
    for (int l : net.edgeLayers) if (l != 0) { anyLayer = true; break; }
    if (anyLayer) {
        json layers = json::array();
        for (int i = 0; i < static_cast<int>(net.edges.size()); ++i)
            layers.push_back(i < static_cast<int>(net.edgeLayers.size()) ? net.edgeLayers[i] : 0);
        j["edge_layers"] = std::move(layers);
    }
    if (!net.tangents.empty()) {
        json tans = json::array();
        for (const Vec2& t : net.tangents) tans.push_back(json::array({t.x, t.y}));
        j["tangents"] = std::move(tans);
    }
    bool anyElev = false;
    for (double e : net.nodeElev) if (std::isfinite(e)) { anyElev = true; break; }
    if (anyElev) {
        json elev = json::array();
        for (std::size_t i = 0; i < net.nodes.size(); ++i)
            if (i < net.nodeElev.size() && std::isfinite(net.nodeElev[i]))
                elev.push_back(net.nodeElev[i]);
            else
                elev.push_back(nullptr);           // at-grade node
        j["node_elev"] = std::move(elev);
    }
    j["width"] = net.width;
    j["sidewalk"] = net.sidewalk;
    j["curb"] = net.curb;
    j["corner_radius"] = net.cornerRadius;
    j["lift"] = net.lift;
    j["markings"] = net.markings;
    j["crosswalks"] = net.crosswalks;
    j["color"] = json::array({net.color.x, net.color.y, net.color.z});
    return j;
}

// Bow a generated grid into curved streets: split each long edge at its midpoint and push that point
// off the chord by a smooth position-driven field (low-frequency sine), so neighbouring streets sweep
// together into coherent curves rather than random wiggles. Short junction-internal edges are left
// straight, so the intersections (degree >= 3, straight tangents) stay clean — the curve lives in the
// degree-2 mid-spans where the Catmull-Rom sampler renders it.
static RoadGraph warpGraph(const RoadGraph& in, double curviness) {
    const double f = 0.016;                 // warp field wavelength ~ 1/f ~ 60 m
    const double spacing = 34.0;            // sample along each edge so the spline has points to bend
    const double amp = curviness * 42.0;    // displacement magnitude (m)
    // DOMAIN WARP: displace every node — and points sampled along each edge — by a smooth low-frequency
    // vector field. Because the displacement is a continuous function of POSITION, neighbours move
    // together, so a gentle warp is injective: the grid deforms into flowing organic streets WITHOUT
    // roads crossing each other, and a capDegree cluster shifts as one unit instead of spiralling.
    // (Per-edge bowing moved each edge on its own and so could cross its neighbours.) The Catmull-Rom
    // sampler renders the warped chains as curves; junctions keep straight tangents and stay clean.
    auto warp = [&](const Vec2& p) {
        double dx = std::sin(p.y * f) + 0.5 * std::sin(p.y * f * 2.1 + 1.3);
        double dy = std::sin(p.x * f + 2.4) + 0.5 * std::sin(p.x * f * 1.7 + 4.1);
        return Vec2(dx * amp, dy * amp);
    };
    RoadGraph out;
    out.nodes.reserve(in.nodes.size());
    for (const RoadNode& n : in.nodes) out.nodes.push_back(RoadNode{n.pos + warp(n.pos)});
    for (const RoadEdge& e : in.edges) {
        Vec2 a = in.nodes[e.a].pos, b = in.nodes[e.b].pos;   // ORIGINAL positions: sample, then warp
        double len = (b - a).length();
        int segs = std::max(1, static_cast<int>(std::lround(len / spacing)));
        int prev = e.a;
        for (int i = 1; i < segs; ++i) {
            double t = static_cast<double>(i) / segs;
            Vec2 p = a + (b - a) * t;
            int mi = static_cast<int>(out.nodes.size());
            out.nodes.push_back(RoadNode{p + warp(p)});
            out.edges.push_back(RoadEdge{prev, mi, e.width, e.klass, e.layer});
            prev = mi;
        }
        out.edges.push_back(RoadEdge{prev, e.b, e.width, e.klass, e.layer});
    }
    return out;
}

// Spread acute junctions so the mesher's corners stay weldable: at an acute intersection the curb
// returns overrun and the sidewalk/pad slivers (no per-corner math fully saves a <~30deg crossing).
// For each real junction (degree >= 3) any adjacent arm pair closer than `minAngle` is opened up —
// both far nodes rotated around the junction by half the deficit — over a few capped passes so the
// corrections settle. NOT a roundabout: topology is untouched, only the approach angles are relaxed.
static RoadGraph deAcute(const RoadGraph& in, double minAngle) {
    const double kTwoPi = 6.283185307179586;
    RoadGraph g = in;
    auto rotateFar = [&](int v, int far, double ang) {
        Vec2 c = g.nodes[v].pos, d = g.nodes[far].pos - c;
        double cs = std::cos(ang), sn = std::sin(ang);
        g.nodes[far].pos = c + Vec2(d.x * cs - d.y * sn, d.x * sn + d.y * cs);
    };
    for (int pass = 0; pass < 10; ++pass) {
        std::vector<std::vector<int>> inc(g.nodes.size());
        for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
            inc[g.edges[e].a].push_back(e);
            inc[g.edges[e].b].push_back(e);
        }
        bool changed = false;
        for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
            if (static_cast<int>(inc[v].size()) < 3) continue;            // only real junctions
            struct Arm { int far; double ang; };
            std::vector<Arm> arms;
            for (int e : inc[v]) {
                int far = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
                Vec2 d = g.nodes[far].pos - g.nodes[v].pos;
                if (d.lengthSquared() > 1e-9) arms.push_back({far, std::atan2(d.y, d.x)});
            }
            int n = static_cast<int>(arms.size());
            if (n < 3) continue;
            std::sort(arms.begin(), arms.end(), [](const Arm& a, const Arm& b) { return a.ang < b.ang; });
            for (int k = 0; k < n; ++k) {
                Arm& a0 = arms[k];
                Arm& a1 = arms[(k + 1) % n];
                double gap = a1.ang - a0.ang;
                if (gap <= 0) gap += kTwoPi;
                if (gap >= minAngle || gap < 1e-3) continue;
                double corr = std::min((minAngle - gap) * 0.5, 0.12);     // half each, capped per pass
                rotateFar(v, a0.far, -corr);
                rotateFar(v, a1.far, corr);
                a0.ang -= corr; a1.ang += corr;
                changed = true;
            }
        }
        if (!changed) break;
    }
    return g;
}

// The HARD angle constraint behind deAcute (device: "we should have more rules
// in the road graph that disallow sharp angles"): relaxation rotates arms
// apart, but a hemmed-in junction can be un-relaxable — two arms stay nearly
// parallel and the sidewalk crotch between them is a razor no corner math can
// weld. Any arm pair still tighter than `hardMin` after relaxation loses its
// SHORTER edge: a clean cul-de-sac beats a broken junction.
static RoadGraph pruneAcuteArms(const RoadGraph& in, double hardMin) {
    const double kTwoPi = 6.283185307179586;
    RoadGraph g = in;
    std::vector<char> drop(g.edges.size(), 0);
    std::vector<std::vector<int>> inc(g.nodes.size());
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
        inc[g.edges[e].a].push_back(e);
        inc[g.edges[e].b].push_back(e);
    }
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) {
        if (static_cast<int>(inc[v].size()) < 3) continue;
        struct Arm { int edge; double ang, len; };
        std::vector<Arm> arms;
        for (int e : inc[v]) {
            if (drop[e]) continue;
            int far = (g.edges[e].a == v) ? g.edges[e].b : g.edges[e].a;
            Vec2 d = g.nodes[far].pos - g.nodes[v].pos;
            if (d.lengthSquared() > 1e-9)
                arms.push_back({e, std::atan2(d.y, d.x), d.length()});
        }
        int n = static_cast<int>(arms.size());
        if (n < 3) continue;
        std::sort(arms.begin(), arms.end(),
                  [](const Arm& a, const Arm& b) { return a.ang < b.ang; });
        for (int k = 0; k < n; ++k) {
            const Arm& a0 = arms[k];
            const Arm& a1 = arms[(k + 1) % n];
            if (drop[a0.edge] || drop[a1.edge]) continue;
            double gap = a1.ang - a0.ang;
            if (gap <= 0) gap += kTwoPi;
            if (gap >= hardMin || gap < 1e-3) continue;
            drop[a0.len <= a1.len ? a0.edge : a1.edge] = 1;
        }
    }
    RoadGraph out;
    out.nodes = g.nodes;
    for (int e = 0; e < static_cast<int>(g.edges.size()); ++e)
        if (!drop[e]) out.edges.push_back(g.edges[e]);
    return out;
}

void applyGenerateRecipe(RoadNet& net, const json& g) {
    if (!g.is_object()) return;
    // Pick the generator. "district" (default) subdivides one footprint; "metro"
    // grows organic arterials between hotspots and fills the blocks between them.
    // Both hand a raw RoadGraph to the SHARED junction-policy tail below.
    const std::string kind = g.value("kind", std::string("district"));
    RoadGraph base;
    if (kind == "metro") {
        MetroParams mp;
        if (g.contains("center")) {
            const json& c = g["center"];
            mp.center = Vec2(c.value("x", 0.0), c.value("z", 0.0));
        }
        mp.radius      = g.value("radius", 700.0);
        mp.hotspots    = g.value("hotspots", 6);
        mp.blockSize   = g.value("block_size", 70.0);
        mp.arteryWidth = g.value("artery_width", net.width * 1.6);
        mp.streetWidth = g.value("street_width", net.width);
        mp.ringRoad    = g.value("ring_road", false);
        mp.seed        = g.value("seed", 5u);
        // Metropolis tier: freeway backbone + collector streets. Defaults on for
        // 2 km-class footprints, off for the small metros that predate the tier.
        mp.freeways           = g.value("freeways", mp.radius >= 550.0);
        mp.freewayWidth       = g.value("freeway_width", 22.0);
        mp.collectorWidth     = g.value("collector_width", 9.5);
        mp.collectorSpan      = g.value("collector_span", 0.0);
        mp.interchangeSpacing = g.value("interchange_spacing", 520.0);
        mp.corridorFreeways   = g.value("corridor_freeways", false);
        mp.arterialsOnly      = g.value("arterials_only", false);   // v2 stage 1
        // Growth spacing (the "room to breathe" dials): a big metro roughly
        // doubles the small-metro defaults so faces are parcel-sized.
        mp.segLength       = g.value("seg_length", mp.segLength);
        mp.influence       = g.value("influence", mp.influence);
        mp.killRadius      = g.value("kill_radius", mp.killRadius);
        mp.mergeRadius     = g.value("merge_radius", mp.mergeRadius);
        mp.corridorSpacing = g.value("corridor_spacing", mp.corridorSpacing);
        mp.ambientPer500   = g.value("ambient_per_500", mp.ambientPer500);
        mp.loopMin         = g.value("loop_min", mp.loopMin);
        mp.loopMax         = g.value("loop_max", mp.loopMax);
        mp.outHubs = &net.cityHubs;   // polycentric zoning reads these (city_lots)
        // Terrain-aware layout: when the road is draped on terrain, gate the city
        // on buildability so it hugs buildable land and avoids water / steep
        // mountain instead of marching over them. Opt out with terrain_aware:false.
        if (net.heightAt && g.value("terrain_aware", true)) {
            mp.ground = net.heightAt;
            mp.build.maxSlope  = g.value("max_slope", 0.32);
            mp.build.seaLevel  = g.value("sea_level", -1e30);
            mp.build.beachRise = g.value("beach_rise", 2.5);
            mp.build.riverWidth = g.value("river_width", 20.0);
            if (g.contains("rivers") && g["rivers"].is_array())
                for (const auto& rv : g["rivers"]) {
                    std::vector<Vec2> line;
                    if (rv.is_array())
                        for (const auto& pt : rv)
                            if (pt.is_array() && pt.size() >= 2)
                                line.push_back(Vec2(pt[0].get<double>(), pt[1].get<double>()));
                    if (line.size() >= 2) mp.build.rivers.push_back(std::move(line));
                }
        }
        base = buildMetro(mp, &net.freewayPlans);
    } else {
        DistrictParams dp;
        if (g.contains("center")) {
            const json& c = g["center"];
            dp.center = Vec2(c.value("x", 0.0), c.value("z", 0.0));
        }
        dp.radius       = g.value("radius", 130.0);
        dp.arterials    = g.value("arterials", 3);
        double bs       = g.value("block_size", 36.0);   // nominal target; min/max bracket it
        dp.blockSizeMax = g.value("block_size_max", bs);
        dp.blockSizeMin = g.value("block_size_min", bs * 0.55);
        dp.irregular    = g.value("irregular", 0.22);
        dp.jitter       = g.value("jitter", 0.16);
        dp.seed         = g.value("seed", 1u);
        dp.arteryWidth  = g.value("artery_width", net.width * 1.6);
        dp.streetWidth  = g.value("street_width", net.width);
        base = buildDistrict(dp).graph;
    }
    double curviness = g.value("curviness", 0.0);    // 0 = straight grid; >0 sweeps streets into curves
    // City-generation junction policy (road-network-v2-plan T1.1/T1.2): no auto roundabouts;
    // planarize every crossing into shared nodes, then cap degree to <=4 LAST — planarize itself can
    // lift a node past the cap when a street T's into it, so capping has to bind on the final noded
    // graph. The result is one connected, editable, degree-capped network.
    RoadRules rules;
    rules.autoRoundabout = false;
    RoadGraph cg = capDegree(planarize(applyConstraints(base, rules), 1.0), rules);
    if (kind == "metro") {
        // The metro hands us a clean planar graph: organic arterials + streets that
        // SUBDIVIDE the blocks between them. The district cleanup below (acute-arm
        // prune, deAcute, relax) is tuned for one footprint's regular grid and
        // shreds the irregular-face subdivision — the local streets meet arterials
        // at every angle by design. So only de-sliver lightly (drop sub-metre edges
        // planarize leaves at street/arterial tees) and keep the rest intact.
        // Glenn (repeatedly): "some of the short roads shouldn't exist." The old
        // 3.0 m floor kept sub-5 m stubs (the double-stoplight collinear stub was
        // 4.94 m) that ride other edges' ribbons. 10 m is well under the ~70 m
        // blocks (no subdivision shredding) and well over stub scale.
        cg = mergeShortEdges(cg, g.value("min_road_len", 10.0), rules.maxDegree);
        // MINIMUM ROAD CLEARANCE (Glenn, repeatedly: "why are there two multilane
        // roads literally right next to each other?"). The growth radii are
        // width-BLIND, so the generator can route chains closer than their
        // ribbons are wide — measured as 46% of all surface overlap. Drop any
        // edge whose interior rides INSIDE a longer edge's ribbon: interior
        // samples projecting onto the other edge's INTERIOR (u in 0.05..0.95 —
        // chain continuations project onto endpoints and are never flagged)
        // within clearance * the combined half-widths.
        {
            const double clear = g.value("min_road_clearance", 0.8);
            std::vector<char> drop(cg.edges.size(), 0);
            for (std::size_t i = 0; i < cg.edges.size(); ++i) {
                const Vec2 a1 = cg.nodes[cg.edges[i].a].pos, b1 = cg.nodes[cg.edges[i].b].pos;
                const double len1 = (b1 - a1).length();
                if (len1 < 1e-6) continue;
                for (std::size_t j = 0; j < cg.edges.size() && !drop[i]; ++j) {
                    if (i == j || drop[j]) continue;
                    const Vec2 a2 = cg.nodes[cg.edges[j].a].pos, b2 = cg.nodes[cg.edges[j].b].pos;
                    const Vec2 d2v = b2 - a2;
                    const double L2 = d2v.lengthSquared();
                    const double len2 = std::sqrt(L2);
                    if (L2 < 1e-9) continue;
                    // Keep the longer/wider of a shadowed pair; equal -> lower
                    // index survives (deterministic, never drops both).
                    const bool jWins =
                        len2 > len1 * 1.05 ||
                        (len2 > len1 * 0.7 &&
                         (cg.edges[j].width > cg.edges[i].width ||
                          (cg.edges[j].width == cg.edges[i].width && j < i)));
                    if (!jWins) continue;
                    const double thr =
                        (cg.edges[i].width + cg.edges[j].width) * 0.5 * clear;
                    bool inside = true;
                    for (double t = 0.25; t <= 0.76 && inside; t += 0.25) {
                        const Vec2 p = a1 + (b1 - a1) * t;
                        const double u = dot(p - a2, d2v) / L2;
                        if (u < 0.05 || u > 0.95) { inside = false; break; }
                        if ((p - (a2 + d2v * u)).length() > thr) inside = false;
                    }
                    if (inside) drop[i] = 1;
                }
            }
            RoadGraph kept;
            kept.nodes = cg.nodes;
            int nDrop = 0;
            for (std::size_t e = 0; e < cg.edges.size(); ++e)
                if (!drop[e]) kept.edges.push_back(cg.edges[e]); else ++nDrop;
            if (nDrop > 0) cg = std::move(kept);
        }
    } else {
        // Minimum road length (device: "really short roads ... should be merged"):
        // fold crossings that landed close together into one junction, or stretch a
        // stub that can't merge (a capDegree stagger link) out to a drivable length.
        // Runs BEFORE the warp so it sees real junction-to-junction edges, not the
        // curve samples warping introduces.
        const double minRoadLen = g.value("min_road_len", 14.0);
        if (minRoadLen > 0.0) cg = mergeShortEdges(cg, minRoadLen, rules.maxDegree);
        if (curviness > 0.0) cg = warpGraph(cg, curviness);   // domain-warp the grid into organic curves
        cg = deAcute(cg, 0.6);                                 // open up acute junctions so corners stay clean
        // Hard floor (device: "disallow sharp angles like that ... some
        // constraints"): drop the shorter arm of any junction pair relaxation
        // couldn't open past ~20 deg, so no razor sidewalk crotch survives. Opt-out
        // via "prune_acute_deg": 0.
        const double pruneDeg = g.value("prune_acute_deg", 20.0);
        if (pruneDeg > 0.0) cg = pruneAcuteArms(cg, pruneDeg * 3.14159265358979323846 / 180.0);
        // No hairpins (device: "sharp bends ... creating some really bad overlap"):
        // a degree-2 corner sharper than ~52 deg folds the stroked carriageway over
        // itself. Relax such through-nodes toward their chord until drivable.
        cg = relaxSharpBends(cg);
    }
    // Carry the generator's junction policy onto the net so the mesh + conform
    // passes don't re-promote roundabouts it deliberately disabled (ADR-0075 P0).
    net.autoRoundabout = rules.autoRoundabout;
    net.nodes.clear(); net.edges.clear(); net.edgeWidths.clear();
    net.edgeClasses.clear(); net.edgeLayers.clear();
    for (const RoadNode& n : cg.nodes) net.nodes.push_back(n.pos);
    for (const RoadEdge& e : cg.edges) {
        net.edges.push_back({e.a, e.b});
        net.edgeWidths.push_back(e.width);          // arterials wider than local streets
        net.edgeClasses.push_back(e.klass);         // carry the grown class (P1 unification)
        net.edgeLayers.push_back(e.layer);          // and its grade-separation tier
    }
}

// Diagnostic accessors (RT_POKE_REPORT): the poke instrument must read the
// EXACT chain decomposition + constrained graph the mesher builds from — a
// near-miss decomposition gave it deck heights that drifted from the real road
// on hills and poisoned the LOD0 numbers (plan P3.2 round 6).
RoadGraph roadNetConstrainedGraph(const RoadNet& net) { return constrainedNetGraph(net); }
std::vector<UnionSpine> roadNetWeldSpines(const RoadGraph& g) { return weldChainSpines(g); }

json roadRecipeForSave(const std::string& currentRecipe, const RoadNet& net) {
    json recipe = json::parse(currentRecipe, nullptr, false);
    if (!recipe.is_object() || !recipe.contains("generate"))
        return roadNetToJson(net);                  // hand-authored: the net IS the saved form
    // Generated road: keep the "generate" block and refresh only the look from the net — never bake
    // the nodes (baking froze the city and lost the recipe, the grown.json "save changed" bug).
    json look = roadNetToJson(net);
    for (const char* k : {"nodes", "edges", "edge_layers", "tangents"}) look.erase(k);
    look["generate"] = recipe["generate"];
    return look;
}

}  // namespace engine
