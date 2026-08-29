#include "road_net.h"

#include "road_semantics.h"
#include "../../../log.h"
#include "../../../profile.h"
#include "road_lattice.h"
#include "road_offset.h"   // ribbonOutline + polygonUnion (S5 curb/sidewalk band)        // swept-lattice street mesher (stage 3)
#include "../../mesh_builder.h"  // MeshBuilder::append

#include "road_network.h"       // RoadGraph, RoadEdge
#include "road_constraints.h"   // applyConstraints, capDegree, RoadRules
#include "road_rules.h"         // DesignRules (clearance, deck thickness, ramp grade)
#include "district.h"           // DistrictParams, buildDistrict (generate recipe)
#include "metro.h"              // MetroParams, buildMetro ("kind":"metro" recipe)
#include <algorithm>
#include <cmath>
#include <set>
#include <limits>
#include <cstdlib>
#include <unordered_map>
#include <cstdio>

namespace engine {

// The curvature cap must clear the WIDEST ribbon in the road, not the default
// width: the metro recipe writes 13 m arterials beside 7 m streets, and a bend
// with radius between the two folds the arterial's band over itself (the
// "side of the road raised up vertically" flap). Half-width + sidewalk + margin
// of the widest edge. (Widths are always resolved now, so this is a plain max.)
static double netMinTurnRadius(const RoadEntity& road) {
    double maxW = road.look.defaultWidth;
    for (const RoadEdge& e : road.graph.edges) maxW = std::max(maxW, static_cast<double>(e.width));
    return maxW * 0.5 + road.look.sidewalk + 0.5;
}


using json = nlohmann::json;

namespace {

bool isZero(const Vec2& v) { return v.x == 0.0 && v.y == 0.0; }

// Valid, non-degenerate edge indices of a control graph.
std::vector<int> validEdgeIndices(const RoadGraph& g) {
    std::vector<int> out;
    const int n = static_cast<int>(g.nodes.size());
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        const RoadEdge& e = g.edges[ei];
        if (e.a >= 0 && e.b >= 0 && e.a < n && e.b < n && e.a != e.b) out.push_back(ei);
    }
    return out;
}

// The road's graph for the mesher: EVERY edge is a Catmull-Rom spline, sampled as a Hermite cubic
// through its endpoints' tangents (Catmull-Rom auto on a degree-2 through-road, a straight chord into
// a junction/dead-end). The sampler is ADAPTIVE — an edge that doesn't actually bend collapses back to
// a single segment, so the grid's straight runs don't densify and clog the junction meshes. The
// original nodes keep their indices (so junction degree is preserved); curve samples append.
// This is the real work navRoadGraph is kept for (spline sampling, not a
// representation change): the entity's CONTROL graph in, the fine SAMPLED graph out.
RoadGraph sampleNetGraph(const RoadEntity& road, double minTurnRadius = 0.0) {
    const RoadGraph& cg = road.graph;
    const int n = static_cast<int>(cg.nodes.size());
    auto P = [&](int i) { return cg.nodes[i].pos; };
    const std::vector<int> ev = validEdgeIndices(cg);
    // S8 walk permission from the BAND MODEL: an edge is walkable iff its
    // resolved spec carries a Sidewalk band on either side. The legacy shim
    // synthesizes sidewalks from look.sidewalk for ordinary streets, so roads
    // without specs keep pedestrians; freeway3/ramp1/dirt presets have no
    // Sidewalk band and correctly shut walkers out at the DATA level.
    auto ewalk = [&](int ei) {
        const RoadSpec sp = roadNetEdgeSpec(road, ei);
        return sp.hasSidewalk(-1) || sp.hasSidewalk(1);
    };
    // Kerbside PARKING band, resolved here so nav/sim never need the spec table.
    // Right-hand side (+1): a two-way street is symmetric, so one number serves
    // both directed links.
    auto epark = [&](int ei, Real& off, Real& w) {
        const RoadSpec sp = roadNetEdgeSpec(road, ei);
        double o = 0, bw = 0;
        if (roadSpecParkingBand(sp, +1, o, bw)) {
            off = static_cast<Real>(o);
            w = static_cast<Real>(bw);
        } else {
            off = 0; w = 0;
        }
    };

    RoadGraph g;
    g.nodes.resize(n);
    for (int i = 0; i < n; ++i) {
        g.nodes[i].pos = cg.nodes[i].pos;
        // Authored ABSOLUTE deck Y (elevated roads): the node rides at that
        // world Y; the weld carries it as UnionSpine.yAbs.
        if (cg.nodes[i].elevAbsolute) { g.nodes[i].elev = cg.nodes[i].elev;
                                        g.nodes[i].elevAbsolute = true; }
        // Semantic hints ride onto the ORIGINAL control nodes (0..n-1);
        // appended curve samples stay Auto for classifyRoadGraph.
        g.nodes[i].kind = cg.nodes[i].kind;
    }

    // Per-node neighbours (for degree + the Catmull-Rom "other" neighbour).
    std::vector<std::vector<int>> nbr(n);
    for (int ei : ev) {
        const RoadEdge& e = cg.edges[ei];
        nbr[e.a].push_back(e.b); nbr[e.b].push_back(e.a);
    }
    auto stored = [&](int i) { return cg.nodes[i].tangent; };
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
        const RoadEdge& e = cg.edges[ei];
        int a = e.a, b = e.b;
        Real w = e.width;
        Vec2 m0 = outTan(a, b), m1 = inTan(b, a);
        double len = (P(b) - P(a)).length();
        int segs = std::max(4, static_cast<int>(std::ceil(len / 5.0)));
        // Sample the spline with its curvature capped so the half-width (plus sidewalk)
        // ribbon can't fold on an over-tight bend. Endpoints — the shared junction nodes —
        // are preserved, so the graph stays stitched.
        std::vector<Vec2> poly = fairHermite(P(a), m0, P(b), m1, segs, minTurnRadius);
        int lay = e.layer;
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

        RoadClass kls = e.klass;
        // Elevated span: interior samples ride the authored deck, interpolated by
        // arc-length between the two authored endpoints. Only when BOTH ends are
        // authored — a span with an at-grade end drapes (a homogeneous chain is
        // what the weld's yAbs path needs; a mixed one falls back to the ground).
        const bool elevSpan = cg.nodes[a].elevAbsolute && cg.nodes[b].elevAbsolute;
        double ea = 0.0, eb = 0.0;
        std::vector<double> arc;
        if (elevSpan) {
            ea = cg.nodes[a].elev; eb = cg.nodes[b].elev;
            arc.assign(poly.size(), 0.0);
            for (std::size_t s = 1; s < poly.size(); ++s)
                arc[s] = arc[s - 1] + (poly[s] - poly[s - 1]).length();
        }
        Real parkOff = 0, parkW = 0;
        epark(ei, parkOff, parkW);
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
            RoadEdge ge{prev, idx, w, kls, lay};
            ge.spec = e.spec;                    // roads-v2: band model rides the graph
            ge.walkable = ewalk(ei);
            ge.parkOffset = parkOff; ge.parkWidth = parkW;
            g.edges.push_back(ge);
            prev = idx;
        }
        RoadEdge geLast{prev, b, w, kls, lay};
        geLast.spec = e.spec;
        geLast.walkable = ewalk(ei);
        geLast.parkOffset = parkOff; geLast.parkWidth = parkW;
        g.edges.push_back(geLast);               // last -> shared node b
    }
    // The sampled graph carries the spec table too, so a consumer holding only
    // the sampled graph can still resolve RoadEdge::spec through it.
    g.specs = cg.specs;
    return g;
}

}  // namespace

// Roads-v2 S3b: strip the BAKED corridor edges (klass Freeway/Ramp) so the
// street mesher, terrain conform, and street nav see only streets — the
// corridor still draws/carves/navigates itself until S4-S6 unify. Without this
// a baked net double-meshes, double-carves, and double-counts nav edges (the
// corridor's carriageway chains are merged into LevelRoadGraph separately).
// Also keeps the curvature cap honest: netMinTurnRadius scans the edge widths,
// and a 27 m freeway width would inflate every street's minimum turn radius.
// Nodes are kept (indices stay stable); orphaned nodes emit no geometry.
// Profile slope limit (rise/run) for chain-height smoothing — the value the
// deleted weld used (WeldSolidParams::maxGrade); mesh and terrain carve both
// grade to it, so they stay in agreement.
static constexpr double kRoadMaxGrade = 0.08;

RoadEntity roadNetStreetsOnly(const RoadEntity& road) {
    bool any = false;
    for (const RoadEdge& e : road.graph.edges)
        if (e.baked) { any = true; break; }
    if (!any) return road;
    RoadEntity out = road;
    out.graph.edges.clear();
    for (const RoadEdge& e : road.graph.edges) {
        if (e.baked) continue;                     // baked: corridor's own
        out.graph.edges.push_back(e);
    }
    return out;
}

namespace {

// Dead ends that stop inside another road become T junctions (or are pulled
// back) on EVERY derived graph — constrainedGraph (nav, lots, census,
// conform) and the mesher's own promotion below — so all of them agree
// where a road ends. (The first cut joined only in constrainedGraph; the
// map then showed the arterial's flat end cap still at the OLD position while
// the census's graph had it pulled back: two graphs, two answers.) Each end
// is reported once per process, not once per derivation.
void joinDanglingEndsLogged(RoadGraph& g, const RoadEntity& road) {
    static std::set<long long> reported;
    for (const DanglingJoin& j : joinDanglingEnds(g, road.look.sidewalk)) {
        const long long key = (static_cast<long long>(std::lround(j.end.x * 10.0)) << 32) ^
                              (static_cast<long long>(std::lround(j.end.y * 10.0)) & 0xffffffffLL);
        if (!reported.insert(key).second) continue;
        if (j.trimmed)
            LOG_INFO << "[roadgraph] pulled a dangling class-" << static_cast<int>(j.endClass)
                     << " end at (" << j.end.x << ", " << j.end.y << ") back " << j.pulledBack
                     << " m: a class-" << static_cast<int>(j.throughClass)
                     << " road skirts past its cap " << j.gap << " m away, not a T";
        else
            LOG_INFO << "[roadgraph] joined a dangling class-" << static_cast<int>(j.endClass)
                     << " end at (" << j.end.x << ", " << j.end.y << ") to the class-"
                     << static_cast<int>(j.throughClass) << " road at (" << j.at.x << ", "
                     << j.at.y << "): " << j.gap << " m bridged, a T junction now";
    }
}

// The graph the mesher AND the terrain-conform both build from: the sampled graph put
// through the local-constraints pass (ADR-0052), so a promoted roundabout is reflected
// identically in the carriageway and in the ground it grades. One source keeps them in sync.
RoadGraph constrainedGraph(const RoadEntity& road, const RoadGroundFn& heightAt) {
    // Roads-v2.1 2e: baked corridor edges stay IN — nav routes the freeway
    // natively from the graph now that the corridor renderer is gone. Only
    // the terrain conform still strips them (roadNetConformRegions):
    // corridorAuthor's engineered flatten owns the corridor's carve.
    double minR = netMinTurnRadius(road);
    RoadRules rules;
    rules.autoRoundabout = road.look.autoRoundabout;   // honour the policy (ADR-0075 P0)
    RoadGraph g = applyConstraints(sampleNetGraph(road, minR), rules);
    joinDanglingEndsLogged(g, road);
    classifyRoadGraph(g, heightAt);   // semantic layer (#17): kinds + access
    return g;
}

}  // namespace

// Public accessor (ADR-0059): hand runtime consumers the same sampled+constrained
// graph the mesher uses, without exposing the file-local builder above.
RoadGraph navRoadGraph(const RoadEntity& road, const RoadGroundFn& heightAt) {
    return constrainedGraph(road, heightAt);
}

// The FULL sampled+constrained graph, baked corridor edges INCLUDED — the
// graph the unified mesher builds from (roads-v2.1 R1) and the surface the
// bake-fidelity gates measure against. (Identical to navRoadGraph since 2e;
// both names kept for their distinct call-site intents.)
RoadGraph roadNetFullGraph(const RoadEntity& road, const RoadGroundFn& heightAt) {
    return constrainedGraph(road, heightAt);
}

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
        s.access = g.edges[e0].access;           // semantic access bits (#17/#21)
        s.closed = closed;
        s.points.push_back(g.nodes[v].pos);
        // 3-D channel (welder-goes-3D): a chain with ANY absolute deck Y
        // (corridor decks / ramps, elevAbsolute) rides authored heights where
        // they exist; non-absolute nodes get NaN here and are FILLED from the
        // drape profile below — so a baked ramp chain that TERMINATES at an
        // at-grade street junction keeps its authored descent instead of
        // draping the whole elevated span onto the ground (the old all-or-
        // nothing rule did exactly that: every landing node is at-grade, so
        // every baked ramp meshed at ground level and the drive probe fell
        // through the sky where the deck should be). Pure streets have no
        // absolute nodes and keep the unchanged drape path.
        auto yOf = [&](int ni) {
            return g.nodes[ni].elevAbsolute
                       ? static_cast<double>(g.nodes[ni].elev)
                       : std::numeric_limits<double>::quiet_NaN();
        };
        std::vector<double> ys{ yOf(v) };
        bool anyAbs = g.nodes[v].elevAbsolute;
        int prev = v, e = e0, startNode = v, lastE = e0;
        while (!used[e]) {
            used[e] = 1;
            lastE = e;
            s.layer = std::max(s.layer, g.edges[e].layer);
            int nx = (g.edges[e].a == prev) ? g.edges[e].b : g.edges[e].a;
            if (closed && nx == startNode) break;
            s.points.push_back(g.nodes[nx].pos);
            ys.push_back(yOf(nx));
            anyAbs = anyAbs || g.nodes[nx].elevAbsolute;
            if (!closed && breaksChain(nx)) break;
            int ne = -1;
            for (int ee : inc[nx]) if (ee != e && !used[ee]) { ne = ee; break; }
            if (ne < 0) break;
            prev = nx; e = ne;
        }
        s.accessBack = g.edges[lastE].access;   // the back end's edge (#21)
        s.authoredDeck = anyAbs;
        if (anyAbs && ys.size() == s.points.size()) s.yAbs = std::move(ys);
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
    o.access = s.access; o.accessBack = s.accessBack;
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

namespace {
// Nearest-asphalt-edge height sampler for the S5 curb/sidewalk band: segments
// (chain centrelines with their reconciled profile heights + pad boundary
// loops) in a coarse grid hash; sample = height at the nearest point on the
// nearest segment. The band hugs the asphalt within ~7 m, so a small search
// window finds its segment; brute force is the (rare) fallback.
struct EdgeHeightField {
    struct Seg { Vec2 a, b; double ya, yb; };
    std::vector<Seg> segs;
    std::unordered_map<long long, std::vector<int>> cells;
    static constexpr double kCell = 12.0;
    static long long key(int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^
               (static_cast<long long>(cz) & 0xffffffffLL);
    }
    void addPolyline(const std::vector<Vec2>& pts, const std::vector<double>& ys) {
        for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
            const int si = static_cast<int>(segs.size());
            segs.push_back({ pts[i], pts[i + 1], ys[i], ys[i + 1] });
            const double x0 = std::min(pts[i].x, pts[i + 1].x) - 1.0;
            const double x1 = std::max(pts[i].x, pts[i + 1].x) + 1.0;
            const double z0 = std::min(pts[i].y, pts[i + 1].y) - 1.0;
            const double z1 = std::max(pts[i].y, pts[i + 1].y) + 1.0;
            for (int cx = (int)std::floor(x0 / kCell); cx <= (int)std::floor(x1 / kCell); ++cx)
                for (int cz = (int)std::floor(z0 / kCell); cz <= (int)std::floor(z1 / kCell); ++cz)
                    cells[key(cx, cz)].push_back(si);
        }
    }
    void addLoop(const std::vector<Vec3>& loop) {
        if (loop.size() < 2) return;
        std::vector<Vec2> pts;
        std::vector<double> ys;
        for (const Vec3& p : loop) { pts.push_back(Vec2(p.x, p.z)); ys.push_back(p.y); }
        pts.push_back(Vec2(loop.front().x, loop.front().z));
        ys.push_back(loop.front().y);
        addPolyline(pts, ys);
    }
    double sample(double x, double z) const {
        // Inverse-distance BLEND over the nearby segments, not nearest-only:
        // at a junction corner the nearest chain flips between arms meeting at
        // different heights, and a nearest-snap put that height CLIFF straight
        // into the band (steep slab quads on hills). Blending keeps the band
        // flush where it hugs one edge (that weight dominates) and smooth
        // where two arms compete.
        const int cx = (int)std::floor(x / kCell), cz = (int)std::floor(z / kCell);
        std::vector<int> cand;                   // small: the 3x3 window's lists
        for (int r = 1; r <= 3 && cand.empty(); ++r)
            for (int dx = -r; dx <= r; ++dx)
                for (int dz = -r; dz <= r; ++dz) {
                    auto it = cells.find(key(cx + dx, cz + dz));
                    if (it != cells.end())
                        cand.insert(cand.end(), it->second.begin(), it->second.end());
                }
        if (cand.empty()) {                      // fallback: brute force
            cand.resize(segs.size());
            for (std::size_t i = 0; i < segs.size(); ++i) cand[i] = (int)i;
        }
        std::sort(cand.begin(), cand.end());
        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
        double wsum = 0.0, hsum = 0.0;
        for (int si : cand) {
            const Seg& sg = segs[si];
            const Vec2 ab = sg.b - sg.a;
            const double l2 = ab.lengthSquared();
            double t = l2 > 1e-12 ? dot(Vec2(x, z) - sg.a, ab) / l2 : 0.0;
            t = t < 0 ? 0 : (t > 1 ? 1 : t);
            const double d = (sg.a + ab * t - Vec2(x, z)).length();
            const double w = 1.0 / (d * d + 0.5);
            wsum += w;
            hsum += w * (sg.ya + (sg.yb - sg.ya) * t);
        }
        return wsum > 0 ? hsum / wsum : 0.0;
    }
};
}  // namespace

RenderMesh buildRoadNetLattice(const RoadGraph& gIn,
                               const std::function<Real(Real, Real)>& heightAt,
                               std::vector<std::size_t>* chainTriEndsOut,
                               double sidewalkWidth, double curbHeight,
                               bool crosswalks, CurbBandAudit* auditOut,
                               double cornerRadius,
                               RoadDeckField* deckOut) {
    // Semantic self-heal (#17): tests and tools hand this hand-built graphs
    // that never went through a producer — classify so kind-gated styling
    // behaves identically for them. Classified inputs pass through untouched
    // (classifyRoadGraph is idempotent; hints are preserved).
    RoadGraph healed;
    const RoadGraph& g = roadGraphUnclassified(gIn)
                             ? (healed = gIn,
                                classifyRoadGraph(
                                    healed,
                                    heightAt ? GroundFn([&heightAt](double x, double y) {
                                        return static_cast<double>(heightAt(x, y));
                                    })
                                             : GroundFn()),
                                healed)
                             : gIn;
    const int N = static_cast<int>(g.nodes.size());
    // Stage 1 of the junction re-architecture (docs/junction-weld-decision.md):
    // the junction owns the FULL cross-section. Trim each body by its full
    // half-width (carriageway + sidewalk), so an arm's raised sidewalk pulls
    // back out of the junction disc instead of sweeping into the pad and
    // double-covering it — the measured 85% of the 16% overlap.
    const double kSidewalkW = sidewalkWidth;
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
    // Mixed chains carry NaN at their at-grade nodes (a baked ramp's street
    // landing). weldChainProfiles READS yAbs as its authored input, so the
    // NaNs must die HERE, before any consumer: fill them with the draped
    // street surface at that point. NaN must never reach a profile or a mesh.
    for (UnionSpine& c : chains)
        for (std::size_t i = 0; i < c.yAbs.size(); ++i)
            if (std::isnan(c.yAbs[i]))
                c.yAbs[i] = groundFn(c.points[i].x, c.points[i].y) + 0.15;
    {
        std::vector<std::vector<double>> profiles = weldChainProfiles(
            chains, groundFn, 0.0, kRoadMaxGrade, 3.0 + 4.0);
        for (std::size_t si = 0; si < chains.size(); ++si) {
            const bool sized = profiles[si].size() == chains[si].points.size();
            if (chains[si].yAbs.empty()) {
                if (sized) chains[si].yAbs = profiles[si];
                continue;
            }
            // Mixed chain: authored deck heights win; the at-grade holes
            // (NaN — e.g. a ramp's street landing node) ride the same drape
            // profile a street would, so the descent meets the street exactly
            // where the street's own surface sits. If the profile pass
            // resampled (size mismatch), bridge NaN runs from the finite
            // neighbours instead — NaN must NEVER reach the mesh.
            std::vector<double>& ys = chains[si].yAbs;
            for (std::size_t i = 0; i < ys.size(); ++i) {
                if (!std::isnan(ys[i])) continue;
                if (sized) { ys[i] = profiles[si][i]; continue; }
                std::size_t lo = i; while (lo > 0 && std::isnan(ys[lo - 1])) --lo;
                std::size_t hi = i; while (hi + 1 < ys.size() && std::isnan(ys[hi + 1])) ++hi;
                const double before = lo > 0 ? ys[lo - 1]
                                             : (hi + 1 < ys.size() ? ys[hi + 1] : 0.0);
                const double after = hi + 1 < ys.size() ? ys[hi + 1] : before;
                for (std::size_t k = lo; k <= hi; ++k) {
                    const double t = (hi >= lo)
                        ? (double(k - lo) + 1.0) / (double(hi - lo) + 2.0) : 0.5;
                    ys[k] = before + (after - before) * t;
                }
                i = hi;
            }
        }
    }
    // THE DECK, exported (RoadDeckField): these chains' yAbs ARE the heights
    // every body ring below is swept at, so a consumer sampling them stands on
    // the drawn asphalt by construction.
    if (deckOut) deckOut->spines = chains;
    // S4: junction pairs closer than their combined radii merge into ONE
    // COMPOUND pad. trimSpine clamps its trims at 45% of the chain, so S1 kept
    // a squeezed sub-metre body with BOTH pads stacked over it — the measured
    // pad-pad overlap (52 cells on the real graph). Union-find the too-close
    // pairs; the link body between merged nodes is skipped (the compound pad
    // owns that asphalt), and the cluster's outer arms build one pad.
    std::vector<int> uf(N);
    for (int v = 0; v < N; ++v) uf[v] = v;
    std::function<int(int)> find = [&](int v) {
        while (uf[v] != v) v = uf[v] = uf[uf[v]];
        return v;
    };
    for (const UnionSpine& s : chains) {
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        if (a < 0 || b < 0 || a == b || deg[a] < 3 || deg[b] < 3) continue;
        double len = 0;
        for (std::size_t i = 1; i < s.points.size(); ++i)
            len += (s.points[i] - s.points[i - 1]).length();
        if (len <= rad[a] + rad[b] + 0.25) uf[find(a)] = find(b);
    }
    // S5: the curb/sidewalk band is swept along the union outline of these
    // footprints (body ribbons + pad boundaries), with heights sampled from
    // this field — asphalt and band meet flush by construction.
    EdgeHeightField bandHeights;
    std::vector<Poly2> bandRibbons;
    std::vector<std::pair<Vec2, Vec2>> bandGaps;   // non-street mouths (S6)
    std::vector<int> streetArms(N, 0);             // per cluster root
    // Which chains are REALLY elevated (deck rides above the ground — a
    // viaduct or layered bridge)? weldChainProfiles assigns yAbs to EVERY
    // chain (the one-profile-source ride), so yAbs presence alone says
    // nothing — compare against the ground. At-grade chains double as the
    // pier KEEP-OUT: no column may stand in a road below (S6).
    std::vector<char> chainElevated(chains.size(), 0);
    struct KeepOutSeg { Vec2 a, b; double hw; };
    std::vector<KeepOutSeg> pierKeepOut;
    int nElevated = 0, nElevatedStreets = 0;
    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const UnionSpine& cs = chains[ci];
        bool up = false;
        double lift = 0.0;
        Vec2 liftAt(0, 0);
        if (cs.yAbs.size() == cs.points.size())
            for (std::size_t i = 0; i < cs.points.size(); ++i) {
                const double d = cs.yAbs[i] - ground(cs.points[i].x, cs.points[i].y);
                if (d > lift) { lift = d; liftAt = cs.points[i]; }
                if (d > 1.5) up = true;
            }
        // A BRIDGE is structural, not a height difference: authored deck
        // heights (corridor decks/ramps) or a layer above grade. A plain
        // street whose grade-limited profile rides a fill over a dip is still
        // an at-grade street — the conform pass carves the ground up to it.
        // The old test (any point > 1.5 m above the RAW terrain) turned two
        // metro locals crossing valleys into "bridges": no sidewalk band, so
        // the arterial's band swept straight across their mouths at all four
        // ends (the city map's deepest four sidewalk-on-asphalt places).
        up = up && (cs.authoredDeck || cs.layer > 0);
        chainElevated[ci] = up;
        if (up) {
            ++nElevated;
            const bool street = cs.klass != RoadClass::Freeway && cs.klass != RoadClass::Ramp;
            if (street) ++nElevatedStreets;
            if (std::getenv("RT_LATTICE_DEBUG"))
                LOG_INFO << "[lattice] chain " << ci << " class " << static_cast<int>(cs.klass)
                         << " flagged ELEVATED: rides " << lift << " m above ground at ("
                         << liftAt.x << ", " << liftAt.y << "), " << cs.points.size()
                         << " pts from (" << cs.points.front().x << ", " << cs.points.front().y
                         << ") to (" << cs.points.back().x << ", " << cs.points.back().y << ")"
                         << (street ? " — a STREET: bridge treatment, no sidewalk band" : "");
        }
        if (!up)
            for (std::size_t i = 0; i + 1 < cs.points.size(); ++i)
                pierKeepOut.push_back({ cs.points[i], cs.points[i + 1], cs.halfWidth });
    }
    if (nElevatedStreets > 0)
        LOG_INFO << "[lattice] " << nElevated << " chains elevated, " << nElevatedStreets
                 << " of them STREETS (bridge treatment: no sidewalk band; the junction "
                    "pad's mouth becomes a band boundary — RT_LATTICE_DEBUG=1 lists them)";
    auto pierBlocked = [&](const Vec2& p) {
        for (const KeepOutSeg& k : pierKeepOut) {
            const Vec2 ab = k.b - k.a;
            const double l2 = ab.lengthSquared();
            double t2 = l2 > 1e-12 ? dot(p - k.a, ab) / l2 : 0.0;
            t2 = t2 < 0 ? 0 : (t2 > 1 ? 1 : t2);
            if ((k.a + ab * t2 - p).length() < k.hw + 2.5) return true;
        }
        return false;
    };
    // ACUTE-PAIR TRIM (2c nose trim, generalized in R2): two arms leaving a
    // junction NEARLY PARALLEL — a ramp along its freeway at a gore, or two
    // streets forking at a sharp angle (drive feedback A1: "one of the roads
    // remains a flat ribbon... It floats above it") — overlap ribbons far
    // beyond the standard junction radius. Each such chain end extends its
    // trim to where its centreline actually CLEARS the sibling's ribbon
    // (half + half + margin); the junction pad then fills the wedge between
    // the real mouths. Perpendicular and opposite arms are untouched (their
    // ribbons separate within the standard radius already).
    struct ArmRef { std::size_t chain; Vec2 dir; };
    std::vector<std::vector<ArmRef>> armsAt(N);
    for (std::size_t ci2 = 0; ci2 < chains.size(); ++ci2) {
        const UnionSpine& cs2 = chains[ci2];
        if (cs2.points.size() < 2) continue;
        const int fa = nodeAt(cs2.points.front()), fb = nodeAt(cs2.points.back());
        if (fa >= 0) {
            Vec2 d = cs2.points[1] - cs2.points[0];
            const double l = d.length();
            if (l > 1e-9) armsAt[fa].push_back({ ci2, d * (1.0 / l) });
        }
        if (fb >= 0) {
            Vec2 d = cs2.points[cs2.points.size() - 2] - cs2.points.back();
            const double l = d.length();
            if (l > 1e-9) armsAt[fb].push_back({ ci2, d * (1.0 / l) });
        }
    }
    // Near-node segments per chain (for the clearance walk).
    auto chainSegsNear = [&](std::size_t ci2, int node, double reach) {
        std::vector<std::pair<Vec2, Vec2>> segs;
        const UnionSpine& cs2 = chains[ci2];
        const bool fromA = nodeAt(cs2.points.front()) == node;
        double acc = 0;
        for (std::size_t i = 1; i < cs2.points.size() && acc < reach; ++i) {
            const std::size_t k = fromA ? i : cs2.points.size() - 1 - i;
            const std::size_t kp = fromA ? k - 1 : k + 1;
            segs.push_back({ cs2.points[kp], cs2.points[k] });
            acc += (cs2.points[k] - cs2.points[kp]).length();
        }
        return segs;
    };
    struct Wedge { int node; std::size_t subCi, domCi; double subTrim; double side = 0; };
    std::vector<Wedge> wedges;
    auto pairTrim = [&](const UnionSpine& mine, std::size_t myCi, int node,
                        bool fromFront) {
        double trim = 0;
        std::size_t domBest = SIZE_MAX;
        Vec2 myDir(0, 0);
        for (const ArmRef& ar : armsAt[node])
            if (ar.chain == myCi) { myDir = ar.dir; break; }
        if (myDir.x == 0 && myDir.y == 0) return trim;
        for (const ArmRef& other : armsAt[node]) {
            if (other.chain == myCi) continue;
            if (dot(myDir, other.dir) < 0.82) continue;   // ~35 deg: not acute
            // DOMINANCE: only the narrower-or-equal arm gives way — a ramp
            // trims to clear its freeway, never the freeway to clear the
            // ramp (the symmetric rule cut the mainline's body and parapets
            // ~60 m back from every gore). Equal street forks both extend.
            if (mine.halfWidth > chains[other.chain].halfWidth + 0.1) continue;
            const double clearL = mine.halfWidth +
                                  chains[other.chain].halfWidth + 0.4;
            const auto segs = chainSegsNear(other.chain, node, 220.0);
            double acc = 0;
            const std::size_t n2 = mine.points.size();
            for (std::size_t i = 0; i < n2; ++i) {
                const std::size_t k = fromFront ? i : n2 - 1 - i;
                if (i > 0) {
                    const std::size_t kp = fromFront ? k - 1 : k + 1;
                    acc += (mine.points[k] - mine.points[kp]).length();
                }
                double dmin = 1e30;
                for (const auto& seg : segs) {
                    const Vec2 ab = seg.second - seg.first;
                    const double l2 = ab.lengthSquared();
                    double t2 = l2 > 1e-12
                        ? dot(mine.points[k] - seg.first, ab) / l2 : 0.0;
                    t2 = t2 < 0 ? 0 : (t2 > 1 ? 1 : t2);
                    dmin = std::min(dmin,
                                    (seg.first + ab * t2 - mine.points[k]).length());
                }
                if (dmin > clearL) {
                    if (acc > trim) { trim = acc; domBest = other.chain; }
                    break;
                }
            }
        }
        if (domBest != SIZE_MAX && trim > 0)
            wedges.push_back({ node, myCi, domBest, trim });
        return trim;
    };
    // TRIM PRE-PASS (R2): all trims — and therefore ALL wedge records — are
    // computed before any chain sweeps, because a DOMINANT chain's parapet
    // needs gap windows from wedges its SUBORDINATE (which may come later in
    // the loop) records.
    std::vector<double> trimA(chains.size(), 0.0), trimB(chains.size(), 0.0);
    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const UnionSpine& s = chains[ci];
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        trimA[ci] = (a >= 0 && deg[a] >= 3) ? rad[a] : 0.0;
        trimB[ci] = (b >= 0 && deg[b] >= 3) ? rad[b] : 0.0;
        if (a >= 0 && deg[a] >= 3)
            trimA[ci] = std::max(trimA[ci], pairTrim(s, ci, a, true));
        if (b >= 0 && deg[b] >= 3)
            trimB[ci] = std::max(trimB[ci], pairTrim(s, ci, b, false));
    }
    // WEDGE RAILS (R2): for each acute pair, the pad's corner between the
    // dominant and subordinate arms follows the DOMINANT chain's edge curve
    // (sampled from its mouth out to the subordinate's mouth arc, offset by
    // its half-width on the subordinate's side) instead of a straight chord
    // the curved arm bows off of. Keyed to the pad root; matched by arm ids.
    std::map<int, std::vector<JunctionRail>> nodeRails;
    for (Wedge& w : wedges) {
        const UnionSpine& dom = chains[w.domCi];
        const UnionSpine& sub = chains[w.subCi];
        if (dom.points.size() < 2 || sub.points.size() < 2) continue;
        if (dom.yAbs.size() != dom.points.size()) continue;
        const bool domFromA = nodeAt(dom.points.front()) == w.node;
        if (!domFromA && nodeAt(dom.points.back()) != w.node) continue;
        auto dirAt = [&](const UnionSpine& c, bool fromA) {
            const Vec2 d = fromA ? c.points[1] - c.points[0]
                                 : c.points[c.points.size() - 2] - c.points.back();
            const double l = d.length();
            return l > 1e-9 ? d * (1.0 / l) : Vec2(1, 0);
        };
        const bool subFromA = nodeAt(sub.points.front()) == w.node;
        const Vec2 dDir = dirAt(dom, domFromA);
        const Vec2 sDir = dirAt(sub, subFromA);
        const double side =
            (dDir.x * sDir.y - dDir.y * sDir.x) >= 0 ? 1.0 : -1.0;
        w.side = side;
        JunctionRail rail;
        rail.fromId = static_cast<int>(w.domCi);
        rail.toId = static_cast<int>(w.subCi);
        const double s0 = (w.node < N ? rad[w.node] : 8.0) + 1.5;
        const double s1 = w.subTrim - 1.5;
        double acc = 0;
        for (std::size_t i = 1; i < dom.points.size() && acc < s1; ++i) {
            const std::size_t k = domFromA ? i : dom.points.size() - 1 - i;
            const std::size_t kp = domFromA ? k - 1 : k + 1;
            const Vec2 seg = dom.points[k] - dom.points[kp];
            const double segL = seg.length();
            if (segL < 1e-9) continue;
            const double a0 = acc;
            acc += segL;
            for (double st = std::max(s0, std::ceil(a0 / 4.0) * 4.0); st < acc && st < s1;
                 st += 4.0) {
                const double t = (st - a0) / segL;
                const Vec2 c2 = dom.points[kp] + seg * t;
                const Vec2 dirN = seg * (1.0 / segL);
                const Vec2 leftN(-dirN.y, dirN.x);
                const double y =
                    dom.yAbs[kp] + (dom.yAbs[k] - dom.yAbs[kp]) * t;
                // seg is oriented AWAY from the node in BOTH walk
                // orientations, so leftN is already node-relative — the side
                // sign (from the away-from-node dir cross) applies directly.
                const Vec2 p = c2 + leftN * (side * dom.halfWidth);
                rail.pts.push_back(Vec3(p.x, y, p.y));
            }
        }
        if (rail.pts.size() >= 2)
            nodeRails[find(w.node)].push_back(std::move(rail));
    }
    for (std::size_t ci = 0; ci < chains.size(); ++ci) {
        const UnionSpine& s = chains[ci];
        if (s.points.size() < 2) continue;
        const int a = nodeAt(s.points.front()), b = nodeAt(s.points.back());
        if (a >= 0 && b >= 0 && a != b && find(a) == find(b))
            continue;                            // compound link: the pad owns it
        const double rA = trimA[ci], rB = trimB[ci];
        UnionSpine t = trimSpine(s, rA, rB);
        if (t.points.size() < 2) continue;

        // LANES come from the road's CLASS — the one lanesForClass() source the
        // nav lanes already use, so the painted dividers and the lanes cars
        // actually drive in agree by construction. Dividing the width by a
        // nominal 3.6 m (what this did) is a guess that disagreed with both:
        // it made every 12 m street "2 lanes per side" and every road, however
        // wide, was painted as a plain two-laner because the strip that carries
        // a divider was never emitted at all.
        const int laneTotal = std::max(1, lanesForClass(s.klass, /*perDirection=*/false));
        const int lanesPerSide = std::max(1, laneTotal / 2);
        const bool elevated = chainElevated[ci] != 0;
        const bool isFwy = s.klass == RoadClass::Freeway;
        const bool isRamp = s.klass == RoadClass::Ramp;
        std::vector<Vec3> ring0, ringN;
        if (isFwy || isRamp) {
            // S6 ONE MESHER: a Freeway/Ramp chain gets the full structure the
            // corridor kit builds — deck, viaduct underside, edge parapets,
            // (freeway) median, piers under elevated spans. Parapets stop at
            // the junction trim, so a gore's merge stays open by construction.
            const int lanes = isRamp ? 1
                : std::max(1, static_cast<int>(std::lround((s.halfWidth - 1.0) / 3.6)));
            MeshBuilder::append(out, sweepRoadLattice(t, freewayDeckProfile(lanes),
                                                      ground, 2.0, nullptr, &ring0, &ringN));
            // STRUCTURE exists only where the deck FLIES (the old corridor's
            // 35 cm rule, as gap data): at-grade stretches get no underside
            // (buried soffit + an open fascia ring at the street were the
            // zoo's landing rims) and no PARAPET either — an at-grade ramp
            // edge wants a guardrail (R6 side grammar), not 0.85 m of
            // concrete running into the junction.
            std::vector<GapWindow> ground0;
            {
                double acc2 = 0, runStart = -1;
                const bool hasY2 = t.yAbs.size() == t.points.size();
                for (std::size_t i = 0; i < t.points.size(); ++i) {
                    if (i > 0) acc2 += (t.points[i] - t.points[i - 1]).length();
                    const double clr2 = hasY2
                        ? t.yAbs[i] - ground(t.points[i].x, t.points[i].y)
                        : 0.0;
                    if (clr2 < 0.35) {
                        if (runStart < 0) runStart = std::max(0.0, acc2 - 1.0);
                    } else if (runStart >= 0) {
                        ground0.push_back({ runStart, acc2 });
                        runStart = -1;
                    }
                }
                if (runStart >= 0) ground0.push_back({ runStart, acc2 + 1.0 });
                MeshBuilder::append(out, sweepRoadLattice(
                    t, freewayUndersideProfile(0.5), ground, 2.0,
                    ground0.empty() ? nullptr : &ground0));
            }
            // Parapet WEDGE GAPS (R2): where this chain is the DOMINANT of
            // an acute pair, its wedge-side parapet opens over the whole
            // merge window (gore trim -> subordinate mouth), not just the
            // junction trim — or the wall stands across the merge (the
            // probe's blocked=1 at three gores).
            double Lt = 0;
            for (std::size_t i = 1; i < t.points.size(); ++i)
                Lt += (t.points[i] - t.points[i - 1]).length();
            std::vector<GapWindow> gapsL, gapsR;
            for (const Wedge& w : wedges) {
                if (w.domCi != ci || w.side == 0) continue;
                const bool atA = a >= 0 && nodeAt(s.points.front()) == w.node;
                const bool atB = b >= 0 && nodeAt(s.points.back()) == w.node;
                if (!atA && !atB) continue;
                const double span = w.subTrim - (atA ? rA : rB);
                if (span <= 1.0) continue;
                const GapWindow g = atA ? GapWindow{ 0.0, span }
                                        : GapWindow{ Lt - span, Lt };
                // side is in away-from-node coords; sweep runs a->b, so the
                // b-end flips.
                const double sweepSide = atA ? w.side : -w.side;
                (sweepSide > 0 ? gapsL : gapsR).push_back(g);
            }
            for (const GapWindow& g : ground0) {   // no parapets at grade
                gapsL.push_back(g);
                gapsR.push_back(g);
            }
            MeshBuilder::append(out, sweepRoadLattice(t, parapetProfile(+1), ground, 2.0,
                                                      gapsL.empty() ? nullptr : &gapsL));
            MeshBuilder::append(out, sweepRoadLattice(t, parapetProfile(-1), ground, 2.0,
                                                      gapsR.empty() ? nullptr : &gapsR));
            // R6d side grammar: box girders hang under both deck edges (the
            // flat soffit alone read as paper), gapped where the deck sits
            // at grade; and a railing POST tops each parapet run every
            // ~3.2 m — skipping exactly the windows the walls skip.
            MeshBuilder::append(out, sweepRoadLattice(t, girderProfile(+1), ground, 2.0,
                                                      ground0.empty() ? nullptr : &ground0));
            MeshBuilder::append(out, sweepRoadLattice(t, girderProfile(-1), ground, 2.0,
                                                      ground0.empty() ? nullptr : &ground0));
            {
                const Vec3 postCol(0.55, 0.56, 0.58);
                auto inGaps = [](const std::vector<GapWindow>& gs, double sArc) {
                    for (const GapWindow& g : gs)
                        if (sArc >= g.s0 - 0.4 && sArc <= g.s1 + 0.4) return true;
                    return false;
                };
                auto post = [&](const Vec3& base, const Vec2& d) {
                    const Vec2 l(-d.y, d.x);
                    const double hx = 0.06, hz = 0.06, h = 0.5;
                    const Vec3 dx(l.x * hx, 0, l.y * hx);
                    const Vec3 dz(d.x * hz, 0, d.y * hz);
                    const Vec3 top = base + Vec3(0, h, 0);
                    auto face = [&](const Vec3& A, const Vec3& B, const Vec3& C,
                                    const Vec3& D, const Vec3& n) {
                        MeshBuilder::emitQuad(out, A, B, C, D, n, postCol);
                    };
                    face(base - dx - dz, base + dx - dz, top + dx - dz,
                         top - dx - dz, Vec3(-d.x, 0, -d.y));
                    face(base + dx + dz, base - dx + dz, top - dx + dz,
                         top + dx + dz, Vec3(d.x, 0, d.y));
                    face(base - dx + dz, base - dx - dz, top - dx - dz,
                         top - dx + dz, Vec3(-l.x, 0, -l.y));
                    face(base + dx - dz, base + dx + dz, top + dx + dz,
                         top + dx - dz, Vec3(l.x, 0, l.y));
                    face(top - dx - dz, top + dx - dz, top + dx + dz,
                         top - dx + dz, Vec3(0, 1, 0));
                };
                const bool hasY = t.yAbs.size() == t.points.size();
                const bool hasW = t.hw.size() == t.points.size();
                double acc = 0, nextPost = 1.6;
                for (std::size_t i = 1; i < t.points.size() && hasY; ++i) {
                    const Vec2 A = t.points[i - 1], B = t.points[i];
                    const double seg = (B - A).length();
                    while (seg > 1e-6 && nextPost <= acc + seg) {
                        const double f = (nextPost - acc) / seg;
                        const Vec2 P = A + (B - A) * f;
                        const Vec2 d = normalize(B - A);
                        const Vec2 l(-d.y, d.x);
                        const double y =
                            t.yAbs[i - 1] + (t.yAbs[i] - t.yAbs[i - 1]) * f;
                        const double hwHere =
                            hasW ? t.hw[i - 1] + (t.hw[i] - t.hw[i - 1]) * f
                                 : t.halfWidth;
                        for (int side = -1; side <= 1; side += 2) {
                            if (inGaps(side > 0 ? gapsL : gapsR, nextPost))
                                continue;
                            const Vec2 c2 = P + l * (side * (hwHere - 0.14));
                            post(Vec3(c2.x, y + 0.85, c2.y), d);
                        }
                        nextPost += 3.2;
                    }
                    acc += seg;
                }
            }
            if (isFwy)
                MeshBuilder::append(out, sweepRoadLattice(t, medianProfile(), ground, 2.0));
            MeshBuilder::append(out, latticeChainPiers(t, groundFn, 0.5, nullptr, pierBlocked));
            // No sidewalk band along a freeway or ramp — and where this chain
            // enters a STREET junction, the band must GAP across its mouth
            // (else a curb + slab curbs the ramp entrance shut).
            if (ring0.size() >= 2)
                bandGaps.push_back({ Vec2(ring0.front().x, ring0.front().z),
                                     Vec2(ring0.back().x, ring0.back().z) });
            if (ringN.size() >= 2)
                bandGaps.push_back({ Vec2(ringN.front().x, ringN.front().z),
                                     Vec2(ringN.back().x, ringN.back().z) });
        } else {
            // S5 OWNERSHIP: street bodies sweep the CARRIAGEWAY only. The curb
            // + sidewalk band owns everything outside the asphalt — a body can
            // no longer sweep a raised sidewalk into a pad or a neighbour.
            const std::size_t v0 = out.vertices.size();
            MeshBuilder::append(out,
                                sweepRoadLattice(t, carriagewayProfile(lanesPerSide, laneTotal,
                                                                       /*oneWay=*/false),
                                                 ground, 2.0, nullptr, &ring0, &ringN));
            // LANDING/gore MOUTH GAP (#20, review S4b): where a STREET body
            // meets a ramp landing, gap the sidewalk band across its mouth —
            // exactly as the freeway/ramp branch does. Otherwise the street's
            // closed-capsule ribbon caps with a CURB WALL straight across the
            // carriageway at the landing, sealing the ramp entrance ("holes
            // where it thinks it's an intersection"). The zebra is already
            // suppressed there (S3); this opens the curb too.
            auto landingEnd = [&](int nd) {
                return nd >= 0 &&
                       (g.nodes[nd].kind == JunctionKind::Landing ||
                        isGore(g.nodes[nd].kind));
            };
            if (landingEnd(a) && ring0.size() >= 2)
                bandGaps.push_back({ Vec2(ring0.front().x, ring0.front().z),
                                     Vec2(ring0.back().x, ring0.back().z) });
            if (landingEnd(b) && ringN.size() >= 2)
                bandGaps.push_back({ Vec2(ringN.front().x, ringN.front().z),
                                     Vec2(ringN.back().x, ringN.back().z) });
            // Crosswalk band UV (ADR-0062, ported from the weld): the shader
            // stripes a zebra where mv lands in the set-back window past a
            // junction mouth. The sweep bakes v = chain arc length; remap it
            // to metres-past-the-NEAREST-JUNCTION-mouth (dead ends and chain
            // interiors stay out of the window), or shift everything clear of
            // the window when crosswalks are off — the paint stays gated.
            {
                double Lc = 0;
                for (std::size_t i = 1; i < t.points.size(); ++i)
                    Lc += (t.points[i] - t.points[i - 1]).length();
                // Semantic zebra gate (#21): a crossing window opens at a
                // chain end ONLY where that end is a street INTERSECTION and
                // the edge is crossable — never at a ramp landing or a gore
                // (both are deg >= 3, so the old trim-radius test lit a zebra
                // there: "the freeways have crosswalks?"). Per-end, so a
                // street that is an Intersection at A and a Landing at B gets
                // its zebra only at A.
                const bool jA = rA > 0.0 &&
                                (s.access & road_access::kCrossable) && a >= 0 &&
                                g.nodes[a].kind == JunctionKind::Intersection;
                const bool jB = rB > 0.0 &&
                                (s.accessBack & road_access::kCrossable) && b >= 0 &&
                                g.nodes[b].kind == JunctionKind::Intersection;
                for (std::size_t vi = v0; vi < out.vertices.size(); ++vi) {
                    const double sAt = out.vertices[vi].v;
                    double d = 1e4;
                    if (jA) d = std::min(d, sAt);
                    if (jB) d = std::min(d, Lc - sAt);
                    out.vertices[vi].v = static_cast<float>(crosswalks ? d : d + 64.0);
                }
            }
            if (elevated) {
                // A layered street bridge: give the deck an underside and legs.
                // (Its sidewalk is omitted — bridge sides are barrier/grammar
                // territory, not a kerbed slab hanging over the void.)
                MeshBuilder::append(out, sweepRoadLattice(t, freewayUndersideProfile(0.4), ground, 2.0));
                MeshBuilder::append(out, latticeChainPiers(t, groundFn, 0.4, nullptr, pierBlocked));
            } else {   // footprint + edge heights for the band union
                std::vector<double> ys(t.points.size(), 0.0);
                if (t.yAbs.size() == t.points.size()) ys = t.yAbs;
                else for (std::size_t i = 0; i < t.points.size(); ++i)
                    ys[i] = ground(t.points[i].x, t.points[i].y);
                // APPROACH BAND CUT (#20): a street that climbs to a ramp
                // LANDING or gore must not carry its sidewalk up the grade
                // ("the sidewalk floats as it elevates, not attached to the
                // road"). Trim the RISING portion off each Landing/gore end,
                // keeping only the flat interior (within 0.5 m of the chain's
                // low point). A plain flat street trims nothing.
                std::vector<Vec2> bpts = t.points;
                std::vector<double> bys = ys;
                {
                    int lo = 0, hi = static_cast<int>(bpts.size()) - 1;
                    const double base =
                        *std::min_element(bys.begin(), bys.end());
                    auto isLand = [&](int nd) {
                        return nd >= 0 &&
                               (g.nodes[nd].kind == JunctionKind::Landing ||
                                isGore(g.nodes[nd].kind));
                    };
                    if (isLand(a))
                        while (lo < hi && bys[lo] > base + 0.5) ++lo;
                    if (isLand(b))
                        while (hi > lo && bys[hi] > base + 0.5) --hi;
                    if (lo > 0 || hi < static_cast<int>(bpts.size()) - 1) {
                        std::vector<Vec2> kp(bpts.begin() + lo,
                                             bpts.begin() + hi + 1);
                        std::vector<double> ky(bys.begin() + lo,
                                               bys.begin() + hi + 1);
                        bpts.swap(kp);
                        bys.swap(ky);
                    }
                }
                if (bpts.size() >= 2) {
                    bandHeights.addPolyline(bpts, bys);
                    Poly2 rib = ribbonOutline(bpts, t.halfWidth + 0.02, 2.0);
                    if (rib.size() >= 3) {
                        if (signedArea(rib) < 0)
                            std::reverse(rib.begin(), rib.end());
                        bandRibbons.push_back(std::move(rib));
                    }
                }
            }
        }
        // The mouth is the FULL profile ring (sidewalk|curb|lanes|curb|sidewalk),
        // reversed to run left -> right looking OUTWARD. Slicing only the
        // carriageway here was the boundary bug: each arm swept its raised
        // sidewalk into a pad that covered carriageway only, so sidewalks
        // double-covered at every corner. The pad now spans verge to verge and
        // its corner points sit at the sidewalk outer edge, where the kerb
        // returns will fillet (stage 2).
        // The body ring IS the carriageway (S5 ownership), so the mouth is the
        // whole ring. The contract is left->right looking OUTWARD along the
        // arm — and "outward" FLIPS between the chain's two ends: at ring0
        // outward is +sweep (the ring needs reversing), at ringN outward is
        // -sweep (it does not). S1 reversed BOTH, so every chain-END arm
        // crossed its mouth backwards and the boundary loop bowtied there
        // (found by the compound-pad area check: 149 vs 213 filled).
        auto mouth = [&](const std::vector<Vec3>& ring, bool flip) {
            return flip ? std::vector<Vec3>(ring.rbegin(), ring.rend()) : ring;
        };
        if (chainTriEndsOut) chainTriEndsOut->push_back(out.indices.size());
        const std::size_t tn = t.points.size();
        // Arms gather on the cluster ROOT: a compound pad (merged too-close
        // junctions) collects the outer arms of ALL its member nodes.
        if (a >= 0 && deg[a] >= 3 && ring0.size() >= 2) {
            arms[find(a)].push_back({ normalize(t.points[1] - t.points[0]), mouth(ring0, true), static_cast<int>(ci) });
            if (!isFwy && !isRamp && !elevated) ++streetArms[find(a)];
        }
        if (b >= 0 && deg[b] >= 3 && ringN.size() >= 2) {
            arms[find(b)].push_back({ normalize(t.points[tn - 2] - t.points[tn - 1]), mouth(ringN, false), static_cast<int>(ci) });
            if (!isFwy && !isRamp && !elevated) ++streetArms[find(b)];
        }
    }

    int nCoons = 0, nT = 0, nFan = 0, nStub = 0, nDeg2 = 0, nMismatch = 0,
        nCompound = 0;
    std::vector<int> clusterSize(N, 0);
    for (int v = 0; v < N; ++v)
        if (deg[v] >= 3) ++clusterSize[find(v)];
    // Cluster KIND (#21): the most-specific member kind over each union-find
    // cluster of too-close junctions. Landings/gores styling-differ from
    // street Intersections — the pad still fills the disc, but only an
    // Intersection cluster joins the sidewalk band and hosts zebras.
    auto kindPrio = [](JunctionKind k) {
        switch (k) {
            case JunctionKind::Landing: return 6;
            case JunctionKind::Diverge: return 5;
            case JunctionKind::Merge: return 4;
            case JunctionKind::Intersection: return 3;
            case JunctionKind::DeadEnd: return 2;
            case JunctionKind::None: return 1;
            default: return 0;
        }
    };
    std::vector<JunctionKind> clusterKind(N, JunctionKind::None);
    for (int v = 0; v < N; ++v) {
        if (deg[v] < 3) continue;
        const int r = find(v);
        if (kindPrio(g.nodes[v].kind) > kindPrio(clusterKind[r]))
            clusterKind[r] = g.nodes[v].kind;
    }
    std::vector<int> degHist(12, 0);
    for (int v = 0; v < N; ++v) {
        if (deg[v] < 12) ++degHist[deg[v]];
        if (deg[v] < 3) { if (deg[v] == 2) ++nDeg2; continue; }
        if (find(v) != v) continue;                  // compound member: root emits
        const int na = static_cast<int>(arms[v].size());
        if (na < 3) { ++nStub; continue; }          // arms not gathered (trim/lookup failed)
        if (clusterSize[v] > 1) ++nCompound;         // merged too-close junctions
        else if (na != deg[v]) ++nMismatch;          // gathered != incident: a real bug
        if (na == 3) ++nT;
        else if (na == 4) ++nCoons;
        else ++nFan;
        std::vector<Vec3> padFootprint;
        // The authored kerb return is only APPLIED where it is geometrically
        // possible. The curb/sidewalk band is an INWARD offset of this corner
        // (the sidewalk sits on the concave side of a kerb return), and a curve
        // cannot be offset inward by more than its own radius: with a 3.5 m walk
        // around a 3.0 m return the band's outer edge is degenerate before it is
        // drawn. Measured on metro_v2 (docs/curb-weld-analysis.md): rounding at
        // r=3.0 with a 3.5 m walk took folded band quads 462 -> 832 and stacked
        // band 884 -> 1084 m2, while the SAME rounding with a 2.0 m walk costs
        // almost nothing (464 folds, 254 m2). So round when the radius can carry
        // the walk, and keep the old corner when it cannot.
        const double padCorner =
            (std::getenv("RT_KERB_RETURN") && cornerRadius > sidewalkWidth * 1.05)
                ? cornerRadius : 0.0;
        RenderMesh pad = junctionPatch(arms[v], 2.0f, Vec3(0.10, 0.10, 0.11),
                                       &padFootprint,
                                       nodeRails.count(v) ? &nodeRails[v]
                                                          : nullptr,
                                       padCorner);
        // 2c: an ELEVATED pad (a gore on the deck) closes its bottom — a
        // mirrored down-facing belly at deck thickness plus a rim skirt, so
        // the viaduct has structure under the junction fill too, not a
        // floating carpet between the chains' undersides.
        if (padFootprint.size() >= 3) {
            double avgY = 0, gnd = 0;
            for (const Vec3& p : padFootprint) {
                avgY += p.y;
                gnd += ground(p.x, p.z);
            }
            avgY /= padFootprint.size();
            gnd /= padFootprint.size();
            if (avgY - gnd > 2.5) {
                const double thk = 0.5;
                RenderMesh belly = pad;
                for (auto& vtx : belly.vertices) vtx.position.y -= thk;
                // Flip winding so the belly faces DOWN.
                for (std::size_t t = 0; t + 2 < belly.indices.size(); t += 3)
                    std::swap(belly.indices[t + 1], belly.indices[t + 2]);
                MeshBuilder::append(out, belly);
                // Rim skirt: quads around the footprint loop.
                RenderMesh skirt;
                const std::size_t nfp = padFootprint.size();
                for (std::size_t i = 0; i < nfp; ++i) {
                    const Vec3& A = padFootprint[i];
                    const Vec3& B = padFootprint[(i + 1) % nfp];
                    Vec3 d = B - A;
                    const double dl = std::sqrt(d.x * d.x + d.z * d.z);
                    if (dl < 1e-6) continue;
                    const Vec3 nrm(d.z / dl, 0.0, -d.x / dl);   // outward (CCW loop)
                    MeshBuilder::emitQuad(
                        skirt, A, B, Vec3(B.x, B.y - thk, B.z),
                        Vec3(A.x, A.y - thk, A.z), nrm, Vec3(0.32, 0.32, 0.34));
                }
                MeshBuilder::append(out, skirt);
            }
        }
        // The pad's drivable faces join the deck field (RoadDeckField::pads);
        // an elevated pad's bottom closure faces down and is skipped.
        if (deckOut) {
            for (std::size_t ti = 0; ti + 2 < pad.indices.size(); ti += 3) {
                const Vec3& a = pad.vertices[pad.indices[ti]].position;
                const Vec3& b = pad.vertices[pad.indices[ti + 1]].position;
                const Vec3& c = pad.vertices[pad.indices[ti + 2]].position;
                if (pad.vertices[pad.indices[ti]].normal.y <= 0.0) continue;
                deckOut->pads.push_back({a, b, c});
            }
        }
        MeshBuilder::append(out, pad);
        // Only a street INTERSECTION's pad joins the sidewalk band union
        // (#21). A Landing/gore pad has a ramp arm; joining it wrapped the
        // sidewalk band around and ONTO the ramp mouth (the curb "sealed the
        // ramp entrance shut", and a zebra painted across it). The pad still
        // meshes as the drivable disc — it just stops recruiting sidewalk.
        if (padFootprint.size() >= 3 && streetArms[v] > 0 &&
            clusterKind[v] == JunctionKind::Intersection) {
            bandHeights.addLoop(padFootprint);
            Poly2 fp;
            for (const Vec3& p : padFootprint) fp.push_back(Vec2(p.x, p.z));
            if (signedArea(fp) < 0) std::reverse(fp.begin(), fp.end());
            bandRibbons.push_back(std::move(fp));
        }
    }
    // S5: sweep the curb + sidewalk band along every union boundary loop
    // (exterior outlines AND block-interior holes — the band rides each
    // loop's right normal, outward from the asphalt in both cases).
    if (!bandRibbons.empty()) {
        std::vector<Poly2> loops;
        int nSliverLoops = 0;
        for (Poly2& L : polygonUnion(bandRibbons)) {
            // snap-round + de-spur so band quads never fold on a hairline
            // vertex the union left behind.
            //
            // MERGE TOLERANCE > SNAP QUANTUM (docs/curb-weld-analysis.md). The
            // old pair — snap to 1 cm, then merge closer than 0.5 cm — cannot
            // converge: two union vertices a hair apart land on ADJACENT grid
            // points, exactly 1 cm apart, which is above the merge threshold, so
            // the snap MANUFACTURES the hairline pairs the merge exists to
            // remove. The kerb then reverses across a 1 cm edge and the band's
            // mitre blows that reversal out to metres. 2 cm clears the 1.41 cm
            // grid diagonal, so no adjacent-grid-point pair can survive; real
            // kerb detail is metres, never centimetres.
            const double kSnap = 0.01, kMerge = 0.02;
            Poly2 c;
            for (const Vec2& p : L) {
                const Vec2 q(std::round(p.x / kSnap) * kSnap, std::round(p.y / kSnap) * kSnap);
                if (c.empty() || (q - c.back()).length() > kMerge) c.push_back(q);
            }
            while (c.size() >= 2 && (c.front() - c.back()).length() <= kMerge) c.pop_back();
            bool changed = true;
            while (changed && c.size() > 3) {
                changed = false;
                const int nn2 = static_cast<int>(c.size());
                for (int i = 0; i < nn2; ++i) {
                    const Vec2 e0 = c[i] - c[(i + nn2 - 1) % nn2];
                    const Vec2 e1 = c[(i + 1) % nn2] - c[i];
                    const double l0 = e0.length(), l1 = e1.length();
                    if (l0 < 1e-9 || l1 < 1e-9) { c.erase(c.begin() + i); changed = true; break; }
                    const double cosT = dot(e0, e1) / (l0 * l1);
                    // A SPUR is a shape, not an angle: the kerb goes out and comes
                    // straight back, enclosing nothing. The old test fired only
                    // past ~170 deg (cosT < -0.985) — but the reversals that
                    // wreck the band measure 90-166 deg and sailed through. Judge
                    // the sliver instead: a reversal that encloses under 0.05 m2,
                    // or reverses across a sub-decimetre edge, is a union
                    // artifact. A LEGITIMATE sharp corner (an acute Y arriving at
                    // 30 deg) reverses just as hard but spans metres, so its
                    // triangle is ~1 m2 and it is kept.
                    const double sliver = 0.5 * std::fabs(cross(e0, e1));
                    // A sub-decimetre edge is noise at ANY angle: real kerb
                    // detail — an arc segment, a mouth, a corner — is metres.
                    // (Gating this on the angle too left a band of 90-120 deg
                    // reversals alive, 155 of them on metro_v2.)
                    // Judged against the BAND's scale, not centimetres (the
                    // sidewalk census, metro_v2): 24 of the 29 remaining
                    // sidewalk-on-asphalt places were reversals of 120-170 deg
                    // on 0.2-0.6 m edges enclosing 0.1-0.5 m2 — out-and-back
                    // hooks the union leaves where two roads' verges cross at a
                    // junction corner — and each one took the band a metre into
                    // the carriageway. A reversal sharper than 120 deg on an
                    // edge under 1 m, or enclosing under 0.5 m2, is noise: a
                    // real kerb never reverses inside a metre.
                    if (cosT < -0.985 || std::min(l0, l1) < 0.10 ||
                        (cosT < -0.5 && (sliver < 0.5 || std::min(l0, l1) < 1.0))) {
                        c.erase(c.begin() + i);
                        changed = true;
                        break;
                    }
                }
            }
            // A loop smaller than the band it would carry is not a kerb. The
            // union leaves a 10 cm triangle where two verges cross at a sharp
            // junction corner; the de-spur above never touches a triangle, and
            // the sweeper built a 2 m sidewalk ring around it — the concrete
            // wedge in the middle of metro's collector/local junction (census #1,
            // 4.4 m onto the deck).
            if (c.size() >= 3) {
                double perim = 0.0;
                for (std::size_t i = 0; i < c.size(); ++i)
                    perim += (c[(i + 1) % c.size()] - c[i]).length();
                if (std::fabs(signedArea(c)) < 1.0 || perim < 2.0) {
                    ++nSliverLoops;
                    continue;
                }
                loops.push_back(std::move(c));
            }
        }
        if (nSliverLoops > 0)
            LOG_INFO << "[lattice] dropped " << nSliverLoops
                     << " sliver curb loop(s) (area < 1 m2 or perimeter < 2 m)";
        if (auditOut) {
            auditOut->loops = loops;
            auditOut->mouthGaps = bandGaps;
            auditOut->sidewalkWidth = sidewalkWidth;
            auditOut->curbHeight = curbHeight;
            for (int v = 0; v < N; ++v)
                if (deg[v] >= 3) {
                    auditOut->junctions.push_back(g.nodes[v].pos);
                    auditOut->junctionDegree.push_back(deg[v]);
                    // Smallest gap between adjacent arms, from the SAME arm
                    // directions the pad was built from.
                    double minGap = -1.0;
                    if (arms[v].size() >= 2) {
                        std::vector<double> ang;
                        ang.reserve(arms[v].size());
                        for (const JunctionArm& A : arms[v])
                            ang.push_back(std::atan2(A.dir.y, A.dir.x));
                        std::sort(ang.begin(), ang.end());
                        minGap = 360.0;
                        for (std::size_t k = 0; k < ang.size(); ++k) {
                            double d = ang[(k + 1) % ang.size()] - ang[k];
                            if (k + 1 == ang.size()) d += 2.0 * PI;
                            minGap = std::min(minGap, d * 180.0 / PI);
                        }
                    }
                    auditOut->junctionMinAngle.push_back(minGap);
                }
        }
        MeshBuilder::append(out, sweepCurbSidewalkBand(
            loops, [&](double x, double z) { return bandHeights.sample(x, z); },
            sidewalkWidth, curbHeight, bandGaps.empty() ? nullptr : &bandGaps,
            heightAt ? &heightAt : nullptr));   // outer skirt drapes to terrain
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
                 << " pad3=" << nT << " pad4=" << nCoons << " pad5+=" << nFan
                 << " compound=" << nCompound << " stub(arms<3)=" << nStub
                 << " armMismatch=" << nMismatch
                 << " | tris=" << (out.indices.size() / 3) << " degenerate=" << degen;
        std::string h;
        for (int d = 0; d < 12; ++d) if (degHist[d]) h += " d" + std::to_string(d) + "=" + std::to_string(degHist[d]);
        LOG_INFO << "[lattice] degree histogram:" << h;
    }
    return out;
}

RenderMesh buildRoadNetMesh(const RoadEntity& road, const RoadGroundFn& heightAt,
                            CurbBandAudit* auditOut, RoadDeckField* deckOut) {
    // Roads-v2.1 2e: the ONE mesher builds EVERYTHING, baked corridor edges
    // included — the corridor renderer is gone. (Conform still strips baked
    // edges: corridorAuthor's engineered flatten owns that carve.)
    // Cap centerline curvature so neither the carriageway nor the sidewalk outer rail can
    // fold: keep the turn radius above the widest offset (half-width + sidewalk) + margin.
    double minR = netMinTurnRadius(road);
    RoadGraph raw = sampleNetGraph(road, minR);
    // Does the local-constraints pass (ADR-0052) promote any node to a roundabout?
    // Honour the junction policy (ADR-0075 P0): a generated road set
    // autoRoundabout=false, so we must NOT probe/promote with default rules here
    // (that re-promoted roundabouts the generator disabled).
    RoadRules rules;
    rules.autoRoundabout = road.look.autoRoundabout;
    RoadGraph g = applyConstraints(raw, rules);   // promoted graph (= the constrained graph)
    joinDanglingEndsLogged(g, road);              // the same joins/pull-backs the other graphs got

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
                         rampGrade = dr.rampGrade, groundSurf = road.look.lift;
            auto groundFn = [&](const Vec2& q) { return heightAt ? heightAt(q.x, q.y) : 0.0; };
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
                        // 2e: AUTHORED deck heights win. A baked corridor
                        // node already carries its solved profile — the
                        // clearance re-derivation must not clobber it (it
                        // would flatten the engineered viaduct onto generic
                        // layer heights).
                        if (g.nodes[ns[i]].elevAbsolute) continue;
                        g.nodes[ns[i]].elev = static_cast<Real>(deckY[i]);
                        g.nodes[ns[i]].elevAbsolute = true;                     // ride it through the weld
                    }
                }
            }
        }
    }

    // Semantic layer (#17): classify AFTER the layer-elevation stamping so
    // access bits see the final elevAbsolute truth.
    classifyRoadGraph(g, heightAt);

    // ONE MESHER (roads-v2 S6): every street net goes through the swept
    // lattice — carriageway bodies, junction pads, curb/sidewalk band loops,
    // and per-class structure (freeway kit, ramp decks, layered bridges).
    // The union weld (weldSolid), the SDF grid and the analytic fallback are
    // deleted from this path; the lattice IS the road mesher.
    RenderMesh rm = buildRoadNetLattice(g, heightAt, nullptr, road.look.sidewalk,
                                        road.look.curb, road.look.crosswalks, auditOut,
                                        road.look.cornerRadius,
                                        deckOut);
    if (deckOut) deckOut->buildIndex();
    // Bounds + NaN audit: one NaN vertex poisons the bounding sphere and the
    // whole road entity frustum-culls to nothing, silently.
    std::size_t nans = 0;
    Vec3 lo(1e30, 1e30, 1e30), hi(-1e30, -1e30, -1e30);
    for (const Vertex& v : rm.vertices) {
        if (!std::isfinite(v.position.x) || !std::isfinite(v.position.y) ||
            !std::isfinite(v.position.z)) { ++nans; continue; }
        lo.x = std::min(lo.x, v.position.x); hi.x = std::max(hi.x, v.position.x);
        lo.y = std::min(lo.y, v.position.y); hi.y = std::max(hi.y, v.position.y);
        lo.z = std::min(lo.z, v.position.z); hi.z = std::max(hi.z, v.position.z);
    }
    LOG_INFO << "[roadmesh] " << g.edges.size() << " edges -> "
             << rm.vertices.size() << " verts, " << rm.indices.size() / 3
             << " tris, nan " << nans << ", y [" << lo.y << ", " << hi.y
             << "], x [" << lo.x << ", " << hi.x << "], z [" << lo.z << ", "
             << hi.z << "]";
    // RT_DUMP_ROADMESH=<path>: write the mesh's XZ vertex cloud so coverage can
    // be plotted against the road graph (which chains actually meshed?).
    if (const char* dp = std::getenv("RT_DUMP_ROADMESH")) {
        if (FILE* f = std::fopen(dp, "w")) {
            for (std::size_t i = 0; i < rm.vertices.size(); i += 16)
                std::fprintf(f, "%.1f %.1f\n", rm.vertices[i].position.x,
                             rm.vertices[i].position.z);
            std::fclose(f);
        }
    }
    return rm;
}

// ---- RoadDeckField ------------------------------------------------------------
void RoadDeckField::buildIndex() {
    cells_.clear();
    padCells_.clear();
    auto key = [](int cx, int cz) {
        return (static_cast<long long>(cx) << 32) ^
               (static_cast<long long>(cz) & 0xffffffffLL);
    };
    for (int ti = 0; ti < static_cast<int>(pads.size()); ++ti) {
        const Tri& t = pads[ti];
        const double x0 = std::min({t.a.x, t.b.x, t.c.x}), x1 = std::max({t.a.x, t.b.x, t.c.x});
        const double z0 = std::min({t.a.z, t.b.z, t.c.z}), z1 = std::max({t.a.z, t.b.z, t.c.z});
        for (int cz = static_cast<int>(std::floor(z0 / cell_)); cz <= static_cast<int>(std::floor(z1 / cell_)); ++cz)
            for (int cx = static_cast<int>(std::floor(x0 / cell_)); cx <= static_cast<int>(std::floor(x1 / cell_)); ++cx)
                padCells_[key(cx, cz)].push_back(ti);
    }
    for (int si = 0; si < static_cast<int>(spines.size()); ++si) {
        const UnionSpine& sp = spines[si];
        if (sp.yAbs.size() != sp.points.size()) continue;   // no deck: skip
        for (int i = 0; i + 1 < static_cast<int>(sp.points.size()); ++i) {
            const Vec2& a = sp.points[i];
            const Vec2& b = sp.points[i + 1];
            const int x0 = static_cast<int>(std::floor(std::min(a.x, b.x) / cell_)) - 1;
            const int x1 = static_cast<int>(std::floor(std::max(a.x, b.x) / cell_)) + 1;
            const int z0 = static_cast<int>(std::floor(std::min(a.y, b.y) / cell_)) - 1;
            const int z1 = static_cast<int>(std::floor(std::max(a.y, b.y) / cell_)) + 1;
            for (int cz = z0; cz <= z1; ++cz)
                for (int cx = x0; cx <= x1; ++cx)
                    cells_[key(cx, cz)].push_back({si, i});
        }
    }
}

double RoadDeckField::depthInside(double x, double z, int* spineOut) const {
    if (spineOut) *spineOut = -1;
    const long long k = (static_cast<long long>(static_cast<int>(std::floor(x / cell_))) << 32) ^
                        (static_cast<long long>(static_cast<int>(std::floor(z / cell_))) & 0xffffffffLL);
    double depth = 0.0;
    if (auto pit = padCells_.find(k); pit != padCells_.end()) {
        for (int ti : pit->second) {
            const Tri& t = pads[ti];
            const double d1 = (x - t.b.x) * (t.a.z - t.b.z) - (t.a.x - t.b.x) * (z - t.b.z);
            const double d2 = (x - t.c.x) * (t.b.z - t.c.z) - (t.b.x - t.c.x) * (z - t.c.z);
            const double d3 = (x - t.a.x) * (t.c.z - t.a.z) - (t.c.x - t.a.x) * (z - t.a.z);
            const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            if (neg && pos) continue;
            auto edgeDist = [&](const Vec3& p, const Vec3& q) {
                const double ex = q.x - p.x, ez = q.z - p.z;
                const double l2 = ex * ex + ez * ez;
                double u = l2 < 1e-12 ? 0.0 : ((x - p.x) * ex + (z - p.z) * ez) / l2;
                u = u < 0 ? 0 : (u > 1 ? 1 : u);
                const double dx = p.x + ex * u - x, dz = p.z + ez * u - z;
                return std::sqrt(dx * dx + dz * dz);
            };
            depth = std::max(depth, std::min({edgeDist(t.a, t.b), edgeDist(t.b, t.c), edgeDist(t.c, t.a)}));
        }
    }
    auto it = cells_.find(k);
    if (it == cells_.end()) return depth;
    const Vec2 q(x, z);
    for (const Seg& sg : it->second) {
        const UnionSpine& sp = spines[sg.spine];
        const Vec2& a = sp.points[sg.i];
        const Vec2& b = sp.points[sg.i + 1];
        const Vec2 ab = b - a;
        const double L2 = ab.lengthSquared();
        if (L2 < 1e-12) continue;
        const double t = dot(q - a, ab) / L2;
        if (t < 0.0 || t > 1.0) continue;   // past an end: a cap, not the carriageway
        const double d = (a + ab * t - q).length();
        const double hw = sp.hw.size() == sp.points.size()
                              ? sp.hw[sg.i] + (sp.hw[sg.i + 1] - sp.hw[sg.i]) * t
                              : sp.halfWidth;
        if (hw - d > depth) {
            depth = hw - d;
            if (spineOut) *spineOut = sg.spine;
        }
    }
    return depth;
}

bool RoadDeckField::heightAt(double x, double z, double margin, double* outY) const {
    const long long k = (static_cast<long long>(static_cast<int>(std::floor(x / cell_))) << 32) ^
                        (static_cast<long long>(static_cast<int>(std::floor(z / cell_))) & 0xffffffffLL);
    // Pads first: inside a pad the drawn triangle IS the surface.
    if (auto pit = padCells_.find(k); pit != padCells_.end()) {
        bool found = false;
        double best = -1e30;
        for (int ti : pit->second) {
            const Tri& t = pads[ti];
            const double d1 = (x - t.b.x) * (t.a.z - t.b.z) - (t.a.x - t.b.x) * (z - t.b.z);
            const double d2 = (x - t.c.x) * (t.b.z - t.c.z) - (t.b.x - t.c.x) * (z - t.c.z);
            const double d3 = (x - t.a.x) * (t.c.z - t.a.z) - (t.c.x - t.a.x) * (z - t.a.z);
            const bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
            const bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
            if (neg && pos) continue;
            const double den = (t.b.z - t.c.z) * (t.a.x - t.c.x) + (t.c.x - t.b.x) * (t.a.z - t.c.z);
            if (std::fabs(den) < 1e-12) continue;
            const double w0 = ((t.b.z - t.c.z) * (x - t.c.x) + (t.c.x - t.b.x) * (z - t.c.z)) / den;
            const double w1 = ((t.c.z - t.a.z) * (x - t.c.x) + (t.a.x - t.c.x) * (z - t.c.z)) / den;
            const double y = w0 * t.a.y + w1 * t.b.y + (1.0 - w0 - w1) * t.c.y;
            if (!found || y > best) { best = y; found = true; }   // stacked pads: the top
        }
        if (found) { *outY = best; return true; }
    }
    auto it = cells_.find(k);
    if (it == cells_.end()) return false;
    const Vec2 q(x, z);
    double bestD = 1e30, bestY = 0.0;
    for (const Seg& sg : it->second) {
        const UnionSpine& sp = spines[sg.spine];
        const Vec2& a = sp.points[sg.i];
        const Vec2& b = sp.points[sg.i + 1];
        const Vec2 ab = b - a;
        const double L2 = ab.lengthSquared();
        double t = L2 < 1e-12 ? 0.0 : dot(q - a, ab) / L2;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        const double d = (a + ab * t - q).length();
        const double hw = sp.hw.size() == sp.points.size()
                              ? sp.hw[sg.i] + (sp.hw[sg.i + 1] - sp.hw[sg.i]) * t
                              : sp.halfWidth;
        if (d > hw + margin || d >= bestD) continue;
        bestD = d;
        bestY = sp.yAbs[sg.i] + (sp.yAbs[sg.i + 1] - sp.yAbs[sg.i]) * t;
        // Superelevation: the lattice banks the deck by signedLateral *
        // crossSlope, lateral measured along `left` = fwd rotated +90 in XZ
        // (road_lattice.cpp, rings[i].left), + rising to the left.
        if (sp.crossSlope.size() == sp.points.size() && L2 > 1e-12) {
            const double cs = sp.crossSlope[sg.i] + (sp.crossSlope[sg.i + 1] - sp.crossSlope[sg.i]) * t;
            const Vec2 fwd = ab * (1.0 / std::sqrt(L2));
            const Vec2 left(-fwd.y, fwd.x);
            bestY += dot(q - a, left) * cs;
        }
    }
    if (bestD > 1e29) return false;
    *outY = bestY;
    return true;
}

std::vector<TerrainFlatten> roadNetConformRegions(const RoadEntity& roadIn,
                                                  const RoadGroundFn& heightAt,
                                                  double shoulder,
                                                  double falloff, double maxGrade) {
    RT_PROFILE_ZONE_NAMED("roadNetConformRegions");
    // 2e: the conform is the ONE consumer that still strips baked corridor
    // edges — corridorAuthor's engineered flatten (at-grade windows only,
    // viaducts fly) owns that carve; carving the baked chains here too would
    // double-grade the interchange ground.
    const RoadEntity road = roadNetStreetsOnly(roadIn);
    std::vector<TerrainFlatten> out;
    if (!heightAt) return out;                           // flat road: nothing to carve
    // Mirror the DECK's own profile computation EXACTLY — the same constrained
    // graph, the same weldChainSpines decomposition (curve-sampled points,
    // per-chain widths), the same roadProfile smoothing at the weld's grade —
    // so the carve grades the ground to where the deck actually is. The old
    // independently-densified profile diverged by metres on slopes: the
    // grade-limited deck cut through hills the carve never lowered (device:
    // "the road is being buried by the terrain — it's not conforming").
    RoadGraph g = constrainedGraph(road, heightAt);      // grade to the roundabout, not the raw spokes
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
        weldChainProfiles(spines, heightAt, 0.0, kRoadMaxGrade,
                          road.look.sidewalk + 4.0);
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
            sp.points, profile, sp.halfWidth + road.look.sidewalk + 2.0, shoulder,
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
            const double r = jw[v] * 0.5 * 1.02 + road.look.sidewalk + 2.0;
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

StructureSet buildRoadWalls(const RoadEntity& roadIn, const RoadGroundFn& heightAt,
                            const StructureParams& p) {
    StructureSet set;
    // CO-DESIGNED with the SMOOTHSTEP conform (roads-v2.1 R6a; the old
    // version priced walls off the DaylightBatter reach the conform no
    // longer uses). roadNetConformRegions feathers deck -> natural over
    // `falloff` metres past the graded corridor; a DEEP CUT leaves that
    // feather face near-vertical and raw (hillcity's first frame). The wall
    // stands just OUTSIDE the graded corridor and rises to the natural
    // ground at the feather's END, fronting the steep face — the terrain
    // still smoothsteps behind it, hidden. Shallow cuts (drop <= minWall)
    // keep their grass; fills stay open embankments (they read as slopes).
    const RoadEntity road = roadNetStreetsOnly(roadIn);   // corridor flies; no walls
    if (!heightAt) return set;                            // flat road: no walls
    RoadGraph g = constrainedGraph(road, heightAt);
    std::vector<UnionSpine> spines = weldChainSpines(g);
    std::vector<std::vector<double>> profiles =
        weldChainProfiles(spines, heightAt, 0.0, kRoadMaxGrade,
                          road.look.sidewalk + 4.0);
    const double falloff = 8.0;   // roadNetConformRegions' feather span

    for (std::size_t si = 0; si < spines.size(); ++si) {
        const UnionSpine& sp = spines[si];
        const int n = static_cast<int>(sp.points.size());
        if (static_cast<int>(profiles[si].size()) != n || n < 2) continue;
        if (!sp.yAbs.empty()) continue;   // authored elevated: piers, not walls
        const double flatHalf = sp.halfWidth + road.look.sidewalk + 2.0;   // graded corridor
        const double wallLat = flatHalf + 0.6;    // just outside the graded ground
        // DENSIFIED stations (~6 m): the spine's own points can span a whole
        // straight edge, and a single wall quad would chord an undulating
        // hill line instead of following it.
        std::vector<Vec2> pts;
        std::vector<double> deck;
        for (int k = 0; k + 1 < n; ++k) {
            const Vec2 A = sp.points[k], B = sp.points[k + 1];
            const double len = (B - A).length();
            const int div = std::max(1, static_cast<int>(std::ceil(len / 6.0)));
            for (int t = 0; t < div; ++t) {
                const double f = static_cast<double>(t) / div;
                pts.push_back(A + (B - A) * f);
                deck.push_back(profiles[si][k] +
                               (profiles[si][k + 1] - profiles[si][k]) * f);
            }
        }
        pts.push_back(sp.points[n - 1]);
        deck.push_back(profiles[si][n - 1]);
        const int m = static_cast<int>(pts.size());
        auto stationWall = [&](int side, int k, Vec3& topOut, double& dropOut) {
            int seg = std::min(k, m - 2);
            Vec2 dir = normalize(pts[seg + 1] - pts[seg]);
            Vec2 nrm = perp(dir);
            Vec2 at = pts[k] + nrm * (static_cast<double>(side) * wallLat);
            Vec2 end =
                pts[k] + nrm * (static_cast<double>(side) * (flatHalf + falloff));
            const double naturalEnd = heightAt(end.x, end.y);
            const double drop = naturalEnd - deck[k];   // cut face height
            topOut = Vec3(at.x, deck[k] + std::max(0.0, drop), at.y);
            dropOut = std::max(0.0, drop);
        };
        for (int side = -1; side <= 1; side += 2)
            for (int k = 0; k + 1 < m; ++k) {
                Vec3 topA, topB; double dropA, dropB;
                stationWall(side, k, topA, dropA);
                stationWall(side, k + 1, topB, dropB);
                if (dropA <= p.minWall && dropB <= p.minWall) continue;
                WallSegment w{topA, topB, dropA, dropB, true};
                const int seg = std::min(k, m - 2);
                const Vec2 nrm = perp(normalize(pts[seg + 1] - pts[seg]));
                w.out = Vec3(nrm.x * side, 0, nrm.y * side);
                w.bench = falloff - 0.6 + 0.4;   // cover the feather to natural
                set.walls.push_back(w);
                set.colliderEdges.push_back({topA, topB});
            }
    }
    set.mesh = bakeWallMesh(set.walls, p.color);
    return set;
}

RoadSpec roadNetEdgeSpec(const RoadEntity& road, int ei) {
    const RoadGraph& g = road.graph;
    if (ei >= 0 && ei < static_cast<int>(g.edges.size())) {
        const int si = g.edges[ei].spec;
        if (si >= 0 && si < static_cast<int>(g.specs.size())) return g.specs[si];
        return roadSpecFromLegacy(g.edges[ei].width, /*oneWay=*/false,
                                  road.look.sidewalk, road.look.curb);
    }
    return roadSpecFromLegacy(road.look.defaultWidth, /*oneWay=*/false,
                              road.look.sidewalk, road.look.curb);
}

bool roadNetSetEdgeWidth(RoadEntity& road, int ei, double w) {
    if (ei < 0 || ei >= static_cast<int>(road.graph.edges.size())) return false;
    // <= 0 reverts to the look's default width (widths are always resolved).
    road.graph.edges[ei].width =
        static_cast<Real>((w > 0.0) ? w : road.look.defaultWidth);
    return true;
}

bool roadNetMoveNode(RoadEntity& road, int i, const Vec2& pos) {
    if (i < 0 || i >= static_cast<int>(road.graph.nodes.size())) return false;
    road.graph.nodes[i].pos = pos;
    return true;
}

bool roadNetSetTangent(RoadEntity& road, int i, const Vec2& tangent) {
    if (i < 0 || i >= static_cast<int>(road.graph.nodes.size())) return false;
    road.graph.nodes[i].tangent = tangent;
    return true;
}

Vec2 roadNetTangentAt(const RoadEntity& road, int i) {
    const RoadGraph& g = road.graph;
    const int n = static_cast<int>(g.nodes.size());
    if (i < 0 || i >= n) return Vec2(0, 0);
    if (!isZero(g.nodes[i].tangent)) return g.nodes[i].tangent;
    // Auto: Catmull-Rom on a degree-2 through-road, chord at an end, else zero.
    std::vector<int> nb;
    for (int ei : validEdgeIndices(g)) {
        const RoadEdge& e = g.edges[ei];
        if (e.a == i) nb.push_back(e.b);
        if (e.b == i) nb.push_back(e.a);
    }
    if (nb.size() == 2) return (g.nodes[nb[1]].pos - g.nodes[nb[0]].pos) * 0.5;
    if (nb.size() == 1) return (g.nodes[nb[0]].pos - g.nodes[i].pos) * 0.5;
    return Vec2(0, 0);
}

int roadNetAddNode(RoadEntity& road, const Vec2& pos) {
    road.graph.nodes.push_back(RoadNode{pos});   // zero tangent -> the new node is auto
    return static_cast<int>(road.graph.nodes.size()) - 1;
}

bool roadNetAddEdge(RoadEntity& road, int a, int b) {
    RoadGraph& g = road.graph;
    const int n = static_cast<int>(g.nodes.size());
    if (a < 0 || b < 0 || a >= n || b >= n || a == b) return false;
    for (const RoadEdge& e : g.edges)
        if ((e.a == a && e.b == b) || (e.a == b && e.b == a)) return false;   // already joined
    RoadEdge e;
    e.a = a; e.b = b;
    e.width = static_cast<Real>(road.look.defaultWidth);
    g.edges.push_back(e);
    return true;
}

int roadNetExtend(RoadEntity& road, int from, const Vec2& pos) {
    if (from < 0 || from >= static_cast<int>(road.graph.nodes.size())) return -1;
    int ni = roadNetAddNode(road, pos);
    roadNetAddEdge(road, from, ni);
    return ni;
}

int roadNetSplitEdge(RoadEntity& road, int edgeIndex, const Vec2& pos) {
    RoadGraph& g = road.graph;
    if (edgeIndex < 0 || edgeIndex >= static_cast<int>(g.edges.size())) return -1;
    int ni = roadNetAddNode(road, pos);
    // Both halves inherit the split edge's width/class/spec/layer wholesale —
    // the copy carries every field, which is the whole point of the one graph.
    RoadEdge second = g.edges[edgeIndex];
    const int b = second.b;
    g.edges[edgeIndex].b = ni;                // a -> new
    second.a = ni; second.b = b;              // new -> b
    g.edges.push_back(second);
    return ni;
}

bool roadNetDeleteNode(RoadEntity& road, int i) {
    RoadGraph& g = road.graph;
    const int n = static_cast<int>(g.nodes.size());
    if (i < 0 || i >= n) return false;
    // Edge fields travel WITH their edge, so the delete cannot desync them
    // (review S4b: the parallel arrays once shifted out of order here).
    std::vector<RoadEdge> kept;
    for (const RoadEdge& e : g.edges) {
        if (e.a == i || e.b == i) continue;                   // drop incident edges
        RoadEdge o = e;
        if (o.a > i) --o.a;
        if (o.b > i) --o.b;
        kept.push_back(o);
    }
    g.edges = std::move(kept);
    g.nodes.erase(g.nodes.begin() + i);
    return true;
}

int roadNetNearestEdge(const RoadEntity& road, const Vec2& p, double maxDist) {
    const RoadGraph& g = road.graph;
    int best = -1;
    double bestD2 = maxDist * maxDist;
    for (int ei = 0; ei < static_cast<int>(g.edges.size()); ++ei) {
        const RoadEdge& e = g.edges[ei];
        if (e.a < 0 || e.b < 0 || e.a >= static_cast<int>(g.nodes.size())
            || e.b >= static_cast<int>(g.nodes.size())) continue;
        Vec2 A = g.nodes[e.a].pos, B = g.nodes[e.b].pos, ab = B - A;
        double len2 = ab.lengthSquared();
        double t = len2 > 1e-9 ? std::max(0.0, std::min(1.0, dot(p - A, ab) / len2)) : 0.0;
        double d2 = (p - (A + ab * t)).lengthSquared();
        if (d2 < bestD2) { bestD2 = d2; best = ei; }
    }
    return best;
}

// --- level I/O -----------------------------------------------------------------
// THE ONLY CODE THAT STILL KNOWS THE PARALLEL-ARRAY SHAPE (unification step 6).
// The wire format is unchanged — sparse per-edge overrides, short arrays meaning
// "default" — and every fallback is resolved HERE, on the way in, so nothing
// downstream ever guards a size again.

RoadEntity roadNetFromJson(const json& j) {
    RoadEntity road;
    // The look first: the default width seeds every edge that carries no override.
    road.look.defaultWidth = j.value("width", road.look.defaultWidth);
    road.look.sidewalk = j.value("sidewalk", road.look.sidewalk);
    road.look.curb = j.value("curb", road.look.curb);
    road.look.cornerRadius = j.value("corner_radius", road.look.cornerRadius);
    road.look.lift = j.value("lift", road.look.lift);
    road.look.markings = j.value("markings", road.look.markings);
    road.look.crosswalks = j.value("crosswalks", road.look.crosswalks);
    road.look.autoRoundabout = j.value("auto_roundabout", road.look.autoRoundabout);
    if (j.contains("color") && j["color"].is_array() && j["color"].size() == 3)
        road.look.color = Vec3(j["color"][0].get<double>(), j["color"][1].get<double>(),
                               j["color"][2].get<double>());

    RoadGraph& g = road.graph;
    if (j.contains("nodes") && j["nodes"].is_array())
        for (const json& p : j["nodes"])
            g.nodes.push_back(RoadNode{Vec2(p.value("x", 0.0), p.value("z", 0.0))});
    if (j.contains("edges") && j["edges"].is_array())
        for (const json& e : j["edges"]) {
            double w = 0.0;
            RoadEdge ge;
            if (e.is_array() && e.size() >= 2) {
                ge.a = e[0].get<int>(); ge.b = e[1].get<int>();
                if (e.size() >= 3) w = e[2].get<double>();        // [a, b, width]
            } else if (e.is_object()) {
                ge.a = e.value("a", 0); ge.b = e.value("b", 0);
                w = e.value("width", 0.0);
            } else {
                continue;
            }
            // Resolve the width fallback at construction: override, else default.
            ge.width = static_cast<Real>(w > 0.0 ? w : road.look.defaultWidth);
            g.edges.push_back(ge);
        }
    if (j.contains("edge_layers") && j["edge_layers"].is_array()) {
        const json& el = j["edge_layers"];
        for (std::size_t i = 0; i < el.size() && i < g.edges.size(); ++i)
            g.edges[i].layer = el[i].get<int>();
    }
    if (j.contains("edge_classes") && j["edge_classes"].is_array()) {
        const json& ec = j["edge_classes"];
        auto parseClass = [](const std::string& s) {
            if (s == "freeway")   return RoadClass::Freeway;
            if (s == "arterial")  return RoadClass::Arterial;
            if (s == "collector") return RoadClass::Collector;
            if (s == "ramp")      return RoadClass::Ramp;
            return RoadClass::Local;
        };
        for (std::size_t i = 0; i < ec.size() && i < g.edges.size(); ++i)
            g.edges[i].klass = parseClass(ec[i].get<std::string>());
    }
    if (j.contains("tangents") && j["tangents"].is_array()) {
        const json& ts = j["tangents"];
        for (std::size_t i = 0; i < ts.size() && i < g.nodes.size(); ++i) {
            const json& t = ts[i];
            if (t.is_array() && t.size() >= 2)
                g.nodes[i].tangent = Vec2(t[0].get<double>(), t[1].get<double>());
            else if (t.is_object())
                g.nodes[i].tangent = Vec2(t.value("x", 0.0), t.value("z", 0.0));
        }
    }
    // Per-node absolute deck Y (elevated roads). A null (or non-number) entry
    // means at-grade — resolved straight to elevAbsolute=false (drape).
    if (j.contains("node_elev") && j["node_elev"].is_array()) {
        const json& ne = j["node_elev"];
        for (std::size_t i = 0; i < ne.size() && i < g.nodes.size(); ++i)
            if (ne[i].is_number()) {
                g.nodes[i].elev = static_cast<Real>(ne[i].get<double>());
                g.nodes[i].elevAbsolute = true;
            }
    }
    // Roads-v2 band model: a spec table + per-edge indices. Either a preset
    // name ("freeway3") or an object with a "bands" array per entry.
    if (j.contains("specs") && j["specs"].is_array())
        for (const auto& js : j["specs"]) g.specs.push_back(roadSpecFromJson(js));
    if (j.contains("edge_specs") && j["edge_specs"].is_array()) {
        const json& es = j["edge_specs"];
        for (std::size_t i = 0; i < es.size() && i < g.edges.size(); ++i)
            g.edges[i].spec = es[i].is_number_integer() ? es[i].get<int>() : -1;
    }
    return road;
}

json roadNetToJson(const RoadEntity& road) {
    const RoadGraph& g = road.graph;
    json j;
    json nodes = json::array();
    for (const RoadNode& p : g.nodes) nodes.push_back({{"x", p.pos.x}, {"z", p.pos.y}});
    j["nodes"] = std::move(nodes);
    json edges = json::array();
    for (const RoadEdge& e : g.edges) {
        // Reconstruct the sparse form: only a width that differs from the look's
        // default is an override worth writing ([a, b, width]).
        const double w = e.width;
        if (w != road.look.defaultWidth) edges.push_back(json::array({e.a, e.b, w}));
        else                             edges.push_back(json::array({e.a, e.b}));
    }
    j["edges"] = std::move(edges);
    bool anyLayer = false;
    for (const RoadEdge& e : g.edges) if (e.layer != 0) { anyLayer = true; break; }
    if (anyLayer) {
        json layers = json::array();
        for (const RoadEdge& e : g.edges) layers.push_back(e.layer);
        j["edge_layers"] = std::move(layers);
    }
    bool anyTan = false;
    for (const RoadNode& n : g.nodes) if (!isZero(n.tangent)) { anyTan = true; break; }
    if (anyTan) {
        json tans = json::array();
        for (const RoadNode& n : g.nodes)
            tans.push_back(json::array({n.tangent.x, n.tangent.y}));
        j["tangents"] = std::move(tans);
    }
    bool anyElev = false;
    for (const RoadNode& n : g.nodes) if (n.elevAbsolute) { anyElev = true; break; }
    if (anyElev) {
        json elev = json::array();
        for (const RoadNode& n : g.nodes)
            if (n.elevAbsolute) elev.push_back(static_cast<double>(n.elev));
            else                elev.push_back(nullptr);           // at-grade node
        j["node_elev"] = std::move(elev);
    }
    j["width"] = road.look.defaultWidth;
    j["sidewalk"] = road.look.sidewalk;
    j["curb"] = road.look.curb;
    j["corner_radius"] = road.look.cornerRadius;
    j["lift"] = road.look.lift;
    j["markings"] = road.look.markings;
    j["crosswalks"] = road.look.crosswalks;
    j["color"] = json::array({road.look.color.x, road.look.color.y, road.look.color.z});
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
        // Copy-then-remap (field-carry): the positional re-init dropped every
        // field after `layer` on warped edges.
        auto segEdge = [&](int va, int vb) {
            RoadEdge o = e;
            o.a = va;
            o.b = vb;
            out.edges.push_back(o);
        };
        for (int i = 1; i < segs; ++i) {
            double t = static_cast<double>(i) / segs;
            Vec2 p = a + (b - a) * t;
            int mi = static_cast<int>(out.nodes.size());
            out.nodes.push_back(RoadNode{p + warp(p)});
            segEdge(prev, mi);
            prev = mi;
        }
        segEdge(prev, e.b);
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

void applyGenerateRecipe(RoadEntity& road, const json& g, const RoadGroundFn& heightAt) {
    RT_PROFILE_ZONE_NAMED("applyGenerateRecipe");
    if (!g.is_object()) return;
    // Pick the generator. "district" (default) subdivides one footprint; "metro"
    // grows organic arterials between hotspots and fills the blocks between them.
    // Both hand a raw RoadGraph to the SHARED junction-policy tail below.
    const std::string kind = g.value("kind", std::string("district"));
    // A regenerate must leave a CLEAN road: buildMetro APPENDS freeway plans and
    // hubs (they would accumulate across regenerates), and stale spec/baked/
    // elevation state describes the OLD graph — a stale baked flag would
    // silently drop a fresh street edge from the mesh, a stale elevation would
    // lift a street onto a phantom deck. The graph assignment below replaces
    // nodes/edges wholesale (with per-field normalization), so only the plan
    // and the spec table need explicit clearing here.
    road.plan.freewayPlans.clear();
    road.plan.cityHubs.clear();
    road.plan.siteFootprints.clear();
    road.graph.specs.clear();
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
        mp.arteryWidth = g.value("artery_width", road.look.defaultWidth * 1.6);
        mp.streetWidth = g.value("street_width", road.look.defaultWidth);
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
        mp.minBlockEdge    = g.value("min_block_edge", 0.0);
        // P7 patch-conforming fabric: how city-site faces subdivide.
        // "" = legacy gridFill; "chords" | "bisect" | "court" | "mix"
        // (mix = seeded per-face choice weighted by district kind).
        mp.fabric             = g.value("fabric", std::string());
        mp.fabricCoreLen      = g.value("fabric_core_len",
                                        mp.fabric.empty() ? 0.0 : 110.0);
        mp.fabricConform      = g.value("fabric_conform", 0.15);
        mp.fabricSoftCollapse = g.value("fabric_soft_collapse", 0.8);
        mp.fabricJitter       = g.value("fabric_jitter", 0.12);
        // P8 footprint-first skeleton ("" = legacy colonization). Stage B:
        // footprints derive + export (planner overlay); P8-C swaps the
        // skeleton construction itself.
        mp.skeleton      = g.value("skeleton", std::string());
        mp.footprintCell = g.value("footprint_cell", 80.0);
        mp.footprintWobble = g.value("footprint_wobble", 0.12);
        mp.districtLen   = g.value("district_len", 1500.0);
        mp.gateSpacing   = g.value("gate_spacing", 1100.0);
        mp.rimRoad       = g.value("rim_road", true);
        mp.spineRoad     = g.value("spine", true);
        mp.skeletonSway  = g.value("skeleton_sway", 0.05);
        mp.arterialSpan  = g.value("arterial_span", 0.0);
        mp.stopAfter     = g.value("stop_after", std::string());
        // "backbone": "arterial" keeps the hub-to-hub spine a street (a
        // no-freeway metro); the historical default stays Freeway-class.
        if (g.value("backbone", std::string("freeway")) == std::string("arterial"))
            mp.backboneClass = RoadClass::Arterial;
        // Multi-site metros (8km-city plan P2): sites[0] is the city, later
        // entries are satellite towns; the backbone MST spans them all.
        if (g.contains("sites") && g["sites"].is_array()) {
            for (const auto& sj : g["sites"]) {
                MetroSite ms;
                if (sj.contains("center")) {
                    const json& c = sj["center"];
                    ms.center = Vec2(c.value("x", 0.0), c.value("z", 0.0));
                }
                ms.radius    = sj.value("radius", ms.radius);
                ms.hotspots  = sj.value("hotspots", ms.hotspots);
                ms.blockSize = sj.value("block_size", 0.0);
                ms.density   = sj.value("density", 1.0);
                const std::string bias = sj.value("kind_bias", std::string());
                ms.kindBias = bias == "financial"   ? 0
                            : bias == "commercial"  ? 1
                            : bias == "residential" ? 2
                            : bias == "oldtown"     ? 3
                            : bias == "industrial"  ? 4
                                                    : -1;
                mp.sites.push_back(ms);
            }
        }
        mp.outHubs = &road.plan.cityHubs;   // polycentric zoning reads these (city_lots)
        mp.outFootprints = &road.plan.siteFootprints;   // planner overlay + hand-edit
        // Terrain-aware layout: when the road is draped on terrain, gate the city
        // on buildability so it hugs buildable land and avoids water / steep
        // mountain instead of marching over them. Opt out with terrain_aware:false.
        if (heightAt && g.value("terrain_aware", true)) {
            mp.ground = heightAt;
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
        base = buildMetro(mp, &road.plan.freewayPlans);
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
        dp.arteryWidth  = g.value("artery_width", road.look.defaultWidth * 1.6);
        dp.streetWidth  = g.value("street_width", road.look.defaultWidth);
        // Downtown blocks are bigger (see DistrictParams): `core_block_scale`
        // multiplies the block size at the centre and eases out to 1.0 at
        // `core_radius` (default: 45% of the footprint). 1.0 = uniform, as before.
        dp.coreBlockScale = g.value("core_block_scale", 1.0);
        dp.coreRadius     = g.value("core_radius", 0.0);
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
        // BIG-BLOCK JUNCTION SPACING (8km-city plan P2): min_road_len is stub
        // cleanup and must stay under the colonization step (an arterial is a
        // chain of curve segments — raw edge length is sampling, not junction
        // spacing). min_block_edge owns intersection spacing: the fabric fill
        // floors its cells on it (with headroom, metro.cpp) and the span
        // consolidation fuses/deletes/lengthens junction-to-junction spans
        // under it. Double pass with a re-planarize between, exactly the
        // sequence the standalone probe validated.
        {
            const double minBlockEdge = g.value("min_block_edge", 0.0);
            if (minBlockEdge > 0.0) {
                // Two rounds to a fixpoint: consolidation/merge move junctions
                // (which can mint fresh folds) and relax shortens arcs (which
                // can dip a span under the floor) — one pass of each leaves
                // the other's debris. relaxSharpBends only moves degree-2
                // curve nodes, junctions stay pinned, so the face subdivision
                // survives (the reason the district cleanup was skipped
                // here); 0.5 rad per node ≈ a >=60 m turn radius at the
                // colonization step. dropParallelEdges first: a parallel pair
                // is an un-relaxable 180-degree fold. NO FOLDBACKS is the
                // contract (device: curved roads bending back on themselves
                // break the swept geometry). Gated on minBlockEdge so shipped
                // small-metro levels keep their exact geometry.
                const double minLen = g.value("min_road_len", 10.0);
                // REGION-AWARE floor (P7 density unlock): inside the primary
                // city site the span floor drops to just under the fabric
                // core block edge so dense downtown fabric survives the tail;
                // the periphery keeps big-block spacing. Flat without a core.
                const std::string fab = g.value("fabric", std::string());
                const double coreLen =
                    g.value("fabric_core_len", fab.empty() ? 0.0 : 110.0);
                Vec2 coreC(g.contains("center") ? g["center"].value("x", 0.0) : 0.0,
                           g.contains("center") ? g["center"].value("z", 0.0) : 0.0);
                double coreR = g.value("radius", 700.0);
                if (g.contains("sites") && g["sites"].is_array() &&
                    !g["sites"].empty()) {
                    const auto& s0 = g["sites"][0];
                    if (s0.contains("center"))
                        coreC = Vec2(s0["center"].value("x", 0.0),
                                     s0["center"].value("z", 0.0));
                    coreR = s0.value("radius", coreR);
                }
                auto spanFloor = [coreLen, coreC, coreR,
                                  minBlockEdge](const Vec2& q) -> Real {
                    if (coreLen <= 0) return minBlockEdge;
                    const bool inCore = std::fabs(q.x - coreC.x) <= coreR &&
                                        std::fabs(q.y - coreC.y) <= coreR;
                    return inCore ? std::min(minBlockEdge, coreLen * 0.95)
                                  : minBlockEdge;
                };
                // REALIGN acute junctions (docs/curb-weld-analysis.md). Device:
                // "the sight lines are important and also taking sharp turns
                // around acute angles means your car will tip over. So it's a
                // no-go from a design point of view." dissolveAcuteArms above
                // only reaches pairs under ~32 deg, and answers by DELETING an
                // arm — which is why the measured floor was 35 deg with a third
                // of junctions still under 60. This opens the 32-60 band the way
                // a designer does: bend the approach, pin the far node, keep a
                // straight run in. It runs INSIDE the round loop, before
                // planarize and relaxSharpBends, so a bend that grazes another
                // street gets noded and any kink it leaves gets eased in the
                // same round. 0 opts a level out.
                const double minArmDeg = g.value("min_arm_angle_deg", 60.0);
                const double minArmRad = minArmDeg * 3.14159265358979323846 / 180.0;
                // How far out the bend sits. This is the whole quality knob:
                // the same realignment at 26 m put 56 through-nodes under a 30 m
                // turn radius (a hairpin mid-block — the tipping problem moved
                // rather than solved), at 40 m that fell to 8, and at 60 m to 2,
                // which is the untouched city's own figure. Capped at 40% of the
                // first edge, so a short arm still gets a proportionate bend.
                const double runIn = g.value("realign_run_in", 60.0);
                // CULL threshold, and it is SCALE-DEPENDENT — which is why the
                // default is the conservative one. At 45 deg a city-scale net
                // (metro_v2, 1075 m radius) loses five junctions and in exchange
                // every ring including the rim clears 60 deg, with the tightest
                // curve anywhere IMPROVING on the untouched city (28.9 -> 32.7 m).
                // The same 45 deg on a 250 m test site strips the street fabric
                // to nothing: measured, 0 Local/Collector edges survive, so the
                // city has no parking, no frontage and no blocks. A small net's
                // few streets are near-parallel to each other by construction.
                // So: 32 here, and a big level opts in per recipe once someone
                // has measured it on that level.
                const double dissolveRad = g.value("dissolve_acute_deg", 32.0) *
                                           3.14159265358979323846 / 180.0;
                for (int round = 0; round < 3; ++round) {
                    cg = dropParallelEdges(cg);
                    // CULL: drop the redundant twin of a near-parallel pair when
                    // the far end stays reachable. 0.85 = cos(32 deg) — the
                    // measured floor of the untouched city, which is exactly what
                    // this threshold was setting. Exposed so the cull and the
                    // bend can be traded off against each other with numbers.
                    cg = dissolveAcuteArms(cg, std::cos(dissolveRad), 3);
                    cg = consolidateJunctionSpans(cg, spanFloor, rules.maxDegree);
                    cg = capDegree(planarize(cg, 1.0), rules);
                    cg = mergeShortEdges(cg, minLen, rules.maxDegree);
                    cg = relaxSharpBends(cg, 0.5, 64);
                }
                // LAST, not inside the loop: consolidation and span-merging move
                // junctions, so a realignment done before them is partly undone
                // by them (measured: every round re-found ~110 tight junctions).
                // Bends are validated against crossings, so nothing downstream
                // needs to re-node them.
                if (minArmDeg > 0.0) {
                    cg = realignAcuteJunctions(cg, minArmRad, runIn, 6);
                    cg = relaxSharpBends(cg, 0.5, 64);
                }
            }
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
    // Carry the generator's junction policy onto the look so the mesh + conform
    // passes don't re-promote roundabouts it deliberately disabled (ADR-0075 P0).
    road.look.autoRoundabout = rules.autoRoundabout;
    // The generator's graph BECOMES the entity's graph — same node and edge
    // ORDER (the determinism gate, test_road_graph_order, rides on this).
    // NORMALIZED, not copied raw: the parallel-array era kept only
    // {pos, a, b, width, klass, layer} and dropped every other field
    // (elevations, kinds, tangents, specs, baked flags) — reproduce that
    // truncation exactly, or stale generator side-state changes the city.
    road.graph.nodes.clear();
    road.graph.edges.clear();
    road.graph.nodes.reserve(cg.nodes.size());
    for (const RoadNode& n : cg.nodes) road.graph.nodes.push_back(RoadNode{n.pos});
    road.graph.edges.reserve(cg.edges.size());
    for (const RoadEdge& e : cg.edges) {
        RoadEdge oe;
        oe.a = e.a; oe.b = e.b;
        oe.width = e.width;          // arterials wider than local streets
        oe.klass = e.klass;          // carry the grown class (P1 unification)
        oe.layer = e.layer;          // and its grade-separation tier
        road.graph.edges.push_back(oe);
    }
    // --- REAL CROSS-SECTIONS FOR GENERATED STREETS (the parking round) -------
    // Glenn: "the road doesn't really have any clearance for parking — it's half
    // on the sidewalk". It was: a generated street had no spec, so roadNetEdgeSpec
    // fell back to roadSpecFromLegacy, which splits the WHOLE carriageway into
    // travel lanes. With no Parking band to aim at, the sim parked cars at a
    // hardcoded inset that hung over the kerb. Now every frontage street/collector
    // gets an authored section — sidewalk | curb | PARKING | travel | travel |
    // PARKING | curb | sidewalk — and the bays are placed FROM it.
    //
    // WIDTH AGREEMENT (the correctness crux): lots (city_lots' roadSurfaceDist /
    // pushPolyClearOfRoads), nav lane spacing and the mesher all read the ONE
    // RoadEdge::width. roadSpecStreetParking carves its bands out of the width
    // it is GIVEN, and we write carriagewayWidth() straight back into the edge
    // — so the drawn road can never grow wider than the width buildings keep
    // clear of. A road too narrow to carry parking gets a spec with no Parking
    // band (and, again, its exact original width).
    // Arterials/boulevards/freeways/ramps are left specless: today's look.
    if (kind == "metro" && g.value("street_parking", true)) {
        const double parkW = g.value("parking_width", 2.5);
        const double minLane = g.value("min_lane_width", 3.2);
        road.graph.specs.clear();
        // One spec per distinct (class, width) — a handful of entries, not one
        // per edge, so the table stays inspectable and JSON-friendly.
        std::vector<std::pair<RoadClass, double>> key;
        for (RoadEdge& e : road.graph.edges) {
            const RoadClass k = e.klass;
            if (k != RoadClass::Local && k != RoadClass::Collector) continue;
            if (e.layer != 0) continue;              // a deck/bridge has no frontage
            const double w = e.width;
            int si = -1;
            for (std::size_t t = 0; t < key.size(); ++t)
                if (key[t].first == k && std::fabs(key[t].second - w) < 1e-9) {
                    si = static_cast<int>(t);
                    break;
                }
            if (si < 0) {
                si = static_cast<int>(key.size());
                key.push_back({k, w});
                road.graph.specs.push_back(roadSpecStreetParking(
                    w, std::max(1, lanesForClass(k, /*perDirection=*/true)),
                    road.look.sidewalk, road.look.curb > 0.0 ? 0.25 : 0.0, parkW,
                    minLane));
            }
            e.spec = si;
            // Re-assert the ONE width every consumer reads. Equal by
            // construction; written back so it can never silently drift.
            e.width = static_cast<Real>(road.graph.specs[si].carriagewayWidth());
        }
    }
}

// Diagnostic accessors (RT_POKE_REPORT): the poke instrument must read the
// EXACT chain decomposition + constrained graph the mesher builds from — a
// near-miss decomposition gave it deck heights that drifted from the real road
// on hills and poisoned the LOD0 numbers (plan P3.2 round 6).
RoadGraph roadNetConstrainedGraph(const RoadEntity& road, const RoadGroundFn& heightAt) {
    return constrainedGraph(road, heightAt);
}
std::vector<UnionSpine> roadNetWeldSpines(const RoadGraph& g) { return weldChainSpines(g); }

json roadRecipeForSave(const std::string& currentRecipe, const RoadEntity& road) {
    json recipe = json::parse(currentRecipe, nullptr, false);
    if (!recipe.is_object() || !recipe.contains("generate"))
        return roadNetToJson(road);                 // hand-authored: the graph IS the saved form
    // Generated road: keep the "generate" block and refresh only the look — never bake
    // the nodes (baking froze the city and lost the recipe, the grown.json "save changed" bug).
    json look = roadNetToJson(road);
    for (const char* k : {"nodes", "edges", "edge_layers", "tangents"}) look.erase(k);
    look["generate"] = recipe["generate"];
    return look;
}

}  // namespace engine
