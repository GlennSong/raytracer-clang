// The road entity: the plan, not the pavement (roads module, ADR-0089).
//
// What a road IS — its control graph, the "generate" recipe that grows one, the
// sampled + constrained graphs everything downstream routes on, the editor's
// edit ops and the level JSON — lives here, shared by every builder. What a road
// LOOKS like is a builder's business: the lattice mesher that shared this file
// until 2026-09-20 now sits in procgen/deprecated/roads, reached through
// roads::roadBuilder("lattice").

#include "road_entity.h"

#include "road_net_internal.h"
#include "../road_semantics.h"    // classifyRoadGraph (the semantic layer)
#include "../road_constraints.h"  // applyConstraints, capDegree, joinDanglingEnds
#include "../road_rules.h"        // DesignRules
#include "../district.h"          // DistrictParams, buildDistrict (generate recipe)
#include "../metro.h"             // MetroParams, buildMetro ("kind":"metro" recipe)
#include "../../../../log.h"
#include "../../../../profile.h"

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
namespace roadnet {
double netMinTurnRadius(const RoadEntity& road) {
    double maxW = road.look.defaultWidth;
    for (const RoadEdge& e : road.graph.edges) maxW = std::max(maxW, static_cast<double>(e.width));
    return maxW * 0.5 + road.look.sidewalk + 0.5;
}
}  // namespace roadnet
using namespace roadnet;


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
}  // namespace

namespace roadnet {

RoadGraph sampleNetGraph(const RoadEntity& road, double minTurnRadius) {
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

}  // namespace roadnet

// Roads-v2 S3b: strip the BAKED corridor edges (klass Freeway/Ramp) so the
// street mesher, terrain conform, and street nav see only streets — the
// corridor still draws/carves/navigates itself until S4-S6 unify. Without this
// a baked net double-meshes, double-carves, and double-counts nav edges (the
// corridor's carriageway chains are merged into LevelRoadGraph separately).
// Also keeps the curvature cap honest: netMinTurnRadius scans the edge widths,
// and a 27 m freeway width would inflate every street's minimum turn radius.
// Nodes are kept (indices stay stable); orphaned nodes emit no geometry.

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

namespace roadnet {

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

}  // namespace roadnet

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
namespace roadnet {

std::vector<UnionSpine> weldChainSpines(const RoadGraph& g) {
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
        // Where the travel lanes stop, from the kerbside PARKING band the edge
        // already carries (resolved from the spec so nav/sim never need the
        // table). The paint follows the road's real section from here on: a
        // lane divider belongs on a lane boundary, and the white edge line on
        // the parking line — not at a share of the whole carriageway.
        if (g.edges[e0].parkWidth > 0 && s.halfWidth > 1e-6) {
            const double travelEdge =
                g.edges[e0].parkOffset - g.edges[e0].parkWidth * 0.5;
            s.travelEdgeFrac = std::max(0.15, std::min(1.0, travelEdge / s.halfWidth));
        }
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

}  // namespace roadnet


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
    road.look.perClassGrade = j.value("per_class_grade", road.look.perClassGrade);
    // RT_PER_CLASS_GRADE=0|1 overrides the level for an A/B: the per-class
    // table's cost (pokes, cut/fill) measured against the single 8% on the
    // SAME level without editing it.
    if (const char* ov = std::getenv("RT_PER_CLASS_GRADE"))
        road.look.perClassGrade = std::atoi(ov) != 0;
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
            // Arterials/boulevards/freeways/ramps are left specless: today's
            // look. (Giving arterials kerbside parking needs 19 m to keep the
            // design table's 3.5 m lane — their 17 m is four lanes of 4.25 m —
            // and MEASURED on metro_v2 that costs 97 buildings, puts a signal
            // pole 0.15 m onto the carriageway, and takes the worst lot-over-
            // ground gap 0.97 -> 3.04 m. The city is tuned around 17 m
            // arterials; widening them is its own round, with the sidewalk
            // narrowed per class so the building line does not move.)
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
