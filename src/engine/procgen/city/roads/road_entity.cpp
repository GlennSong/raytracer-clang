// The road entity: the plan, not the pavement (roads module, ADR-0089).
//
// What a road IS — its control graph, the
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
            ge.oneWay = e.oneWay;                // a carriageway stays one-way (a -> b is travel)
            ge.walkable = ewalk(ei);
            ge.parkOffset = parkOff; ge.parkWidth = parkW;
            g.edges.push_back(ge);
            prev = idx;
        }
        RoadEdge geLast{prev, b, w, kls, lay};
        geLast.spec = e.spec;
        geLast.oneWay = e.oneWay;
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

// One UnionSpine per CHAIN (a maximal degree-2 run between junctions/dead-ends), carrying that
// road's WIDTH — so the weld gets smooth per-road ribbons (no per-edge spikes) AND the right width
// (arterials stay wider than streets, which a width-less chain trace loses). A pure cycle with no
// junction (a bare roundabout ring) comes back marked closed, so weldSolid makes it an annulus.
namespace roadnet {

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


}  // namespace engine
