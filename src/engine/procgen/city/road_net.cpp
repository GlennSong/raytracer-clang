#include "road_net.h"

#include "road_network.h"       // RoadGraph, RoadEdge
#include "road_constraints.h"   // applyConstraints, capDegree, RoadRules
#include "road_rules.h"         // DesignRules (clearance, deck thickness, ramp grade)
#include "district.h"           // DistrictParams, buildDistrict (generate recipe)
#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace engine {

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

    RoadGraph g;
    g.nodes.resize(n);
    for (int i = 0; i < n; ++i) g.nodes[i].pos = net.nodes[i];

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
        // Adaptive tessellation: an edge that never leaves its chord (a straight run — the grid's
        // junction-to-junction edges, whose tangents ARE the chord) collapses back to ONE segment.
        // Densifying straight edges into len/5 collinear samples was the real cause of the overlap and
        // terrain gaps at junctions (road-network-v2-plan T3.2); a genuine bend keeps its samples.
        {
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

        int lay = elayer(ei);
        int prev = a;
        for (std::size_t s = 1; s + 1 < poly.size(); ++s) {     // interior -> new nodes
            int idx = static_cast<int>(g.nodes.size());
            g.nodes.push_back(RoadNode{poly[s]});
            g.edges.push_back(RoadEdge{prev, idx, w, RoadClass::Local, lay});
            prev = idx;
        }
        g.edges.push_back(RoadEdge{prev, b, w, RoadClass::Local, lay});   // last -> shared node b
    }
    return g;
}

// The graph the mesher AND the terrain-conform both build from: the sampled net graph put
// through the local-constraints pass (ADR-0052), so a promoted roundabout is reflected
// identically in the carriageway and in the ground it grades. One source keeps them in sync.
RoadGraph constrainedNetGraph(const RoadNet& net) {
    double minR = net.width * 0.5 + net.sidewalk + 0.5;
    return applyConstraints(netGraph(net, minR));
}

// Append `src` triangles into `dst`, offsetting indices.
void appendMesh(RenderMesh& dst, const RenderMesh& src) {
    uint32_t base = static_cast<uint32_t>(dst.vertices.size());
    dst.vertices.insert(dst.vertices.end(), src.vertices.begin(), src.vertices.end());
    for (uint32_t idx : src.indices) dst.indices.push_back(base + idx);
}

// Proper interior crossing of segments p1p2 and p3p4 (shared endpoints excluded); on a hit
// `t` is the parameter along p1p2 (so the crossing's arc position can be interpolated).
bool segCrossAt(const Vec2& p1, const Vec2& p2, const Vec2& p3, const Vec2& p4, double& t) {
    Vec2 d1 = p2 - p1, d2 = p4 - p3;
    double den = cross(d1, d2);
    if (std::fabs(den) < 1e-9) return false;
    Vec2 d13 = p3 - p1;
    t = cross(d13, d2) / den;
    double u = cross(d13, d1) / den;
    // t in (0, 1]: include the far endpoint so a crossing that lands exactly on a sample
    // vertex is caught once (by the segment ending there), not missed by both neighbours.
    // u strictly interior: the bridge must cross the ground road's span, not its endpoint.
    return t > 1e-9 && t <= 1.0 + 1e-9 && u > 1e-9 && u < 1.0 - 1e-9;
}

// Build a road graph that carries grade separations (ADR-0051/0054). Ground edges (layer 0)
// are the normal flat road surface; each higher-layer chain is a BRIDGE — its centerline is
// lifted onto a FLAT deck (a level span over the roads it crosses, with ramps down at grade
// to either side), carried on abutment piers, the way a real overpass sits. The visible
// overpass: a flat structure on supports, not an inclined hump.
RenderMesh buildLayeredRoadNetMesh(const RoadNet& net, const RoadGraph& g) {
    const DesignRules& rules = defaultDesign();          // one source for the grade-sep numbers
    const double clearance = rules.clearance, deckThk = rules.deckThickness, rampGrade = rules.rampGrade;
    const double groundSurf = net.lift;                 // flat road surface height (approx)
    double hw = net.width * 0.5;
    auto groundFn = [&](double x, double z) { return net.heightAt ? net.heightAt(x, z) : 0.0; };

    int maxLayer = 0;
    for (const RoadEdge& e : g.edges) maxLayer = std::max(maxLayer, e.layer);

    // Ground roads (layer 0): the existing analytic surface, with multilane markings baked as
    // geometry (lane count from width) so the road under the bridge reads as multilane.
    RoadGraph ground;
    ground.nodes = g.nodes;
    for (const RoadEdge& e : g.edges) if (e.layer == 0) ground.edges.push_back(e);
    RoadMeshParams p;
    p.lift = net.lift; p.color = net.color; p.sidewalkWidth = net.sidewalk;
    p.curbHeight = net.curb; p.cornerRadius = net.cornerRadius;
    p.laneMarkings = net.markings; p.shaderMarkings = false;
    p.crosswalks = false; p.minSetback = net.width * 0.5 + 0.5; p.heightAt = net.heightAt;
    RenderMesh mesh = buildRoadMesh(ground, p);

    // Each upper layer: trace its chains and lift each onto a flat clearing deck + piers.
    for (int L = 1; L <= maxLayer; ++L) {
        RoadGraph up;
        up.nodes = g.nodes;
        for (const RoadEdge& e : g.edges) if (e.layer == L) up.edges.push_back(e);
        for (const std::vector<Vec2>& chain : traceChains(up)) {
            if (chain.size() < 2) continue;
            // Densify so the ramps and the flat span have enough samples.
            const double step = 3.0;
            std::vector<Vec2> dense;
            for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
                Vec2 a = chain[i], b = chain[i + 1];
                int sub = std::max(1, static_cast<int>(std::ceil((b - a).length() / step)));
                for (int k = 0; k < sub; ++k) dense.push_back(a + (b - a) * (double(k) / sub));
            }
            dense.push_back(chain.back());
            int n = static_cast<int>(dense.size());
            std::vector<double> s(n), minH(n);
            for (int i = 0; i < n; ++i) minH[i] = groundFn(dense[i].x, dense[i].y) + groundSurf;
            s[0] = 0.0;
            for (int i = 1; i < n; ++i) s[i] = s[i - 1] + (dense[i] - dense[i - 1]).length();
            // For each ground road crossed, hold the deck FLAT over a span window centred on the
            // crossing (lower-road half-width + an overhang), so the deck is level across the road
            // below and only ramps on the approaches. Record the window edges for pier placement.
            std::vector<double> support;
            for (int i = 0; i + 1 < n; ++i)
                for (const RoadEdge& e : ground.edges) {
                    double t;
                    if (!segCrossAt(dense[i], dense[i + 1], g.nodes[e.a].pos, g.nodes[e.b].pos, t))
                        continue;
                    double sCross = s[i] + t * (s[i + 1] - s[i]);
                    double spanHalf = e.width * 0.5 + 5.0;
                    double H = groundFn(dense[i].x, dense[i].y) + groundSurf + clearance + deckThk;
                    for (int k = 0; k < n; ++k)
                        if (std::fabs(s[k] - sCross) <= spanHalf) minH[k] = std::max(minH[k], H);
                    support.push_back(sCross - spanHalf);
                    support.push_back(sCross + spanHalf);
                }
            std::vector<double> deckY = clearanceProfile(s, minH, rampGrade);
            appendMesh(mesh, bridgeDeck(dense, deckY, hw, net.color, deckThk));
            // Abutment piers at the span edges (just outside the road below, where the flat deck
            // meets the ramps) — the nearest sample to each recorded window edge.
            std::vector<int> at;
            for (double sp : support) {
                int best = 0; double bd = 1e30;
                for (int k = 0; k < n; ++k) { double d = std::fabs(s[k] - sp); if (d < bd) { bd = d; best = k; } }
                at.push_back(best);
            }
            appendMesh(mesh, bridgePiers(dense, deckY, at, net.width, 3.0, deckThk,
                                         Vec3(0.34, 0.34, 0.36), net.heightAt));
        }
    }
    return mesh;
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
        s.closed = closed;
        s.points.push_back(g.nodes[v].pos);
        int prev = v, e = e0, startNode = v;
        while (!used[e]) {
            used[e] = 1;
            int nx = (g.edges[e].a == prev) ? g.edges[e].b : g.edges[e].a;
            if (closed && nx == startNode) break;
            s.points.push_back(g.nodes[nx].pos);
            if (!closed && breaksChain(nx)) break;
            int ne = -1;
            for (int ee : inc[nx]) if (ee != e && !used[ee]) { ne = ee; break; }
            if (ne < 0) break;
            prev = nx; e = ne;
        }
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

RenderMesh buildRoadNetMesh(const RoadNet& net) {
    // Cap centerline curvature so neither the carriageway nor the sidewalk outer rail can
    // fold: keep the turn radius above the widest offset (half-width + sidewalk) + margin.
    double minR = net.width * 0.5 + net.sidewalk + 0.5;
    RoadGraph raw = netGraph(net, minR);
    // Does the local-constraints pass (ADR-0052) promote any node to a roundabout?
    bool roundabout = false;
    for (int v = 0; v < static_cast<int>(raw.nodes.size()); ++v)
        if (nodeNeedsRoundabout(raw, v, {})) { roundabout = true; break; }
    RoadGraph g = applyConstraints(raw);   // promoted graph (= constrainedNetGraph)

    // Grade separations (ADR-0051/0054): if any edge is on a higher layer, the net carries an
    // overpass — build the ground roads flat and lift each bridge chain onto a clearing deck.
    for (const RoadEdge& e : g.edges)
        if (e.layer > 0) return buildLayeredRoadNetMesh(net, g);

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
    RoadGraph g = constrainedNetGraph(net);              // grade to the roundabout, not the raw spokes
    double hw = net.width * 0.5;
    for (const std::vector<Vec2>& chain : traceChains(g)) {
        if (chain.size() < 2) continue;
        // Densify to a fixed step so even a straight road (2 graph nodes) has enough
        // samples for roadProfile to smooth and grade-limit along its length.
        const double step = 4.0;
        std::vector<Vec2> dense;
        for (std::size_t i = 0; i + 1 < chain.size(); ++i) {
            Vec2 a = chain[i], b = chain[i + 1];
            int sub = std::max(1, static_cast<int>(std::ceil((b - a).length() / step)));
            for (int k = 0; k < sub; ++k) dense.push_back(a + (b - a) * (double(k) / sub));
        }
        dense.push_back(chain.back());
        int n = static_cast<int>(dense.size());
        std::vector<double> ground(n), s(n);
        for (int i = 0; i < n; ++i) ground[i] = net.heightAt(dense[i].x, dense[i].y);
        s[0] = 0.0;
        for (int i = 1; i < n; ++i) s[i] = s[i - 1] + (dense[i] - dense[i - 1]).length();
        std::vector<double> profile = roadProfile(ground, s, maxGrade);
        std::vector<TerrainFlatten> r = roadConformRegions(dense, profile, hw, shoulder, falloff);
        out.insert(out.end(), r.begin(), r.end());
    }
    return out;
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
    return ni;
}

bool roadNetDeleteNode(RoadNet& net, int i) {
    const int n = static_cast<int>(net.nodes.size());
    if (i < 0 || i >= n) return false;
    std::vector<std::array<int, 2>> kept;
    std::vector<double> keptW;
    const bool hasW = !net.edgeWidths.empty();
    for (int ei = 0; ei < static_cast<int>(net.edges.size()); ++ei) {
        const std::array<int, 2>& e = net.edges[ei];
        if (e[0] == i || e[1] == i) continue;                 // drop incident edges
        kept.push_back({ e[0] > i ? e[0] - 1 : e[0], e[1] > i ? e[1] - 1 : e[1] });
        if (hasW) keptW.push_back(ei < static_cast<int>(net.edgeWidths.size())
                                      ? net.edgeWidths[ei] : 0.0);
    }
    net.edges = std::move(kept);
    if (hasW) net.edgeWidths = std::move(keptW);              // stay parallel to edges
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
    if (j.contains("tangents") && j["tangents"].is_array())
        for (const json& t : j["tangents"]) {
            if (t.is_array() && t.size() >= 2)
                net.tangents.push_back(Vec2(t[0].get<double>(), t[1].get<double>()));
            else if (t.is_object())
                net.tangents.push_back(Vec2(t.value("x", 0.0), t.value("z", 0.0)));
        }
    net.width = j.value("width", net.width);
    net.sidewalk = j.value("sidewalk", net.sidewalk);
    net.curb = j.value("curb", net.curb);
    net.cornerRadius = j.value("corner_radius", net.cornerRadius);
    net.lift = j.value("lift", net.lift);
    net.markings = j.value("markings", net.markings);
    net.crosswalks = j.value("crosswalks", net.crosswalks);
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
    for (int pass = 0; pass < 6; ++pass) {
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

void applyGenerateRecipe(RoadNet& net, const json& g) {
    if (!g.is_object()) return;
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
    double curviness = g.value("curviness", 0.0);    // 0 = straight grid; >0 sweeps streets into curves
    DistrictNet d = buildDistrict(dp);
    // City-generation junction policy (road-network-v2-plan T1.1/T1.2): no auto roundabouts;
    // planarize every crossing into shared nodes, then cap degree to <=4 LAST — planarize itself can
    // lift a node past the cap when a street T's into it, so capping has to bind on the final noded
    // graph. The result is one connected, editable, degree-capped network.
    RoadRules rules;
    rules.autoRoundabout = false;
    RoadGraph cg = capDegree(planarize(applyConstraints(d.graph, rules), 1.0), rules);
    // Minimum road length (device: "really short roads ... should be merged"):
    // fold crossings that landed close together into one junction, or stretch a
    // stub that can't merge (a capDegree stagger link) out to a drivable length.
    // Runs BEFORE the warp so it sees real junction-to-junction edges, not the
    // curve samples warping introduces.
    const double minRoadLen = g.value("min_road_len", 14.0);
    if (minRoadLen > 0.0) cg = mergeShortEdges(cg, minRoadLen, rules.maxDegree);
    if (curviness > 0.0) cg = warpGraph(cg, curviness);   // domain-warp the grid into organic curves
    cg = deAcute(cg, 0.6);                                 // open up acute junctions so corners stay clean
    net.nodes.clear(); net.edges.clear(); net.edgeWidths.clear();
    for (const RoadNode& n : cg.nodes) net.nodes.push_back(n.pos);
    for (const RoadEdge& e : cg.edges) {
        net.edges.push_back({e.a, e.b});
        net.edgeWidths.push_back(e.width);          // arterials wider than local streets
    }
}

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
