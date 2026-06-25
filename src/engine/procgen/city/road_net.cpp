#include "road_net.h"

#include "road_network.h"       // RoadGraph, RoadEdge
#include "road_constraints.h"   // applyConstraints (min-arm-angle + roundabout promotion)
#include <algorithm>
#include <cmath>

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

// The road's graph for the mesher: straight edges, or — when `curved` — each edge
// sampled as a Hermite cubic through its endpoints' tangents (Catmull-Rom auto on a
// degree-2 through-road, a straight chord into a junction/dead-end). The original
// nodes keep their indices (so junction degree is preserved); curve samples append.
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

    RoadGraph g;
    g.nodes.resize(n);
    for (int i = 0; i < n; ++i) g.nodes[i].pos = net.nodes[i];

    if (!net.curved) {
        for (int ei : ev) {
            const std::array<int, 2>& e = net.edges[ei];
            g.edges.push_back(RoadEdge{e[0], e[1], ewidth(ei), RoadClass::Local});
        }
        return g;
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

        int prev = a;
        for (std::size_t s = 1; s + 1 < poly.size(); ++s) {     // interior -> new nodes
            int idx = static_cast<int>(g.nodes.size());
            g.nodes.push_back(RoadNode{poly[s]});
            g.edges.push_back(RoadEdge{prev, idx, w, RoadClass::Local});
            prev = idx;
        }
        g.edges.push_back(RoadEdge{prev, b, w, RoadClass::Local});   // last -> shared node b
    }
    return g;
}

}  // namespace

RenderMesh buildRoadNetMesh(const RoadNet& net) {
    // Cap centerline curvature so neither the carriageway nor the sidewalk outer rail can
    // fold: keep the turn radius above the widest offset (half-width + sidewalk) + margin.
    double minTurnRadius = net.width * 0.5 + net.sidewalk + 0.5;
    RoadGraph g = netGraph(net, minTurnRadius);
    // Local-constraints pass (ADR-0052): a node with too many arms — or two arms too
    // acute to share a flat junction — is promoted to a roundabout, so a many-spoke hub
    // opens into a ring instead of overlapping itself (ADR-0044 trim divergence).
    g = applyConstraints(g);
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

std::vector<TerrainFlatten> roadNetConformRegions(const RoadNet& net, double shoulder,
                                                  double falloff, double maxGrade) {
    std::vector<TerrainFlatten> out;
    if (!net.heightAt) return out;                       // flat road: nothing to carve
    double minR = net.width * 0.5 + net.sidewalk + 0.5;
    RoadGraph g = netGraph(net, minR);
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
    net.curved = true;          // a shaped tangent implies the spline is shown
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
    net.curved = j.value("curved", net.curved);
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
    j["curved"] = net.curved;
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

}  // namespace engine
