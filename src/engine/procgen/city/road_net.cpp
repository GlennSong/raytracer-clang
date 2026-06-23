#include "road_net.h"

#include "road_network.h"       // RoadGraph, RoadEdge
#include <algorithm>
#include <cmath>

namespace engine {

using json = nlohmann::json;

namespace {

bool isZero(const Vec2& v) { return v.x == 0.0 && v.y == 0.0; }

Vec2 hermite(const Vec2& p0, const Vec2& m0, const Vec2& p1, const Vec2& m1, double t) {
    double t2 = t * t, t3 = t2 * t;
    double h00 = 2 * t3 - 3 * t2 + 1, h10 = t3 - 2 * t2 + t;
    double h01 = -2 * t3 + 3 * t2, h11 = t3 - t2;
    return p0 * h00 + m0 * h10 + p1 * h01 + m1 * h11;
}

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
RoadGraph netGraph(const RoadNet& net) {
    const int n = static_cast<int>(net.nodes.size());
    std::vector<std::array<int, 2>> edges = validEdges(net);
    auto P = [&](int i) { return net.nodes[i]; };
    auto width = static_cast<Real>(net.width);

    RoadGraph g;
    g.nodes.resize(n);
    for (int i = 0; i < n; ++i) g.nodes[i].pos = net.nodes[i];

    if (!net.curved) {
        for (const std::array<int, 2>& e : edges)
            g.edges.push_back(RoadEdge{e[0], e[1], width, RoadClass::Local});
        return g;
    }

    // Per-node neighbours (for degree + the Catmull-Rom "other" neighbour).
    std::vector<std::vector<int>> nbr(n);
    for (const std::array<int, 2>& e : edges) { nbr[e[0]].push_back(e[1]); nbr[e[1]].push_back(e[0]); }
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

    for (const std::array<int, 2>& e : edges) {
        int a = e[0], b = e[1];
        Vec2 m0 = outTan(a, b), m1 = inTan(b, a);
        double len = (P(b) - P(a)).length();
        int segs = std::max(4, static_cast<int>(std::ceil(len / 5.0)));
        int prev = a;
        for (int s = 1; s < segs; ++s) {
            Vec2 p = hermite(P(a), m0, P(b), m1, static_cast<double>(s) / segs);
            int idx = static_cast<int>(g.nodes.size());
            g.nodes.push_back(RoadNode{p});
            g.edges.push_back(RoadEdge{prev, idx, width, RoadClass::Local});
            prev = idx;
        }
        g.edges.push_back(RoadEdge{prev, b, width, RoadClass::Local});
    }
    return g;
}

}  // namespace

RenderMesh buildRoadNetMesh(const RoadNet& net) {
    RoadGraph g = netGraph(net);
    RoadMeshParams p;
    p.lift = net.lift;
    p.color = net.color;
    p.sidewalkWidth = net.sidewalk;
    p.curbHeight = net.curb;
    p.cornerRadius = net.cornerRadius;
    p.laneMarkings = net.markings;
    p.crosswalks = net.crosswalks;
    p.minSetback = net.width * 0.5 + 0.5;        // pad clears the curb corners
    p.heightAt = net.heightAt;
    return buildRoadMesh(g, p);
}

void roadNetSetWidth(RoadNet& net, double width) {
    net.width = std::max(0.5, width);
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

RoadNet roadNetFromJson(const json& j) {
    RoadNet net;
    if (j.contains("nodes") && j["nodes"].is_array())
        for (const json& p : j["nodes"])
            net.nodes.push_back(Vec2(p.value("x", 0.0), p.value("z", 0.0)));
    if (j.contains("edges") && j["edges"].is_array())
        for (const json& e : j["edges"]) {
            if (e.is_array() && e.size() >= 2)
                net.edges.push_back({e[0].get<int>(), e[1].get<int>()});
            else if (e.is_object())
                net.edges.push_back({e.value("a", 0), e.value("b", 0)});
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
    for (const std::array<int, 2>& e : net.edges) edges.push_back(json::array({e[0], e[1]}));
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
