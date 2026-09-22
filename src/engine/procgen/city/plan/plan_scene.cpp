#include "plan_scene.h"

#include "../roads/lanes/interchange.h"   // ONE ramp generator, shared with the level importer
#include "../roads/lanes/polyline_ops.h"   // stations/pointAt: sampling along the route
#include "../roads/lanes/road_graph_spec.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

namespace engine {
namespace plan {
namespace {

using json = nlohmann::json;

const char* className(RoadClass k) {
    switch (k) {
        case RoadClass::Freeway: return "freeway";
        case RoadClass::Arterial: return "arterial";
        case RoadClass::Collector: return "collector";
        default: return "local";
    }
}

Vec2 leftOf(const Vec2& d) { return Vec2(-d.y, d.x); }

using Chain = PlanChain;   // chainsOf(graph) is the plan's own: one road, not forty edges

// A polyline pushed `d` to its left, mitred at the joints: the two carriageways of
// a divided road, which is what makes the freeway a wall traffic must use a ramp to
// cross rather than a painted line drivers turn across.
std::vector<Vec2> offsetPolyline(const std::vector<Vec2>& p, Real d) {
    std::vector<Vec2> out;
    out.reserve(p.size());
    for (std::size_t i = 0; i < p.size(); ++i) {
        Vec2 nPrev, nNext;
        bool hasPrev = false, hasNext = false;
        if (i > 0) { nPrev = leftOf(normalize(p[i] - p[i - 1])); hasPrev = true; }
        if (i + 1 < p.size()) { nNext = leftOf(normalize(p[i + 1] - p[i])); hasNext = true; }
        Vec2 n = hasPrev && hasNext ? normalize(nPrev + nNext) : (hasPrev ? nPrev : nNext);
        Real scale = 1;
        if (hasPrev && hasNext) {
            const Real c = dot(n, nNext);
            scale = c > 0.3 ? 1 / c : 1 / Real(0.3);   // clamp the mitre on a hairpin
        }
        out.push_back(p[i] + n * (d * scale));
    }
    return out;
}

json pointsJson(const std::vector<Vec2>& pts) {
    json a = json::array();
    for (const Vec2& p : pts) a.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100});
    return a;
}

}  // namespace

nlohmann::json planToLanesScene(const CityPlan& plan, const SceneOptions& opt) {
    const Brief& b = plan.brief;
    json scene;
    scene["name"] = b.name;
    // The map, with room for the sidewalks outside the outermost road.
    const Real half = b.size * Real(0.5) + 60;
    scene["terrain"] = {{"type", "flat"}, {"bounds", {b.center.x - half, b.center.x + half, b.center.y - half, b.center.y + half}}};
    // Lane counts and widths that add up to the brief's carriageway widths.
    scene["classes"] = {
        {"local", {{"w", b.localWidth / 2}, {"fwd", 1}, {"back", 1}, {"sidewalk", b.sidewalk}, {"g_max", 0.12}, {"rank", 1}, {"thick", 0.5}, {"window", 40}}},
        {"collector", {{"w", b.collectorWidth / 4}, {"fwd", 2}, {"back", 2}, {"sidewalk", b.sidewalk}, {"g_max", 0.1}, {"rank", 1}, {"thick", 0.5}, {"window", 80}}},
        {"arterial", {{"w", b.arterialWidth / 6}, {"fwd", 3}, {"back", 3}, {"sidewalk", b.sidewalk}, {"g_max", 0.08}, {"rank", 2}, {"thick", 0.6}, {"window", 120}}},
        {"freeway", {{"w", b.freewayWidth / 8}, {"fwd", 4}, {"back", 0}, {"shoulder", 2.5}, {"sidewalk", 0.0}, {"g_max", 0.06}, {"rank", 3}, {"thick", 1.2}, {"window", 200.0}}},
        {"ramp", {{"w", 4.5}, {"fwd", 1}, {"back", 0}, {"shoulder", 2.5}, {"g_max", 0.08}, {"rank", 0}, {"thick", 1.0}, {"window", 40}}},
    };
    scene["rules"] = {{"closing", 3.0}};

    json edges = json::array();
    std::vector<std::string> streetIds;
    int id = 0;
    for (const Chain& c : chainsOf(plan.streets)) {
        streetIds.push_back("s" + std::to_string(id++));
        edges.push_back({{"id", streetIds.back()}, {"class", className(c.klass)}, {"path", {{"points", pointsJson(c.pts)}}}});
    }
    // The freeway: two one-way carriageways either side of its centreline, riding OVER the
    // streets it crosses (`floor` holds the deck clear above the ground there), with diamond
    // interchanges onto the streets beside it. Without those it is a wall with no way on —
    // which is what the first planned city was.
    const std::vector<PlanChain> streetChains = chainsOf(plan.streets);
    std::vector<roads::lanes::RampStreet> rampStreets;
    rampStreets.reserve(streetChains.size());
    for (std::size_t i = 0; i < streetChains.size() && i < streetIds.size(); ++i)
        rampStreets.push_back({streetIds[i], streetChains[i].pts});
    int route = 0;
    for (const Chain& c : chainsOf(plan.freeway)) {
        const std::string a = "fw" + std::to_string(route) + "_a", d = "fw" + std::to_string(route) + "_b";
        std::vector<Vec2> back = offsetPolyline(c.pts, -opt.carriagewayGap);
        std::reverse(back.begin(), back.end());
        // AN ELEVATED RING. Two designs failed before this one: holding the freeway's deck
        // clear only where it crosses a street put 1.4 km of a 2.1 km ring on piers anyway
        // (at 6% it cannot come back down between crossings 350 m apart), and making the
        // STREET climb instead asked a 25 m chain — cut that short by the frontage roads
        // either side — to gain 8 m, which is a 24% ramp. On a flat city the honest answer
        // is the one real cities build: the motorway runs above the streets for its whole
        // length, every street passes under it, and the ramps come down to the frontage
        // roads beside it.
        json floors = json::array();
        {
            const std::vector<double> st = roads::lanes::stations(c.pts);
            for (double s0 = 0; s0 < st.back(); s0 += 60.0) {
                const Vec2 q = roads::lanes::pointAt(c.pts, st, s0);
                floors.push_back(json::array({std::round(q.x * 100) / 100, std::round(q.y * 100) / 100,
                                              opt.clearance, 60.0}));
            }
        }
        edges.push_back({{"id", a}, {"class", "freeway"}, {"floor", floors},
                         {"path", {{"points", pointsJson(offsetPolyline(c.pts, opt.carriagewayGap))}}}});
        edges.push_back({{"id", d}, {"class", "freeway"}, {"floor", floors},
                         {"path", {{"points", pointsJson(back)}}}});
        if (opt.ramps) {
            roads::lanes::DiamondOptions dop;
            dop.aId = a;
            dop.bId = d;
            dop.idPrefix = "d" + std::to_string(route);
            dop.carriage = opt.carriagewayGap;
            dop.edgeReach = b.freewayWidth / 2 + 2.5;
            dop.maxDiamonds = opt.diamondsPerRoute;
            dop.spacing = opt.interchangeSpacing;
            dop.clearance = opt.clearance;
            const roads::lanes::DiamondResult dr = roads::lanes::diamondRamps(c.pts, rampStreets, dop);
            for (const json& r : dr.ramps) edges.push_back(r);
        }
        ++route;
    }
    scene["edges"] = std::move(edges);
    return scene;
}

}  // namespace plan
}  // namespace engine
