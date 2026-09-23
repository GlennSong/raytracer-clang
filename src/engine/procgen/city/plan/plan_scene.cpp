#include "plan_scene.h"

#include "../roads/lanes/interchange.h"   // ONE ramp generator, shared with the level importer
#include "../roads/lanes/polyline_ops.h"   // stations/pointAt: sampling along the route
#include "../roads/lanes/road_graph_spec.h"
#include "../roads/lanes/terrain_recipe.h"   // makeTerrain: the same ground the builder will make

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

// The same, for a CLOSED polyline (the ring): every vertex mitred against its wrap-around
// neighbour, so chains cut from it share their end points exactly.
std::vector<Vec2> offsetClosed(const std::vector<Vec2>& p, Real d) {
    if (p.size() < 3) return offsetPolyline(p, d);
    std::vector<Vec2> ext;
    ext.reserve(p.size() + 2);
    ext.push_back(p.back());
    ext.insert(ext.end(), p.begin(), p.end());
    ext.push_back(p.front());
    std::vector<Vec2> o = offsetPolyline(ext, d);
    return std::vector<Vec2>(o.begin() + 1, o.end() - 1);
}

// The crossing of an open polyline with a closed one: the station along the open one, and the
// closed one's segment index. False if they do not cross.
bool crossingWithRing(const std::vector<Vec2>& open, const std::vector<Vec2>& ring, double& sOpen, std::size_t& ringSeg) {
    const std::vector<double> st = roads::lanes::stations(open);
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i + 1 < open.size(); ++i)
        for (std::size_t k = 0; k < n; ++k) {
            const Vec2 a = open[i], b = open[i + 1], c = ring[k], d = ring[(k + 1) % n];
            const Real den = cross(b - a, d - c);
            if (std::fabs(den) < 1e-12) continue;
            const Real u = cross(c - a, d - c) / den, v = cross(c - a, b - a) / den;
            if (u < 0 || u > 1 || v < 0 || v > 1) continue;
            sOpen = st[i] + (st[i + 1] - st[i]) * u;
            ringSeg = k;
            return true;
        }
    return false;
}

std::vector<Vec2> cubic(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, int n) {
    std::vector<Vec2> out;
    for (int i = 0; i <= n; ++i) {
        const Real t = Real(i) / n, u = 1 - t;
        out.push_back(p0 * (u * u * u) + p1 * (3 * u * u * t) + p2 * (3 * u * t * t) + p3 * (t * t * t));
    }
    return out;
}

json pointsJson(const std::vector<Vec2>& pts) {
    json a = json::array();
    for (const Vec2& p : pts) a.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100});
    return a;
}

}  // namespace

FreewaySection freewaySection(const Brief& b, Real medianGap) {
    FreewaySection f;
    f.laneW = b.freewayWidth / 8;
    f.lanes = 4;
    f.carriageHalf = f.lanes * f.laneW / 2 + Real(2.5);
    f.carriage = f.carriageHalf + medianGap / 2;
    f.edgeReach = f.carriage + f.carriageHalf;
    return f;
}

roads::lanes::SystemOptions systemOptions() {
    roads::lanes::SystemOptions so;
    so.rampHalf = 4.5 / 2 + 2.5;   // the ramp class: one 4.5 m lane, 2.5 m shoulders
    return so;
}

std::vector<std::vector<Vec2>> ringChains(const CityPlan& plan) {
    const std::vector<Vec2>& ring = plan.ring;
    const std::size_t n = ring.size();
    if (n < 8) return {};
    std::vector<std::size_t> at;   // ring vertices where an expressway crosses
    for (const std::vector<Vec2>& sp : plan.spurs) {
        double s = 0; std::size_t k = 0;
        if (crossingWithRing(sp, ring, s, k)) at.push_back(k);
    }
    std::sort(at.begin(), at.end());
    if (at.empty()) at = {0, n / 2};
    if (at.size() == 1) at.push_back((at[0] + n / 2) % n);
    std::sort(at.begin(), at.end());
    // Where the ring crosses a street: a SEAM goes on one of these. A diamond cannot span a seam
    // (its ramps anchor to one chain), so a seam midway between two arterials cost both their
    // diamonds; on an arterial it costs that one, and the builder carries a deck joint over a street.
    std::vector<char> crossed(n, 0);
    for (const RoadEdge& e : plan.streets.edges) {
        const Vec2 p0 = plan.streets.nodes[static_cast<std::size_t>(e.a)].pos, p1 = plan.streets.nodes[static_cast<std::size_t>(e.b)].pos;
        for (std::size_t k = 0; k < n; ++k) {
            const Vec2 c = ring[k], d = ring[(k + 1) % n];
            const Real den = cross(p1 - p0, d - c);
            if (std::fabs(den) < 1e-12) continue;
            const Real u = cross(c - p0, d - c) / den, v = cross(c - p0, p1 - p0) / den;
            if (u >= 0 && u <= 1 && v >= 0 && v <= 1) crossed[k] = 1;
        }
    }
    std::vector<std::size_t> seams;
    for (std::size_t i = 0; i < at.size(); ++i) {
        const std::size_t a = at[i], b = i + 1 < at.size() ? at[i + 1] : at[0] + n;
        const std::size_t mid = (a + b) / 2, reach = std::min<std::size_t>((b - a) / 4, 15);   // vertices are ~20 m apart
        std::size_t seam = mid % n;
        for (std::size_t d = 0; d <= reach; ++d) {
            if (crossed[(mid + d) % n]) { seam = (mid + d) % n; break; }
            if (crossed[(mid + n - d) % n]) { seam = (mid + n - d) % n; break; }
        }
        seams.push_back(seam);
    }
    std::sort(seams.begin(), seams.end());
    std::vector<std::vector<Vec2>> chains;
    for (std::size_t i = 0; i < seams.size(); ++i) {
        const std::size_t a = seams[i], b = i + 1 < seams.size() ? seams[i + 1] : seams[0] + n;
        std::vector<Vec2> c;
        for (std::size_t k = a; k <= b; ++k) c.push_back(ring[k % n]);
        chains.push_back(std::move(c));
    }
    return chains;
}

constexpr double kTownPair = 150.0;   // the boulevard pair's run from a town's main street to where the freeway starts

std::vector<Vec2> spurFreeway(const CityPlan& plan, std::size_t j) {
    const std::vector<Vec2>& sp = plan.spurs[j];
    double sx = 0; std::size_t k = 0;
    if (!crossingWithRing(sp, plan.ring, sx, k)) return sp;
    // past the loops' lanes on the stem, and short of midtown's boulevard by room for the
    // one-way pair to splay out to its two Ts
    const FreewaySection f = freewaySection(plan.brief);
    const roads::lanes::SystemOptions so = systemOptions();
    const double loopGore = f.edgeReach + so.rampHalf + so.bandGap + so.loopR - so.diverge;
    const std::vector<double> st = roads::lanes::stations(sp);
    const double sEnd = std::min(sx + loopGore + 40.0, st.back() - 80.0);
    // and, at a town, short of its main street by the same room for the pair at that end
    const double sStart = j < plan.towns.size() && plan.towns[j].built ? kTownPair : 0.0;
    std::vector<Vec2> out{roads::lanes::pointAt(sp, st, sStart)};
    for (std::size_t i = 0; i < sp.size() && st[i] < sEnd; ++i) if (st[i] > sStart + 0.5) out.push_back(sp[i]);
    out.push_back(roads::lanes::pointAt(sp, st, sEnd));
    return out;
}

std::vector<SystemAt> systemInterchanges(const CityPlan& plan, const std::vector<std::vector<Vec2>>& chains) {
    std::vector<SystemAt> out;
    const FreewaySection f = freewaySection(plan.brief);
    auto road = [&](const std::vector<Vec2>& route, std::size_t id) {
        roads::lanes::SystemRoad r;
        r.route = route;
        r.aId = "fw" + std::to_string(id) + "_a";
        r.bId = "fw" + std::to_string(id) + "_b";
        r.carriage = f.carriage;
        r.edgeReach = f.edgeReach;
        r.lanes = f.lanes;
        r.laneW = f.laneW;
        return r;
    };
    for (std::size_t j = 0; j < plan.spurs.size(); ++j) {
        const std::vector<Vec2> stem = spurFreeway(plan, j);
        for (std::size_t k = 0; k < chains.size(); ++k) {
            if (roads::lanes::crossings(chains[k], stem).empty()) continue;
            roads::lanes::SystemOptions so = systemOptions();
            so.idPrefix = "x" + std::to_string(j);
            // an expressway is drawn from the map edge inward, so its outer leg runs back up its stations
            SystemAt at{k, j, roads::lanes::systemInterchange(road(chains[k], k), road(stem, chains.size() + j), -1.0, so)};
            out.push_back(std::move(at));
            break;
        }
    }
    return out;
}

nlohmann::json sceneTerrain(const Brief& b) {
    // The map, with room for the sidewalks outside the outermost road — or, in a world, out to
    // where the city's hills have faded into it.
    const json& world = b.world;
    const Real half = world.is_null() ? b.size * Real(0.5) + 60 : world.value("grid", b.size * 0.5 + 60);
    // THE GROUND THE CITY IS BUILT ON. Never "flat": a scene whose terrain is `flat` makes no
    // ground grid at all (lanes.cpp: hasTerrain = type != "flat"), and the loader's lane-city
    // path publishes its blocks only when there IS one — so the first planned level loaded
    // with 0 blocks, 0 lots and not one building in 3 km of streets. The brief's relief is
    // rolling hills the roads conform to and the lots grade their pads off.
    const Real relief = std::max(Real(2), b.relief);
    nlohmann::json octaves = json::array();
    octaves.push_back({relief * 0.34, 900.0});
    octaves.push_back({relief * 0.12, 360.0});
    octaves.push_back({relief * 0.04, 140.0});
    json terrain = {{"type", "procedural"},
                   {"bounds", {b.center.x - half, b.center.x + half, b.center.y - half, b.center.y + half}},
                   {"res", 10.0},
                   {"seed", static_cast<int>(b.seed)},
                   {"octaves", octaves}};
    if (!world.is_null()) {
        // the world underneath, the same block the level renders beyond this grid, and the hills
        // laid over it inside the city only — faded out before the grid's edge, calmed toward the coast
        terrain["base"] = world.at("base");
        const json fade = world.value("reliefFade", json::array({b.size * 0.5, half - 100}));
        terrain["relief_fade"] = {{"centre", {b.center.x, b.center.y}}, {"r0", fade[0]}, {"r1", fade[1]}};
        if (world.contains("calm")) terrain["relief_calm"] = world["calm"];
    }
    return terrain;
}

HeightField sceneGround(const Brief& b) {
    const json t = sceneTerrain(b);
    roads::lanes::TerrainSpec tspec;
    tspec.type = "procedural";
    tspec.res = t["res"].get<double>();
    tspec.seed = t["seed"].get<int>();
    for (const json& o2 : t["octaves"]) tspec.octaves.emplace_back(o2[0].get<double>(), o2[1].get<double>());
    if (t.contains("base")) {
        tspec.base = t["base"];
        tspec.hasFade = true;
        tspec.fadeX = t["relief_fade"]["centre"][0].get<double>(); tspec.fadeY = t["relief_fade"]["centre"][1].get<double>();
        tspec.fadeR0 = t["relief_fade"]["r0"].get<double>(); tspec.fadeR1 = t["relief_fade"]["r1"].get<double>();
        if (t.contains("relief_calm")) {
            tspec.hasCalm = true;
            tspec.calmDx = t["relief_calm"]["dir"][0].get<double>(); tspec.calmDy = t["relief_calm"]["dir"][1].get<double>();
            tspec.calmFrom = t["relief_calm"]["from"].get<double>(); tspec.calmTo = t["relief_calm"]["to"].get<double>();
        }
    }
    const json& bd = t["bounds"];
    return roads::lanes::makeTerrain(tspec, {bd[0].get<double>(), bd[1].get<double>(), bd[2].get<double>(), bd[3].get<double>()});
}

nlohmann::json planToLanesScene(const CityPlan& plan, const SceneOptions& opt) {
    const Brief& b = plan.brief;
    json scene;
    scene["name"] = b.name;
    const json& world = b.world;
    scene["terrain"] = sceneTerrain(b);
    // Lane counts and widths that add up to the brief's carriageway widths.
    scene["classes"] = {
        {"local", {{"w", b.localWidth / 2}, {"fwd", 1}, {"back", 1}, {"sidewalk", b.sidewalk}, {"g_max", 0.12}, {"rank", 1}, {"thick", 0.5}, {"window", 40}}},
        {"collector", {{"w", b.collectorWidth / 4}, {"fwd", 2}, {"back", 2}, {"sidewalk", b.sidewalk}, {"g_max", 0.1}, {"rank", 1}, {"thick", 0.5}, {"window", 80}}},
        {"arterial", {{"w", b.arterialWidth / 6}, {"fwd", 3}, {"back", 3}, {"sidewalk", b.sidewalk}, {"g_max", 0.08}, {"rank", 2}, {"thick", 0.6}, {"window", 120}}},
        {"boulevard", {{"w", b.arterialWidth / 6}, {"fwd", 3}, {"back", 0}, {"sidewalk", b.sidewalk}, {"g_max", 0.08}, {"rank", 2}, {"thick", 0.6}, {"window", 120}}},
        {"freeway", {{"w", b.freewayWidth / 8}, {"fwd", 4}, {"back", 0}, {"shoulder", 2.5}, {"sidewalk", 0.0}, {"g_max", 0.06}, {"rank", 3}, {"thick", 1.2}, {"window", 200.0}}},
        {"ramp", {{"w", 4.5}, {"fwd", 1}, {"back", 0}, {"shoulder", 2.5}, {"g_max", 0.08}, {"rank", 0}, {"thick", 1.0}, {"window", 40}}},
    };
    scene["rules"] = {{"closing", 3.0}};

    // The ground this scene declares, evaluated here so the freeway's floors can ride over it.
    const HeightField ground = sceneGround(b);

    const FreewaySection fs = freewaySection(b, opt.medianGap);
    const std::vector<std::vector<Vec2>> chains = ringChains(plan);
    const std::vector<SystemAt> systems = systemInterchanges(plan, chains);
    json edges = json::array();

    // THE STREETS, one edge per chain between junctions — except where an expressway's boulevard
    // pair meets midtown's boulevard, whose chain is cut in three at the pair's two Ts.
    const std::vector<PlanChain> streetChains = chainsOf(plan.streets);
    // A pair's Ts on a street chain, in station order: at an expressway's inner end on midtown's
    // boulevard (end 1), and at a town's main street (end 0).
    struct Tee { std::size_t chain; double s0, s1; std::size_t spur; int end; };
    std::vector<Tee> tees;
    constexpr double kTee = 20.0;   // each T this far along the street from the expressway's end
    for (std::size_t j = 0; j < plan.spurs.size(); ++j) {
        for (int end = 0; end < 2; ++end) {
            if (end == 0 && !(j < plan.towns.size() && plan.towns[j].built)) continue;
            const Vec2 at = end == 0 ? plan.spurs[j].front() : plan.spurs[j].back();
            // The nearest stretch of street with room for both Ts, 15 m clear of its junctions,
            // slid along to within 60 m of the expressway's end: where it lands on a junction, the
            // pair splays a little further to one side of it.
            double best = 1e30; std::size_t bc = 0; double bs = 0;
            for (std::size_t i = 0; i < streetChains.size(); ++i) {
                const std::vector<double> st = roads::lanes::stations(streetChains[i].pts);
                if (st.back() < 2 * kTee + 30.0) continue;
                const roads::lanes::Projection pr = roads::lanes::project(streetChains[i].pts, st, at);
                const double c = std::clamp(pr.station, kTee + 15.0, st.back() - kTee - 15.0);
                const double d = (roads::lanes::pointAt(streetChains[i].pts, st, c) - at).length();
                if (d < best) { best = d; bc = i; bs = c; }
            }
            if (best < 60.0) tees.push_back({bc, bs - kTee, bs + kTee, j, end});
            else if (std::getenv("RT_PLAN_WHY"))
                std::printf("[plan] expressway %zu: no Ts for its %s boulevard pair (no street with room within %.0f m of its end)\n", j,
                            end == 0 ? "town" : "midtown", best);
        }
    }
    std::vector<roads::lanes::RampStreet> rampStreets;
    auto street = [&](const std::string& sid, RoadClass k, const std::vector<Vec2>& pts) {
        edges.push_back({{"id", sid}, {"class", className(k)}, {"path", {{"points", pointsJson(pts)}}}});
        rampStreets.push_back({sid, pts});
    };
    auto piece = [](const std::vector<Vec2>& pts, double s0, double s1) {
        const std::vector<double> st = roads::lanes::stations(pts);
        std::vector<Vec2> out{roads::lanes::pointAt(pts, st, s0)};
        for (std::size_t i = 0; i < pts.size(); ++i) if (st[i] > s0 + 0.5 && st[i] < s1 - 0.5) out.push_back(pts[i]);
        out.push_back(roads::lanes::pointAt(pts, st, s1));
        return out;
    };
    std::vector<std::array<std::array<Vec2, 2>, 2>> teePoints(plan.spurs.size());   // for each expressway, each end, its two Ts
    for (std::size_t i = 0; i < streetChains.size(); ++i) {
        const PlanChain& c = streetChains[i];
        const std::string sid = "s" + std::to_string(i);
        const Tee* t = nullptr;
        for (std::size_t j = 0; j < tees.size(); ++j) if (tees[j].chain == i) t = &tees[j];
        if (!t) { street(sid, c.klass, c.pts); continue; }
        const double len = roads::lanes::stations(c.pts).back();
        street(sid + "_0", c.klass, piece(c.pts, 0, t->s0));
        street(sid + "_1", c.klass, piece(c.pts, t->s0, t->s1));
        street(sid + "_2", c.klass, piece(c.pts, t->s1, len));
        const std::vector<double> st = roads::lanes::stations(c.pts);
        teePoints[t->spur][static_cast<std::size_t>(t->end)] = {roads::lanes::pointAt(c.pts, st, t->s0), roads::lanes::pointAt(c.pts, st, t->s1)};
    }

    if (std::getenv("RT_PLAN_WHY") && ground) {
        // how high the city stands: every street point's ground, and against the sea
        double lo = 1e30, hi = -1e30;
        for (const PlanChain& c : streetChains) for (const Vec2& p : c.pts) { const double z = ground(p.x, p.y); lo = std::min(lo, z); hi = std::max(hi, z); }
        const double sea = world.is_null() ? -1e30 : world.value("seaLevel", -1e30);
        std::printf("[plan] streets stand %.1f to %.1f m (%.0f m of relief)%s\n", lo, hi, hi - lo,
                    sea > -1e29 ? (", the lowest " + std::to_string(static_cast<int>(std::round(lo - sea))) + " m above the sea").c_str() : "");
    }

    // THE FREEWAY: two one-way carriageways either side of its centreline — right-hand traffic
    // in plan coordinates (keep-left on screen: roads/lanes/interchange.h), a with the route on
    // its RIGHT, b against it on its left (offsetPolyline's +d is left) — standing medianGap
    // apart so each is its own deck. Placed the other way, every ramp anchored to a
    // carriageway's right-hand lane left from beside the median and crossed the deck.
    //
    // AN ELEVATED RING. Two designs failed before this one: holding the freeway's deck clear
    // only where it crosses a street put 1.4 km of a 2.1 km ring on piers anyway (at 6% it
    // cannot come back down between crossings 350 m apart), and making the STREET climb
    // instead asked a 25 m chain — cut short by the frontage roads either side — to gain 8 m,
    // a 24% ramp. So the motorway runs above the streets its whole length, every street passes
    // under it, and the ramps come down beside it to the streets that pass under it.
    auto floorsOver = [&](const std::vector<Vec2>& c, double from, double to) {
        // OVER THE GROUND, not at an absolute height: on rolling ground a flat 8 m floor is
        // under the hills and pointless over the hollows. Sampled from the same terrain the
        // builder will make from this scene's own spec.
        json floors = json::array();
        const std::vector<double> st = roads::lanes::stations(c);
        for (double s0 = std::max(0.0, from); s0 < std::min(st.back(), to); s0 += 60.0) {
            const Vec2 q = roads::lanes::pointAt(c, st, s0);
            const double z = (ground ? ground(q.x, q.y) : 0.0) + opt.clearance;
            floors.push_back(json::array({std::round(q.x * 100) / 100, std::round(q.y * 100) / 100, std::round(z * 100) / 100, 60.0}));
        }
        return floors;
    };
    auto carriageways = [&](std::size_t id, const std::vector<Vec2>& ca, std::vector<Vec2> cb, const json& floors) {
        std::reverse(cb.begin(), cb.end());
        edges.push_back({{"id", "fw" + std::to_string(id) + "_a"}, {"class", "freeway"}, {"floor", floors}, {"path", {{"points", pointsJson(ca)}}}});
        edges.push_back({{"id", "fw" + std::to_string(id) + "_b"}, {"class", "freeway"}, {"floor", floors}, {"path", {{"points", pointsJson(cb)}}}});
    };
    // the ring, offset once as the closed loop it is, then cut into its chains
    {
        const std::vector<Vec2> ra = offsetClosed(plan.ring, -fs.carriage), rb = offsetClosed(plan.ring, fs.carriage);
        const std::size_t n = plan.ring.size();
        std::size_t start = 0;
        for (std::size_t k = 0; k < chains.size(); ++k) {
            for (std::size_t i = 0; i < n; ++i) if ((plan.ring[i] - chains[k].front()).lengthSquared() < 1e-6) start = i;
            std::vector<Vec2> ca, cb;
            for (std::size_t i = 0; i < chains[k].size(); ++i) { ca.push_back(ra[(start + i) % n]); cb.push_back(rb[(start + i) % n]); }
            json floors = floorsOver(chains[k], 0, 1e30);
            // Over an expressway the ring holds 2.5 m higher. The expressway must be at the ring's
            // height again 200 m out, to bridge the suburbs' first street, and at design grade on
            // rolling ground it cannot also get down to the ground under the ring: it passes under
            // at about 1.3 m, and the ring's girders need their clearance above that.
            for (const SystemAt& at : systems) {
                if (at.chain != k || !at.r.built) continue;
                const Vec2 q = roads::lanes::pointAt(chains[k], roads::lanes::stations(chains[k]), at.r.sThrough);
                const double z = (ground ? ground(q.x, q.y) : 0.0) + opt.clearance + 2.5;
                floors.push_back(json::array({std::round(q.x * 100) / 100, std::round(q.y * 100) / 100, std::round(z * 100) / 100, 40.0}));
            }
            carriageways(k, ca, cb, floors);
        }
    }
    // THE EXPRESSWAYS: at ground level where they pass under the ring — its deck leaves 5 m
    // beneath — and at the ring's height from 200 m out, to bridge the suburbs' streets.
    // Inside the ring, past the interchange's loops, the carriageways carry on as a one-way
    // boulevard pair that splays to meet midtown's boulevard at two Ts 40 m apart. (Closed
    // into one two-way road, two edges' lanes share one centreline at the node and overlap.)
    for (std::size_t j = 0; j < plan.spurs.size(); ++j) {
        const std::vector<Vec2> fwy = spurFreeway(plan, j);
        double sx = 0; std::size_t seg = 0;
        const bool crosses = crossingWithRing(fwy, plan.ring, sx, seg);
        const std::vector<Vec2> ca = offsetPolyline(fwy, -fs.carriage), cb = offsetPolyline(fwy, fs.carriage);
        const std::size_t id = chains.size() + j;
        // floors to 200 m out: at 8 m to 140 m (past the suburbs' first street, 200 m out on
        // rolling ground), then down at design grade to pass under the ring, held higher there —
        // and, approaching a town, from 250 m out, down to meet its main street at ground level
        const bool town = j < plan.towns.size() && plan.towns[j].built;
        carriageways(id, ca, cb, floorsOver(fwy, town ? 250.0 : 0.0, crosses ? sx - 200.0 : 1e30));
        // A pair of one-way boulevards between a freeway end and its two Ts, `dir` pointing along
        // the freeway away from the Ts; a (the carriageway with the route) takes the T on its right.
        auto pair = [&](const std::string& idp, const std::array<Vec2, 2>& tp, const Vec2& freewayEnd, const Vec2& dir,
                        const Vec2& aEnd, const Vec2& bEnd, bool aLeavesTee) {
            const Vec2 route = aLeavesTee ? dir : dir * Real(-1);   // the way a runs, whose right is a's side
            const bool firstRight = cross(route, tp[0] - freewayEnd) < 0;
            const Vec2 tA = firstRight ? tp[0] : tp[1], tB = firstRight ? tp[1] : tp[0];
            const Real reach = (tA - aEnd).length() * Real(0.45);
            if (aLeavesTee) {   // a runs from its T onto the freeway, b off it to its T
                edges.push_back({{"id", idp + "_in"}, {"class", "boulevard"}, {"path", {{"points", pointsJson(cubic(tA, tA + dir * reach, aEnd - dir * reach, aEnd, 16))}}}});
                edges.push_back({{"id", idp + "_out"}, {"class", "boulevard"}, {"path", {{"points", pointsJson(cubic(bEnd, bEnd - dir * reach, tB + dir * reach, tB, 16))}}}});
            } else {            // a runs off the freeway to its T, b from its T onto it
                edges.push_back({{"id", idp + "_in"}, {"class", "boulevard"}, {"path", {{"points", pointsJson(cubic(aEnd, aEnd - dir * reach, tA + dir * reach, tA, 16))}}}});
                edges.push_back({{"id", idp + "_out"}, {"class", "boulevard"}, {"path", {{"points", pointsJson(cubic(tB, tB + dir * reach, bEnd - dir * reach, bEnd, 16))}}}});
            }
        };
        const auto& tp = teePoints[j];
        auto found = [](const std::array<Vec2, 2>& t) { return t[0].lengthSquared() != 0 || t[1].lengthSquared() != 0; };
        if (town && found(tp[0])) {
            // THE TOWN END: the expressway comes down to its main street as the same pair
            pair("bt" + std::to_string(j), tp[0], fwy.front(), normalize(fwy[1] - fwy.front()), ca.front(), cb.front(), true);
        } else {
            // NO TOWN: out in the country, a teardrop turns the outbound carriageway back into the
            // inbound one. Without it the outbound carriageway ended at the map edge with no way
            // off, and every car that took it — and every home whose only road led there — was
            // stranded (the census found 40 street links so). Looking outward, b (outbound) is on
            // the right and a (inbound) on the left: the loop runs on past b's end, round the far
            // side of a 40 m circle and back into a's start.
            const Vec2 out = normalize(fwy.front() - fwy[1]);   // the expressway is drawn from its far end inward
            const Vec2 left = Vec2(-out.y, out.x);
            const Vec2 bEnd = cb.front(), aStart = ca.front();
            const Real R = 40;
            const Vec2 c = (bEnd + aStart) * Real(0.5) + out * Real(60);
            std::vector<Vec2> loop = cubic(bEnd, bEnd + out * Real(30), c - left * R - out * Real(20), c - left * R, 12);
            for (int i = 1; i <= 24; ++i) {
                const Real th = M_PI * i / 24;   // from the right of the circle, round its far side, to its left
                loop.push_back(c - left * (R * std::cos(th)) + out * (R * std::sin(th)));
            }
            const std::vector<Vec2> back = cubic(c + left * R, c + left * R - out * Real(20), aStart + out * Real(30), aStart, 12);
            loop.insert(loop.end(), back.begin() + 1, back.end());
            edges.push_back({{"id", "tn" + std::to_string(j)}, {"class", "boulevard"}, {"path", {{"points", pointsJson(loop)}}}});
        }
        // THE MIDTOWN END
        if (found(tp[1])) pair("bv" + std::to_string(j), tp[1], fwy.back(), normalize(fwy[fwy.size() - 2] - fwy.back()), ca.back(), cb.back(), false);
    }

    // THE RAMPS, after every edge they anchor to. Each ring chain gets diamonds onto the streets
    // beside it, clear of the system interchange; each expressway gets its interchange.
    if (opt.ramps) {
        for (std::size_t k = 0; k < chains.size(); ++k) {
            roads::lanes::DiamondOptions dop;
            dop.aId = "fw" + std::to_string(k) + "_a";
            dop.bId = "fw" + std::to_string(k) + "_b";
            dop.idPrefix = "d" + std::to_string(k);
            dop.carriage = fs.carriage;
            dop.edgeReach = fs.edgeReach;
            dop.freewayLanes = fs.lanes;
            dop.freewayLaneW = fs.laneW;
            dop.maxDiamonds = opt.diamondsPerRoute;
            dop.spacing = opt.interchangeSpacing;
            dop.clearance = opt.clearance;
            dop.rampHalf = 4.5 / 2 + 2.5;   // the ramp class below: one 4.5 m lane, 2.5 m shoulders
            dop.gRamp = 0.08;               // and its g_max
            dop.ground = ground;            // the climb is to the ground the ramp lands on
            dop.window = 60.0;              // and from the deck as built: floors every 60 m over the ground
            for (const SystemAt& at : systems)
                if (at.chain == k && at.r.built) dop.keepOut.push_back({at.r.sThrough - at.r.reach - 60.0, at.r.sThrough + at.r.reach + 60.0});
            const roads::lanes::DiamondResult dr = roads::lanes::diamondRamps(chains[k], rampStreets, dop);
            for (const json& r : dr.ramps) edges.push_back(r);
            if (std::getenv("RT_PLAN_WHY"))
                std::printf("[plan] %s: %d diamonds of %d crossings (oblique %d, spacing %d, no terminal %d, no room %d, over a street %d)\n",
                            dop.aId.c_str(), dr.built, dr.candidates, dr.rejectedOblique, dr.rejectedSpacing, dr.rejectedTerminal,
                            dr.rejectedRoom, dr.rejectedConflict);
        }
        for (const SystemAt& at : systems) {
            for (const json& r : at.r.ramps) edges.push_back(r);
            if (std::getenv("RT_PLAN_WHY"))
                std::printf("[plan] expressway %zu x ring chain %zu: %s\n", at.spur, at.chain,
                            at.r.built ? "system interchange, four ramps" : ("no interchange: " + at.r.why).c_str());
        }
    }
    scene["edges"] = std::move(edges);
    return scene;
}

}  // namespace plan
}  // namespace engine
