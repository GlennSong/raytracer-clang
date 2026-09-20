#include "engine/procgen/city/roads/lanes/level_import.h"
#include "engine/procgen/city/roads/lanes/polyline_ops.h"
#include "engine/procgen/city/roads/lanes/geom2d.h"          // the design-loop redraw (closing + opening)
#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"

#include "engine/level_params.h"
#include "engine/procgen/city/roads/road_entity.h"
#include "engine/procgen/city/road_mesh.h"
#include "engine/procgen/noise.h"
#include "engine/procgen/terrain.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <cstdio>
#include <sstream>

namespace engine {
namespace roads::lanes {

namespace {

const char* className(RoadClass c) {
    switch (c) { case RoadClass::Freeway: return "arterial"; case RoadClass::Arterial: return "arterial"; case RoadClass::Collector: return "collector";
                 case RoadClass::Local: return "local"; case RoadClass::Ramp: return "ramp"; case RoadClass::Alley: return "alley"; }
    return "local";
}
int classPriority(RoadClass c) { return c == RoadClass::Arterial ? 3 : c == RoadClass::Collector ? 2 : c == RoadClass::Local ? 1 : 0; }

nlohmann::json pointList(const std::vector<Vec2>& xy) {
    nlohmann::json a = nlohmann::json::array(); for (const Vec2& p : xy) a.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100}); return a;
}

struct Chain { std::vector<Vec2> xy; std::vector<double> s; RoadClass klass; double hw; };


// Offset a closed CCW loop point-for-point (index i of the result belongs to index i of the loop), with wrap-around tangents.
std::vector<Vec2> offsetLoop(const std::vector<Vec2>& loop, double d) {
    size_t n = loop.size(); std::vector<Vec2> out(n);
    for (size_t i = 0; i < n; ++i) { Vec2 t = normalize(loop[(i + 1) % n] - loop[(i + n - 1) % n]); out[i] = loop[i] + perp(t) * d; }
    return out;
}

// The slice of a ring between two loop indices (wrapping), as a contiguous open polyline.
std::vector<Vec2> ringSlice(const std::vector<Vec2>& ring, size_t i0, size_t i1) {
    std::vector<Vec2> out; size_t n = ring.size(); for (size_t i = i0; ; i = (i + 1) % n) { out.push_back(ring[i]); if (i == i1) break; } return out;
}

// A freeway alignment is not a street's polyline: the perimeter ring's notches and corners are redrawn as arcs
// of at least the design radius. Morphological closing (grow R, shrink R) fills concave notches narrower than 2R
// and rounds concave corners at R; opening (shrink R, grow R) removes protrusions narrower than 2R and rounds
// convex corners at R. Round joins make every corner an arc of radius R. Blocks the redraw cuts across sit
// outside the freeway afterwards, which is what happens to a city's corner blocks in reality.
std::vector<Vec2> designLoop(const std::vector<Vec2>& loop, double R) {
    if (loop.size() < 8 || R <= 0) return loop;
    PolySet set = closing(fromRing(loop), R);
    set = offsetSet(offsetSet(set, -R), R);
    if (set.empty()) return loop;
    size_t best = 0; for (size_t i = 1; i < set.size(); ++i) if (std::fabs(ringArea(set[i].outer)) > std::fabs(ringArea(set[best].outer))) best = i;
    return set[best].outer;
}

double minRadius(const std::vector<Vec2>& loop, double step) {
    size_t n = loop.size(); double best = 1e300; int k = std::max(1, static_cast<int>(20.0 / step));   // curvature over 40 m chords
    for (size_t i = 0; i < n; ++i) {
        Vec2 a = loop[(i + n - static_cast<size_t>(k)) % n], b = loop[i], c = loop[(i + static_cast<size_t>(k)) % n];
        double ab = distance(a, b), bc = distance(b, c), ca = distance(c, a); double area2 = std::fabs(cross(b - a, c - a));
        if (area2 > 1e-9) best = std::min(best, ab * bc * ca / (2 * area2));
    }
    return best;
}

// Outer face of the street graph: start at the leftmost junction heading down, take the RIGHTMOST exit
// at every junction (exterior on the right), stop when back at the start. Returns the ring's chain
// indices in walk order with their direction (true = reversed) — or empty if the walk did not close.
std::vector<std::pair<int, bool>> outerFace(const std::vector<Chain>& chains) {
    struct End { int chain; bool atStart; Vec2 p; };
    std::vector<End> ends; for (size_t i = 0; i < chains.size(); ++i) { ends.push_back({static_cast<int>(i), true, chains[i].xy.front()}); ends.push_back({static_cast<int>(i), false, chains[i].xy.back()}); }
    std::vector<int> node(ends.size(), -1); std::vector<Vec2> nodes;
    for (size_t i = 0; i < ends.size(); ++i) {
        for (size_t k = 0; k < nodes.size(); ++k) if (distance(nodes[k], ends[i].p) < 1.5) { node[i] = static_cast<int>(k); break; }
        if (node[i] < 0) { node[i] = static_cast<int>(nodes.size()); nodes.push_back(ends[i].p); }
    }
    std::vector<std::vector<size_t>> at(nodes.size()); for (size_t i = 0; i < ends.size(); ++i) at[static_cast<size_t>(node[i])].push_back(i);
    auto outDir = [&](size_t e) { const Chain& c = chains[static_cast<size_t>(ends[e].chain)]; return ends[e].atStart ? tangentAt(c.xy, 0) : tangentAt(c.xy, c.xy.size() - 1) * -1.0; };
    auto otherEnd = [&](size_t e) { return ends[e].atStart ? e + 1 : e - 1; };
    int startNode = 0; for (size_t k = 1; k < nodes.size(); ++k) if (nodes[k].x < nodes[static_cast<size_t>(startNode)].x) startNode = static_cast<int>(k);
    // first exit: the one heading most nearly straight down
    size_t cur = at[static_cast<size_t>(startNode)][0]; double best = -2; for (size_t e : at[static_cast<size_t>(startNode)]) { double sc = dot(outDir(e), Vec2(0, -1)); if (sc > best) { best = sc; cur = e; } }
    std::vector<std::pair<int, bool>> walk; std::vector<char> usedChain(chains.size(), 0);
    for (size_t step = 0; step < chains.size() * 2; ++step) {
        walk.emplace_back(ends[cur].chain, !ends[cur].atStart); usedChain[static_cast<size_t>(ends[cur].chain)] = 1;
        size_t arrive = otherEnd(cur); int n = node[arrive]; if (n == startNode) return walk;
        Vec2 heading = outDir(arrive) * -1.0;                      // direction of travel arriving at the node
        size_t next = arrive; double bestTurn = 1e300;
        for (size_t e : at[static_cast<size_t>(n)]) {
            if (e == arrive) continue; Vec2 o = outDir(e); double turn = std::atan2(cross(heading, o), dot(heading, o));   // + = left
            if (turn < bestTurn) { bestTurn = turn; next = e; }   // rightmost exit
        }
        if (next == arrive) next = arrive;                         // a dead-end stub: the outer face U-turns and comes back along it
        cur = next;
    }
    return {};
}

// The ring proper: the outer-face walk minus every chain it traversed out AND back (dead-end stubs).
std::vector<std::pair<int, bool>> ringChains(const std::vector<std::pair<int, bool>>& walk) {
    std::map<int, int> count; for (const auto& w : walk) ++count[w.first];
    std::vector<std::pair<int, bool>> ring; for (const auto& w : walk) if (count[w.first] == 1) ring.push_back(w);
    return ring;
}

}  // namespace

nlohmann::json graphFromLevel(const std::string& levelPath, const std::string& outDir, const ImportOptions& oIn, ImportReport& rep) {
    const ImportOptions& o = oIn;
    std::ifstream in(levelPath); if (!in) throw std::runtime_error("cannot open " + levelPath);
    nlohmann::json level = nlohmann::json::parse(in);
    std::string name = level.value("name", std::string("level")); for (char& ch : name) if (ch == ' ' || ch == '/') ch = '_';
    // --- the level's own ground ---
    RoadGroundFn ground; std::shared_ptr<TerrainParams> tp; std::shared_ptr<Noise> noise; bool hasTerrain = level.contains("terrain");
    if (hasTerrain) {
        tp = std::make_shared<TerrainParams>(readTerrainParams(level["terrain"])); tp->erodedBase = readErodedBase(level);
        noise = std::make_shared<Noise>(level["terrain"].value("seed", 0u));
        ground = [tp, noise](double x, double z) { return terrainHeight(*tp, *noise, x, z); };
    }
    // --- the level's own recipe ---
    nlohmann::json gen; double roadWidth = 8.0, sidewalk = 3.5;
    for (const auto& ent : level.value("entities", nlohmann::json::array())) {
        if (ent.value("shape", std::string()) != "road") continue; const nlohmann::json& road = ent["road"];
        gen = road.value("generate", nlohmann::json()); roadWidth = road.value("width", roadWidth); sidewalk = road.value("sidewalk", sidewalk); break;
    }
    if (gen.is_null()) throw std::runtime_error("level has no generated road entity");
    RoadEntity net; net.look.defaultWidth = roadWidth; net.look.autoRoundabout = false;
    applyGenerateRecipe(net, gen, ground);
    RoadGraph g = navRoadGraph(net, ground);
    std::vector<UnionSpine> spines = roadNetWeldSpines(g);
    if (const char* probe = std::getenv("LANELAB_IMPORT_PROBE")) {   // LANELAB_IMPORT_PROBE=x,y: the source graph around a point (nodes, arms, chain ends)
        double px = 0, py = 0; if (std::sscanf(probe, "%lf,%lf", &px, &py) == 2) {
            std::ostringstream ps; ps.setf(std::ios::fixed); ps.precision(1);
            for (size_t ni = 0; ni < g.nodes.size(); ++ni) {
                const Vec2 q = g.nodes[ni].pos; if (distance(q, Vec2(px, py)) > 12.0) continue;
                ps << "node " << ni << " at " << q.x << "," << q.y << ":";
                for (size_t ei = 0; ei < g.edges.size(); ++ei) { const RoadEdge& e = g.edges[ei]; if (e.a != static_cast<int>(ni) && e.b != static_cast<int>(ni)) continue; const Vec2 o = g.nodes[static_cast<size_t>(e.a == static_cast<int>(ni) ? e.b : e.a)].pos; ps << " edge " << ei << " class " << static_cast<int>(e.klass) << " w " << e.width << " layer " << e.layer << " to " << o.x << "," << o.y << " (" << distance(q, o) << " m)"; }
                ps << "; ";
            }
            for (size_t si = 0; si < spines.size(); ++si) { const UnionSpine& sp = spines[si]; if (sp.points.size() < 2) continue; for (int end = 0; end < 2; ++end) { const Vec2 q = end ? sp.points.back() : sp.points.front(); if (distance(q, Vec2(px, py)) <= 12.0) ps << "spine " << si << (end ? " ends" : " starts") << " at " << q.x << "," << q.y << " class " << static_cast<int>(sp.klass) << " n " << sp.points.size() << "; "; } }
            rep.notes += "PROBE " + ps.str();
        }
    }
    std::vector<Chain> chains;
    for (const UnionSpine& sp : spines) { if (sp.points.size() < 2) continue; Chain c; c.xy = sp.points; c.s = stations(c.xy); c.klass = sp.klass; c.hw = sp.halfWidth; chains.push_back(std::move(c)); }
    rep.chains = static_cast<int>(chains.size());
    Vec2 centre; size_t np = 0; for (const Chain& c : chains) for (const Vec2& p : c.xy) { centre += p; ++np; }
    if (np) centre = centre / static_cast<double>(np);
    // --- output skeleton ---
    nlohmann::json out; out["name"] = name + "_lanelab";
    out["classes"] = {
        {"freeway",   {{"w", o.freewayLaneW}, {"fwd", o.freewayLanes}, {"back", 0}, {"shoulder", 2.5}, {"sidewalk", 0.0}, {"rank", 3}, {"g_max", o.freewayGMax}, {"window", o.freewayWindow}, {"thick", 1.2}}},
        {"arterial",  {{"w", 3.25}, {"fwd", 2}, {"back", 2}, {"gap", o.turnPockets ? 3.25 : 0.0}, {"sidewalk", sidewalk}, {"rank", 2}, {"g_max", 0.08}, {"window", 120}, {"thick", 0.6}}},
        {"collector", {{"w", 3.25}, {"fwd", 1}, {"back", 1}, {"shoulder", 2.5}, {"sidewalk", sidewalk}, {"rank", 1}, {"g_max", 0.10}, {"window", 80}, {"thick", 0.5}}},
        {"local",     {{"w", 3.5}, {"fwd", 1}, {"back", 1}, {"shoulder", 2.5}, {"sidewalk", sidewalk}, {"rank", 1}, {"g_max", 0.12}, {"window", 40}, {"thick", 0.5}}},
        {"alley",     {{"w", 3.5}, {"fwd", 1}, {"back", 0}, {"sidewalk", 0.0}, {"rank", 0}, {"g_max", 0.12}, {"window", 40}, {"thick", 0.4}}},
        {"ramp",      {{"w", o.rampLaneW}, {"fwd", 1}, {"back", 0}, {"shoulder", o.rampShoulder}, {"rank", 0}, {"g_max", 0.08}, {"window", 40}, {"thick", 1.0}}}};
    out["rules"] = {{"closing", 3.0}, {"bridge_h", 4.0}, {"pier_spacing", 24.0}, {"conform_w", 14.0}, {"same_level_dz", 1.0}};
    nlohmann::json edges = nlohmann::json::array();
    // --- perimeter loop -> freeway ---
    std::vector<Vec2> loop; std::vector<bool> isLoop(chains.size(), false); std::vector<double> loopS;
    if (o.freewayLoop) {
        { int cnt[6] = {0, 0, 0, 0, 0, 0}; for (const Chain& c : chains) ++cnt[static_cast<int>(c.klass)]; std::ostringstream cs; cs << "chains by class: freeway " << cnt[0] << " arterial " << cnt[1] << " collector " << cnt[2] << " local " << cnt[3] << " ramp " << cnt[4] << " alley " << cnt[5] << "; "; rep.notes += cs.str(); }
        std::vector<std::pair<int, bool>> walk = ringChains(outerFace(chains));
        for (const auto& w : walk) { std::vector<Vec2> part = chains[static_cast<size_t>(w.first)].xy; if (w.second) std::reverse(part.begin(), part.end()); if (loop.empty()) loop = part; else loop.insert(loop.end(), part.begin() + 1, part.end()); isLoop[static_cast<size_t>(w.first)] = true; }
        if (!loop.empty() && distance(loop.front(), loop.back()) < 2.0) loop.pop_back();
        double area = 0; for (size_t i = 0; i < loop.size(); ++i) { const Vec2& p = loop[i]; const Vec2& q = loop[(i + 1) % loop.size()]; area += p.x * q.y - q.x * p.y; }
        if (area < 0) std::reverse(loop.begin(), loop.end());
        { std::ostringstream cs; cs << "outer face: " << walk.size() << " chains, " << (loop.empty() ? 0.0 : stations(loop).back()) << " m; "; rep.notes += cs.str(); }
        if (loop.size() < 8 || !pointInRing(loop, centre)) { rep.notes += "no perimeter ring found; "; loop.clear(); std::fill(isLoop.begin(), isLoop.end(), false); }
        else rep.loopChains = static_cast<int>(walk.size());
    }
    struct Landing { int chain; double loopStation; std::vector<Vec2> xy; double footIn, footOut, gore; };   // the diamond fitted to its climb
    std::vector<Landing> landings; std::vector<std::vector<Vec2>> streets; std::vector<int> streetChain;
    std::vector<Vec2> rawLoopClosed; std::vector<double> rawLoopClosedS;
    if (!loop.empty()) {
        loop = resample(loop, 2.0); rawLoopClosed = loop; rawLoopClosed.push_back(loop.front()); rawLoopClosedS = stations(rawLoopClosed);
        loop = resample(designLoop(loop, o.freewayRadius), 2.0);
        { double area = 0; for (size_t i = 0; i < loop.size(); ++i) { const Vec2& p = loop[i]; const Vec2& q = loop[(i + 1) % loop.size()]; area += p.x * q.y - q.x * p.y; } if (area < 0) std::reverse(loop.begin(), loop.end()); }
        loopS = stations(loop); rep.loopLength = loopS.back() + distance(loop.back(), loop.front());
        { std::ostringstream cs; cs << "ring redrawn at R >= " << o.freewayRadius << " m: " << static_cast<int>(rep.loopLength) << " m, min radius " << static_cast<int>(minRadius(loop, 2.0)) << " m; "; rep.notes += cs.str(); }
        {   // where the ring is still tighter than a freeway curve, as station runs, with the raw ring's turn there
            std::ostringstream cs; size_t n = loop.size(); int k = 10; bool in = false; double runR = 1e9; size_t runS = 0; int runs = 0;
            for (size_t i = 0; i <= n; ++i) {
                double R = 1e9;
                if (i < n) { Vec2 a = loop[(i + n - k) % n], b = loop[i], c = loop[(i + k) % n]; double area2 = std::fabs(cross(b - a, c - a)); if (area2 > 1e-9) R = distance(a, b) * distance(b, c) * distance(c, a) / (2 * area2); }
                if (R < 150.0) { if (!in) { in = true; runS = i; runR = R; } runR = std::min(runR, R); }
                else if (in) { in = false; if (runs++ < 12) cs << " " << static_cast<int>(loopS[runS]) << "-" << static_cast<int>(loopS[i - 1]) << " (R " << static_cast<int>(runR) << ")"; }
            }
            rep.notes += "tight (< 150 m) at stations" + (runs ? cs.str() : std::string(" none")) + "; ";
        }
        std::vector<Vec2> loopClosed = loop; loopClosed.push_back(loop.front()); std::vector<double> loopClosedS = stations(loopClosed);
        double dCarriage = o.medianGap / 2 + o.freewayLanes * o.freewayLaneW / 2, edgeReach = dCarriage + o.freewayLanes * o.freewayLaneW / 2 + 2.5;
        // streets that end on the loop: candidates for landings, the rest trimmed short of the freeway
        struct Touch { int chain; bool atEnd; double loopStation; int prio; Vec2 xpt; double sCut; std::vector<Vec2> line; std::vector<double> lineS; std::vector<int> absorbed; int headCount = 0, headDeg = 0; };
        std::vector<Touch> touches;
        // where a street's line, extended, crosses the design loop: the landing's crossing (first crossing along the street)
        auto reach = [&](const Chain& c, bool atEnd, Touch& t) {
            std::vector<Vec2> xy = c.xy; if (!atEnd) std::reverse(xy.begin(), xy.end());
            Vec2 tan = tangentAt(xy, xy.size() - 1); std::vector<Vec2> ext = xy; ext.push_back(xy.back() + tan * 800.0); std::vector<double> es = stations(ext);
            std::vector<Vec2> xs = crossings(ext, loopClosed); if (xs.empty()) return false;
            t.sCut = 1e300; for (const Vec2& x : xs) { Projection pr = project(ext, es, x); if (pr.station < t.sCut) { t.sCut = pr.station; t.xpt = x; } }
            t.line = ext; t.lineS = es;
            return t.sCut > 30.0;   // a street that crosses within 30 m of its start is not a street reaching the ring
        };
        for (size_t i = 0; i < chains.size(); ++i) {
            if (isLoop[i]) continue; const Chain& c = chains[i]; if (c.s.back() <= 60) continue;
            Projection r0 = project(rawLoopClosed, rawLoopClosedS, c.xy.front()), r1 = project(rawLoopClosed, rawLoopClosedS, c.xy.back());
            bool atEnd; if (r1.distance < 3.0) atEnd = true; else if (r0.distance < 3.0) atEnd = false; else continue;
            Touch t; t.chain = static_cast<int>(i); t.atEnd = atEnd; t.prio = classPriority(c.klass); if (!reach(c, atEnd, t)) continue;
            t.loopStation = project(loopClosed, loopClosedS, t.xpt).station; touches.push_back(std::move(t));
        }
        std::sort(touches.begin(), touches.end(), [](const Touch& a, const Touch& b) { return a.prio != b.prio ? a.prio > b.prio : a.loopStation < b.loopStation; });
        // estimate the freeway's profile along the loop the way lanelab will build it; a landing's ramps must be able to climb the difference
        std::vector<double> loopZ; if (ground) { HeightField hf = ground; loopZ = profileAlong(loopClosed, hf, o.freewayWindow, o.freewayGMax); }
        std::vector<int> chosen; std::vector<double> chosenS; std::map<int, Landing> fitted;
        // The diamond's inside foot sits ON the landing edge, and a weld chain is one block long: grow the landing
        // street inward through its junctions along the straightest continuation (within 60 deg) until it is long
        // enough. The absorbed chains become part of the landing edge if the landing is chosen.
        auto extendInward = [&](Touch& t, double needLen) {
            while (t.sCut < needLen && t.line.size() >= 2) {
                Vec2 head = t.line.front(); Vec2 dirIn = normalize(t.line.front() - t.line[1]); int best = -1; bool bestAtEnd = false; double bestDot = std::cos(60.0 * M_PI / 180.0);   // a city arterial jogs at its junctions; the foot may sit past a bend
                for (size_t j = 0; j < chains.size(); ++j) {
                    if (isLoop[j] || static_cast<int>(j) == t.chain || chains[j].xy.size() < 2) continue;
                    if (std::find(t.absorbed.begin(), t.absorbed.end(), static_cast<int>(j)) != t.absorbed.end()) continue;
                    const Chain& cj = chains[j];
                    if (distance(cj.xy.front(), head) < 1.5) { double dt = dot(tangentAt(cj.xy, 0), dirIn); if (dt > bestDot) { bestDot = dt; best = static_cast<int>(j); bestAtEnd = false; } }
                    else if (distance(cj.xy.back(), head) < 1.5) { double dt = dot(tangentAt(cj.xy, cj.xy.size() - 1) * -1.0, dirIn); if (dt > bestDot) { bestDot = dt; best = static_cast<int>(j); bestAtEnd = true; } }
                }
                if (best < 0) {   // why not: what meets the head, and how squarely
                    t.headCount = 0; double bd = -1;
                    for (size_t j = 0; j < chains.size(); ++j) { if (isLoop[j] || static_cast<int>(j) == t.chain || chains[j].xy.size() < 2) continue; const Chain& cj = chains[j];
                        if (distance(cj.xy.front(), head) < 1.5) { ++t.headCount; bd = std::max(bd, dot(tangentAt(cj.xy, 0), dirIn)); } else if (distance(cj.xy.back(), head) < 1.5) { ++t.headCount; bd = std::max(bd, dot(tangentAt(cj.xy, cj.xy.size() - 1) * -1.0, dirIn)); } }
                    t.headDeg = bd > -1 ? static_cast<int>(std::acos(std::max(-1.0, std::min(1.0, bd))) * 180.0 / M_PI) : -1; return;
                }
                std::vector<Vec2> add = chains[static_cast<size_t>(best)].xy; if (!bestAtEnd) std::reverse(add.begin(), add.end());   // add now ENDS at head
                double added = stations(add).back(); std::vector<Vec2> line(add.begin(), add.end() - 1); line.insert(line.end(), t.line.begin(), t.line.end());
                t.line = line; t.lineS = stations(line); t.sCut += added; t.absorbed.push_back(best);
            }
        };
        auto climbAt = [&](const Touch& t, double zf, double dIn, double dOut) {
            Vec2 a = pointAt(t.line, t.lineS, t.sCut - dIn), b = pointAt(t.line, t.lineS, t.sCut + dOut);
            double ga = ground ? ground(a.x, a.y) : 0.0, gb = ground ? ground(b.x, b.y) : 0.0; return std::max(std::fabs(zf - ga), std::fabs(zf - gb));
        };
        for (Touch& t : touches) {
            if (static_cast<int>(chosen.size()) >= o.diamonds) break; bool ok = true;
            Vec2 q = pointAt(loopClosed, loopClosedS, t.loopStation); double zf = loopZ.empty() ? 0.0 : interp(loopClosedS, loopZ, t.loopStation) + o.clearance;
            const Chain& tc = chains[static_cast<size_t>(t.chain)]; Vec2 tanOut = t.atEnd ? tangentAt(tc.xy, tc.xy.size() - 1) : tangentAt(tc.xy, 0) * -1.0;
            bool cliff = false; double prev = ground ? ground(q.x, q.y) : 0.0;
            for (double d = 10.0; d <= 330.0; d += 10) { Vec2 pt = pointAt(t.line, t.lineS, t.sCut + d); double gz = ground ? ground(pt.x, pt.y) : 0.0; if (std::fabs(gz - prev) > 4.0) cliff = true; prev = gz; }   // 4 m in 10 m: a scarp
            double crossAngle = std::acos(std::min(1.0, std::fabs(dot(tanOut, tangentAtStation(loopClosed, loopClosedS, t.loopStation))))) * 180.0 / M_PI;   // 90 = square to the ring
            if (crossAngle < 50.0) { rep.notes += "landing at loop station " + std::to_string(static_cast<int>(t.loopStation)) + " skipped (meets the ring at " + std::to_string(static_cast<int>(crossAngle)) + " deg); "; continue; }
            if (cliff) { rep.notes += "landing at loop station " + std::to_string(static_cast<int>(t.loopStation)) + " skipped (the extension runs off a cliff); "; continue; }
            // the ramps climb from the ground at their FEET; a ramp needs free length for its climb (the lab's rule:
            // climb / g_max x 1.5), so the diamond grows to the climb, and the climb is re-measured at the grown feet
            Landing fit{t.chain, t.loopStation, {}, o.footInside, o.footOutside, o.goreOffset}; double worst = 0;
            for (int pass = 0; pass < 3; ++pass) {
                worst = climbAt(t, zf, fit.footIn, fit.footOut); double need = worst / 0.08 * 1.5 + 20.0;
                extendInward(t, std::max(o.footInside, need + 30.0) + 40.0);
                // the inside foot: the default reach when the street has it, never less than the climb needs, never past the street
                fit.footIn = std::min(std::max(o.footInside, need + 30.0), t.sCut - 40.0); fit.footOut = std::max(o.footOutside, need); fit.gore = std::max(o.goreOffset, need - 60.0);
            }
            double need = worst / 0.08 * 1.5 + 20.0;
            if (worst > o.maxRampClimb) { rep.notes += "landing at loop station " + std::to_string(static_cast<int>(t.loopStation)) + " skipped (ramps would climb " + std::to_string(static_cast<int>(worst)) + " m); "; continue; }
            if (t.sCut < need + 70.0) { std::ostringstream cs; cs << "landing at loop station " << static_cast<int>(t.loopStation) << " skipped (street too short inside the ring: " << static_cast<int>(t.sCut) << " m for a " << static_cast<int>(worst) << " m climb needing " << static_cast<int>(need + 70) << "; its head meets " << t.headCount << " streets, straightest at " << t.headDeg << " deg); "; rep.notes += cs.str(); continue; }
            for (double s : chosenS) { double d = std::fabs(s - t.loopStation); d = std::min(d, rep.loopLength - d); if (d < o.gateSpacing) ok = false; }
            if (!ok) continue;
            bool clash = false; for (int a : t.absorbed) if (std::find(chosen.begin(), chosen.end(), a) != chosen.end() || chains[static_cast<size_t>(a)].xy.empty()) clash = true;
            if (clash) { rep.notes += "landing at loop station " + std::to_string(static_cast<int>(t.loopStation)) + " skipped (shares a street with another landing); "; continue; }
            chosen.push_back(t.chain); chosenS.push_back(t.loopStation); fitted[t.chain] = fit;
            { std::ostringstream as; as << "landing c" << t.chain << " at loop station " << static_cast<int>(t.loopStation) << " absorbed"; for (int a : t.absorbed) as << " c" << a; as << " (line " << static_cast<int>(t.sCut) << " m to the crossing); "; rep.notes += as.str(); }
            for (int a : t.absorbed) chains[static_cast<size_t>(a)].xy.clear();   // part of the landing edge now
        }
        for (const Touch& t : touches) {
            if (std::find(chosen.begin(), chosen.end(), t.chain) == chosen.end()) continue;
            // the landing street (grown inward) runs straight through its crossing and on beyond it to the diamond's outer feet
            Landing ld = fitted[t.chain]; double landLen = ld.footOut + 60.0;
            Vec2 tan = tangentAt(t.line, t.line.size() - 1); std::vector<Vec2> xy;
            for (size_t k = 0; k < t.line.size() && t.lineS[k] < t.sCut - 1.0; ++k) xy.push_back(t.line[k]);
            xy.push_back(t.xpt); int n = static_cast<int>(landLen / 5); for (int k = 1; k <= n; ++k) xy.push_back(t.xpt + tan * (landLen * k / n));
            ld.xy = xy; landings.push_back(ld); chains[static_cast<size_t>(t.chain)].xy = xy;
        }
        // every other street keeps its longest run inside the frontage band and ends on the frontage road's
        // centreline as a T: no street reaches the freeway at grade, and none stops short of anything.
        double dFront = edgeReach + o.frontageSetback;
        auto inside = [&](const Vec2& p) { return pointInRing(loop, p) && project(loopClosed, loopClosedS, p).distance >= dFront; };
        auto toFrontage = [&](std::vector<Vec2>& poly, bool atEnd) {
            Vec2 last = atEnd ? poly.back() : poly.front(); Projection pr = project(loopClosed, loopClosedS, last);
            Vec2 towards = pointAt(loopClosed, loopClosedS, pr.station); Vec2 dir = normalize(towards - last); Vec2 p = last + dir * std::max(0.0, pr.distance - dFront);
            if (atEnd) poly.push_back(p); else poly.insert(poly.begin(), p);
        };
        // Perimeter chains are not exempt: the ring is redrawn at the design radius, so a concave notch of the
        // old perimeter ends up deep inside the freeway — its streets stay streets (a 4-way crossing on the old
        // perimeter kept only its two interior arms before, meeting at a point in the void: "a chunk taken out
        // of a bend", 2026-09-07). A perimeter chain the freeway actually replaces has no run inside the band.
        int ringKept = 0;
        for (size_t i = 0; i < chains.size(); ++i) {
            if (chains[i].xy.size() < 2) continue;
            bool landing = false; for (const Landing& ld : landings) if (ld.chain == static_cast<int>(i)) landing = true; if (landing) continue;
            std::vector<Vec2>& xy = chains[i].xy; size_t bs = 0, bl = 0, cs = 0, cl = 0;
            for (size_t k = 0; k <= xy.size(); ++k) { if (k < xy.size() && inside(xy[k])) { if (!cl) cs = k; ++cl; } else { if (cl > bl) { bl = cl; bs = cs; } cl = 0; } }
            auto keepRing = [&](double len) { if (!isLoop[i]) return; isLoop[i] = false; ++ringKept; std::ostringstream rs; rs.setf(std::ios::fixed); rs.precision(0); rs << "perimeter chain c" << i << " kept as a street (" << len << " m inside the band); "; rep.notes += rs.str(); };
            if (bl == xy.size()) { keepRing(stations(xy).back()); continue; }
            std::vector<Vec2> kept; if (bl >= 2) kept.assign(xy.begin() + static_cast<long>(bs), xy.begin() + static_cast<long>(bs + bl));
            if (isLoop[i] && (kept.size() < 2 || stations(kept).back() < 30)) { xy.clear(); continue; }   // the ring proper: the freeway replaces it
            if (kept.size() < 2 || stations(kept).back() < 30) {
                { std::ostringstream ds; ds.setf(std::ios::fixed); ds.precision(0); ds << "dropped chain " << i << " (" << (kept.size() < 2 ? 0.0 : stations(kept).back()) << " m";
                  if (!xy.empty()) ds << " of " << stations(xy).back() << " m, " << xy.front().x << "," << xy.front().y << " to " << xy.back().x << "," << xy.back().y; ds << "); "; rep.notes += ds.str(); }
                xy.clear(); ++rep.dropped; continue;
            }
            if (bs + bl < xy.size()) toFrontage(kept, true); if (bs > 0) toFrontage(kept, false);
            const bool wasRing = isLoop[i]; keepRing(stations(kept).back());
            xy = kept; if (!wasRing) ++rep.trimmed;
        }
        if (ringKept) rep.notes += std::to_string(ringKept) + " perimeter chains kept as streets inside the redraw's notches; ";
        {   // Two arms of one class meeting at a dead point are ONE street that bends there: the arm that made the
            // point a junction was consumed (the ring redraw, a dropped stub, a landing). Joined into one chain
            // with the joint rounded at the class's radius, so the lab builds a bend, not a junction box with no
            // third road. A class change is left alone and reported below.
            auto radiusFor = [](RoadClass k) { return k == RoadClass::Arterial ? 30.0 : k == RoadClass::Collector ? 20.0 : 12.0; };
            // Round the joint at index j: trim R*tan(turn/2) of polyline either side (never more than 45 % of the
            // shorter leg) and bridge with a quadratic curve controlled from the joint — smooth, radius ~R.
            auto fillet = [&](std::vector<Vec2>& xy, size_t j, double R) {
                if (j == 0 || j + 1 >= xy.size()) return 0.0;
                std::vector<double> s = stations(xy);
                const Vec2 d1 = normalize(xy[j] - xy[j - 1]), d2 = normalize(xy[j + 1] - xy[j]);
                const double theta = std::acos(std::max(-1.0, std::min(1.0, dot(d1, d2))));   // 0 = straight through
                if (theta < 10.0 * M_PI / 180.0) return 0.0;
                double t = R * std::tan(std::min(theta, 150.0 * M_PI / 180.0) / 2); const double maxT = 0.45 * std::min(s[j], s.back() - s[j]);
                if (t > maxT) { t = maxT; R = t / std::tan(std::min(theta, 150.0 * M_PI / 180.0) / 2); }
                if (t < 1.0) return 0.0;
                const double s0 = s[j] - t, s1 = s[j] + t; const Vec2 p0 = pointAt(xy, s, s0), p1 = pointAt(xy, s, s1);
                // control point: where the legs' tangents meet (the joint itself when the legs are straight)
                const Vec2 t0 = tangentAtStation(xy, s, s0), t1 = tangentAtStation(xy, s, s1); Vec2 c = xy[j];
                { const double den = t0.x * t1.y - t0.y * t1.x; if (std::fabs(den) > 1e-6) { const Vec2 w = p1 - p0; const double u = (w.x * t1.y - w.y * t1.x) / den; if (u > 0 && u < 3 * t) c = p0 + t0 * u; } }
                const int n = std::max(8, static_cast<int>(std::ceil(R * theta)));   // ~1 m per sample
                std::vector<Vec2> out; for (size_t i = 0; i < xy.size(); ++i) if (s[i] < s0 - 0.5) out.push_back(xy[i]);   // no slivers beside the curve
                for (int k = 0; k <= n; ++k) { const double u = static_cast<double>(k) / n; out.push_back(p0 * ((1 - u) * (1 - u)) + c * (2 * u * (1 - u)) + p1 * (u * u)); }
                for (size_t i = 0; i < xy.size(); ++i) if (s[i] > s1 + 0.5) out.push_back(xy[i]);
                xy = out; return R;
            };
            int joined = 0; std::ostringstream js; js.setf(std::ios::fixed); js.precision(0);
            for (bool again = true; again;) {
                again = false;
                struct End { Vec2 p; int chain; bool atEnd; }; std::vector<End> ends;
                for (size_t i = 0; i < chains.size(); ++i) {
                    if (isLoop[i] || chains[i].xy.size() < 2) continue;
                    bool landing = false; for (const Landing& ld : landings) if (ld.chain == static_cast<int>(i)) landing = true; if (landing) continue;   // landings carry ramps by id: never joined
                    ends.push_back({chains[i].xy.front(), static_cast<int>(i), false}); ends.push_back({chains[i].xy.back(), static_cast<int>(i), true});
                }
                std::vector<char> seen(ends.size(), 0);
                for (size_t a = 0; a < ends.size() && !again; ++a) {
                    if (seen[a]) continue; std::vector<size_t> grp{a}; seen[a] = 1;
                    for (size_t b = a + 1; b < ends.size(); ++b) if (!seen[b] && distance(ends[a].p, ends[b].p) < 1.5) { grp.push_back(b); seen[b] = 1; }
                    if (grp.size() != 2) continue;
                    const End e0 = ends[grp[0]], e1 = ends[grp[1]]; if (e0.chain == e1.chain) continue;
                    Chain& A = chains[static_cast<size_t>(e0.chain)]; Chain& B = chains[static_cast<size_t>(e1.chain)]; if (A.klass != B.klass) continue;
                    std::vector<Vec2> ax = A.xy; if (!e0.atEnd) std::reverse(ax.begin(), ax.end());   // ends at the joint
                    std::vector<Vec2> bx = B.xy; if (e1.atEnd) std::reverse(bx.begin(), bx.end());    // starts at the joint
                    const double turn = std::acos(std::max(-1.0, std::min(1.0, dot(normalize(ax.back() - ax[ax.size() - 2]), normalize(bx[1] - bx[0]))))) * 180.0 / M_PI;
                    std::vector<Vec2> xy(ax.begin(), ax.end()); xy.insert(xy.end(), bx.begin() + 1, bx.end());
                    const double R = fillet(xy, ax.size() - 1, radiusFor(A.klass));
                    ++joined; if (joined <= 12) js << " c" << e0.chain << "+c" << e1.chain << " at " << e0.p.x << "," << e0.p.y << " (" << turn << " deg turn" << (R > 0 ? ", R " + std::to_string(static_cast<int>(R)) + " m" : "") << ")";
                    A.xy = xy; A.s = stations(xy); B.xy.clear(); again = true;
                }
            }
            if (joined) rep.notes += std::to_string(joined) + " two-arm dead points joined into bends:" + js.str() + "; ";
        }
        {   // Two-arm dead points: exactly two chain ends meet and nothing passes through — the lab would build a
            // junction box with no third road ("a chunk taken out of a bend"). Reported, never silently fixed.
            struct End { Vec2 p; int chain; bool atEnd; };
            std::vector<End> ends;
            for (size_t i = 0; i < chains.size(); ++i) { if (isLoop[i] || chains[i].xy.size() < 2) continue; ends.push_back({chains[i].xy.front(), static_cast<int>(i), false}); ends.push_back({chains[i].xy.back(), static_cast<int>(i), true}); }
            std::vector<char> seen(ends.size(), 0); int twoArm = 0; std::ostringstream ts; ts.setf(std::ios::fixed); ts.precision(0);
            for (size_t a = 0; a < ends.size(); ++a) {
                if (seen[a]) continue; std::vector<size_t> grp{a}; seen[a] = 1;
                for (size_t b = a + 1; b < ends.size(); ++b) if (!seen[b] && distance(ends[a].p, ends[b].p) < 1.5) { grp.push_back(b); seen[b] = 1; }
                if (grp.size() != 2) continue;
                const End& e0 = ends[grp[0]]; const End& e1 = ends[grp[1]]; if (e0.chain == e1.chain) continue;
                auto away = [&](const End& e) { const std::vector<Vec2>& xy = chains[static_cast<size_t>(e.chain)].xy; return e.atEnd ? tangentAt(xy, xy.size() - 1) * -1.0 : tangentAt(xy, 0); };
                const double turn = std::acos(std::max(-1.0, std::min(1.0, dot(away(e0), away(e1))))) * 180.0 / M_PI;   // 180 = straight through, 0 = hairpin
                ++twoArm; if (twoArm <= 12) ts << " c" << e0.chain << "/c" << e1.chain << " at " << e0.p.x << "," << e0.p.y << " (" << turn << " deg between the arms)";
            }
            if (twoArm) rep.notes += std::to_string(twoArm) + " two-arm dead points:" + ts.str() + "; ";
        }
        rep.landings = static_cast<int>(landings.size());
        // carriageways: inner clockwise (centre on the right), outer counter-clockwise; split at the two largest gaps between landings
        std::vector<Vec2> inner = offsetLoop(loop, +dCarriage), outer = offsetLoop(loop, -dCarriage);   // inner runs clockwise: each inner arc is reversed when sliced
        std::vector<double> cuts;
        if (chosenS.size() >= 2) {
            std::vector<double> ss = chosenS; std::sort(ss.begin(), ss.end()); std::vector<std::pair<double, double>> gaps;
            for (size_t i = 0; i < ss.size(); ++i) { double a = ss[i], b = i + 1 < ss.size() ? ss[i + 1] : ss[0] + rep.loopLength; gaps.push_back({b - a, std::fmod((a + b) / 2, rep.loopLength)}); }
            std::sort(gaps.rbegin(), gaps.rend()); cuts = {gaps[0].second, gaps[1].second};
        } else cuts = {0.0, rep.loopLength / 2};
        std::sort(cuts.begin(), cuts.end());
        // arcs are index slices of the offset rings between the two cut stations: contiguous by construction, the seam wraps
        size_t c0 = static_cast<size_t>(std::lower_bound(loopS.begin(), loopS.end(), cuts[0]) - loopS.begin()) % loop.size();
        size_t c1 = static_cast<size_t>(std::lower_bound(loopS.begin(), loopS.end(), cuts[1]) - loopS.begin()) % loop.size();
        auto arcsOf = [&](const std::vector<Vec2>& ring, const char* prefix, bool cw) {
            std::vector<std::vector<Vec2>> arcs = {ringSlice(ring, c0, c1), ringSlice(ring, c1, c0)};
            for (int k = 0; k < 2; ++k) {
                std::vector<Vec2> a = arcs[static_cast<size_t>(k)]; if (cw) std::reverse(a.begin(), a.end());
                if (a.size() < 2) continue;
                nlohmann::json e; e["id"] = std::string(prefix) + (k == 0 ? "_a" : "_b"); e["class"] = "freeway"; e["path"] = {{"type", "polyline"}, {"points", pointList(a)}};
                e["floor"] = nlohmann::json::array(); edges.push_back(e);
            }
        };
        arcsOf(inner, "in", true); arcsOf(outer, "out", false);
        if (o.frontage) {
            std::vector<Vec2> fr = offsetLoop(loop, +(edgeReach + o.frontageSetback));
            std::vector<std::vector<Vec2>> arcs = {ringSlice(fr, c0, c1), ringSlice(fr, c1, c0)};
            for (int k = 0; k < 2; ++k) { if (arcs[static_cast<size_t>(k)].size() < 2) continue; nlohmann::json e; e["id"] = std::string("fr") + (k == 0 ? "_a" : "_b"); e["class"] = "collector"; e["path"] = {{"type", "polyline"}, {"points", pointList(arcs[static_cast<size_t>(k)])}}; edges.push_back(e); }
        }
        { std::ostringstream dbg; dbg << "landings at loop stations"; for (double s : chosenS) dbg << " " << static_cast<int>(s); dbg << "; cuts at " << static_cast<int>(cuts[0]) << " " << static_cast<int>(cuts[1]) << "; arcs";
          for (const auto& e : edges) if (e["class"] == "freeway") { std::vector<Vec2> a; for (const auto& p : e["path"]["points"]) a.emplace_back(p.at(0).get<double>(), p.at(1).get<double>()); double jump = 0; for (size_t i = 0; i + 1 < a.size(); ++i) jump = std::max(jump, distance(a[i], a[i+1])); dbg << " " << e["id"].get<std::string>() << "=" << static_cast<int>(stations(a).back()) << "m(jump " << static_cast<int>(jump) << ")"; }
          rep.notes += dbg.str() + "; "; }
        // landing streets pass UNDER the freeway: each carriageway gets a floor tent where the street crosses it
        for (const Landing& ld : landings) {
            for (auto& e : edges) {
                if (e["class"] != "freeway") continue; std::vector<Vec2> arc; for (const auto& p : e["path"]["points"]) arc.emplace_back(p.at(0).get<double>(), p.at(1).get<double>());
                for (const Vec2& x : crossings(arc, ld.xy)) { double zStreet = ground ? ground(x.x, x.y) : 0.0; e["floor"].push_back({x.x, x.y, zStreet + o.clearance + 0.6}); }
            }
        }
    }
    // --- streets (arterials get left-turn pockets on every approach to another arterial) ---
    std::map<int, nlohmann::json> pockets; int pocketCount = 0;
    // chains END at junctions (they never cross mid-chain): an arterial chain whose end meets another arterial gets a
    // left-turn pocket on that approach — forward traffic approaching the chain's end, backward traffic approaching its start
    if (o.turnPockets) for (size_t i = 0; i < chains.size(); ++i) {
        if (isLoop[i] || chains[i].xy.size() < 2 || chains[i].klass != RoadClass::Arterial) continue;
        const Chain& c = chains[i]; std::vector<double> cs = stations(c.xy); double L = cs.back(); if (L < 70) continue;
        // a signalised approach: the chain's end meets another arterial or a collector
        bool meets[2] = {false, false};
        for (int end = 0; end < 2; ++end) {
            Vec2 p = end ? c.xy.back() : c.xy.front();
            for (size_t j = 0; j < chains.size() && !meets[end]; ++j) {
                if (j == i || isLoop[j] || chains[j].xy.size() < 2) continue;
                if (chains[j].klass != RoadClass::Arterial && chains[j].klass != RoadClass::Collector) continue;
                if (project(chains[j].xy, stations(chains[j].xy), p).distance < 1.5) meets[end] = true;
            }
        }
        int n = (meets[0] ? 1 : 0) + (meets[1] ? 1 : 0); if (n == 0) continue;
        // both approaches share the ONE median (gap = one lane): the block is split between them, each pocket plus its
        // taper staying 15 m clear of its junction and clear of the other pocket. Short blocks get short pockets.
        double P = std::min(85.0, (L - 30.0) / (1.5 * n)); if (P < 20) continue; double taper = std::min(30.0, P / 2);
        if (meets[1]) pockets[static_cast<int>(i)].push_back({{"id", "t" + std::to_string(pocketCount++)}, {"side", "left"}, {"s0", L - 15 - P}, {"s1", L - 15}, {"taper", taper}, {"kind", "turn"}});
        if (meets[0]) pockets[static_cast<int>(i)].push_back({{"id", "t" + std::to_string(pocketCount++)}, {"side", "left"}, {"dir", "back"}, {"s0", 15.0}, {"s1", 15.0 + P}, {"taper", taper}, {"kind", "turn"}});
    }
    rep.notes += std::to_string(pocketCount) + " turn pockets; ";
    for (size_t i = 0; i < chains.size(); ++i) {
        if (isLoop[i] || chains[i].xy.size() < 2) continue; const Chain& c = chains[i];
        nlohmann::json e; e["id"] = "c" + std::to_string(i); e["class"] = className(c.klass);
        double lw = c.klass == RoadClass::Arterial || c.klass == RoadClass::Collector ? 3.25 : 3.5; int per = std::max(1, std::min(3, static_cast<int>(std::lround(c.hw / lw))));
        if (c.klass == RoadClass::Alley) e["lanes"] = {{"w", lw}, {"fwd", 1}, {"back", 0}}; else e["lanes"] = {{"w", lw}, {"fwd", per}, {"back", per}};
        if (c.klass == RoadClass::Arterial && o.turnPockets) { e["lanes"]["gap"] = 3.25; if (pockets.count(static_cast<int>(i))) e["lanes"]["pockets"] = pockets[static_cast<int>(i)]; }
        for (const Landing& ld : landings) if (ld.chain == static_cast<int>(i) && !loop.empty()) {   // a landing street: the city stops at the outer frontage road
            std::vector<Vec2> lc = loop; lc.push_back(loop.front()); std::vector<double> lcs = stations(lc); std::vector<double> cs2 = stations(c.xy);
            double sQ = project(c.xy, cs2, pointAt(lc, lcs, ld.loopStation)).station;
            double dOuter = o.medianGap / 2 + o.freewayLanes * o.freewayLaneW + 2.5 + o.frontageSetback;
            e["lots_range"] = {0.0, sQ + dOuter + 5.0};
        }
        e["path"] = {{"type", "polyline"}, {"points", pointList(c.xy)}}; edges.push_back(e); ++rep.streets; 
    }
    // --- diamonds ---
    if (!loop.empty()) {
        std::vector<Vec2> loopClosed = loop; loopClosed.push_back(loop.front()); std::vector<double> loopClosedS = stations(loopClosed);
        auto arcPolyline = [&](const std::string& id) { std::vector<Vec2> a; for (const auto& e : edges) if (e["id"] == id) for (const auto& p : e["path"]["points"]) a.emplace_back(p.at(0).get<double>(), p.at(1).get<double>()); return a; };
        std::vector<std::pair<std::string, std::vector<Vec2>>> arcs; for (const char* id : {"in_a", "in_b", "out_a", "out_b"}) { auto a = arcPolyline(id); if (!a.empty()) arcs.emplace_back(id, a); }
        auto nearestArc = [&](const Vec2& p, bool inner) { std::string best; double bd = 1e300; for (const auto& a : arcs) { if ((a.first.rfind("in_", 0) == 0) != inner) continue; double d = project(a.second, stations(a.second), p).distance; if (d < bd) { bd = d; best = a.first; } } return best; };
        auto loopPoint = [&](double st) { double L = loopClosedS.back(); st = std::fmod(std::fmod(st, L) + L, L); return pointAt(loopClosed, loopClosedS, st); };
        int di = 0;
        // Ramps live in the band beside the freeway and end on the first road they meet (Glenn, 2026-09-07: "they
        // should descend to the city road level and intersect with the first road"): inside, the frontage road — a
        // T 70 m before or after the landing junction; outside, the landing street itself, 35 and 75 m beyond the
        // freeway's edge. Each ramp is as long as its climb needs (8 % with the design margin, plus the approach),
        // no longer, so none crosses a block. The old feet — dovetail merges 170–230 m along the landing street,
        // reached by 300–600 m diagonals — put an on-ramp down the centreline of local street c6 for 75 m.
        const double L = loopClosedS.back();
        const double dCarriage = o.medianGap / 2 + o.freewayLanes * o.freewayLaneW / 2, edgeReach = dCarriage + o.freewayLanes * o.freewayLaneW / 2 + 2.5, dFront = edgeReach + o.frontageSetback;
        auto wrap = [&](double st) { return std::fmod(std::fmod(st, L) + L, L); };
        auto inward = [&](double st) { return perp(tangentAtStation(loopClosed, loopClosedS, wrap(st))); };   // offsetLoop(+d) is inward: toward the centre and the frontage road
        auto frontPoint = [&](double st) { return loopPoint(st) + inward(st) * dFront; };
        std::vector<double> loopZ; if (ground) { HeightField hf = ground; loopZ = profileAlong(loopClosed, hf, o.freewayWindow, o.freewayGMax); }
        auto zAt = [&](double st) { return loopZ.empty() ? 0.0 : interp(loopClosedS, loopZ, wrap(st)) + o.clearance; };   // the deck over a landing street: ground profile + clearance
        auto gAt = [&](const Vec2& p) { return ground ? ground(p.x, p.y) : 0.0; };
        auto ringDist = [&](double a, double b) { double d = std::fabs(wrap(a) - wrap(b)); return std::min(d, L - d); };
        std::vector<double> tStations;   // street Ts on the frontage road, as loop stations
        if (o.frontage) { std::vector<Vec2> fr = offsetLoop(loop, dFront); std::vector<Vec2> frClosed = fr; frClosed.push_back(fr.front()); std::vector<double> frS = stations(frClosed);
            for (size_t i = 0; i < chains.size(); ++i) { if (isLoop[i] || chains[i].xy.size() < 2) continue; for (const Vec2& q : {chains[i].xy.front(), chains[i].xy.back()}) if (project(frClosed, frS, q).distance < 3.0) tStations.push_back(project(loopClosed, loopClosedS, q).station); } }
        auto clearOfTs = [&](double st, double need) { for (double m : tStations) if (ringDist(st, m) < need) return false; return true; };
        auto clearOfEnds = [&](const std::string& arcId, const Vec2& gore, double need) {
            for (const auto& a : arcs) if (a.first == arcId) { std::vector<double> as = stations(a.second); double st = project(a.second, as, gore).station; return st > need && as.back() - st > need; }
            return false;
        };
        const double dec = 80, tapOff = 72, aux = 110, tapOn = 90, appr = 60, tOut = 45.0;   // the outer terminal: one four-way on the landing street, 45 m past the freeway's edge
        const double bandClimbMax = 32.0;   // a band ramp beside a high viaduct is simply long (32 m ~ 600 m); the old 18 m cap sized diamond feet
        // the frontage terminal: the first offset from the landing junction (70 … 250 m) that keeps 45 m from every street T
        auto terminalOffset = [&](double S, double sign) { for (double t : {70.0, 100.0, 130.0, 160.0, 190.0, 220.0, 250.0}) if (!o.frontage || clearOfTs(S + sign * t, 45.0)) return t; return -1.0; };
        // band occupancy: two diamonds' band runs never overlap (inner and outer kept apart, 10 m between); intervals in loop stations
        std::vector<std::pair<double, double>> usedInner, usedOuter;
        auto overlaps = [&](const std::vector<std::pair<double, double>>& used, double a, double b) {
            auto norm = [&](double x, double y) { std::vector<std::pair<double, double>> v; x = wrap(x); y = wrap(y); if (x <= y) v.push_back({x, y}); else { v.push_back({x, L}); v.push_back({0, y}); } return v; };
            for (const auto& iv : norm(a, b)) for (const auto& u : used) for (const auto& uv : norm(u.first, u.second)) if (iv.first < uv.second + 10.0 && uv.first < iv.second + 10.0) return true;
            return false;
        };
        // A band ramp is planned at the lab's design grade for ramps (g_max / 1.5, the free-length rule). Beside a
        // freeway that climbs away from it at 5-6 % that never converges — at 5.3 % a ramp cannot catch a 4.8 % freeway —
        // so it falls back to the class grade less a little (7.5 %); the lab's free-length line then reports it short
        // against the 1.5 margin, which is the honest thing to say about a ramp beside a steep freeway.
        const double designGrade = 0.08 / 1.5, steepGrade = 0.075;
        // the ramp's run from its T (at loop station sT, ground point tp) to its gore, `sign` stations away: the gore height
        // depends on where the gore lands, so iterate to a fixed point; false when the freeway outruns the ramp at both grades
        auto fitAt = [&](double grade, double sT, const Vec2& tp, double sign, double& climb, double& run) {
            auto runFor = [&](double c) { return std::max(150.0, std::fabs(c) / grade * 1.05 + appr); };   // 5 % over: the deck rides a little above the ground-profile estimate
            run = 200;
            for (int it = 0; it < 80; ++it) { climb = zAt(sT + sign * run) - gAt(tp); const double next = runFor(climb); if (std::fabs(next - run) < 0.5) { run = next; return run <= 900.0; } run = next; if (run > 900.0) return false; }
            return run <= 900.0;
        };
        // the design grade wins when it lands a ramp of ordinary length; a fixed point 700 m away beside a climbing freeway
        // is not a design, it is the steep case
        auto fitRamp = [&](double sT, const Vec2& tp, double sign, double& climb, double& run) {
            double c1 = 0, r1 = 0; const bool ok1 = fitAt(designGrade, sT, tp, sign, c1, r1) && r1 <= 600.0 && std::fabs(c1) <= bandClimbMax;
            if (ok1) { climb = c1; run = r1; return true; }
            return fitAt(steepGrade, sT, tp, sign, climb, run);
        };
        // A band ramp's spine: from the gore (the lab supplies the departure point itself) it eases from the gore lane's
        // radial rG out to the band's radial rBand over 80 m and follows the loop's curvature; the last `approach` metres
        // are a cubic from the band into the T point, arriving along `endDir` — INWARD for a T on the frontage road (a
        // tangential road, met squarely), ALONG THE LOOP for a T on the landing street (a radial road, met squarely by
        // simply continuing, shifted sideways). A bezier chord between points 600 m apart on a 220 m ring cut 100 m
        // inside the band; a quarter circle at a radial street hooked the ramp away from the freeway and jogged back.
        // `sign` is the direction of travel in loop stations; radials are positive inward. Reversed for an on-ramp.
        auto bandPath = [&](double sGore, double sT, double sign, double rG, double rBand, const Vec2& tPoint, const Vec2& endDir, double approach, bool toGore) {
            std::vector<Vec2> pts; const double Lrun = std::fabs(sT - sGore);
            auto at = [&](double st, double r) { return loopPoint(st) + inward(st) * r; };
            for (double d = 0; d < Lrun - approach; d += 5.0) { const double u = std::min(1.0, d / 80.0), w = u * u * (3 - 2 * u); pts.push_back(at(sGore + sign * d, rG + (rBand - rG) * w)); }
            const double sA = sT - sign * approach; const Vec2 A = at(sA, rBand), tA = tangentAtStation(loopClosed, loopClosedS, wrap(sA)) * sign;
            const double k = distance(tPoint, A);   // Hermite tangents scaled to the chord: a gentle S or a square turn, no overshoot
            for (int i = 1; i <= 14; ++i) {
                const double u = static_cast<double>(i) / 14, h00 = 2 * u * u * u - 3 * u * u + 1, h10 = u * u * u - 2 * u * u + u, h01 = -2 * u * u * u + 3 * u * u, h11 = u * u * u - u * u;
                pts.push_back(A * h00 + tA * (h10 * k) + tPoint * h01 + endDir * (h11 * k));
            }
            if (!toGore) std::reverse(pts.begin(), pts.end());   // an on-ramp reads T → band → gore
            return pts;
        };
        for (const Landing& ld : landings) {
            std::string sid = "c" + std::to_string(ld.chain); const Chain& c = chains[static_cast<size_t>(ld.chain)]; std::vector<double> ss = stations(ld.xy);
            double sQ = project(ld.xy, ss, loopPoint(ld.loopStation)).station;
            double lw = c.klass == RoadClass::Arterial || c.klass == RoadClass::Collector ? 3.25 : 3.5; int per = std::max(1, std::min(3, static_cast<int>(std::lround(c.hw / lw))));
            double outerLane = (per - 0.5) * lw;                 // |offset| of the outermost lane of each direction (no median)
            double inFoot = std::max(20.0, sQ - ld.footIn), outFoot = std::min(ss.back() - 20.0, sQ + ld.footOut);
            (void)inFoot; (void)outFoot; (void)outerLane;
            const double S = ld.loopStation;
            // inner carriageway (clockwise = decreasing station; the frontage road on its right): off-ramp from a gore upstream
            // to a T on the frontage road at S + tOff; on-ramp from a T at S - tOn to a merge downstream
            const double tOffIn = terminalOffset(S, +1.0), tOnIn = terminalOffset(S, -1.0);
            if (tOffIn < 0 || tOnIn < 0) { rep.notes += "diamond at loop station " + std::to_string(static_cast<int>(S)) + " skipped (no frontage terminal 45 m clear of a street T within 250 m); "; continue; }
            const Vec2 tInOff = frontPoint(S + tOffIn), tInOn = frontPoint(S - tOnIn);
            double cInOff = 0, LInOff = 0, cInOn = 0, LInOn = 0; bool fits = fitRamp(S + tOffIn, tInOff, +1.0, cInOff, LInOff); fits = fitRamp(S - tOnIn, tInOn, -1.0, cInOn, LInOn) && fits;
            // outer carriageway (counter-clockwise = increasing station; right side outward): off-ramp from a gore upstream and
            // on-ramp to a merge downstream, meeting the landing street at ONE point 45 m outside the freeway's edge — a four-way
            // ramp terminal, the ramps on opposite sides of the street
            if (ss.back() < sQ + edgeReach + tOut + 40.0) { rep.notes += "diamond at loop station " + std::to_string(static_cast<int>(S)) + " skipped (landing street too short outside for its ramp terminals); "; continue; }
            const Vec2 tOutOff = pointAt(ld.xy, ss, sQ + edgeReach + tOut), tOutOn = tOutOff;
            double cOutOff = 0, LOutOff = 0, cOutOn = 0, LOutOn = 0; fits = fitRamp(S, tOutOff, -1.0, cOutOff, LOutOff) && fits; fits = fitRamp(S, tOutOn, +1.0, cOutOn, LOutOn) && fits;
            if (!fits) { rep.notes += "diamond at loop station " + std::to_string(static_cast<int>(S)) + " skipped (the freeway climbs away faster than a band ramp can follow, or a ramp would pass 900 m); "; continue; }
            const double worstClimb = std::max({cInOff, cInOn, cOutOff, cOutOn});
            if (worstClimb > bandClimbMax) { std::ostringstream cs; cs << "diamond at loop station " << static_cast<int>(S) << " skipped (a band ramp would climb " << static_cast<int>(worstClimb) << " m); "; rep.notes += cs.str(); continue; }
            // the stretches of band each ramp occupies: T to gore. Gore runs (the decel and aux lanes on the carriageway) may
            // meet or overlap between neighbouring diamonds — an on-ramp's aux lane running into the next off-ramp's decel lane
            // is a weave lane, which is how close interchanges are built
            const std::pair<double, double> bInOff{S + tOffIn, S + tOffIn + LInOff}, bInOn{S - tOnIn - LInOn, S - tOnIn}, bOutOff{S - LOutOff, S}, bOutOn{S, S + LOutOn};
            if (overlaps(usedInner, bInOff.first, bInOff.second) || overlaps(usedInner, bInOn.first, bInOn.second) || overlaps(usedOuter, bOutOff.first, bOutOff.second) || overlaps(usedOuter, bOutOn.first, bOutOn.second)) {
                rep.notes += "diamond at loop station " + std::to_string(static_cast<int>(S)) + " skipped (its band ramps would run into another diamond's); "; continue; }
            const Vec2 gInOff = loopPoint(S + tOffIn + LInOff), gInOn = loopPoint(S - tOnIn - LInOn), gOutOff = loopPoint(S - LOutOff), gOutOn = loopPoint(S + LOutOn);
            const std::string aInOff = nearestArc(gInOff, true), aInOn = nearestArc(gInOn, true), aOutOff = nearestArc(gOutOff, false), aOutOn = nearestArc(gOutOn, false);
            bool ok = clearOfEnds(aInOff, gInOff, dec + tapOff + 20) && clearOfEnds(aInOn, gInOn, aux + tapOn + 20) && clearOfEnds(aOutOff, gOutOff, dec + tapOff + 20) && clearOfEnds(aOutOn, gOutOn, aux + tapOn + 20);
            if (!ok) { rep.notes += "diamond at loop station " + std::to_string(static_cast<int>(S)) + " skipped (a gore too close to an arc joint); "; continue; }
            // approach controls: the inner Ts are met moving inward off the band, the outer Ts moving along the loop (the landing street is radial there)
            const Vec2 along = tangentAtStation(loopClosed, loopClosedS, wrap(S));   // increasing station = the outer carriageway's direction of travel
            const Vec2 cpInOff = tInOff - inward(S + tOffIn) * 14.0, cpInOn = tInOn - inward(S - tOnIn) * 14.0, cpOutOff = tOutOff - along * 14.0, cpOutOn = tOutOn + along * 14.0;
            usedInner.push_back(bInOff); usedInner.push_back(bInOn); usedOuter.push_back(bOutOff); usedOuter.push_back(bOutOn);
            std::string pre = "d" + std::to_string(di++);
            (void)cpInOff; (void)cpInOn; (void)cpOutOff; (void)cpOutOn;
            const double rGin = dCarriage + 2.0 * o.freewayLaneW, rBandIn = edgeReach + 6.7;            // inner: radials inward; the ramp lane after the taper, then the band centre
            const double rGout = -(dCarriage + 2.0 * o.freewayLaneW), rBandOut = -(edgeReach + 6.7);    // outer: mirrored
            auto offRamp = [&](const std::string& id, const std::string& arc, const Vec2& gore, const std::vector<Vec2>& spine) {
                nlohmann::json e; e["id"] = id; e["class"] = "ramp";
                e["from"] = {{"edge", arc}, {"at", {gore.x, gore.y}}, {"side", "right"}, {"decel", dec}, {"taper", tapOff}, {"approach", appr}};
                e["path"] = {{"type", "polyline"}, {"points", pointList(spine)}}; return e;
            };
            auto onRamp = [&](const std::string& id, const std::string& arc, const Vec2& gore, const std::vector<Vec2>& spine) {
                nlohmann::json e; e["id"] = id; e["class"] = "ramp";
                e["to"] = {{"edge", arc}, {"at", {gore.x, gore.y}}, {"side", "right"}, {"aux", aux}, {"taper", tapOn}, {"approach", appr}};
                e["path"] = {{"type", "polyline"}, {"points", pointList(spine)}}; return e;
            };
            const double inApproach = std::max(30.0, (dFront - rBandIn) * 1.6), outApproach = 70.0;   // a square turn needs ~1.6x its radial offset; a sideways shift, 70 m
            const Vec2 alongS = tangentAtStation(loopClosed, loopClosedS, wrap(S));
            edges.push_back(offRamp(pre + "_in_off", aInOff, gInOff, bandPath(S + tOffIn + LInOff, S + tOffIn, -1.0, rGin, rBandIn, tInOff, inward(S + tOffIn), inApproach, true)));
            edges.push_back(onRamp(pre + "_in_on", aInOn, gInOn, bandPath(S - tOnIn - LInOn, S - tOnIn, +1.0, rGin, rBandIn, tInOn, inward(S - tOnIn), inApproach, false)));
            edges.push_back(offRamp(pre + "_out_off", aOutOff, gOutOff, bandPath(S - LOutOff, S, +1.0, rGout, rBandOut, tOutOff, alongS, outApproach, true)));
            edges.push_back(onRamp(pre + "_out_on", aOutOn, gOutOn, bandPath(S + LOutOn, S, -1.0, rGout, rBandOut, tOutOn, alongS * -1.0, outApproach, false)));
            rep.ramps += 4;
            { std::ostringstream ns; ns.setf(std::ios::fixed); ns.precision(0);
              ns << pre << " at loop station " << S << " on " << sid << ": band ramps in_off " << LInOff << " m (climb " << cInOff << ") to the frontage T at " << tInOff.x << "," << tInOff.y
                 << ", in_on " << LInOn << " m (climb " << cInOn << ") from the frontage T at " << tInOn.x << "," << tInOn.y << ", out_off " << LOutOff << " m (climb " << cOutOff << ") to the street T at " << tOutOff.x << "," << tOutOff.y << ", out_on " << LOutOn << " m (climb " << cOutOn << "); ";
              rep.notes += ns.str(); }
        }
    }
    out["edges"] = edges;
    // --- terrain: the level ground baked to a grid over the roads' extent ---
    double x0 = 1e300, x1 = -1e300, y0 = 1e300, y1 = -1e300;
    for (const auto& e : edges) if (e["path"].contains("points")) for (const auto& p : e["path"]["points"]) { if (!p.is_array()) continue; double x = p.at(0).get<double>(), y = p.at(1).get<double>(); if (x == 0 && y == 0) continue; x0 = std::min(x0, x); x1 = std::max(x1, x); y0 = std::min(y0, y); y1 = std::max(y1, y); }
    x0 -= o.margin; x1 += o.margin; y0 -= o.margin; y1 += o.margin;
    if (ground) {
        int nx = static_cast<int>((x1 - x0) / o.terrainRes) + 1, ny = static_cast<int>((y1 - y0) / o.terrainRes) + 1; std::vector<double> z(static_cast<size_t>(nx) * ny);
        for (int j = 0; j < ny; ++j) for (int i = 0; i < nx; ++i) z[static_cast<size_t>(j) * nx + i] = std::round(ground(x0 + i * o.terrainRes, y0 + j * o.terrainRes) * 1000) / 1000;
        std::string tfile = outDir + "/" + name + "_terrain.json"; std::ofstream tf(tfile); tf << nlohmann::json{{"x0", x0}, {"y0", y0}, {"res", o.terrainRes}, {"nx", nx}, {"ny", ny}, {"z", z}}.dump();
        out["terrain"] = {{"type", "grid"}, {"file", name + "_terrain.json"}, {"bounds", {x0, x1, y0, y1}}, {"res", o.terrainRes}};
    } else out["terrain"] = {{"type", "flat"}, {"bounds", {x0, x1, y0, y1}}};
    std::ostringstream n; n << "loop " << rep.loopChains << " chains, " << rep.loopLength << " m; " << rep.streets << " streets (" << rep.trimmed << " trimmed short of the freeway, " << rep.dropped << " dropped), " << rep.landings << " landings, " << rep.ramps << " ramps";
    rep.notes += n.str(); return out;
}

}  // namespace roads::lanes
}  // namespace engine
