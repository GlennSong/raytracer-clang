#include "island_rail.h"

#include <algorithm>
#include <functional>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <queue>

namespace engine {

namespace {

using json = nlohmann::json;

std::vector<Vec2> pointsOf(const json& e) {
    std::vector<Vec2> p;
    if (!e.contains("path") || !e["path"].contains("points")) return p;
    for (const json& q : e["path"]["points"]) p.push_back(Vec2(q[0].get<double>(), q[1].get<double>()));
    return p;
}

double dist(Vec2 a, Vec2 b) { return (a - b).length(); }

// The freeway's closed chains: its edges joined end to start (an island scene cuts each carriageway in two).
std::vector<std::vector<Vec2>> freewayLoops(const json& graph) {
    std::vector<std::vector<Vec2>> pieces;
    for (const json& e : graph["edges"])
        if (e.value("class", "") == "freeway") { std::vector<Vec2> p = pointsOf(e); if (p.size() > 1) pieces.push_back(p); }
    std::vector<std::vector<Vec2>> loops;
    std::vector<char> used(pieces.size(), 0);
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        if (used[i]) continue;
        used[i] = 1;
        std::vector<Vec2> chain = pieces[i];
        for (bool grew = true; grew && dist(chain.front(), chain.back()) > 1.0;) {
            grew = false;
            for (std::size_t j = 0; j < pieces.size(); ++j)
                if (!used[j] && dist(pieces[j].front(), chain.back()) < 1.0) {
                    chain.insert(chain.end(), pieces[j].begin() + 1, pieces[j].end());
                    used[j] = 1;
                    grew = true;
                }
        }
        if (dist(chain.front(), chain.back()) <= 1.0 && chain.size() >= 4) { chain.pop_back(); loops.push_back(chain); }
    }
    return loops;
}

double loopArea(const std::vector<Vec2>& p) {
    double a = 0;
    for (std::size_t i = 0; i < p.size(); ++i) { const Vec2& u = p[i]; const Vec2& v = p[(i + 1) % p.size()]; a += u.x * v.y - v.x * u.y; }
    return 0.5 * a;
}

std::vector<Vec2> resampleClosed(const std::vector<Vec2>& p, double step) {
    std::vector<Vec2> out{p[0]};
    Vec2 at = p[0];
    for (std::size_t i = 1; i <= p.size(); ++i) {
        const Vec2 to = p[i % p.size()];
        double d = dist(at, to);
        while (d >= step) { at = at + (to - at) * (step / d); out.push_back(at); d = dist(at, to); }
    }
    if (dist(out.back(), out.front()) < 0.5 * step) out.pop_back();
    return out;
}

bool segCross(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 p4, Vec2* at) {
    const double d = (p2.x - p1.x) * (p4.y - p3.y) - (p2.y - p1.y) * (p4.x - p3.x);
    if (std::fabs(d) < 1e-12) return false;
    const double t = ((p3.x - p1.x) * (p4.y - p3.y) - (p3.y - p1.y) * (p4.x - p3.x)) / d;
    const double u = ((p3.x - p1.x) * (p2.y - p1.y) - (p3.y - p1.y) * (p2.x - p1.x)) / d;
    if (t < 0 || t >= 1 || u < 0 || u > 1) return false;   // (half-open along the line: a road through a vertex counts once)
    if (at) *at = p1 + (p2 - p1) * t;
    return true;
}

// A street's half width to its kerb (the lanes builder's hw): its class's lanes, from the graph's class table
double streetHalfWidth(const json& graph, const std::string& cls) {
    // (a class the graph does not define is the builder's built-in: road_graph_spec.cpp)
    double w = cls == "arterial" ? 3.0 : 3.5, fwd = cls == "arterial" ? 2 : 1, back = fwd, gap = 0;
    if (graph.contains("classes") && graph["classes"].contains(cls)) {
        const json& c = graph["classes"][cls];
        w = c.value("w", w); fwd = c.value("fwd", fwd); back = c.value("back", back); gap = c.value("gap", gap);
    }
    // laneSlots: travel bands side by side, the median gap between directions; the outermost lane's edge
    return (back > 0 && fwd > 0) ? std::max(fwd, back) * w + gap / 2 : 0.5 * (fwd + back) * w;
}

// ...and its sidewalk beyond the kerb
double streetSidewalk(const json& graph, const std::string& cls) {
    const double builtIn = cls == "arterial" ? 2.6 : cls == "local" ? 1.8 : 2.0;
    if (graph.contains("classes") && graph["classes"].contains(cls)) return graph["classes"][cls].value("sidewalk", builtIn);
    return builtIn;
}

// from a light-rail pair's centreline to the outside of a station's flights (the pair 1.8 m either side, the platform
// gap and a 4 m platform, the 1.9 m flight behind it): the line must keep this clear of a street's sidewalk
constexpr double kCityStationBack = 1.8 + 1.55 + 4.0 + 2.1;

void wayIn(const json& graph, const IslandRailOptions& o, IslandRailResult& r, const std::function<Vec2(double)>& at) {
    // the streets a way may leave from, and every centreline it must not cross
    struct Line { std::string id, cls; std::vector<Vec2> p; };
    std::vector<Line> streets, all;
    for (const json& e : graph["edges"]) {
        const std::string cls = e.value("class", "");
        Line l{e.value("id", ""), cls, pointsOf(e)};
        if (l.p.size() < 2 || cls == "footway") continue;
        if (cls != "freeway" && cls != "ramp" && cls != "rail" && cls != "light_rail" && cls != "mountain" && !e.contains("from") && !e.contains("to")) streets.push_back(l);
        all.push_back(l);
    }
    for (const json& e : r.edges) all.push_back({e["id"].get<std::string>(), e.value("class", "rail"), pointsOf(e)});
    // A crossing of the street the way leaves from counts too, but where it leaves (within a metre of `from`): a
    // street bending round a station was recrossed 31 m on, and the way lifted over it at 49 %.
    auto crossesAny = [&](Vec2 a, Vec2 b, const std::string& own, Vec2 from) {
        for (const Line& l : all)
            for (std::size_t j = 0; j + 1 < l.p.size(); ++j) {
                Vec2 x;
                if (segCross(a, b, l.p[j], l.p[j + 1], &x) && !(l.id == own && dist(x, from) < 1.0)) return true;
            }
        return false;
    };
    for (std::size_t si = 0; si < r.stations.size(); ++si) {
        IslandRailStation& sn = r.stations[si];
        const Vec2 m = at(sn.station), t0 = at(sn.station + 5) - at(sn.station - 5);
        const Vec2 t = t0 * (1.0 / std::max(1e-9, t0.length())), n(-t.y, t.x);
        // per side and direction along the track, the leg's far end B, and the nearest street point to it that a
        // straight path reaches without crossing anything
        struct Pick { double d = 1e30; int side = 0, dir = 0; Vec2 a, b; std::string id, cls; } best;
        // the way past B: along the track to the line's crossing, over (or under) it to beyond the far platform
        auto legOf = [&](int side, int dir) {
            const Vec2 corner = m + n * (side * o.wayOffset);
            std::vector<Vec2> pts{m + t * (dir * o.wayLeg) + n * (side * o.wayOffset)};
            for (double s = o.wayLeg - 10; s > 4; s -= 10) pts.push_back(corner + t * (dir * s));
            pts.push_back(corner + t * (dir * 3.0));
            pts.push_back(corner - n * (side * 3.0));   // a 3 m chamfer round the corner
            pts.push_back(m - n * (side * o.wayReach));   // (straight across the line, square to it)
            return pts;
        };
        // UNDER: no height to gain, so no leg along the track -- from the nearest street on either side straight to
        // just outside the flights behind the platform, then square across under the line. (With the leg, a way set
        // out clear of the flights ran along the sidewalk of the street the line is set in beside.)
        if (o.wayUnder)
            for (int side = -1; side <= 1; side += 2) {
                const Vec2 c = m + n * (side * o.wayOffset);
                // the way under clear of everything but the line (the track pair is within 3 m of its centreline)
                if (crossesAny(c, m + n * (side * 3.0), "", Vec2(0, 0)) || crossesAny(m - n * (side * 3.0), m - n * (side * o.wayReach), "", Vec2(0, 0))) continue;
                for (const Line& l : streets)
                    for (std::size_t j = 0; j + 1 < l.p.size(); ++j) {
                        const Vec2 u = l.p[j], v = l.p[j + 1], uv = v - u;
                        const double k = std::clamp(((c - u).x * uv.x + (c - u).y * uv.y) / std::max(1e-9, uv.x * uv.x + uv.y * uv.y), 0.0, 1.0);
                        const Vec2 q = u + uv * k; const double d = dist(q, c);
                        if (d >= best.d || d > o.wayMaxStreet || d < streetHalfWidth(graph, l.cls) + 2.0) continue;
                        if (((q - m).x * n.x + (q - m).y * n.y) * side < o.wayOffset) continue;   // beyond the flights, on its side
                        if (crossesAny(q, c, l.id, q)) continue;
                        best = {d, side, 0, q, c, l.id, l.cls};
                    }
            }
        else
        for (int side = -1; side <= 1; side += 2)
            for (int dir = -1; dir <= 1; dir += 2) {
                const std::vector<Vec2> leg = legOf(side, dir);
                bool legClear = true;
                for (std::size_t i = 0; i + 2 < leg.size() && legClear; ++i)   // (the last piece crosses the line)
                    if (crossesAny(leg[i], leg[i + 1], "", Vec2(0, 0))) legClear = false;
                if (!legClear) continue;
                const Vec2 b = leg.front();
                for (const Line& l : streets)
                    for (std::size_t j = 0; j + 1 < l.p.size(); ++j) {
                        const Vec2 u = l.p[j], v = l.p[j + 1], uv = v - u;
                        const double k = std::clamp(((b - u).x * uv.x + (b - u).y * uv.y) / std::max(1e-9, uv.x * uv.x + uv.y * uv.y), 0.0, 1.0);
                        const Vec2 q = u + uv * k; const double d = dist(q, b);
                        if (d >= best.d || d > o.wayMaxStreet) continue;
                        if (((q - m).x * n.x + (q - m).y * n.y) * side < o.wayOffset) continue;   // behind the leg, on its side
                        if (crossesAny(q, b, l.id, q)) continue;
                        best = {d, side, dir, q, b, l.id, l.cls};
                    }
            }
        if (best.side == 0) { r.skipped.push_back(sn.name + ": no street within reach of the platforms"); continue; }
        // the way: from just inside the street's kerb (its end on the sidewalk, so the builder joins it to the
        // street), to B, along the track to the line's crossing, over it to beyond the far platform
        const Vec2 toB = (best.b - best.a) * (1.0 / std::max(1e-9, dist(best.b, best.a)));
        const Vec2 a = best.a + toB * (streetHalfWidth(graph, best.cls) + 1.0);
        std::vector<Vec2> pts{a};
        if (o.wayUnder) { pts.push_back(best.b); pts.push_back(m - n * (best.side * o.wayReach)); }
        else for (const Vec2& q : legOf(best.side, best.dir)) pts.push_back(q);
        json path = json::array();
        double len = 0;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            path.push_back({std::round(pts[i].x * 100) / 100, std::round(pts[i].y * 100) / 100});
            if (i) len += dist(pts[i], pts[i - 1]);
        }
        // the hold over the line: the builder sets its height (the footway goes OVER the track), this its reach --
        // or, under, the hold is the track's: level along its platforms, as high as the way under it needs
        json e = {{"id", o.idPrefix + "_way_" + std::to_string(si)}, {"class", "footway"}, {"path", {{"points", path}}}};
        if (!o.wayUnder) e["floor"] = json::array({json::array({m.x, m.y, -1000.0, o.wayHold})});
        else
            for (json& t : r.edges) {
                const std::vector<Vec2> tp = pointsOf(t);
                bool on = false;
                for (std::size_t j = 0; j + 1 < tp.size() && !on; ++j) {
                    const Vec2 u = tp[j], v = tp[j + 1], uv = v - u;
                    const double k = std::clamp(((m - u).x * uv.x + (m - u).y * uv.y) / std::max(1e-9, uv.x * uv.x + uv.y * uv.y), 0.0, 1.0);
                    on = dist(u + uv * k, m) < 0.5;
                }
                if (!on) continue;
                if (!t.contains("floor")) t["floor"] = json::array();
                t["floor"].push_back(json::array({std::round(m.x * 100) / 100, std::round(m.y * 100) / 100, -1000.0, o.platformHalf}));
                break;
            }
        r.ways.push_back(e);
        sn.street = best.id;
        sn.way = len;
    }
}

}  // namespace

IslandRailResult islandRailLoop(const json& graph, const std::vector<RailTown>& towns, const IslandRailOptions& o) {
    IslandRailResult r;
    r.railClass = {{"g_max", o.gMax}, {"balance", o.balance}};
    const std::vector<std::vector<Vec2>> loops = freewayLoops(graph);
    if (loops.empty()) { r.skipped.push_back("no closed freeway in the graph"); return r; }
    // THE OUTER CARRIAGEWAY: of the freeway's loops, the one enclosing the most -- the line goes beside it, outward
    const std::vector<Vec2>* outer = &loops[0];
    for (const auto& l : loops) if (std::fabs(loopArea(l)) > std::fabs(loopArea(*outer))) outer = &l;
    const std::vector<Vec2> fw = resampleClosed(*outer, o.spacing);
    const std::size_t n = fw.size();
    const double turn = loopArea(fw) > 0 ? 1.0 : -1.0;   // counter-clockwise: the left normal points in
    // the offset, on a tangent read over +-30 m (a carriageway's 1 m wiggles are not the line's)
    const int k = std::max(1, static_cast<int>(std::lround(30.0 / o.spacing)));
    std::vector<Vec2> line(n);
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 t = fw[(i + static_cast<std::size_t>(k)) % n] - fw[(i + n - static_cast<std::size_t>(k)) % n];
        const double L = std::max(1e-9, t.length());
        const Vec2 left(-t.y / L, t.x / L);
        line[i] = fw[i] - left * (turn * o.offset);
    }
    // smoothed once more, so an offset round a bend with a kink in it does not carry the kink out 45 m
    {
        std::vector<Vec2> s(n);
        for (std::size_t i = 0; i < n; ++i) s[i] = (line[(i + n - 1) % n] + line[i] * 2.0 + line[(i + 1) % n]) * 0.25;
        line = s;
    }
    std::vector<double> st(n + 1, 0.0);
    for (std::size_t i = 0; i < n; ++i) st[i + 1] = st[i] + dist(line[i], line[(i + 1) % n]);
    r.length = st[n];

    // WHAT IT CROSSES: every other road's centreline (each a grade separation the builder makes)
    std::vector<double> crossAt;
    for (const json& e : graph["edges"]) {
        if (e.value("class", "") == "freeway" || e.value("class", "") == "rail" || e.value("class", "") == "footway") continue;   // (nor a line written before)
        const std::vector<Vec2> p = pointsOf(e);
        if (p.size() < 2) continue;
        double x0 = 1e30, x1 = -1e30, z0 = 1e30, z1 = -1e30;
        for (const Vec2& q : p) { x0 = std::min(x0, q.x); x1 = std::max(x1, q.x); z0 = std::min(z0, q.y); z1 = std::max(z1, q.y); }
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = line[i], b = line[(i + 1) % n];
            if (std::max(a.x, b.x) < x0 || std::min(a.x, b.x) > x1 || std::max(a.y, b.y) < z0 || std::min(a.y, b.y) > z1) continue;
            for (std::size_t j = 0; j + 1 < p.size(); ++j) {
                Vec2 at;
                if (segCross(a, b, p[j], p[j + 1], &at)) { crossAt.push_back(st[i] + dist(a, at)); ++r.crossings; }
            }
        }
    }

    // THE RIVERS: the freeway's water floors (wider holds than the 30 m a road under it asks), onto the line
    json floors = json::array();
    std::vector<std::pair<double, double>> floorSpan;   // (station, half length) along the line
    for (const json& e : graph["edges"]) {
        if (e.value("class", "") != "freeway" || !e.contains("floor")) continue;
        for (const json& f : e["floor"]) {
            const Vec2 p(f[0].get<double>(), f[1].get<double>());
            const double h = f[2].get<double>(), hold = f[3].get<double>();
            if (hold <= 35.0) continue;   // a road under the freeway: the line meets that road for itself
            std::size_t best = 0;
            double bd = 1e30;
            for (std::size_t i = 0; i < n; ++i) { const double d = dist(line[i], p); if (d < bd) { bd = d; best = i; } }
            if (bd > o.offset + hold) continue;
            bool dup = false;
            for (const auto& fs : floorSpan) if (std::fabs(fs.first - st[best]) < 5.0) dup = true;
            if (dup) continue;
            floors.push_back({line[best].x, line[best].y, h, hold});
            floorSpan.push_back({st[best], hold});
        }
    }
    r.floors = static_cast<int>(floors.size());

    // THE STATIONS: per town, the nearest place on the line a platform fits -- on the straight, clear of crossings
    // and bridges, and not on another town's platform
    auto at = [&](double s) {
        s = std::fmod(s + r.length * 4.0, r.length);
        const std::size_t i = static_cast<std::size_t>(std::upper_bound(st.begin(), st.end(), s) - st.begin()) - 1;
        const double seg = st[i + 1] - st[i];
        const double t = seg > 1e-9 ? (s - st[i]) / seg : 0.0;
        return line[i % n] + (line[(i + 1) % n] - line[i % n]) * t;
    };
    auto ringGap = [&](double a, double b) { const double d = std::fabs(a - b); return std::min(d, r.length - d); };
    const double half = 0.5 * o.platformLength;
    auto fits = [&](double s) {
        for (double c : crossAt) if (ringGap(c, s) < half + o.crossingClear) return false;
        for (const auto& fs : floorSpan) if (ringGap(fs.first, s) < half + fs.second + 40.0) return false;
        // straight enough: the turn over the platform (and 40 m either side) against the radius asked for
        const Vec2 a = at(s - half - 40), m = at(s), b = at(s + half + 40);
        const Vec2 u = m - a, v = b - m;
        const double ang = std::fabs(std::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y));
        const double arc = o.platformLength + 80.0;
        return ang < 1e-6 || arc / ang >= o.straightRadius;
    };
    for (const RailTown& t : towns) {
        double bestS = -1, bestD = 1e30;
        for (std::size_t i = 0; i < n; ++i) {
            const double d = dist(line[i], t.at);
            if (d >= bestD || d > o.maxTownDistance) continue;
            bool taken = false;
            for (const IslandRailStation& sn : r.stations) if (ringGap(sn.station, st[i]) < o.platformLength + 400.0) taken = true;
            if (taken || !fits(st[i])) continue;
            bestD = d;
            bestS = st[i];
        }
        if (bestS < 0) {
            char why[160];
            std::snprintf(why, sizeof why, "%s: no straight stretch clear of crossings and bridges within %.0f m", t.name.c_str(), o.maxTownDistance);
            r.skipped.push_back(why);
            continue;
        }
        r.stations.push_back({t.name, at(bestS), bestS, bestD, "", 0});
    }
    std::sort(r.stations.begin(), r.stations.end(), [](const IslandRailStation& a, const IslandRailStation& b) { return a.station < b.station; });

    // THE EDGES: cut half way between stations (each edge holds one station's platform whole), the first cut at 0
    // when there are no stations
    std::vector<double> cuts;
    if (r.stations.size() >= 2) {
        for (std::size_t i = 0; i < r.stations.size(); ++i) {
            const double a = r.stations[i].station, b = r.stations[(i + 1) % r.stations.size()].station;
            const double gap = std::fmod(b - a + r.length, r.length);
            cuts.push_back(std::fmod(a + 0.5 * gap, r.length));
        }
    } else {
        const double s0 = r.stations.empty() ? 0.0 : r.stations[0].station + 0.5 * r.length;
        cuts = {std::fmod(s0, r.length), std::fmod(s0 + r.length / 3, r.length), std::fmod(s0 + 2 * r.length / 3, r.length)};
    }
    std::sort(cuts.begin(), cuts.end());
    for (std::size_t c = 0; c < cuts.size(); ++c) {
        const double s0 = cuts[c], s1 = c + 1 < cuts.size() ? cuts[c + 1] : cuts[0] + r.length;
        json pts = json::array();
        auto push = [&](Vec2 p) { pts.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100}); };
        push(at(s0));
        for (double s = std::ceil(s0 / o.spacing) * o.spacing; s < s1 - 0.5 * o.spacing; s += o.spacing) if (s > s0 + 0.5 * o.spacing) push(at(s));
        push(at(s1));
        json e = {{"id", o.idPrefix + "_" + std::to_string(c)}, {"class", "rail"}, {"path", {{"points", pts}}}};
        json fl = json::array();
        for (std::size_t q = 0; q < floorSpan.size(); ++q) {
            const double fs = floorSpan[q].first;
            const double inEdge = std::fmod(fs - s0 + 2 * r.length, r.length);
            if (inEdge <= s1 - s0 + floorSpan[q].second) fl.push_back(floors[q]);
        }
        if (!fl.empty()) e["floor"] = fl;
        json plats = json::array();
        for (const IslandRailStation& sn : r.stations) {
            const double inEdge = std::fmod(sn.station - s0 + 2 * r.length, r.length);
            if (inEdge - half < 0 || inEdge + half > s1 - s0) continue;
            plats.push_back({{"name", sn.name}, {"s0", std::round(inEdge - half)}, {"s1", std::round(inEdge + half)}, {"side", "both"}, {"w", o.platformWidth}});
        }
        if (!plats.empty()) e["platforms"] = plats;
        r.edges.push_back(e);
    }
    if (o.ways) wayIn(graph, o, r, at);
    return r;
}


// ---- A CITY'S LIGHT RAIL --------------------------------------------------------------------------------------------
namespace {

// the street graph the loop follows: arterials and collectors, joined where their points meet
struct StreetGraph {
    std::vector<Vec2> nodes;
    std::vector<std::vector<std::pair<int, double>>> adj;
    std::map<std::pair<long, long>, int> key;
    int node(Vec2 p) {
        const std::pair<long, long> k(std::lround(p.x * 2), std::lround(p.y * 2));
        auto it = key.find(k);
        if (it != key.end()) return it->second;
        nodes.push_back(p);
        adj.emplace_back();
        return key[k] = static_cast<int>(nodes.size()) - 1;
    }
    int nearest(Vec2 p) const {
        int b = -1; double bd = 1e30;
        for (std::size_t i = 0; i < nodes.size(); ++i) if (dist(nodes[i], p) < bd) { bd = dist(nodes[i], p); b = static_cast<int>(i); }
        return b;
    }
    std::vector<Vec2> route(int a, int b) const {
        std::vector<double> d(nodes.size(), 1e30);
        std::vector<int> prev(nodes.size(), -1);
        using Q = std::pair<double, int>;
        std::priority_queue<Q, std::vector<Q>, std::greater<Q>> pq;
        d[static_cast<std::size_t>(a)] = 0;
        pq.push({0, a});
        while (!pq.empty()) {
            const auto [du, u] = pq.top(); pq.pop();
            if (u == b) break;
            if (du > d[static_cast<std::size_t>(u)]) continue;
            for (const auto& [v, w] : adj[static_cast<std::size_t>(u)])
                if (du + w < d[static_cast<std::size_t>(v)]) { d[static_cast<std::size_t>(v)] = du + w; prev[static_cast<std::size_t>(v)] = u; pq.push({du + w, v}); }
        }
        if (d[static_cast<std::size_t>(b)] >= 1e29) return {};
        std::vector<Vec2> out;
        for (int v = b; v >= 0; v = prev[static_cast<std::size_t>(v)]) out.push_back(nodes[static_cast<std::size_t>(v)]);
        std::reverse(out.begin(), out.end());
        return out;
    }
};

// Douglas-Peucker on an open polyline
void simplify(const std::vector<Vec2>& p, std::size_t a, std::size_t b, double tol, std::vector<char>& keep) {
    if (b <= a + 1) return;
    double worst = -1; std::size_t at = a;
    const Vec2 u = p[b] - p[a];
    const double L = std::max(1e-9, u.length());
    for (std::size_t i = a + 1; i < b; ++i) {
        const double d = std::fabs(u.x * (p[i] - p[a]).y - u.y * (p[i] - p[a]).x) / L;
        if (d > worst) { worst = d; at = i; }
    }
    if (worst > tol) { keep[at] = 1; simplify(p, a, at, tol, keep); simplify(p, at, b, tol, keep); }
}

// a closed polygon offset by `d` toward its inside (mitred, the miter capped), then any loop the offset tied cut out
std::vector<Vec2> insetClosed(const std::vector<Vec2>& p, double d) {
    const std::size_t n = p.size();
    const double sgn = loopArea(p) > 0 ? 1.0 : -1.0;   // ccw: the inside is on the left
    std::vector<Vec2> out;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = p[(i + n - 1) % n], b = p[i], c = p[(i + 1) % n];
        Vec2 t0 = b - a, t1 = c - b;
        t0 = t0 * (1.0 / std::max(1e-9, t0.length())); t1 = t1 * (1.0 / std::max(1e-9, t1.length()));
        const Vec2 n0(-t0.y * sgn, t0.x * sgn), n1(-t1.y * sgn, t1.x * sgn);
        Vec2 m = n0 + n1;
        const double ml = m.length();
        if (ml < 1e-6) { out.push_back(b + n0 * d); continue; }
        m = m * (1.0 / ml);
        const double cosHalf = std::max(0.35, m.x * n0.x + m.y * n0.y);   // the miter capped at ~2.9 d
        out.push_back(b + m * (d / cosHalf));
    }
    // untie: where two non-neighbouring segments cross, drop the stretch between them (the shorter way round)
    for (int guard = 0; guard < 200; ++guard) {
        const std::size_t m = out.size();
        bool cut = false;
        for (std::size_t i = 0; i < m && !cut; ++i)
            for (std::size_t k = 2; k < m / 2 && !cut; ++k) {
                const std::size_t j = (i + k) % m;
                Vec2 x;
                if (!segCross(out[i], out[(i + 1) % m], out[j], out[(j + 1) % m], &x)) continue;
                // the two loops the crossing splits it into; the bigger one is the line, the other the knot
                std::vector<Vec2> a1, a2;
                for (std::size_t q = (j + 1) % m; q != (i + 1) % m; q = (q + 1) % m) a1.push_back(out[q]);
                a1.push_back(x);
                for (std::size_t q = (i + 1) % m; q != (j + 1) % m; q = (q + 1) % m) a2.push_back(out[q]);
                a2.push_back(x);
                out = std::fabs(loopArea(a1)) >= std::fabs(loopArea(a2)) ? a1 : a2;
                cut = true;
            }
        if (!cut) break;
    }
    return out;
}

// corners rounded: each vertex replaced by an arc of `radius` (smaller where the legs are short), sampled every few m
std::vector<Vec2> filletClosed(const std::vector<Vec2>& p, double radius) {
    const std::size_t n = p.size();
    std::vector<Vec2> out;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = p[(i + n - 1) % n], b = p[i], c = p[(i + 1) % n];
        const double la = dist(a, b), lc = dist(b, c);
        Vec2 u = (a - b) * (1.0 / std::max(1e-9, la)), v = (c - b) * (1.0 / std::max(1e-9, lc));
        const double cosT = std::clamp(u.x * v.x + u.y * v.y, -1.0, 1.0);
        const double theta = std::acos(cosT);                 // the angle AT the corner (pi: straight on)
        const double turn = 3.14159265358979 - theta;
        if (turn < 0.03) { out.push_back(b); continue; }
        double t = radius / std::tan(theta / 2);              // tangent length
        t = std::min({t, 0.45 * la, 0.45 * lc});
        const double r = t * std::tan(theta / 2);
        const Vec2 p0 = b + u * t, p1 = b + v * t;
        Vec2 bis = u + v;
        bis = bis * (1.0 / std::max(1e-9, bis.length()));
        const Vec2 centre = b + bis * (r / std::sin(theta / 2));
        const double a0 = std::atan2(p0.y - centre.y, p0.x - centre.x), a1 = std::atan2(p1.y - centre.y, p1.x - centre.x);
        double sweep = a1 - a0;
        while (sweep > 3.14159265358979) sweep -= 6.28318530717959;
        while (sweep < -3.14159265358979) sweep += 6.28318530717959;
        const int steps = std::max(2, static_cast<int>(std::ceil(std::fabs(sweep) * r / 4.0)));
        for (int k = 0; k <= steps; ++k) {
            const double ang = a0 + sweep * k / steps;
            out.push_back(centre + Vec2(std::cos(ang), std::sin(ang)) * r);
        }
    }
    return out;
}

}  // namespace

IslandRailResult cityRailLoop(const json& graph, const RailTown& city, Vec2 interchange, const CityRailOptions& o) {
    IslandRailResult r;
    // the streets it follows, and the street length round each point (where the city is built up)
    StreetGraph sg;
    std::vector<std::pair<Vec2, double>> mids;   // segment midpoints and lengths, for density
    for (const json& e : graph["edges"]) {
        const std::string cls = e.value("class", "");
        const std::vector<Vec2> p = pointsOf(e);
        for (std::size_t i = 0; i + 1 < p.size(); ++i) mids.push_back({(p[i] + p[i + 1]) * 0.5, dist(p[i], p[i + 1])});
        if (cls != "arterial" && cls != "collector") continue;
        for (std::size_t i = 0; i + 1 < p.size(); ++i) {
            const int a = sg.node(p[i]), b = sg.node(p[i + 1]);
            const double w = dist(p[i], p[i + 1]);
            sg.adj[static_cast<std::size_t>(a)].push_back({b, w});
            sg.adj[static_cast<std::size_t>(b)].push_back({a, w});
        }
    }
    if (sg.nodes.size() < 8) { r.skipped.push_back(city.name + ": no arterials to follow"); return r; }
    auto density = [&](Vec2 q, double rad) {
        double s = 0;
        for (const auto& m : mids) if (dist(m.first, q) < rad) s += m.second;
        return s;
    };
    // WAYPOINTS: the interchange, then in each of eight sectors round the hub the most built-up street node
    struct Wp { double ang; int node; };
    std::vector<Wp> wps;
    const int ia = sg.nearest(interchange);
    const double angI = std::atan2(sg.nodes[static_cast<std::size_t>(ia)].y - city.at.y, sg.nodes[static_cast<std::size_t>(ia)].x - city.at.x);
    wps.push_back({angI, ia});
    auto angGap = [](double a, double b) { return std::fabs(std::remainder(a - b, 6.28318530717959)); };
    for (int k = 0; k < 8; ++k) {
        const double ang = k * 3.14159265358979 / 4;
        double best = -1e30; int bn = -1; double bang = 0;
        for (std::size_t i = 0; i < sg.nodes.size(); ++i) {
            const Vec2 q = sg.nodes[i] - city.at;
            const double rr = q.length();
            if (rr < o.ringMin || rr > o.ringMax) continue;
            const double qa = std::atan2(q.y, q.x);
            if (angGap(qa, ang) > 3.14159265358979 / 8) continue;
            const double sc = density(sg.nodes[i], 250.0) - std::fabs(rr - 0.5 * (o.ringMin + o.ringMax)) * 2.0;
            if (sc > best) { best = sc; bn = static_cast<int>(i); bang = qa; }
        }
        if (bn >= 0 && angGap(bang, angI) > 3.14159265358979 / 8) wps.push_back({bang, bn});
    }
    if (wps.size() < 3) { r.skipped.push_back(city.name + ": too few streets round its hub for a loop"); return r; }
    std::sort(wps.begin() + 1, wps.end(), [&](const Wp& a, const Wp& b) {
        return std::fmod(a.ang - angI + 12.566, 6.28318530717959) < std::fmod(b.ang - angI + 12.566, 6.28318530717959);
    });
    std::vector<Vec2> ring;
    for (std::size_t i = 0; i < wps.size(); ++i) {
        const std::vector<Vec2> seg = sg.route(wps[i].node, wps[(i + 1) % wps.size()].node);
        if (seg.empty()) { r.skipped.push_back(city.name + ": its streets do not join up round the hub"); return r; }
        ring.insert(ring.end(), ring.empty() ? seg.begin() : seg.begin() + 1, seg.end());
    }
    // no out-and-back spurs: a point that returns to the one before last cuts the spur off
    std::vector<Vec2> st;
    for (const Vec2& q : ring) {
        if (st.size() >= 2 && dist(q, st[st.size() - 2]) < 1.0) { st.pop_back(); continue; }
        if (!st.empty() && dist(q, st.back()) < 0.5) continue;
        st.push_back(q);
    }
    if (st.size() > 2 && dist(st.front(), st.back()) < 0.5) st.pop_back();
    if (st.size() < 4) { r.skipped.push_back(city.name + ": the loop folded onto itself"); return r; }
    {
        std::vector<Vec2> open = st;
        open.push_back(st.front());
        std::vector<char> keep(open.size(), 0);
        keep.front() = keep.back() = 1;
        // a closed line's two ends are one point: split it at the point farthest from them, and simplify each half
        std::size_t far = 1;
        for (std::size_t i = 1; i + 1 < open.size(); ++i) if (dist(open[i], open[0]) > dist(open[far], open[0])) far = i;
        keep[far] = 1;
        simplify(open, 0, far, 4.0, keep);
        simplify(open, far, open.size() - 1, 4.0, keep);
        std::vector<Vec2> s2;
        for (std::size_t i = 0; i + 1 < open.size(); ++i) if (keep[i]) s2.push_back(open[i]);
        st = s2;
    }
    // set in from the streets, corners rounded, resampled. The set-in, unless given: the widest of the streets it follows
    // to the back of its sidewalk, the station behind the platform and a metre (island 8's arterials are 3+3 lanes and
    // a 5 m sidewalk: 16 m to the back; a fixed 21 m ran the deck edge over their sidewalks)
    double setIn = o.setIn;
    if (setIn <= 0) {
        double widest = 0;
        for (const char* cls : {"arterial", "collector"}) widest = std::max(widest, streetHalfWidth(graph, cls) + streetSidewalk(graph, cls));
        setIn = widest + kCityStationBack + 1.0;
    }
    std::vector<Vec2> inset = insetClosed(st, setIn);
    // no cusps: where the set-in line turns back sharply (an inset corner the untie left a spike at), cut the corner
    // twice (Chaikin) before rounding -- a 40 m fillet shrinks to nothing on legs a few metres long
    for (int pass = 0; pass < 3; ++pass) {
        const std::size_t m = inset.size();
        std::vector<Vec2> sm;
        for (std::size_t i = 0; i < m; ++i) {
            const Vec2 a = inset[(i + m - 1) % m], b = inset[i], c = inset[(i + 1) % m];
            Vec2 u = a - b, v = c - b;
            const double cosT = (u.x * v.x + u.y * v.y) / std::max(1e-9, u.length() * v.length());
            if (cosT < -0.5) { sm.push_back(b); continue; }   // a gentle corner (over 120 degrees): the fillet's
            sm.push_back(b + u * 0.25);
            sm.push_back(b + v * 0.25);
        }
        inset = sm;
    }
    std::vector<Vec2> line0 = resampleClosed(filletClosed(inset, o.radius), o.spacing);
    // NO CURVE TIGHTER THAN A TRAM TAKES: where the circle through a point and its neighbours two either side is
    // under the minimum, relax the point toward its neighbours' middle, until none is (the fillet cannot round a
    // corner whose legs are a few metres long)
    const double rMin = 0.6 * o.radius;
    auto radiusAt = [](Vec2 a, Vec2 b, Vec2 c) {
        const double ab = dist(a, b), bc = dist(b, c), ca = dist(c, a);
        const double cr = std::fabs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        return cr < 1e-9 ? 1e9 : ab * bc * ca / (2 * cr);
    };
    auto relax = [&]() {
        for (int it = 0; it < 400; ++it) {
            const std::size_t m = line0.size();
            bool tight = false;
            std::vector<Vec2> nx = line0;
            for (std::size_t i = 0; i < m; ++i) {
                const Vec2 a = line0[(i + m - 2) % m], b = line0[i], c = line0[(i + 2) % m];
                if (radiusAt(a, b, c) >= rMin) continue;
                tight = true;
                for (int k = -2; k <= 2; ++k) {   // the point and its neighbours, each toward its own neighbours' middle
                    const std::size_t j = (i + m + static_cast<std::size_t>(k + 2) - 2) % m;
                    nx[j] = line0[j] * 0.5 + (line0[(j + m - 1) % m] + line0[(j + 1) % m]) * 0.25;
                }
            }
            line0 = nx;
            if (!tight) break;
        }
        line0 = resampleClosed(line0, o.spacing);
    };
    relax();
    // KEEP CLEAR OF THE STREETS IT RUNS BESIDE: rounding and relaxing pull a set-in line back toward the street on the
    // inside of a bend (Weyby's came to 13 m of a 16 m arterial's centreline, its deck over the sidewalk). Where the
    // line runs along a street (within 30 degrees of it) nearer than the street's back of sidewalk and the pair's own
    // half width, push it out, and relax again. Crossing a street square is what it should do and is left alone.
    {
        struct Seg { Vec2 a, b; double need; };
        std::vector<Seg> segs;
        for (const json& e : graph["edges"]) {
            const std::string cls = e.value("class", "");
            if (cls == "footway" || cls == "rail" || cls == "light_rail") continue;
            const std::vector<Vec2> p = pointsOf(e);
            const double need = streetHalfWidth(graph, cls) + streetSidewalk(graph, cls) + 3.6 + 1.0;
            for (std::size_t j = 0; j + 1 < p.size(); ++j) segs.push_back({p[j], p[j + 1], need});
        }
        for (int round = 0; round < 6; ++round) {
            const std::size_t m = line0.size();
            std::vector<Vec2> push(m, Vec2(0, 0));
            int pushed = 0;
            for (std::size_t i = 0; i < m; ++i) {
                const Vec2 p = line0[i], tl = line0[(i + 1) % m] - line0[(i + m - 1) % m];
                const double tll = std::max(1e-9, tl.length());
                for (const Seg& sg : segs) {
                    const Vec2 uv = sg.b - sg.a; const double L2 = uv.x * uv.x + uv.y * uv.y; if (L2 < 1e-9) continue;
                    const double k = std::clamp(((p - sg.a).x * uv.x + (p - sg.a).y * uv.y) / L2, 0.0, 1.0);
                    const Vec2 q = sg.a + uv * k; const double d = dist(p, q);
                    if (d >= sg.need || d < 0.5) continue;
                    if (std::fabs((tl.x * uv.x + tl.y * uv.y) / (tll * std::sqrt(L2))) < 0.866) continue;   // across it, not along
                    const Vec2 away = (p - q) * (1.0 / d);
                    const double want = sg.need - d;
                    if (want > push[i].length()) push[i] = away * want;
                }
                if (push[i].length() > 0.05) ++pushed;
            }
            if (!pushed) break;
            for (std::size_t i = 0; i < m; ++i) line0[i] = line0[i] + push[i];
            relax();
            if (std::getenv("RT_CITY_RAIL_DEBUG")) std::fprintf(stderr, "[city rail] %s: keep-clear round %d pushed %d of %zu points\n", city.name.c_str(), round, pushed, m);
        }
    }
    if (std::getenv("RT_CITY_RAIL_DEBUG")) {
        auto len = [](const std::vector<Vec2>& p) { double L = 0; for (std::size_t i = 0; i < p.size(); ++i) L += dist(p[i], p[(i + 1) % p.size()]); return L; };
        std::fprintf(stderr, "[city rail] %s: %zu waypoints, streets %zu pts %.0f m, inset %zu pts %.0f m, line %zu pts\n", city.name.c_str(),
                     wps.size(), st.size(), len(st), inset.size(), len(inset), line0.size());
    }
    if (line0.size() < 20) { r.skipped.push_back(city.name + ": the loop came out too small"); return r; }
    // run counter-clockwise from the point nearest the interchange (so the first station is there)
    std::vector<Vec2> line;
    {
        std::size_t k0 = 0;
        for (std::size_t i = 0; i < line0.size(); ++i) if (dist(line0[i], interchange) < dist(line0[k0], interchange)) k0 = i;
        for (std::size_t i = 0; i < line0.size(); ++i) line.push_back(line0[(k0 + i) % line0.size()]);
        if (loopArea(line) < 0) std::reverse(line.begin() + 1, line.end());
    }
    const std::size_t n = line.size();
    std::vector<double> sAt(n + 1, 0);
    for (std::size_t i = 1; i <= n; ++i) sAt[i] = sAt[i - 1] + dist(line[i - 1], line[i % n]);
    r.length = sAt[n];
    auto at = [&](double s) {
        s = std::fmod(std::fmod(s, r.length) + r.length, r.length);
        const std::size_t i = static_cast<std::size_t>(std::upper_bound(sAt.begin(), sAt.end(), s) - sAt.begin()) - 1;
        const double seg = sAt[i + 1] - sAt[i];
        const double t = seg > 1e-9 ? (s - sAt[i]) / seg : 0.0;
        return line[i % n] + (line[(i + 1) % n] - line[i % n]) * t;
    };
    // crossings (each a lift over a street), and where along the loop
    std::vector<double> crossAt;
    for (const json& e : graph["edges"]) {
        const std::string cls = e.value("class", "");
        if (cls == "footway") continue;
        const std::vector<Vec2> p = pointsOf(e);
        for (std::size_t j = 0; j + 1 < p.size(); ++j)
            for (std::size_t i = 0; i < n; ++i) {
                Vec2 x;
                if (!segCross(line[i], line[(i + 1) % n], p[j], p[j + 1], &x)) continue;
                ++r.crossings;
                crossAt.push_back(sAt[i] + dist(line[i], x));
            }
    }
    auto clearOfCrossings = [&](double s) {
        double d = 1e30;
        for (double c : crossAt) { const double a = std::fabs(std::fmod(s - c + 2 * r.length, r.length)); d = std::min(d, std::min(a, r.length - a)); }
        return d;
    };
    // STATIONS: at the interchange, then the most built-up straight 550-900 m on, round to the start
    const double half = 0.5 * o.platformLength;
    auto straight = [&](double s) {
        const Vec2 a = at(s - half - 20), m = at(s), b = at(s + half + 20);
        const Vec2 u = m - a, v = b - m;
        const double ang = std::fabs(std::atan2(u.x * v.y - u.y * v.x, u.x * v.x + u.y * v.y));
        return ang < 1e-6 || (o.platformLength + 40.0) / ang >= o.straightRadius;
    };
    auto stationNear = [&](double s0, double s1) {
        double best = -1e30, bs = -1;
        for (double s = s0; s <= s1; s += 10.0) {
            // no street under the platforms (its flights would land on it; Saltwood 3 stood over a bending arterial)
            if (!straight(s) || clearOfCrossings(s) < half + o.platformClear) continue;
            const double sc = density(at(s), 300.0);
            if (sc > best) { best = sc; bs = s; }
        }
        return bs;
    };
    {
        double s0 = -1;
        auto ok = [&](double s) { return straight(s) && clearOfCrossings(s) >= half + o.platformClear; };
        for (double s = 0; s < 400.0 && s0 < 0; s += 10.0) { if (ok(s)) s0 = s; if (s0 < 0 && ok(r.length - s)) s0 = r.length - s; }
        if (s0 < 0) s0 = stationNear(0, 400);
        if (s0 >= 0) r.stations.push_back({city.name + " Central", at(s0), s0, dist(at(s0), city.at), "", 0});
        double cur = s0 < 0 ? 0 : s0;
        int k = 1;
        while (true) {
            const double first = r.stations.empty() ? 0 : r.stations.front().station;
            const double end = (first <= cur ? first + r.length : first) - o.stopMin;
            const double lo = cur + o.stopMin, hi = std::min(cur + o.stopMax, end);
            if (lo > hi) break;
            double s = stationNear(lo, hi);
            if (s < 0) s = stationNear(hi, std::min(hi + 400.0, end));   // (none clear of the streets: a longer run, not a gap)
            if (s < 0) break;
            r.stations.push_back({city.name + " " + std::to_string(++k), at(s), std::fmod(s, r.length), dist(at(s), city.at), "", 0});
            cur = s;
        }
    }
    std::sort(r.stations.begin(), r.stations.end(), [](const IslandRailStation& a, const IslandRailStation& b) { return a.station < b.station; });
    // THE EDGES: cut half way between stations
    std::vector<double> cuts;
    if (r.stations.size() >= 2) {
        for (std::size_t i = 0; i < r.stations.size(); ++i) {
            // A JOINT CLEAR OF THE STREETS: two sections meeting where one is lifted over a street settle at different
            // heights (Saltwood's joints stepped 1.6-4 m, and a crossing beside one came 2.6 m short of clearance).
            // The point between the platforms farthest from any crossing, up to o.jointClear, nearest the middle.
            const double a = r.stations[i].station, gap = std::fmod(r.stations[(i + 1) % r.stations.size()].station - a + r.length, r.length);
            const double lo = half + 10.0, hi = gap - half - 10.0;
            double bestS = 0.5 * gap, bestScore = -1e30;
            for (double u = lo; u <= hi; u += 2.0) {
                const double score = std::min(o.jointClear, clearOfCrossings(a + u)) * 1000.0 - std::fabs(u - 0.5 * gap);
                if (score > bestScore) { bestScore = score; bestS = u; }
            }
            cuts.push_back(std::fmod(a + bestS, r.length));
        }
    } else {
        cuts = {0, r.length / 3, 2 * r.length / 3};
    }
    std::sort(cuts.begin(), cuts.end());
    for (std::size_t c = 0; c < cuts.size(); ++c) {
        const double s0 = cuts[c], s1 = c + 1 < cuts.size() ? cuts[c + 1] : cuts[0] + r.length;
        json pts = json::array();
        auto push = [&](Vec2 p) { pts.push_back({std::round(p.x * 100) / 100, std::round(p.y * 100) / 100}); };
        push(at(s0));
        for (double s = std::ceil(s0 / o.spacing) * o.spacing; s < s1 - 0.5 * o.spacing; s += o.spacing) if (s > s0 + 0.5 * o.spacing) push(at(s));
        push(at(s1));
        json e = {{"id", o.idPrefix + "_" + std::to_string(c)}, {"class", "light_rail"}, {"path", {{"points", pts}}}};
        json plats = json::array();
        for (const IslandRailStation& sn : r.stations) {
            const double inEdge = std::fmod(sn.station - s0 + 2 * r.length, r.length);
            if (inEdge - half < 0 || inEdge + half > s1 - s0) continue;
            plats.push_back({{"name", sn.name}, {"s0", std::round(inEdge - half)}, {"s1", std::round(inEdge + half)}, {"side", "both"}, {"w", o.platformWidth}});
        }
        if (!plats.empty()) e["platforms"] = plats;
        r.edges.push_back(e);
    }
    // THE WAYS IN: the main line's, scaled to a light-rail pair (3.6 m centres, 4 m platforms): the leg between the
    // platforms' backs and the street's sidewalk
    IslandRailOptions wo;
    wo.idPrefix = o.idPrefix;
    // (the line is held up over its side streets: the way passes under it, a flight up to each platform beside it)
    wo.wayUnder = true;
    wo.platformHalf = half + 5.0;
    const double back = 1.8 + 1.55 + o.platformWidth;   // half the pair, the platform gap (rail_track.h), the platform
    wo.wayOffset = back + 2.2 + 1.5 + 1.0;   // the leg clear of the flight behind the platform (deck_mesh.cpp's 1.9 m)
    wo.wayLeg = 30.0;
    wo.wayReach = wo.wayOffset;              // and on as far beyond the far platform: its flight leaves from the way
    wo.wayMaxStreet = 120.0;
    wayIn(graph, wo, r, at);
    return r;
}

}  // namespace engine
