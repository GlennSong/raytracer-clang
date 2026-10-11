#include "island_rail.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

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
        if (e.value("class", "") == "freeway" || e.value("class", "") == "rail") continue;   // (nor a line written before)
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
        r.stations.push_back({t.name, at(bestS), bestS, bestD});
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
    return r;
}

}  // namespace engine
