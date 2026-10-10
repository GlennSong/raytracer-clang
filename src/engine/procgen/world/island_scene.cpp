#include "island_scene.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <set>
#include <string>

namespace engine {

using json = nlohmann::json;

namespace {

json pointsJson(const std::vector<Vec2>& pts) {
    json a = json::array();
    for (const Vec2& p : pts) a.push_back({p.x, p.y});
    return a;
}

double distToPolyline(const Vec2& p, const std::vector<Vec2>& P, Vec2* foot = nullptr) {
    double best = 1e30;
    for (std::size_t i = 0; i + 1 < P.size(); ++i) {
        const Vec2 a = P[i], ab = P[i + 1] - a;
        const double L2 = ab.x * ab.x + ab.y * ab.y;
        const double t = L2 > 1e-12 ? std::clamp(((p - a).x * ab.x + (p - a).y * ab.y) / L2, 0.0, 1.0) : 0.0;
        const Vec2 f = a + ab * t;
        const double d = (p - f).length();
        if (d < best) { best = d; if (foot) *foot = f; }
    }
    return best;
}

bool segCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, Vec2& x) {
    const Vec2 r = b - a, s = d - c;
    const double den = r.x * s.y - r.y * s.x;
    if (std::fabs(den) < 1e-12) return false;
    const double t = ((c - a).x * s.y - (c - a).y * s.x) / den, u = ((c - a).x * r.y - (c - a).y * r.x) / den;
    if (t < 0 || t > 1 || u < 0 || u > 1) return false;
    x = a + r * t;
    return true;
}

}  // namespace

json islandLanesScene(const IslandWorld& w, const std::vector<json>& placeScenes, const IslandSceneOptions& o) {
    json scene;
    scene["name"] = "island";
    for (const json& ps : placeScenes)
        if (ps.is_object() && ps.contains("classes")) {
            scene["classes"] = ps["classes"];
            scene["rules"] = ps.value("rules", json::object());
            break;
        }
    // nothing meets the island freeway at grade: the pass rose to it inside its own diamond and crossed
    // both carriageways as a level junction (Glenn: "literally crosses through two multilane freeways")
    scene["rules"]["freeway_separates"] = true;
    // THE MOUNTAIN CLASS: the rural road's section, graded as mountain roads are (15%, a design grade of
    // 12%). At the rural 10% the pass -- climbing 8.2% on average up its valley -- could not keep to the
    // ground and rode a 2.5 km viaduct 45 m up
    if (scene.contains("classes") && scene["classes"].contains("rural")) {
        json m = scene["classes"]["rural"];
        m["g_max"] = 0.15;
        m["balance"] = 0.5;   // cut through what it cannot climb, as a mountain road does (ADR-0116)
        // GUARDRAILS (ADR-0117, Glenn: "the mountain road needs railguards at some places"): on its bridges,
        // and at grade where the ground 5 m out drops 2 m or more -- nowhere else
        m["edges"] = {{"seam", {{"kind", "none"}}}, {"median", {{"kind", "none"}}}, {"vs_street", {{"kind", "none"}}},
                      {"elevated", {{"kind", "guardrail"}}}, {"at_grade", {{"kind", "guardrail"}, {"min_drop", 2.0}}}};
        scene["classes"]["mountain"] = m;
    }
    // ROAD EDGES (the city's next ten #5; Glenn's guardrails, "in places"): the freeway, its ramps and the roads
    // between towns get a guardrail at grade where the ground 5 m out drops 2.5 m or more -- an embankment, a bank
    // above a river, a cut's far side -- and nowhere else. Their other edges keep their defaults.
    for (const char* k : {"freeway", "ramp", "rural"})
        if (scene.contains("classes") && scene["classes"].contains(k) && !(scene["classes"][k].contains("edges") &&
                                                                           scene["classes"][k]["edges"].contains("at_grade")))
            scene["classes"][k]["edges"]["at_grade"] = {{"kind", "guardrail"}, {"min_drop", 2.5}};
    json edges = json::array();
    const bool windowed = o.windowHalf > 0.0;
    auto inWin = [&](const Vec2& p, double margin) {
        return !windowed || (std::fabs(p.x - o.windowCentre.x) <= o.windowHalf + margin && std::fabs(p.y - o.windowCentre.y) <= o.windowHalf + margin);
    };
    // a polyline clipped to the window: its runs inside, each carried one point past the edge. "Inside"
    // reaches 120 m past the window, within the 150 m of ground the window's grid adds: Saltwood's pass
    // runs along the east edge, stepping 66 m out and back, and clipping it AT the edge cut it into three
    // pieces with 100 and 290 m gaps (Glenn: "broken when we get into the mountains")
    auto clip = [&](const std::vector<Vec2>& P) {
        std::vector<std::vector<Vec2>> runs;
        if (!windowed) { runs.push_back(P); return runs; }
        std::vector<Vec2> cur;
        for (std::size_t i = 0; i < P.size(); ++i) {
            if (inWin(P[i], 120.0)) {
                if (cur.empty() && i > 0) cur.push_back(P[i - 1]);
                cur.push_back(P[i]);
            } else if (!cur.empty()) {
                cur.push_back(P[i]);
                if (cur.size() >= 2) runs.push_back(cur);
                cur.clear();
            }
        }
        if (cur.size() >= 2) runs.push_back(cur);
        return runs;
    };

    // BRIDGES over rivers: along a road, each run within a river's banks gets a floor at the water
    // plus bridgeOverWater, held across the run and 25 m either side
    // `freeway`: a freeway carriageway -- its pavement reaches ~8 m either side of the line tested, so "over the
    // water" starts 10 m from the bank, and a long wet run is floored along its length at the LOCAL water level
    // (#90, Glenn: "another part of the freeway is on the ground and goes right into a river"): the 200 m rule
    // below is the pass's, and a freeway meeting a river obliquely or after running beside it got no floor
    auto riverFloors = [&](const std::vector<Vec2>& P, bool freeway = false) {
        const double wetAt = freeway ? 10.0 : 3.0;
        json floors = json::array();
        if (!w.hydro || P.size() < 2) return floors;
        std::vector<Vec2> dense;
        for (std::size_t i = 0; i + 1 < P.size(); ++i) {
            const int m = std::max(1, static_cast<int>(std::ceil((P[i + 1] - P[i]).length() / 8.0)));
            for (int k = 0; k < m; ++k) dense.push_back(P[i] + (P[i + 1] - P[i]) * (static_cast<double>(k) / m));
        }
        dense.push_back(P.back());
        std::size_t k = 0;
        while (k < dense.size()) {
            double level = 0;
            if (w.hydro->distanceToRiver(dense[k].x, dense[k].y, 60.0, &level) >= wetAt) { ++k; continue; }
            std::size_t j = k;
            double top = level;
            while (j + 1 < dense.size()) {
                double lv = 0;
                if (w.hydro->distanceToRiver(dense[j + 1].x, dense[j + 1].y, 60.0, &lv) >= wetAt) break;
                if (std::isfinite(lv)) top = std::max(top, lv);
                ++j;
            }
            // a CROSSING is a short wet run; a long one is a road running beside (or along) the river -- the
            // pass follows one up its valley, and flooring that whole run at its highest water put 2.2 km
            // of it on a bridge 110 m up
            const double runLen = 8.0 * static_cast<double>(j - k);
            if (freeway && runLen > 200.0) {
                // a freeway along the water: held over it all the way, each floor at the water beside it
                for (std::size_t q = k; q <= j; q += 8) {
                    double lv = 0;
                    w.hydro->distanceToRiver(dense[q].x, dense[q].y, 60.0, &lv);
                    const double water = std::isfinite(lv) ? lv : std::isfinite(top) ? top : w.heightAt(dense[q].x, dense[q].y);
                    floors.push_back({dense[q].x, dense[q].y, water + o.bridgeOverWater, 40.0});
                }
                double lv = 0;
                w.hydro->distanceToRiver(dense[j].x, dense[j].y, 60.0, &lv);
                floors.push_back({dense[j].x, dense[j].y, (std::isfinite(lv) ? lv : top) + o.bridgeOverWater, 40.0});
            } else if (runLen <= 200.0) {
                const Vec2 mid = (dense[k] + dense[j]) * 0.5;
                double lvMid = 0;
                w.hydro->distanceToRiver(mid.x, mid.y, 60.0, &lvMid);
                const double water = std::isfinite(lvMid) ? lvMid : std::isfinite(top) ? top : w.heightAt(mid.x, mid.y);
                floors.push_back({mid.x, mid.y, water + o.bridgeOverWater, runLen * 0.5 + 25.0});
            }
            k = j + 1;
        }
        // LAKES (#79, Glenn: "part of the freeway cuts below a lake ... It should be elevated over the lake"):
        // the router keeps the ROUTE 12 m off a lake, but a carriageway stands a dozen metres to one side of
        // it, and one crossed the lake's edge on its own profile, down in the basin with a retaining wall
        // holding the water back beside it. Every run over (or within a few metres of) a lake is floored at
        // the lake's level plus the bridge clearance -- all of it, however long: a road cannot run IN a lake.
        k = 0;
        while (k < dense.size()) {
            const double lv0 = w.hydro->lakeLevelAt(dense[k].x, dense[k].y, 6.0);
            if (!std::isfinite(lv0)) { ++k; continue; }
            std::size_t j = k;
            double top = lv0;
            while (j + 1 < dense.size()) {
                const double lv = w.hydro->lakeLevelAt(dense[j + 1].x, dense[j + 1].y, 6.0);
                if (!std::isfinite(lv)) break;
                top = std::max(top, lv);
                ++j;
            }
            // one floor every <= 60 m along the run, each holding 40 m either side, so a long crossing is held
            // all the way over
            for (std::size_t q = k; q <= j; q += 8) floors.push_back({dense[q].x, dense[q].y, top + o.bridgeOverWater, 40.0});
            floors.push_back({dense[j].x, dense[j].y, top + o.bridgeOverWater, 40.0});
            k = j + 1;
        }
        return floors;
    };

    // THE PLACES: their streets, ids prefixed
    for (std::size_t k = 0; k < placeScenes.size(); ++k) {
        const json& ps = placeScenes[k];
        if (!ps.is_object() || !ps.contains("edges")) continue;
        const std::string pre = "p" + std::to_string(k) + "_";
        for (json e : ps["edges"]) {
            if (windowed) {
                bool any = false;
                for (const json& q : e["path"]["points"]) if (inWin(Vec2(q[0].get<double>(), q[1].get<double>()), 0.0)) { any = true; break; }
                if (!any) continue;
            }
            e["id"] = pre + e.value("id", std::string());
            for (const char* end : {"from", "to"}) {
                if (!e.contains(end)) continue;
                if (e[end].is_string()) e[end] = pre + e[end].get<std::string>();
                else if (e[end].is_object() && e[end].contains("edge")) e[end]["edge"] = pre + e[end]["edge"].get<std::string>();
            }
            edges.push_back(e);
        }
    }

    // THE FREEWAY: two carriageways off its route, each split into two chains
    const std::vector<Vec2>& R0 = w.freewayRoute;
    std::vector<std::vector<Vec2>> chains[2];   // [a | b] -> its chains
    std::vector<std::pair<std::string, std::vector<Vec2>>> freewayPieces;   // what of them the scene holds
    if (R0.size() >= 8) {
        std::vector<Vec2> R = R0;
        const bool closed = (R.front() - R.back()).length() < 1.0;
        if (closed) R.pop_back();
        const std::size_t N = R.size();
        std::vector<double> st(N, 0.0);
        for (std::size_t i = 1; i < N; ++i) st[i] = st[i - 1] + (R[i] - R[i - 1]).length();
        const double L = st.back() + (closed ? (R.front() - R.back()).length() : 0.0);
        std::vector<Vec2> A(N), B(N);
        for (std::size_t i = 0; i < N; ++i) {
            const Vec2 prev = R[i > 0 ? i - 1 : (closed ? N - 1 : 0)], next = R[i + 1 < N ? i + 1 : (closed ? 0 : N - 1)];
            Vec2 t = next - prev;
            t = t * (1.0 / std::max(1e-9, t.length()));
            const Vec2 n(-t.y, t.x);
            A[i] = R[i] - n * o.carriage;   // with the route, on its right
            B[i] = R[i] + n * o.carriage;   // against it
        }
        // the second split: the vertex farthest (round the ring) from every gore and from the first (index 0)
        std::vector<double> gores{0.0};
        for (const IslandInterchange& ic : w.interchanges) for (const IslandInterchange::Ramp& rp : ic.ramps) gores.push_back(rp.gore);
        std::size_t m = N / 2;
        double bestClear = -1;
        for (std::size_t i = N / 8; i < N - N / 8; ++i) {
            double clear = 1e30;
            for (double g : gores) { const double d = std::fabs(st[i] - g); clear = std::min(clear, std::min(d, L - d)); }
            if (clear > bestClear) { bestClear = clear; m = i; }
        }
        auto slice = [&](const std::vector<Vec2>& P, std::size_t from, std::size_t to) {   // from..to inclusive, wrapping
            std::vector<Vec2> out;
            for (std::size_t i = from;; i = (i + 1) % N) { out.push_back(P[i]); if (i == to) break; }
            return out;
        };
        chains[0] = {slice(A, 0, m), slice(A, m, 0)};
        std::vector<Vec2> b1 = slice(B, m, 0), b2 = slice(B, 0, m);   // b runs against the route: reversed
        std::reverse(b1.begin(), b1.end());
        std::reverse(b2.begin(), b2.end());
        chains[1] = {b1, b2};
        if (!closed) { chains[0] = {A}; std::vector<Vec2> rb = B; std::reverse(rb.begin(), rb.end()); chains[1] = {rb}; }
        // floors where a road passes under: every island road's crossings of the route
        std::vector<Vec2> crossings, crossDir;
        for (const IslandRoad& rd : w.roads) {
            if (rd.kind == "freeway" || rd.points.size() < 2) continue;
            for (std::size_t i = 0; i + 1 < rd.points.size(); ++i)
                for (std::size_t j = 0; j + 1 < R0.size(); ++j) {
                    Vec2 x;
                    if (segCross(rd.points[i], rd.points[i + 1], R0[j], R0[j + 1], x)) {
                        crossings.push_back(x);
                        const Vec2 d = rd.points[i + 1] - rd.points[i];
                        crossDir.push_back(d * (1.0 / std::max(1e-9, d.length())));
                    }
                }
        }
        // the road under's highest ground beneath both carriageways (it climbs; the builder clears its top)
        auto underHigh = [&](const Vec2& x) {
            std::size_t k = 0;
            for (std::size_t q = 0; q < crossings.size(); ++q) if ((crossings[q] - x).length() < 1e-6) k = q;
            double z = -1e30;
            for (int q = -5; q <= 5; ++q) { const Vec2 y = x + crossDir[k] * (5.0 * q); z = std::max(z, w.heightAt(y.x, y.y)); }
            return z;
        };
        const char* names[2][2] = {{"fw0_a", "fw0_a2"}, {"fw0_b", "fw0_b2"}};
        // ONE PROFILE OVER WATER FOR BOTH CARRIAGEWAYS: each side's water floors go to both sides, so the two
        // directions of one freeway cannot part company over a river or a lake (#79: one crossed on piers at
        // 34 m while the other ran 28 m below it, under the water)
        json sharedWater = json::array();
        for (int side = 0; side < 2; ++side)
            for (const std::vector<Vec2>& P : chains[side])
                for (const json& f : riverFloors(P, /*freeway*/ true)) {
                    sharedWater.push_back(f);
                    static const bool listWet = std::getenv("RT_FREEWAY_WATER") != nullptr;   // where the freeway meets water
                    if (listWet) std::fprintf(stderr, "[freeway] over water at (%.0f, %.0f): deck floor %.1f m, holds %.0f m\n",
                                              f[0].get<double>(), f[1].get<double>(), f[2].get<double>(), f[3].get<double>());
                }
        for (int side = 0; side < 2; ++side)
            for (std::size_t c = 0; c < chains[side].size(); ++c) {
                const std::vector<Vec2>& P = chains[side][c];
                json floors = json::array();
                for (const json& f : sharedWater) {   // the other side's floors, where they reach this one
                    Vec2 at;
                    if (distToPolyline(Vec2(f[0].get<double>(), f[1].get<double>()), P, &at) > f[3].get<double>() + o.carriage) continue;
                    floors.push_back({at.x, at.y, f[2], f[3]});
                }
                for (const Vec2& x : crossings) {
                    Vec2 f;
                    if (distToPolyline(x, P, &f) > o.carriage + 20.0) continue;
                    floors.push_back({f.x, f.y, underHigh(x) + o.underClearance, 30.0});
                }
                const std::vector<std::vector<Vec2>> pieces = clip(P);
                for (std::size_t q = 0; q < pieces.size(); ++q) {
                    json fl = json::array();
                    for (const json& f : floors) if (inWin(Vec2(f[0].get<double>(), f[1].get<double>()), 60.0)) fl.push_back(f);
                    const std::string id = q == 0 ? std::string(names[side][c]) : std::string(names[side][c]) + "_" + std::to_string(q);
                    edges.push_back({{"id", id}, {"class", "freeway"}, {"floor", fl}, {"path", {{"points", pointsJson(pieces[q])}}}});
                    freewayPieces.push_back({id, pieces[q]});
                }
            }
    }

    // LOOKOUTS (ADR-0119, Glenn: "widened at other parts -- maybe a lookout over the city and mountain
    // lakes"): on the pass and the mountain road, the straight-ish stretches whose valley side falls
    // furthest over the next 400 m, facing a town or a lake best -- up to two a road, 1.2 km apart.
    // Each becomes a lay-by: a 70 m pocket lane widening the pavement on that side (its outline then
    // carries the guardrail, the drop being what it is).
    struct Lookout { Vec2 at, out; };
    std::vector<std::vector<Lookout>> lookouts(w.roads.size());
    for (std::size_t k = 0; k < w.roads.size(); ++k) {
        const IslandRoad& rd = w.roads[k];
        if ((rd.kind != "pass" && rd.kind != "mountain") || rd.points.size() < 4) continue;
        std::vector<Vec2> targets;
        for (const auto& site : w.sites) targets.push_back(site.at);
        if (w.hydro) for (const Lake& lk : w.hydro->lakes()) targets.push_back(Vec2(0.5 * (lk.minX + lk.maxX), 0.5 * (lk.minZ + lk.maxZ)));
        std::vector<double> st{0.0};
        for (std::size_t i = 1; i < rd.points.size(); ++i) st.push_back(st.back() + (rd.points[i] - rd.points[i - 1]).length());
        auto at = [&](double s) {
            const std::size_t i = std::min<std::size_t>(rd.points.size() - 2, static_cast<std::size_t>(std::upper_bound(st.begin(), st.end(), s) - st.begin()) - 1);
            const double t = std::clamp((s - st[i]) / std::max(1e-9, st[i + 1] - st[i]), 0.0, 1.0);
            return rd.points[i] + (rd.points[i + 1] - rd.points[i]) * t;
        };
        struct Cand { double score, s; Vec2 p, out; };
        std::vector<Cand> cands;
        for (double s = 300.0; s + 300.0 < st.back(); s += 20.0) {
            const Vec2 p = at(s), a = at(s - 40.0), b = at(s + 40.0);
            Vec2 t = b - a; if (t.length() < 1e-6) continue; t = t * (1.0 / t.length());
            const Vec2 t0 = (p - a) * (1.0 / std::max(1e-9, (p - a).length())), t1 = (b - p) * (1.0 / std::max(1e-9, (b - p).length()));
            if (t0.x * t1.x + t0.y * t1.y < std::cos(15.0 * 3.14159265358979 / 180.0)) continue;   // too curved for a lay-by
            if (w.water(p.x, p.y)) continue;
            const double g0 = w.heightAt(p.x, p.y);
            for (double side : {-1.0, 1.0}) {
                const Vec2 out = Vec2(-t.y, t.x) * side;
                if (w.heightAt(p.x + out.x * 15.0, p.y + out.y * 15.0) > g0 + 1.0) continue;   // a cut bank, not a view
                double low = 1e30;
                for (double d : {80.0, 150.0, 250.0, 400.0}) low = std::min(low, w.heightAt(p.x + out.x * d, p.y + out.y * d));
                const double drop = g0 - low;
                if (drop < 40.0) continue;
                double facing = 0.0;
                for (const Vec2& tg : targets) {
                    Vec2 d = tg - p; const double L = d.length(); if (L < 200.0 || L > 6000.0) continue;
                    facing = std::max(facing, (d.x * out.x + d.y * out.y) / L);
                }
                cands.push_back({drop * (0.4 + 0.6 * facing), s, p, out});
            }
        }
        std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) { return x.score > y.score; });
        for (const Cand& c : cands) {
            bool farEnough = true;
            for (const Lookout& l : lookouts[k]) if ((l.at - c.p).length() < 1200.0) farEnough = false;
            if (!farEnough) continue;
            lookouts[k].push_back({c.p, c.out});
            if (lookouts[k].size() >= 2) break;
        }
    }

    // THE COUNTRY ROADS: the pass and the mountain road, and the links -- named as their ramps name them
    for (std::size_t k = 0; k < w.roads.size(); ++k) {
        const IslandRoad& rd = w.roads[k];
        if (rd.kind == "freeway" || rd.points.size() < 2) continue;
        const std::vector<std::vector<Vec2>> pieces = clip(rd.points);
        for (std::size_t q = 0; q < pieces.size(); ++q) {
            json e = {{"id", rd.kind + std::to_string(k) + (q ? "_" + std::to_string(q) : std::string())}, {"class", rd.kind == "link" ? "collector" : "mountain"},
                      {"floor", riverFloors(pieces[q])}, {"path", {{"points", pointsJson(pieces[q])}}}};
            // this piece's lay-bys: the lookout's station along it, the pocket on its valley side (the
            // builder's forward lanes lie right of the path, so a view to the right is a forward pocket)
            json pockets = json::array();
            const std::vector<Vec2>& P = pieces[q];
            double len = 0.0; for (std::size_t i = 1; i < P.size(); ++i) len += (P[i] - P[i - 1]).length();
            for (std::size_t li = 0; li < lookouts[k].size(); ++li) {
                const Lookout& lo = lookouts[k][li];
                double best = 1e30, bestS = 0.0, run = 0.0; Vec2 tan(1, 0);
                for (std::size_t i = 0; i + 1 < P.size(); ++i) {
                    const Vec2 ab = P[i + 1] - P[i]; const double L2 = ab.x * ab.x + ab.y * ab.y, L = std::sqrt(L2);
                    const double t = L2 > 1e-12 ? std::clamp(((lo.at - P[i]).x * ab.x + (lo.at - P[i]).y * ab.y) / L2, 0.0, 1.0) : 0.0;
                    const double d = (lo.at - (P[i] + ab * t)).length();
                    if (d < best) { best = d; bestS = run + L * t; tan = L > 1e-9 ? ab * (1.0 / L) : tan; }
                    run += L;
                }
                if (best > 5.0 || bestS < 60.0 || bestS > len - 60.0) continue;
                const bool right = (-tan.y) * lo.out.x + tan.x * lo.out.y < 0.0;   // out against the left normal: the right
                pockets.push_back({{"id", "look" + std::to_string(li)}, {"kind", "layby"}, {"side", "right"}, {"dir", right ? "fwd" : "back"},
                                   {"s0", bestS - 35.0}, {"s1", bestS + 35.0}, {"taper", 20.0}, {"taper_out", 20.0}});
            }
            if (!pockets.empty()) e["lanes"] = {{"pockets", pockets}};
            edges.push_back(e);
        }
    }
    // THE RAMPS, last (an edge may only refer to edges before it), their anchors on the chain that
    // holds their gore
    std::set<std::string> have;
    for (const json& e : edges) have.insert(e.value("id", std::string()));
    for (json r : w.rampEdges) {
        bool ok = true;
        for (const char* end : {"from", "to"}) {
            if (!r.contains(end)) continue;
            if (r[end].is_string()) { if (!have.count(r[end].get<std::string>())) ok = false; continue; }   // its street must be here
            if (!r[end].is_object() || !r[end].contains("edge")) continue;
            const std::string edge = r[end]["edge"].get<std::string>();
            const bool bSide = edge.size() && edge.back() == 'b';
            const Vec2 at(r[end]["at"][0].get<double>(), r[end]["at"][1].get<double>());
            if (!inWin(at, 0.0)) { ok = false; continue; }
            // the freeway piece on its side that holds its gore
            double best = 1e30;
            std::string pick;
            for (const auto& [id, P] : freewayPieces) {
                if ((id.find("_b") != std::string::npos) != bSide) continue;
                const double d = distToPolyline(at, P);
                if (d < best) { best = d; pick = id; }
            }
            if (pick.empty() || best > o.carriage + 5.0) ok = false; else r[end]["edge"] = pick;
        }
        if (ok) edges.push_back(r);
    }

    scene["edges"] = edges;

    // THE GROUND: the island's land and a margin, at gridRes, on the island's own terrain block
    double x0 = 1e30, x1 = -1e30, z0 = 1e30, z1 = -1e30;
    for (int j = 0; j < w.n; ++j)
        for (int i = 0; i < w.n; ++i)
            if (w.height[static_cast<std::size_t>(j) * w.n + i] > 0.0f) {
                const double x = -w.half + i * w.cell, z = -w.half + j * w.cell;
                x0 = std::min(x0, x); x1 = std::max(x1, x); z0 = std::min(z0, z); z1 = std::max(z1, z);
            }
    const double pad = 400.0;
    json bounds = {std::max(-w.half, x0 - pad), std::min(w.half, x1 + pad), std::max(-w.half, z0 - pad), std::min(w.half, z1 + pad)};
    if (windowed) bounds = {o.windowCentre.x - o.windowHalf - 150.0, o.windowCentre.x + o.windowHalf + 150.0, o.windowCentre.y - o.windowHalf - 150.0, o.windowCentre.y + o.windowHalf + 150.0};
    scene["terrain"] = {{"type", "procedural"},
                        {"bounds", bounds},
                        {"res", o.gridRes},
                        {"seed", static_cast<int>(w.terrain.value("seed", 1u))},
                        {"octaves", json::array({json::array({0.0, 500.0})})},   // no relief of its own: the island is the ground
                        {"base", w.terrain}};
    return scene;
}

std::pair<int, int> clearSignsOfPavement(std::vector<IslandSign>& signs, const json& scene, double carriageHalf) {
    struct Road { std::vector<Vec2> pts; double half; double x0, x1, z0, z1; };
    std::vector<Road> roads;
    const json& classes = scene.value("classes", json::object());
    for (const json& e : scene.value("edges", json::array())) {
        if (!e.contains("path") || !e["path"].contains("points")) continue;
        const json c = classes.value(e.value("class", std::string()), json::object());
        const double lanes = e.value("fwd", c.value("fwd", 1.0)) + e.value("back", c.value("back", 1.0));
        Road r;
        r.half = 0.5 * lanes * e.value("w", c.value("w", 3.5)) + c.value("shoulder", 0.0) + 0.5;
        r.x0 = r.z0 = 1e30; r.x1 = r.z1 = -1e30;
        for (const json& q : e["path"]["points"]) {
            r.pts.emplace_back(q[0].get<double>(), q[1].get<double>());
            r.x0 = std::min(r.x0, r.pts.back().x); r.x1 = std::max(r.x1, r.pts.back().x);
            r.z0 = std::min(r.z0, r.pts.back().y); r.z1 = std::max(r.z1, r.pts.back().y);
        }
        if (r.pts.size() >= 2) roads.push_back(std::move(r));
    }
    auto onPavement = [&](const Vec2& p) {
        for (const Road& r : roads) {
            if (p.x < r.x0 - r.half || p.x > r.x1 + r.half || p.y < r.z0 - r.half || p.y > r.z1 + r.half) continue;
            if (distToPolyline(p, r.pts) < r.half) return true;
        }
        return false;
    };
    auto blocked = [&](const IslandSign& s, const Vec2& at) {
        const Vec2 side(-s.facing.y, s.facing.x);
        const double spread = s.mount == "overhead" ? carriageHalf : 2.0;   // a gantry's uprights; a wide panel's two posts
        for (double k : {-1.0, 0.0, 1.0}) {
            if (s.mount == "overhead" && k == 0.0) continue;   // a gantry's middle spans the road
            if (onPavement(at + side * (spread * k))) return true;
        }
        return false;
    };
    int moved = 0, dropped = 0;
    std::vector<IslandSign> kept;
    for (IslandSign s : signs) {
        if (!blocked(s, s.at)) { kept.push_back(s); continue; }
        // the smallest move that clears: along the road it serves (backing up first -- read AHEAD of where
        // it was, not after), and up to 6 m to either side of it
        bool ok = false;
        const Vec2 side(-s.facing.y, s.facing.x);
        double bestCost = 1e30; Vec2 best = s.at;
        for (double d = 0.0; d <= 60.0; d += 5.0)
            for (double sg : {-1.0, 1.0}) {
                if (d == 0.0 && sg > 0) continue;
                for (double l : {0.0, -2.0, 2.0, -4.0, 4.0, -6.0, 6.0}) {
                    const double cost = d * (sg < 0 ? 1.0 : 1.2) + 3.0 * std::fabs(l);
                    if (cost >= bestCost || (d == 0.0 && l == 0.0)) continue;
                    const Vec2 at = s.at + s.facing * (sg * d) + side * l;
                    if (!blocked(s, at)) { bestCost = cost; best = at; ok = true; }
                }
            }
        if (ok) s.at = best;
        if (ok) { ++moved; kept.push_back(s); } else ++dropped;
    }
    signs = std::move(kept);
    return {moved, dropped};
}

}  // namespace engine
