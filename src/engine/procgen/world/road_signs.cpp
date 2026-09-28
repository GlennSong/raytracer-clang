#include "road_signs.h"

#include "place_names.h"
#include "../../asset_root.h"
#include "../../text/font.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <sstream>

namespace engine {

using json = nlohmann::json;

namespace {

// ---- the freeway as a ring: stations wrap -----------------------------------------------------
struct Ring {
    std::vector<Vec2> pts;
    std::vector<double> st;
    double L = 0;
    explicit Ring(const std::vector<Vec2>& p) : pts(p) {
        st.push_back(0);
        for (std::size_t k = 0; k + 1 < pts.size(); ++k) st.push_back(st.back() + (pts[k + 1] - pts[k]).length());
        L = st.empty() ? 0 : st.back();
    }
    double wrap(double s) const { return L > 0 ? std::fmod(std::fmod(s, L) + L, L) : 0; }
    Vec2 at(double s) const {
        s = wrap(s);
        const std::size_t k = static_cast<std::size_t>(std::upper_bound(st.begin(), st.end(), s) - st.begin());
        const std::size_t i = std::clamp<std::size_t>(k, 1, pts.size() - 1);
        const double t = (s - st[i - 1]) / std::max(1e-9, st[i] - st[i - 1]);
        return pts[i - 1] + (pts[i] - pts[i - 1]) * t;
    }
    Vec2 tangent(double s) const {
        const Vec2 d = at(s + 10) - at(s - 10);
        return d.length() > 1e-9 ? d * (1.0 / d.length()) : Vec2(1, 0);
    }
    // metres from s0 to s1 travelling one way round (+1 with the stations, -1 against)
    double ahead(double s0, double s1, int dir) const { return wrap(dir > 0 ? s1 - s0 : s0 - s1); }
};

Vec2 unit(const Vec2& v) { const double L = v.length(); return L > 1e-9 ? v * (1.0 / L) : Vec2(1, 0); }
double cross2(const Vec2& a, const Vec2& b) { return a.x * b.y - a.y * b.x; }

std::string km(double m) {
    if (m < 950) return std::to_string(static_cast<int>(std::lround(m / 100.0) * 100)) + " m";
    return std::to_string(static_cast<int>(std::lround(m / 1000.0))) + " km";
}

bool insideLoops(const std::vector<std::vector<Vec2>>& loops, const Vec2& p) {
    bool in = false;
    for (const auto& L : loops)
        for (std::size_t i = 0, j = L.size() - 1; i < L.size(); j = i++)
            if ((L[i].y > p.y) != (L[j].y > p.y) && p.x < (L[j].x - L[i].x) * (p.y - L[i].y) / (L[j].y - L[i].y) + L[i].x) in = !in;
    return in;
}

// plan left of travel is the world's right (the mirror): the arrow a driver sees for a turn whose
// plan cross product is `c` (positive: the plan's left)
std::string worldTurn(double c, double threshold = 0.25) {
    if (c > threshold) return "right";
    if (c < -threshold) return "left";
    return "up";
}

}  // namespace

void planIslandSigns(IslandWorld& w) {
    w.signs.clear();
    if (w.freewayRoute.size() < 3 || w.interchanges.empty()) return;
    const Ring R(w.freewayRoute);
    const json& routes = placeNameBook().routes;
    const std::string fwyNo = routes.value("freeway", std::string("1"));
    const std::string cw = routes.value("loop", json::object()).value("clockwise", std::string("Inner Loop"));
    const std::string ccw = routes.value("loop", json::object()).value("counterclockwise", std::string("Outer Loop"));
    auto loopName = [&](int dir) { return (dir > 0) == w.routeClockwise ? cw : ccw; };
    // the freeway's section (plan_scene.h FreewaySection): carriageways 12 m off the centreline, the
    // pavement's edge 22 m; carriageway a (with the route, dir +1) is on the -normal side
    const double carriage = 12.0, edge = 22.0;
    auto side = [&](double s, int dir) { const Vec2 t = R.tangent(s); return Vec2(-t.y, t.x) * (dir > 0 ? -1.0 : 1.0); };
    auto roadside = [&](double s, int dir) { return R.at(s) + side(s, dir) * (edge + 5.0); };
    auto overhead = [&](double s, int dir) { return R.at(s) + side(s, dir) * carriage; };
    auto travel = [&](double s, int dir) { return R.tangent(s) * static_cast<double>(dir); };
    auto sign = [&](const std::string& kind, const Vec2& at, const Vec2& facing, const std::string& mount, json legend) {
        IslandSign sg{kind, at, unit(facing), mount, std::move(legend)};
        w.signs.push_back(sg);
    };
    // what each exit is signed for: its place, and the country road it starts, with the road's far end
    auto roadNo = [&](const std::string& kind) { return routes.value(kind, std::string()); };
    auto farPlace = [&](int road, const Vec2& from) {
        const std::vector<Vec2>& P = w.roads[static_cast<std::size_t>(road)].points;
        const Vec2 far = (P.front() - from).length() > (P.back() - from).length() ? P.front() : P.back();
        int best = -1;
        double bd = 2500.0;
        for (std::size_t k = 0; k < w.sites.size(); ++k) {
            double d = (w.sites[k].at - far).length();
            for (const auto& loop : w.sites[k].limits) for (const Vec2& q : loop) d = std::min(d, (q - far).length());
            if (d < bd) { bd = d; best = static_cast<int>(k); }
        }
        return best;
    };
    std::vector<json> dests(w.interchanges.size());
    for (std::size_t k = 0; k < w.interchanges.size(); ++k) {
        const IslandInterchange& ic = w.interchanges[k];
        const IslandRoad& rd = w.roads[static_cast<std::size_t>(ic.road)];
        json d = json::array();
        if (rd.kind == "pass" || rd.kind == "mountain") {
            d.push_back("#" + roadNo(rd.kind));
            const int far = farPlace(ic.road, ic.at);
            if (far >= 0 && far != ic.site) d.push_back(w.sites[static_cast<std::size_t>(far)].name);
        }
        // a link's exit: the street it lands on, then its place -- three exits into one city are three streets
        if (rd.kind == "link" && !rd.street.empty()) d.push_back(rd.street);
        if (ic.site >= 0) d.push_back(w.sites[static_cast<std::size_t>(ic.site)].name);
        dests[k] = d;
    }
    // the next CITY each way from an interchange: its direction's control city
    // ...and the first place each way: when both ways reach the same city (two cities on a ring do,
    // from either of them), a sign naming it for both ramps says nothing -- each names its first town
    auto nextPlace = [&](std::size_t from, int dir, bool cityOnly) {
        const std::size_t n = w.interchanges.size();
        for (std::size_t step = 1; step <= n; ++step) {
            const std::size_t k = dir > 0 ? (from + step) % n : (from + n - step % n) % n;
            const int site = w.interchanges[k].site;
            if (site < 0 || site == w.interchanges[from].site) continue;
            if (!cityOnly || w.sites[static_cast<std::size_t>(site)].kind == "city") return w.sites[static_cast<std::size_t>(site)].name;
        }
        return std::string();
    };
    auto controlCity = [&](std::size_t from, int dir) {
        const std::string mine = nextPlace(from, dir, true), other = nextPlace(from, -dir, true);
        if (mine.empty() || mine == other) return nextPlace(from, dir, false);
        return mine;
    };

    for (std::size_t k = 0; k < w.interchanges.size(); ++k) {
        const IslandInterchange& ic = w.interchanges[k];
        for (int dir : {+1, -1}) {
            const IslandInterchange::Ramp* off = nullptr;
            const IslandInterchange::Ramp* on = nullptr;
            for (const IslandInterchange::Ramp& rp : ic.ramps)
                if (rp.withRoute == (dir > 0)) (rp.off ? off : on) = &rp;
            // ON THE FREEWAY, BEFORE THE EXIT
            if (off) {
                const double sg = off->gore;
                // the last interchange before this one, this way: no advance sign reaches back past it
                double room = R.L;
                for (const IslandInterchange& o : w.interchanges)
                    if (&o != &ic) for (const IslandInterchange::Ramp& rp : o.ramps) room = std::min(room, R.ahead(rp.gore, sg, dir));
                for (double back : {2000.0, 1000.0})
                    if (back + 150.0 < room) {
                        const double s = sg - dir * back;
                        sign("advance", roadside(s, dir), travel(s, dir), "roadside", {{"exit", ic.exit}, {"dests", dests[k]}, {"dist", km(back)}});
                    }
                const double se = sg - dir * 150.0;
                sign("exit", overhead(se, dir), travel(se, dir), "overhead", {{"exit", ic.exit}, {"dests", dests[k]}, {"arrow", "up-" + worldTurn(-1.0)}});
                const double sgo = sg + dir * 70.0;   // in the gore, between the ramp and the freeway
                sign("gore", roadside(sgo, dir), travel(sgo, dir), "roadside", {{"exit", ic.exit}, {"arrow", "up-" + worldTurn(-1.0)}});
            }
            // AFTER THE ON-RAMP: the route marker, then how far the next places are
            if (on) {
                const double sm = on->gore;
                const double sr = sm + dir * 200.0, sd = sm + dir * 600.0;
                sign("route", roadside(sr, dir), travel(sr, dir), "roadside", {{"route", fwyNo}, {"dir", loopName(dir)}});
                json rows = json::array();
                std::set<std::string> listed;
                if (ic.site >= 0) listed.insert(w.sites[static_cast<std::size_t>(ic.site)].name);
                const std::size_t n = w.interchanges.size();
                for (std::size_t step = 1; step < n && rows.size() < 3; ++step) {
                    const std::size_t j = dir > 0 ? (k + step) % n : (k + n - step) % n;
                    const IslandInterchange& o = w.interchanges[j];
                    if (o.site < 0) continue;
                    const std::string& nm = w.sites[static_cast<std::size_t>(o.site)].name;
                    if (listed.count(nm)) continue;
                    listed.insert(nm);
                    rows.push_back({nm, std::to_string(std::max(1L, std::lround(R.ahead(sd, o.station, dir) / 1000.0)))});
                }
                if (!rows.empty()) sign("distance", roadside(sd, dir), travel(sd, dir), "roadside", {{"rows", rows}, {"unit", "km"}});
            }
        }
        // ON THE ROAD: which ramp goes which way, and the wrong way onto an exit
        const std::vector<Vec2>& P = w.roads[static_cast<std::size_t>(ic.road)].points;
        // the road oriented from its place (or its longer side) towards the freeway and beyond
        Vec2 home = P.front();
        if (ic.site >= 0) {
            const Vec2 c = w.sites[static_cast<std::size_t>(ic.site)].at;
            home = (P.front() - c).length() < (P.back() - c).length() ? P.front() : P.back();
        }
        std::vector<Vec2> road = P;
        if ((road.back() - home).length() < (road.front() - home).length()) std::reverse(road.begin(), road.end());
        auto alongRoad = [&](const Vec2& q, double back, Vec2& at, Vec2& dirOut) {
            std::size_t i = 0;
            double bd = 1e30;
            for (std::size_t m = 0; m < road.size(); ++m) if ((road[m] - q).length() < bd) { bd = (road[m] - q).length(); i = m; }
            double left = back;
            std::size_t m = i;
            while (m > 0 && left > (road[m] - road[m - 1]).length()) { left -= (road[m] - road[m - 1]).length(); --m; }
            at = m > 0 ? road[m] + (road[m - 1] - road[m]) * (left / std::max(1e-9, (road[m] - road[m - 1]).length())) : road[0];
            dirOut = unit(road[std::min(i + 1, road.size() - 1)] - road[i > 0 ? i - 1 : 0]);
        };
        for (const IslandInterchange::Ramp& rp : ic.ramps) {
            const int dir = rp.withRoute ? +1 : -1;
            if (!rp.off) {
                // ENTRANCE: 40 m before the on-ramp, for the traffic coming from the place
                Vec2 at, f;
                alongRoad(rp.terminal, 40.0, at, f);
                const Vec2 into = unit((rp.path.size() > 4 ? rp.path[4] : rp.path.back()) - rp.terminal);
                const Vec2 right(f.y, -f.x);   // the plan's right: the kerb side
                sign("entrance", at + right * 9.0, f, "roadside",
                     {{"route", fwyNo}, {"dir", loopName(dir)}, {"city", controlCity(k, dir)}, {"arrow", worldTurn(cross2(f, into))}});
            } else if (rp.path.size() >= 6) {
                // DO NOT ENTER where the exit meets the road, WRONG WAY 80 m up it, both facing a driver
                // who turned into it
                const Vec2 f = unit(rp.path[rp.path.size() - 5] - rp.terminal);
                const Vec2 right(f.y, -f.x);
                sign("do-not-enter", rp.terminal + f * 12.0 + right * 7.0, f, "roadside", json::object());
                double acc = 0;
                Vec2 q = rp.terminal;
                for (std::size_t m = rp.path.size() - 1; m > 0 && acc < 80.0; --m) { acc += (rp.path[m] - rp.path[m - 1]).length(); q = rp.path[m - 1]; }
                sign("wrong-way", q + right * 7.0, f, "roadside", json::object());
            }
        }
    }
    // TOWN LIMITS: where a road enters a place, facing the traffic coming in
    for (std::size_t si = 0; si < w.sites.size(); ++si) {
        const IslandSite& st = w.sites[si];
        if (st.limits.empty()) continue;
        for (const IslandRoad& rd : w.roads) {
            if (rd.kind == "freeway" || rd.points.size() < 2) continue;
            for (int way : {+1, -1}) {
                const std::vector<Vec2>& P = rd.points;
                for (std::size_t m = 1; m < P.size(); ++m) {
                    const Vec2 a = way > 0 ? P[m - 1] : P[P.size() - m], b = way > 0 ? P[m] : P[P.size() - 1 - m];
                    if (insideLoops(st.limits, a) || !insideLoops(st.limits, b)) continue;
                    const Vec2 f = unit(b - a);
                    sign("limit", b + Vec2(f.y, -f.x) * 8.0, f, "roadside", {{"name", st.name}, {"pop", st.population}, {"kind", st.kind}});
                    break;
                }
            }
        }
    }
    // TRAILBLAZERS: the pass and the mountain road, 150 m out from the freeway, with the far place
    for (std::size_t k = 0; k < w.roads.size(); ++k) {
        const IslandRoad& rd = w.roads[k];
        if ((rd.kind != "pass" && rd.kind != "mountain") || rd.points.size() < 2) continue;
        for (int end : {0, 1}) {
            std::vector<Vec2> P = rd.points;
            if (end) std::reverse(P.begin(), P.end());
            double acc = 0;
            std::size_t m = 1;
            while (m + 1 < P.size() && acc < 150.0) { acc += (P[m] - P[m - 1]).length(); ++m; }
            const Vec2 f = unit(P[m] - P[m - 1]);
            // how far to the far end, and who is there
            double len = 0;
            for (std::size_t q = 1; q < P.size(); ++q) len += (P[q] - P[q - 1]).length();
            const int far = farPlace(static_cast<int>(k), P.front());
            json legend = {{"route", roadNo(rd.kind)}, {"dist", km(len - acc)}};
            if (far >= 0) legend["dest"] = w.sites[static_cast<std::size_t>(far)].name;
            sign("trailblazer", P[m - 1] + Vec2(f.y, -f.x) * 7.0, f, "roadside", legend);
        }
    }
}

// ---- THE FACE ------------------------------------------------------------------------------------
namespace {
// text width in metres at a cap height, measured with the sign font
double textW(const std::string& t, double cap) {
    const Font* f = signFont();
    if (!f) return t.size() * cap * 0.72;
    const float P = 100.0f;
    return f->measure(t, P) * cap / f->capHeight(P);
}
constexpr const char* kGreen = "#00693f";
constexpr const char* kWhite = "#ffffff";
constexpr const char* kRed = "#c1272d";
constexpr const char* kBlack = "#111111";

void border(SignFace& f, const char* color, double inset = 0.06, double width = 0.05) {
    f.elems.push_back({"rect", inset, inset, f.w - 2 * inset, f.h - 2 * inset, "", width, color, "stroke", 0, 0.12});
}
}  // namespace

SignFace layoutSign(const IslandSign& s) {
    SignFace f;
    const json& L = s.legend;
    const double pad = 0.30;
    auto line = [&](const json& d, double cap) {   // a destination: a place, or a route shield
        const std::string t = d.get<std::string>();
        return t.size() > 1 && t[0] == '#' ? cap * 2.2 : textW(t, cap);
    };
    if (s.kind == "advance" || s.kind == "exit") {
        const double cap = s.kind == "exit" ? 0.40 : 0.36, row = cap * 1.75;
        const bool arrow = s.kind == "exit";
        double tw = 0;
        for (const json& d : L["dests"]) tw = std::max(tw, line(d, cap));
        const std::string dist = L.value("dist", std::string());
        if (!dist.empty()) tw = std::max(tw, textW(dist, cap * 0.8));
        const double arrowW = arrow ? cap * 2.4 : 0.0;
        const int rows = static_cast<int>(L["dests"].size()) + (dist.empty() ? 0 : 1);
        f.w = std::max(2.4, tw + 2 * pad + arrowW);
        f.h = rows * row + 2 * pad + 0.1;
        border(f, kWhite);
        // the exit side is the world's left (traffic keeps left): the arrow there, the text beside it
        const double x0 = pad + arrowW;
        double y = pad + cap + 0.1;
        for (const json& d : L["dests"]) {
            const std::string t = d.get<std::string>();
            if (t.size() > 1 && t[0] == '#') f.elems.push_back({"shield", x0 + (tw - cap * 2.0) / 2, y - cap * 1.3, cap * 2.0, cap * 1.7, t.substr(1)});
            else f.elems.push_back({"text", x0 + tw / 2, y, 0, 0, t, cap, kWhite, "middle"});
            y += row;
        }
        if (!dist.empty()) f.elems.push_back({"text", x0 + tw / 2, y, 0, 0, dist, cap * 0.8, kWhite, "middle"});
        if (arrow) f.elems.push_back({"arrow", pad * 0.6, f.h / 2 - cap * 1.1, cap * 2.0, cap * 2.2, "", 0, kWhite, "", L.value("arrow", "up-left") == "up-right" ? 45.0 : -45.0});
        // the EXIT tab, on the exit's side of the top edge
        const std::string tab = "EXIT " + L.value("exit", std::string());
        const double tcap = 0.28, tabW = textW(tab, tcap) + 0.4;
        f.elems.push_back({"tab", 0.0, -tcap * 1.9, tabW, tcap * 1.9, tab, tcap, kWhite});
        return f;
    }
    if (s.kind == "gore") {
        const double cap = 0.30;
        f.w = std::max(1.4, textW("EXIT " + L.value("exit", std::string()), cap) + 0.5); f.h = 1.3;
        border(f, kWhite);
        f.elems.push_back({"text", f.w / 2, 0.55, 0, 0, "EXIT " + L.value("exit", std::string()), cap, kWhite, "middle"});
        f.elems.push_back({"arrow", f.w / 2 - 0.3, 0.68, 0.6, 0.5, "", 0, kWhite, "", L.value("arrow", "up-left") == "up-right" ? 45.0 : -45.0});
        return f;
    }
    if (s.kind == "distance") {
        const double cap = 0.34, row = cap * 1.8;
        double nw = 0, dw = 0;
        for (const json& r : L["rows"]) { nw = std::max(nw, textW(r[0].get<std::string>(), cap)); dw = std::max(dw, textW(r[1].get<std::string>(), cap)); }
        f.w = nw + dw + 0.9 + 2 * pad;
        f.h = L["rows"].size() * row + 2 * pad;
        border(f, kWhite);
        double y = pad + cap + 0.08;
        for (const json& r : L["rows"]) {
            f.elems.push_back({"text", pad, y, 0, 0, r[0].get<std::string>(), cap, kWhite, "start"});
            f.elems.push_back({"text", f.w - pad, y, 0, 0, r[1].get<std::string>(), cap, kWhite, "end"});
            y += row;
        }
        return f;
    }
    if (s.kind == "route") {
        // a route marker: the direction plate over the shield, black on white
        f.background = kWhite;
        const std::string dir = L.value("dir", std::string());
        const double cap = 0.16;
        f.w = std::max(0.9, textW(dir, cap) + 0.3);
        f.h = 1.35;
        border(f, kBlack, 0.03, 0.03);
        f.elems.push_back({"text", f.w / 2, 0.08 + cap, 0, 0, dir, cap, kBlack, "middle"});
        f.elems.push_back({"shield", (f.w - 0.75) / 2, 0.38, 0.75, 0.8, L.value("route", std::string())});
        return f;
    }
    if (s.kind == "entrance") {
        // the route and its direction, the city that way, and which way to turn for it
        const double cap = 0.30;
        const std::string dir = L.value("dir", std::string()), city = L.value("city", std::string());
        const double tw = std::max({textW(dir, cap * 0.8) + 0.8, textW(city, cap), 1.2});
        f.w = tw + 2 * pad;
        // rows: the shield and direction, the city, the arrow -- each its own band
        const double rowShield = pad, rowCity = rowShield + 0.55 + 0.2 + cap, rowArrow = rowCity + 0.25;
        f.h = rowArrow + 0.6 + pad;
        border(f, kWhite);
        f.elems.push_back({"shield", pad, rowShield, 0.6, 0.55, L.value("route", std::string())});
        f.elems.push_back({"text", pad + 0.75, rowShield + 0.4, 0, 0, dir, cap * 0.8, kWhite, "start"});
        f.elems.push_back({"text", f.w / 2, rowCity, 0, 0, city, cap, kWhite, "middle"});
        const std::string a = L.value("arrow", std::string("up"));
        f.elems.push_back({"arrow", f.w / 2 - 0.3, rowArrow, 0.6, 0.6, "", 0, kWhite, "", a == "left" ? -90.0 : a == "right" ? 90.0 : 0.0});
        return f;
    }
    if (s.kind == "do-not-enter") {
        f.background = kWhite;
        f.w = f.h = 0.9;
        f.elems.push_back({"disc", 0.05, 0.05, 0.8, 0.8, "", 0, kRed});
        f.elems.push_back({"rect", 0.15, 0.40, 0.6, 0.12, "", 0, kWhite});
        f.elems.push_back({"text", 0.45, 0.33, 0, 0, "DO NOT", 0.075, kWhite, "middle"});
        f.elems.push_back({"text", 0.45, 0.65, 0, 0, "ENTER", 0.075, kWhite, "middle"});
        return f;
    }
    if (s.kind == "wrong-way") {
        f.background = kRed;
        f.w = 1.1; f.h = 0.75;
        border(f, kWhite, 0.04, 0.03);
        f.elems.push_back({"text", f.w / 2, 0.33, 0, 0, "WRONG", 0.16, kWhite, "middle"});
        f.elems.push_back({"text", f.w / 2, 0.58, 0, 0, "WAY", 0.16, kWhite, "middle"});
        return f;
    }
    if (s.kind == "limit") {
        const double cap = 0.30;
        const std::string name = L.value("name", std::string());
        const int pop = L.value("pop", 0);
        const std::string kind = L.value("kind", std::string()) == "city" ? "CITY LIMIT" : "TOWN LIMIT";
        std::string popText;
        if (pop > 0) {
            std::string n = std::to_string(pop);
            for (int i = static_cast<int>(n.size()) - 3; i > 0; i -= 3) n.insert(static_cast<std::size_t>(i), ",");
            popText = "POP " + n;
        }
        f.w = std::max({textW(name, cap), textW(kind, cap * 0.6), textW(popText, cap * 0.6)}) + 2 * pad;
        f.h = pad * 2 + cap * (popText.empty() ? 2.4 : 3.5);
        border(f, kWhite);
        f.elems.push_back({"text", f.w / 2, pad + cap, 0, 0, name, cap, kWhite, "middle"});
        f.elems.push_back({"text", f.w / 2, pad + cap * 2.1, 0, 0, kind, cap * 0.6, kWhite, "middle"});
        if (!popText.empty()) f.elems.push_back({"text", f.w / 2, pad + cap * 3.1, 0, 0, popText, cap * 0.6, kWhite, "middle"});
        return f;
    }
    if (s.kind == "trailblazer") {
        const double cap = 0.28;
        const std::string dest = L.value("dest", std::string()), dist = L.value("dist", std::string());
        f.w = std::max(textW(dest, cap), textW(dist, cap)) + 2 * pad + 0.9;
        f.h = 1.3;
        border(f, kWhite);
        f.elems.push_back({"shield", pad, (f.h - 0.7) / 2, 0.7, 0.7, L.value("route", std::string())});
        f.elems.push_back({"text", pad + 0.9, pad + cap + 0.05, 0, 0, dest, cap, kWhite, "start"});
        f.elems.push_back({"text", pad + 0.9, pad + cap * 2.5 + 0.05, 0, 0, dist, cap, kWhite, "start"});
        return f;
    }
    f.w = 1; f.h = 1;
    return f;
}

std::string signFaceSvg(const SignFace& f, double x, double y, double k) {
    std::ostringstream o;
    o.setf(std::ios::fixed);
    o.precision(2);
    const Font* font = signFont();
    const double capRatio = font ? font->capHeight(100.0f) / 100.0 : 0.7;   // cap height per em
    // text as the font's own OUTLINES (Font::svgPath): the face renders the same in any viewer
    auto text = [&](const std::string& t, double tx, double baseline, double cap, const std::string& color, const std::string& anchor) {
        if (!font) {
            o << "<text x=\"" << tx << "\" y=\"" << baseline << "\" font-size=\"" << cap / capRatio << "\" fill=\"" << color << "\" text-anchor=\"" << anchor << "\">" << t << "</text>";
            return;
        }
        const float P = static_cast<float>(cap / capRatio);   // the em that gives this cap height
        const double w = font->measure(t, P);
        const double x0 = anchor == "middle" ? tx - w / 2 : anchor == "end" ? tx - w : tx;
        o << "<path fill=\"" << color << "\" d=\"" << font->svgPath(t, static_cast<float>(x0), static_cast<float>(baseline), P) << "\"/>";
    };
    o << "<g transform=\"translate(" << x << ' ' << y << ") scale(" << k << ")\">";
    for (const SignElem& e : f.elems)
        if (e.kind == "tab") o << "<rect x=\"" << e.x << "\" y=\"" << e.y << "\" width=\"" << e.w << "\" height=\"" << e.h + 0.08 << "\" rx=\"0.1\" fill=\"" << kGreen
                               << "\" stroke=\"#fff\" stroke-width=\"0.04\"/>", text(e.text, e.x + e.w / 2, e.y + e.h * 0.5 + e.cap / 2, e.cap, "#fff", "middle");
    o << "<rect width=\"" << f.w << "\" height=\"" << f.h << "\" rx=\"0.12\" fill=\"" << f.background << "\"/>";
    for (const SignElem& e : f.elems) {
        if (e.kind == "rect" && e.anchor == "stroke")
            o << "<rect x=\"" << e.x << "\" y=\"" << e.y << "\" width=\"" << e.w << "\" height=\"" << e.h << "\" rx=\"" << e.radius << "\" fill=\"none\" stroke=\"" << e.color << "\" stroke-width=\"" << e.cap << "\"/>";
        else if (e.kind == "rect")
            o << "<rect x=\"" << e.x << "\" y=\"" << e.y << "\" width=\"" << e.w << "\" height=\"" << e.h << "\" fill=\"" << e.color << "\"/>";
        else if (e.kind == "disc")
            o << "<circle cx=\"" << e.x + e.w / 2 << "\" cy=\"" << e.y + e.h / 2 << "\" r=\"" << e.w / 2 << "\" fill=\"" << e.color << "\"/>";
        else if (e.kind == "text")
            text(e.text, e.x, e.y, e.cap, e.color, e.anchor);
        else if (e.kind == "shield") {
            // a route shield: white, a flat top and a pointed foot, the number in black
            const double X = e.x, Y = e.y, W = e.w, H = e.h;
            o << "<path d=\"M" << X << ' ' << Y << "H" << X + W << "V" << Y + H * 0.62 << "Q" << X + W << ' ' << Y + H * 0.9 << ' ' << X + W / 2 << ' ' << Y + H
              << "Q" << X << ' ' << Y + H * 0.9 << ' ' << X << ' ' << Y + H * 0.62 << "Z\" fill=\"#fff\" stroke=\"#111\" stroke-width=\"0.03\"/>";
            text(e.text, X + W / 2, Y + H * 0.62, H * 0.42, "#111", "middle");
        } else if (e.kind == "arrow") {
            // an up arrow in a w x h box, rotated about its centre
            const double cx = e.x + e.w / 2, cy = e.y + e.h / 2, W = e.w, H = e.h;
            o << "<path transform=\"rotate(" << e.angle << ' ' << cx << ' ' << cy << ")\" fill=\"" << e.color << "\" d=\"M" << cx << ' ' << cy - H / 2
              << "L" << cx + W / 2 << ' ' << cy - H * 0.05 << "L" << cx + W * 0.16 << ' ' << cy - H * 0.05 << "L" << cx + W * 0.16 << ' ' << cy + H / 2
              << "L" << cx - W * 0.16 << ' ' << cy + H / 2 << "L" << cx - W * 0.16 << ' ' << cy - H * 0.05 << "L" << cx - W / 2 << ' ' << cy - H * 0.05 << "Z\"/>";
        }
    }
    o << "</g>";
    return o.str();
}

bool writeSignSheetSvg(const IslandWorld& w, const std::string& svgPath) {
    // the sign font, embedded, so the sheet shows the faces as they will be lettered
    std::string fontFace;
    {
        std::ifstream in(assetPath("assets/fonts/Overpass-Bold.ttf"), std::ios::binary);
        const std::vector<unsigned char> bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        static const char* T = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        std::string b;
        for (std::size_t i = 0; i < bytes.size(); i += 3) {
            const unsigned v = (bytes[i] << 16) | ((i + 1 < bytes.size() ? bytes[i + 1] : 0) << 8) | (i + 2 < bytes.size() ? bytes[i + 2] : 0);
            b += T[(v >> 18) & 63]; b += T[(v >> 12) & 63];
            b += i + 1 < bytes.size() ? T[(v >> 6) & 63] : '='; b += i + 2 < bytes.size() ? T[v & 63] : '=';
        }
        if (!b.empty()) fontFace = "@font-face{font-family:SignFont;src:url(data:font/ttf;base64," + b + ");}";
    }
    // group the signs: by the interchange they stand at (the nearest), then the rest
    struct Item { const IslandSign* s; SignFace f; };
    std::map<std::string, std::vector<Item>> groups;
    std::vector<std::string> order;
    for (const IslandSign& s : w.signs) {
        std::string key = "Other";
        double best = 2600.0;
        for (const IslandInterchange& ic : w.interchanges)
            if ((ic.at - s.at).length() < best && s.kind != "limit" && s.kind != "trailblazer") {
                best = (ic.at - s.at).length();
                key = "Exit " + ic.exit + (ic.site >= 0 ? " — " + w.sites[static_cast<std::size_t>(ic.site)].name : std::string());
            }
        if (s.kind == "limit") key = "Town and city limits";
        if (s.kind == "trailblazer") key = "Country roads";
        if (!groups.count(key)) order.push_back(key);
        groups[key].push_back({&s, layoutSign(s)});
    }
    const double scale = 60.0;   // px per metre on the sheet
    const double pageW = 1800;
    std::ostringstream body;
    body.setf(std::ios::fixed);
    body.precision(1);
    double y = 40;
    for (const std::string& key : order) {
        body << "<text x=\"20\" y=\"" << y + 26 << "\" font-size=\"26\" fill=\"#222\" font-weight=\"bold\">" << key << "</text>\n";
        y += 50;
        double x = 20, rowH = 0;
        for (const Item& it : groups[key]) {
            const double wpx = it.f.w * scale, hpx = it.f.h * scale;
            if (x + wpx > pageW - 20) { x = 20; y += rowH + 50; rowH = 0; }
            body << signFaceSvg(it.f, x, y + 30, scale) << "\n";
            body << "<text x=\"" << x << "\" y=\"" << y + 30 + hpx + 18 << "\" font-size=\"13\" fill=\"#555\">" << it.s->kind << " · " << it.s->mount
                 << " · (" << std::lround(it.s->at.x) << ", " << std::lround(it.s->at.y) << ")</text>\n";
            x += std::max(wpx, 170.0) + 30;
            rowH = std::max(rowH, hpx + 30);
        }
        y += rowH + 60;
    }
    std::ofstream f(svgPath);
    if (!f) return false;
    f << "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " << pageW << ' ' << y << "\" width=\"" << pageW << "\" height=\"" << y << "\">\n"
      << "<style>" << fontFace << "text{font-family:SignFont,Overpass,sans-serif}</style>\n"
      << "<rect width=\"100%\" height=\"100%\" fill=\"#e9e6df\"/>\n" << body.str() << "</svg>\n";
    return static_cast<bool>(f);
}

}  // namespace engine
