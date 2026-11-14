#include "lane_map.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace citysim {

using engine::Real;
using engine::Vec2;

namespace {

// Two stations along a road written from one number (the road file's to the centimetre, the builder's spine) are one.
constexpr Real kSameStation = 0.01;
// How many untied links are listed by name before only the count is.
constexpr int kUntiedListed = 8;

// The point at station `at` of a line whose points have stations `s` (rising): clamped to its ends.
Vec2 pointAt(const std::vector<Vec2>& xy, const std::vector<Real>& s, Real at) {
    if (at <= s.front()) return xy.front();
    if (at >= s.back()) return xy.back();
    const std::size_t i = static_cast<std::size_t>(std::upper_bound(s.begin(), s.end(), at) - s.begin()) - 1;
    const Real span = s[i + 1] - s[i];
    const Real u = span > 1e-12 ? (at - s[i]) / span : 0;
    return xy[i] + (xy[i + 1] - xy[i]) * u;
}

Vec2 laneAt(const LinkLane& l, Real t) {
    return pointAt(l.xy, l.t, std::clamp(t, Real(0), Real(1)));
}

}  // namespace

void LaneMap::build(const engine::NavGraph& nav, const engine::LevelLanes& lanes) {
    const std::size_t nLinks = static_cast<std::size_t>(nav.linkCount());
    links_.assign(nLinks, {});
    outcome_.assign(nLinks, Outcome::NotDrivable);
    drivable = matched = countDiffers = partial = none = 0;
    counts.clear();
    untied.clear();
    // each road's lanes (the travel lanes the builder laid, with their stations)
    std::map<int, std::vector<int>> ofRoad;
    for (std::size_t k = 0; k < lanes.lanes.size(); ++k) {
        const engine::TrafficLane& T = lanes.lanes[k];
        if (T.kind == "connector" || T.parent < 0 || T.xy.size() < 2 || T.stations.size() != T.xy.size()) continue;
        ofRoad[T.parent].push_back(static_cast<int>(k));
    }
    int untiedLinks = 0;
    auto noteUntied = [&](const std::string& why) {
        if (untiedLinks++ < kUntiedListed) untied.push_back(why);
    };
    for (int li = 0; li < nav.linkCount(); ++li) {
        const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
        if (L.footpath) continue;
        ++drivable;
        Outcome& outcome = outcome_[static_cast<std::size_t>(li)];
        if (L.road < 0 || L.city < 0 || static_cast<std::size_t>(L.city) >= lanes.cityRoads.size()) {
            ++none;
            outcome = Outcome::None;
            char buf[128];
            std::snprintf(buf, sizeof buf, "link %d names no road of the road model: untied", li);
            noteUntied(buf);
            continue;
        }
        const int road = lanes.cityRoads[static_cast<std::size_t>(L.city)] + L.road;
        const std::map<int, std::vector<int>>::const_iterator it = ofRoad.find(road);
        if (it == ofRoad.end()) {
            ++none;
            outcome = Outcome::None;
            char buf[128];
            std::snprintf(buf, sizeof buf, "link %d: its road (%d) has no lanes laid: untied", li, road);
            noteUntied(buf);
            continue;
        }
        // its road's lanes running its way over its whole stretch, by slot from the centre-line side
        const int way = L.stationTo >= L.stationFrom ? 1 : -1;
        const Real lo = std::min(L.stationFrom, L.stationTo), hi = std::max(L.stationFrom, L.stationTo);
        const int want = std::max(1, L.lanes);
        std::vector<const engine::TrafficLane*> bySlot(static_cast<std::size_t>(want), nullptr);
        int found = 0;
        bool alongPart = false;
        for (int k : it->second) {
            const engine::TrafficLane& T = lanes.lanes[static_cast<std::size_t>(k)];
            if (T.dir != way || T.slot < 0 || T.slot >= want) continue;
            if (T.spineFrom > lo + kSameStation || T.spineTo < hi - kSameStation) {
                alongPart = alongPart || (T.spineTo > lo && T.spineFrom < hi);
                continue;
            }
            if (bySlot[static_cast<std::size_t>(T.slot)]) continue;
            bySlot[static_cast<std::size_t>(T.slot)] = &T;
            ++found;
        }
        if (found != want) {
            ++counts[{want, found}];
            if (alongPart) { ++partial; outcome = Outcome::Partial; }
            else if (found == 0) { ++none; outcome = Outcome::None; }
            else { ++countDiffers; outcome = Outcome::CountDiffers; }
            continue;
        }
        // each lane's own points between the link's two stations, the way the link runs
        std::vector<LinkLane>& out = links_[static_cast<std::size_t>(li)];
        const Real span = L.stationTo - L.stationFrom;
        for (const engine::TrafficLane* T : bySlot) {
            LinkLane ll;
            ll.w = T->w;
            ll.road = road;
            ll.xy.push_back(pointAt(T->xy, T->stations, L.stationFrom));
            ll.t.push_back(0);
            const std::size_t n = T->xy.size();
            for (std::size_t q = 0; q < n; ++q) {
                const std::size_t i = way > 0 ? q : n - 1 - q;
                const Real s = T->stations[i];
                if (s <= lo + 1e-6 || s >= hi - 1e-6) continue;
                ll.xy.push_back(T->xy[i]);
                ll.t.push_back(std::fabs(span) > 1e-9 ? (s - L.stationFrom) / span : 0.0);
            }
            ll.xy.push_back(pointAt(T->xy, T->stations, L.stationTo));
            ll.t.push_back(1);
            out.push_back(std::move(ll));
        }
        ++matched;
        outcome = Outcome::Matched;
    }
    if (untiedLinks > kUntiedListed) untied.push_back(std::to_string(untiedLinks) + " links untied in all");
}

Vec2 LaneMap::point(int link, Real t, Real laneF) const {
    const std::vector<LinkLane>& ls = links_[static_cast<std::size_t>(link)];
    const int n = static_cast<int>(ls.size());
    const Real fl = std::clamp(laneF, Real(0), Real(n));   // up to one past the last (the freeway's shoulder lane)
    const int k = std::min(static_cast<int>(std::floor(fl)), n - 1);
    const Real f = fl - k;
    const Vec2 p0 = laneAt(ls[static_cast<std::size_t>(k)], t);
    if (f <= 0) return p0;
    if (k + 1 < n) return p0 + (laneAt(ls[static_cast<std::size_t>(k + 1)], t) - p0) * f;
    // past the last lane: on by the spacing of the last two, or a lane's width toward the kerb
    if (n >= 2) return p0 + (p0 - laneAt(ls[static_cast<std::size_t>(n - 2)], t)) * f;
    const Vec2 a = laneAt(ls[0], std::max<Real>(0, t - 0.01)), b = laneAt(ls[0], std::min<Real>(1, t + 0.01));
    const Vec2 dir = b - a;
    const Real dl = dir.length();
    return dl > 1e-9 ? p0 + engine::rightOf(dir * (1.0 / dl)) * (ls[0].w * f) : p0;
}

}  // namespace citysim
