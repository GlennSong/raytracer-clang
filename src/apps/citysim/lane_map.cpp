#include "lane_map.h"

#include "../../engine/procgen/city/polyline.h"   // distToPolyline

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <unordered_map>

namespace citysim {

using engine::Real;
using engine::Vec2;

namespace {

// Where `p` falls on polyline `xy` (cumulative stations `s`): its station, its distance from the line, and how far
// it lies PAST an end along the end segment (> 0: beyond the line's start or end).
struct Proj {
    Real s = 0, dist = 1e30, past = 0;
};
Proj project(const std::vector<Vec2>& xy, const std::vector<Real>& s, const Vec2& p) {
    Proj best;
    for (std::size_t i = 0; i + 1 < xy.size(); ++i) {
        const Vec2 a = xy[i], b = xy[i + 1];
        const Vec2 ab = b - a;
        const Real l2 = ab.x * ab.x + ab.y * ab.y;
        if (l2 < 1e-12) continue;
        const Real raw = ((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2;
        const Real u = std::clamp(raw, Real(0), Real(1));
        const Vec2 q = a + ab * u;
        const Real d = (p - q).length();
        if (d < best.dist) {
            best.dist = d;
            best.s = s[i] + (s[i + 1] - s[i]) * u;
            const Real len = std::sqrt(l2);
            best.past = i == 0 && raw < 0 ? -raw * len : (i + 2 == xy.size() && raw > 1 ? (raw - 1) * len : 0);
        }
    }
    return best;
}

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

void LaneMap::build(const engine::NavGraph& nav, const std::vector<engine::TrafficLane>& lanes,
                    const std::vector<std::vector<Vec2>>& roads, const std::vector<engine::StatedLanes>& stated) {
    const std::size_t nLinks = static_cast<std::size_t>(nav.linkCount());
    links_.assign(nLinks, {});
    roadOf_.assign(nLinks, -1);
    wayOf_.assign(nLinks, 0);
    stated_.assign(nLinks, -1);
    outcome_.assign(nLinks, Outcome::NotDrivable);
    drivable = matched = countDiffers = partial = none = resolved = 0;
    counts.clear();
    // THE ROAD UNDER A LINK: of the roads with lanes running its way at its middle `M` (`byRoad`: road -> (lanes, summed
    // |lateral|)), the one whose centre line lies nearest M -- the link was cut from that line (the road twin keeps a
    // road's spine to 1.5 m). Without the centre lines: the one with the most lanes there (ties: the nearer).
    auto pickRoad = [&roads](const std::map<int, std::pair<int, Real>>& byRoad, const Vec2& M) {
        int road = byRoad.begin()->first;
        Real best = 1e30;
        for (const auto& [p, r] : byRoad) {
            if (static_cast<std::size_t>(p) >= roads.size() || roads[static_cast<std::size_t>(p)].size() < 2) {
                best = 1e30;   // a road without its line: by the lanes, for all of them
                break;
            }
            const Real d = engine::distToPolyline(M, roads[static_cast<std::size_t>(p)]);
            if (d < best) { best = d; road = p; }
        }
        if (best < 1e29) return road;
        road = byRoad.begin()->first;
        for (const auto& [p, r] : byRoad) {
            const std::pair<int, Real>& b = byRoad.at(road);
            if (r.first > b.first || (r.first == b.first && r.second / r.first < b.second / b.first)) road = p;
        }
        return road;
    };
    // the travel lanes, with their stations, and a grid of their points (16 m cells) to find the ones near a link
    // (in TRAVEL order: a lane against its road, dir -1, keeps the road's point order)
    struct Src { const engine::TrafficLane* lane; std::vector<Vec2> xy; std::vector<Real> s; };
    std::vector<Src> src;
    for (const engine::TrafficLane& l : lanes) {
        if (l.kind == "connector" || l.xy.size() < 2) continue;
        Src e{&l, l.xy, {}};
        if (l.dir < 0) std::reverse(e.xy.begin(), e.xy.end());
        e.s.resize(e.xy.size(), 0);
        for (std::size_t i = 1; i < e.xy.size(); ++i) e.s[i] = e.s[i - 1] + (e.xy[i] - e.xy[i - 1]).length();
        src.push_back(std::move(e));
    }
    constexpr Real kCell = 16.0;
    auto key = [](int cx, int cz) { return (static_cast<int64_t>(cx) << 32) ^ static_cast<uint32_t>(cz); };
    std::unordered_map<int64_t, std::vector<int>> grid;
    for (int k = 0; k < static_cast<int>(src.size()); ++k)
        for (const Vec2& p : src[static_cast<std::size_t>(k)].xy) {
            std::vector<int>& cell = grid[key(static_cast<int>(std::floor(p.x / kCell)), static_cast<int>(std::floor(p.y / kCell)))];
            if (cell.empty() || cell.back() != k) cell.push_back(k);
        }
    std::vector<int> cand;
    for (int li = 0; li < nav.linkCount(); ++li) {
        const engine::NavLink& L = nav.links[static_cast<std::size_t>(li)];
        if (L.footpath) continue;
        ++drivable;
        const Vec2 A = nav.nodes[static_cast<std::size_t>(L.from)], B = nav.nodes[static_cast<std::size_t>(L.to)];
        const Real len = (B - A).length();
        Outcome& outcome = outcome_[static_cast<std::size_t>(li)];
        if (len < 1e-6) { ++none; outcome = Outcome::None; continue; }
        const Vec2 d = (B - A) * (1.0 / len), kerb = engine::rightOf(d), M = (A + B) * 0.5;
        // how far from the link's line a lane of it may lie: the whole carriageway either side, and a little
        const Real reach = 0.5 * std::max<Real>(L.width, L.lanes * 3.5 * (L.oneWay ? 1 : 2)) + 2.0;
        cand.clear();
        const int r = static_cast<int>(std::ceil(reach / kCell)) + 1;
        const int cx = static_cast<int>(std::floor(M.x / kCell)), cz = static_cast<int>(std::floor(M.y / kCell));
        for (int dz = -r; dz <= r; ++dz)
            for (int dx = -r; dx <= r; ++dx) {
                auto it = grid.find(key(cx + dx, cz + dz));
                if (it == grid.end()) continue;
                for (int k : it->second) cand.push_back(k);
            }
        std::sort(cand.begin(), cand.end());
        cand.erase(std::unique(cand.begin(), cand.end()), cand.end());
        struct Hit { Real lateral; LinkLane lane; const engine::TrafficLane* src = nullptr; };
        std::vector<Hit> hits;
        bool sawPartial = false;
        std::map<int, std::pair<int, Real>> atMiddle;   // road -> (its lanes running this way at the middle, summed |lateral|)
        std::map<int, int> waySum;                       // road -> the sum of those lanes' ways along it
        for (int k : cand) {
            const Src& e = src[static_cast<std::size_t>(k)];
            const std::vector<Vec2>& xy = e.xy;
            const Proj pm = project(xy, e.s, M);
            if (pm.dist > reach) continue;
            // running this link's way at its middle
            const Vec2 a = pointAt(xy, e.s, pm.s - 1.0), b = pointAt(xy, e.s, pm.s + 1.0);
            const Vec2 tan = b - a;
            const Real tl = tan.length();
            if (tl < 1e-6 || (tan.x * d.x + tan.y * d.y) / tl < 0.8) continue;
            // (a lane of the road under the link, whether or not it runs the link's whole length)
            if (e.lane->parent >= 0) {
                const Vec2 pMid = pointAt(xy, e.s, pm.s);
                std::pair<int, Real>& m = atMiddle[e.lane->parent];
                ++m.first;
                m.second += std::fabs((pMid.x - M.x) * kerb.x + (pMid.y - M.y) * kerb.y);
                waySum[e.lane->parent] += e.lane->dir;
            }
            // ...over its whole length
            const Proj pa = project(xy, e.s, A), pb = project(xy, e.s, B);
            if (pa.past > 1.0 || pb.past > 1.0 || pa.dist > reach || pb.dist > reach || pb.s <= pa.s) { sawPartial = true; continue; }
            LinkLane ll;
            ll.w = e.lane->w;
            ll.road = e.lane->parent;
            const Real span = pb.s - pa.s;
            ll.xy.push_back(pointAt(xy, e.s, pa.s));
            ll.t.push_back(0);
            for (std::size_t i = 0; i < xy.size(); ++i)
                if (e.s[i] > pa.s + 1e-6 && e.s[i] < pb.s - 1e-6) { ll.xy.push_back(xy[i]); ll.t.push_back((e.s[i] - pa.s) / span); }
            ll.xy.push_back(pointAt(xy, e.s, pb.s));
            ll.t.push_back(1);
            const Vec2 pM = pointAt(xy, e.s, pm.s);
            hits.push_back({(pM.x - M.x) * kerb.x + (pM.y - M.y) * kerb.y, std::move(ll), e.lane});
        }
        // THE ROAD UNDER THE LINK, the way the link runs along it, and the lanes its road file states that way
        if (!atMiddle.empty()) {
            const int road = pickRoad(atMiddle, M);
            roadOf_[static_cast<std::size_t>(li)] = road;
            wayOf_[static_cast<std::size_t>(li)] = waySum[road] >= 0 ? 1 : -1;
            if (static_cast<std::size_t>(road) < stated.size()) {
                const engine::StatedLanes& s = stated[static_cast<std::size_t>(road)];
                const int n = wayOf_[static_cast<std::size_t>(li)] > 0 ? s.along : s.against;
                if (n > 0) {
                    stated_[static_cast<std::size_t>(li)] = n;
                    ++resolved;
                }
            }
        }
        if (hits.empty()) {
            ++(sawPartial ? partial : none);
            outcome = sawPartial ? Outcome::Partial : Outcome::None;
            continue;
        }
        // ONE ROAD's lanes: near a junction a crossing road's lane can run this way for a while (its spine bends in);
        // keep the road under the link (a lane always has its road: the road was found from these very lanes)
        {
            const int road = roadOf_[static_cast<std::size_t>(li)];
            hits.erase(std::remove_if(hits.begin(), hits.end(), [&](const Hit& h) { return h.src->parent != road; }), hits.end());
            if (hits.empty()) { ++partial; outcome = Outcome::Partial; continue; }   // its own lanes start or end along it
        }
        std::sort(hits.begin(), hits.end(), [](const Hit& x, const Hit& y) { return x.lateral < y.lateral; });   // offside first
        // THE STATED LANES (the road file's, ADR-0418), all of them travel lanes: none is left over "for the parking" --
        // kerb parking is a band of the kerb lane where a road states it (roads/lanes LaneLayout::parking), never a lane
        // of its own. A link whose road is not known keeps the nav graph's own count.
        const int want = stated_[static_cast<std::size_t>(li)] > 0 ? stated_[static_cast<std::size_t>(li)] : std::max(1, L.lanes);
        ++counts[{want, static_cast<int>(hits.size())}];
        if (static_cast<int>(hits.size()) != want) { ++countDiffers; outcome = Outcome::CountDiffers; continue; }
        std::vector<LinkLane>& out = links_[static_cast<std::size_t>(li)];
        for (Hit& h : hits) out.push_back(std::move(h.lane));
        ++matched;
        outcome = Outcome::Matched;
    }
}

int LaneMap::stateLanes(engine::NavGraph& nav) const {
    int set = 0;
    for (std::size_t li = 0; li < stated_.size() && li < nav.links.size(); ++li)
        if (stated_[li] > 0) {
            nav.links[li].lanes = stated_[li];
            ++set;
        }
    return set;
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
