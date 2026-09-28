#include "engine/procgen/city/roads/lanes/deck_height.h"
#include "engine/procgen/city/roads/lanes/vertical_profile.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace engine {
namespace roads::lanes {

DeckHeight::DeckHeight(const RoadLabGraph& g, const LaneSet& L) : g_(g), L_(L) {
    partners_.assign(L.lanes.size(), {}); roadPartners_.assign(g.edges.size(), {});
    grids_.reserve(L.lanes.size());
    for (const Lane& l : L.lanes) grids_.push_back(std::make_unique<SegmentGrid>(l.xy, 8.0));
    // the near-lane index: each lane's centreline box, grown by its half-width and 15 m of caps and fillets
    // (the footprint's reach past the centreline) and by the index's search reach
    std::vector<Box2> box(L.lanes.size());
    double x0 = 1e300, y0 = 1e300, x1 = -1e300, y1 = -1e300;
    for (size_t li = 0; li < L.lanes.size(); ++li) {
        Box2 b{1e300, 1e300, -1e300, -1e300};
        for (const Vec2& q : L.lanes[li].xy) { b.minX = std::min(b.minX, q.x); b.minY = std::min(b.minY, q.y); b.maxX = std::max(b.maxX, q.x); b.maxY = std::max(b.maxY, q.y); }
        const double grow = L.lanes[li].w / 2 + 15.0 + kIndexReach;
        b.minX -= grow; b.minY -= grow; b.maxX += grow; b.maxY += grow;
        box[li] = b;
        if (L.lanes[li].xy.empty()) continue;
        x0 = std::min(x0, b.minX); y0 = std::min(y0, b.minY); x1 = std::max(x1, b.maxX); y1 = std::max(y1, b.maxY);
    }
    if (x0 < x1) {
        ix0_ = x0; iy0_ = y0;
        inx_ = std::max(1, static_cast<int>(std::ceil((x1 - x0) / kIndexCell)));
        iny_ = std::max(1, static_cast<int>(std::ceil((y1 - y0) / kIndexCell)));
        index_.assign(static_cast<size_t>(inx_) * static_cast<size_t>(iny_), {});
        for (size_t li = 0; li < L.lanes.size(); ++li) {
            if (L.lanes[li].xy.empty()) continue;
            const int i0 = std::clamp(static_cast<int>((box[li].minX - ix0_) / kIndexCell), 0, inx_ - 1), i1 = std::clamp(static_cast<int>((box[li].maxX - ix0_) / kIndexCell), 0, inx_ - 1);
            const int j0 = std::clamp(static_cast<int>((box[li].minY - iy0_) / kIndexCell), 0, iny_ - 1), j1 = std::clamp(static_cast<int>((box[li].maxY - iy0_) / kIndexCell), 0, iny_ - 1);
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) index_[static_cast<size_t>(j) * static_cast<size_t>(inx_) + static_cast<size_t>(i)].push_back(static_cast<int>(li));
        }
    }
}

const std::vector<int>* DeckHeight::candidates(const Vec2& p) const {
    static const std::vector<int> none;
    if (index_.empty()) return nullptr;
    const int i = static_cast<int>(std::floor((p.x - ix0_) / kIndexCell)), j = static_cast<int>(std::floor((p.y - iy0_) / kIndexCell));
    if (i < 0 || j < 0 || i >= inx_ || j >= iny_) return &none;   // beyond every lane's reach
    return &index_[static_cast<size_t>(j) * static_cast<size_t>(inx_) + static_cast<size_t>(i)];
}

void DeckHeight::setPartners(std::vector<std::vector<int>> partners) {
    partners_ = std::move(partners);
    for (auto& v : roadPartners_) v.clear();
    for (size_t li = 0; li < L_.lanes.size(); ++li) {
        int e = L_.lanes[li].parent; if (e < 0) continue;
        for (int q : partners_[li]) roadPartners_[static_cast<size_t>(e)].push_back(q);
    }
    for (auto& v : roadPartners_) {
        std::set<int> u(v.begin(), v.end()); v.assign(u.begin(), u.end());
        std::sort(v.begin(), v.end(), [&](int a, int b) { return L_.rank(a, g_) > L_.rank(b, g_); });
    }
}

double DeckHeight::own(int lane, const Vec2& p) const { return laneHeightAt(L_.lanes[static_cast<size_t>(lane)], g_, p); }
double DeckHeight::ownRoad(int edge, const Vec2& p) const { return projectZ(g_.edges[static_cast<size_t>(edge)], p); }

void DeckHeight::setFootprints(const std::vector<std::vector<Ring>>& outers, const std::vector<std::vector<Ring>>& holes) {
    boundaries_.assign(L_.lanes.size(), {});
    auto closed = [](const Ring& r) { std::vector<Vec2> p(r.begin(), r.end()); if (!r.empty()) p.push_back(r.front()); return p; };
    for (size_t li = 0; li < L_.lanes.size(); ++li) {
        Boundary& b = boundaries_[li];
        for (const Ring& r : outers[li]) { b.outers.emplace_back(closed(r), 8.0); b.outerRings.push_back(r); Polygon2 pg; pg.outer = r; b.outerBoxes.push_back(bounds(pg)); }
        for (const Ring& r : holes[li]) { b.holes.emplace_back(closed(r), 8.0); b.holeRings.push_back(r); }
    }
}

std::vector<char> DeckHeight::edgeFlags(const std::vector<int>& roads) const {
    std::vector<char> want(g_.edges.size(), 0); for (int e : roads) if (e >= 0 && e < static_cast<int>(want.size())) want[static_cast<size_t>(e)] = 1;
    return want;
}

int DeckHeight::nearestLane(const std::vector<int>& roads, const Vec2& p, double radius) const {
    return nearestLane(edgeFlags(roads), p, radius);
}

int DeckHeight::nearestLane(const std::vector<char>& want, const Vec2& p, double radius) const {
    int best = -1; double bd = radius;
    auto test = [&](size_t li) {
        const Lane& l = L_.lanes[li]; if (l.isConnector() || l.parent < 0 || !want[static_cast<size_t>(l.parent)]) return;
        const double d = distanceToLane(static_cast<int>(li), p, radius); if (d < bd) { bd = d; best = static_cast<int>(li); }
    };
    const std::vector<int>* cand = radius <= kIndexReach ? candidates(p) : nullptr;
    if (cand) for (int li : *cand) test(static_cast<size_t>(li));   // ascending lane order: the full scan's answer
    else for (size_t li = 0; li < L_.lanes.size(); ++li) test(li);
    return best;
}

double DeckHeight::layerHeight(const std::vector<int>& roads, const Vec2& p) const {
    return layerHeight(edgeFlags(roads), roads, p);
}

double DeckHeight::layerHeight(const std::vector<char>& want, const std::vector<int>& roads, const Vec2& p) const {
    const int li = nearestLane(want, p); if (li >= 0) return deck(li, p);
    int best = roads.empty() ? -1 : roads[0]; double bd = 1e300;
    for (int e : roads) { const EdgeSpec& E = g_.edges[static_cast<size_t>(e)]; const double d = project(E.xy, E.s, p).distance; if (d < bd) { bd = d; best = e; } }
    return best >= 0 ? ownRoad(best, p) : 0.0;
}

double DeckHeight::distanceToLane(int lane, const Vec2& p, double radius) const {
    const Lane& l = L_.lanes[static_cast<size_t>(lane)];
    if (boundaries_.empty() || boundaries_[static_cast<size_t>(lane)].outers.empty()) {
        double d = grids_[static_cast<size_t>(lane)]->distanceWithin(p, radius + l.w / 2 + 0.02);
        return std::isfinite(d) ? std::max(0.0, d - l.w / 2 - 0.01) : std::numeric_limits<double>::infinity();
    }
    // nearest boundary within the radius; inside (outer ring minus holes) means distance 0
    const Boundary& b = boundaries_[static_cast<size_t>(lane)]; double best = std::numeric_limits<double>::infinity();
    for (const SegmentGrid& g : b.outers) best = std::min(best, g.distanceWithin(p, radius));
    if (!std::isfinite(best)) return best;
    bool inside = false;
    for (size_t k = 0; k < b.outerRings.size() && !inside; ++k) {
        const Box2& bx = b.outerBoxes[k]; if (p.x < bx.minX || p.x > bx.maxX || p.y < bx.minY || p.y > bx.maxY) continue;
        inside = pointInRing(b.outerRings[k], p);
    }
    if (!inside) return best;
    double inHole = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k < b.holeRings.size(); ++k) if (pointInRing(b.holeRings[k], p)) inHole = std::min(inHole, b.holes[k].distanceWithin(p, radius));
    return std::isfinite(inHole) ? inHole : 0.0;
}

double DeckHeight::blendTo(double z, const std::vector<int>& partners, const Vec2& p) const {
    static thread_local int depth = 0; struct Guard { int& d; explicit Guard(int& x) : d(x) { ++d; } ~Guard() { --d; } } guard(depth);
    if (depth > 12) throw std::runtime_error("lanelab: partner chain too deep — the partner relation is not a strict order");
    // The NEAREST partner within blendLen pulls with a smoothstep of distance and takes over exactly at
    // the seam (d = 0), using the partner's DECK height so chains agree. Nearest only: a weighted mix of
    // several partners is smoother inside a lane but is no longer exact where two surfaces meet.
    double Lb = g_.rules.blendLen; int best = -1; double bd = std::numeric_limits<double>::infinity();
    for (int q : partners) { double d = distanceToLane(q, p, Lb); if (d < bd) { bd = d; best = q; } }
    if (best < 0 || !(bd < Lb)) return z;
    double w = bd / Lb; w = w * w * (3 - 2 * w);
    // A partner pulls only where the two are at the same level. Partnership is per ROAD pair (a ramp and
    // the street it lands on), but the blend radius is 2D, so a ramp 20 m from that street and 4 m above
    // it was dragged down to it — a 3.8 m dip in the deck with the pier, placed from the profile, standing
    // 2.8 m proud of the surface (Glenn's cube at the end of the on-ramp). Fade the pull to nothing between
    // same_level_dz and bridge_h of vertical separation.
    // At the seam itself the two surfaces must meet exactly whatever their profiles say (a slip lane two
    // metres off its cross street on a hill still lands on it), so the fade only applies away from the
    // partner: full pull within kSeam of it, the level fade beyond.
    const double zp = deck(best, p), gap = std::fabs(zp - z), lo = g_.rules.sameLevelDz, hi = std::max(lo + 0.5, g_.rules.bridgeH);
    double f = gap <= lo ? 1.0 : gap >= hi ? 0.0 : 1.0 - (gap - lo) / (hi - lo); f = f * f * (3 - 2 * f);
    constexpr double kSeam = 3.0; const double nearSeam = bd >= kSeam ? 0.0 : 1.0 - bd / kSeam;
    const double pull = (1 - w) * std::max(f, nearSeam);
    return pull * zp + (1 - pull) * z;
}

double DeckHeight::deck(int lane, const Vec2& p) const { return blendTo(own(lane, p), partners_[static_cast<size_t>(lane)], p); }
double DeckHeight::deckRoad(int edge, const Vec2& p) const { return blendTo(ownRoad(edge, p), roadPartners_[static_cast<size_t>(edge)], p); }

}  // namespace roads::lanes
}  // namespace engine
