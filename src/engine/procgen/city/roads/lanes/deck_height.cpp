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

int DeckHeight::nearestLane(const std::vector<int>& roads, const Vec2& p, double radius) const {
    std::vector<char> want(g_.edges.size(), 0); for (int e : roads) if (e >= 0 && e < static_cast<int>(want.size())) want[static_cast<size_t>(e)] = 1;
    int best = -1; double bd = radius;
    for (size_t li = 0; li < L_.lanes.size(); ++li) {
        const Lane& l = L_.lanes[li]; if (l.isConnector() || l.parent < 0 || !want[static_cast<size_t>(l.parent)]) continue;
        const double d = distanceToLane(static_cast<int>(li), p, radius); if (d < bd) { bd = d; best = static_cast<int>(li); }
    }
    return best;
}

double DeckHeight::layerHeight(const std::vector<int>& roads, const Vec2& p) const {
    const int li = nearestLane(roads, p); if (li >= 0) return deck(li, p);
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
