#include "city_walk_points.h"

#include "../../engine/procgen/city/roads/road_model.h"

#include <algorithm>
#include <cmath>

namespace citysim {

namespace {

Vec2 walkDirection(const engine::LaneGraph& g, const engine::WalkPoint& p) {
    const Real len = g.walks[p.walk.index].length;
    const Vec2 a = g.pointAt(p.walk, std::max(Real(0), p.along - 0.5));
    const Vec2 b = g.pointAt(p.walk, std::min(len, p.along + 0.5));
    const Vec2 d = b - a;
    const Real l = d.length();
    return l > 1e-6 ? d * (1.0 / l) : Vec2(1, 0);
}

bool streetOrWalkway(const engine::Walk& k) { return k.street != engine::kNoIndex || k.modelWalkway != engine::kNoIndex; }

}  // namespace

void WalkPointMap::build(const engine::RoadModel& model, const std::vector<uint8_t>& open, const engine::ReachableParts& parts) {
    nodes.clear();
    ++version_;
    at.clear();
    along.clear();
    part.clear();
    streetPoints = 0;
    if (!model.built) return;
    const engine::LaneGraph& g = model.laneLevel;
    const auto openWalk = [&](uint32_t w) { return (w >= open.size() || open[w]) && w < parts.partOf.size(); };
    // each part's length of street and walkway walks
    std::vector<Real> partLength(parts.sizes.size(), 0);
    for (uint32_t w = 0; w < g.walks.size(); ++w)
        if (openWalk(w) && streetOrWalkway(g.walks[w]) && parts.partOf[w] < partLength.size())
            partLength[parts.partOf[w]] += g.walks[w].length;
    for (uint32_t w = 0; w < g.walks.size(); ++w) {
        const engine::Walk& k = g.walks[w];
        if (!streetOrWalkway(k) || !openWalk(w) || !(k.length > 0)) continue;   // (a corner, crossing, turn-round)
        const uint32_t p = parts.partOf[w];
        if (p >= partLength.size() || partLength[p] < kWalkPointPartLength) continue;
        const int count = std::max(1, static_cast<int>(std::floor(k.length / kWalkPointSpacing)));
        const Real step = k.length / static_cast<Real>(count);
        for (int i = 0; i < count; ++i)
            add(model, engine::WalkPoint{engine::WalkId(w), step * (static_cast<Real>(i) + 0.5)}, static_cast<int>(p));
    }
    streetPoints = nodeCount();
}

int WalkPointMap::add(const engine::RoadModel& model, const engine::WalkPoint& p, int inPart) {
    const engine::LaneGraph& g = model.laneLevel;
    nodes.push_back(g.pointAt(p.walk, p.along));
    ++version_;
    at.push_back(p);
    along.push_back(walkDirection(g, p));
    part.push_back(inPart);
    return nodeCount() - 1;
}

// THE NEAREST POINT (ADR-0513): rings of 64 m cells outward from p's, until no ring further out can hold a nearer one.
// Ties go to the lower point number, as the scan of every point did (it kept the first of equals). Every brain person
// setting out asked this, a scan of the island's every point each time.
int WalkPointMap::nearestNode(Vec2 p) const {
    if (nodes.empty()) return -1;
    constexpr Real kCell = 64.0;
    auto key = [](int64_t cx, int64_t cz) { return (cx << 32) ^ (cz & 0xffffffff); };
    if (cellsVersion_ != version_ || cellsFor_ != nodes.size()) {
        cells_.clear();
        for (int i = 0; i < nodeCount(); ++i) {
            const Vec2 q = nodes[static_cast<std::size_t>(i)];
            cells_[key(static_cast<int64_t>(std::floor(q.x / kCell)), static_cast<int64_t>(std::floor(q.y / kCell)))].push_back(i);
        }
        cellsVersion_ = version_;
        cellsFor_ = nodes.size();
    }
    const int64_t cx = static_cast<int64_t>(std::floor(p.x / kCell)), cz = static_cast<int64_t>(std::floor(p.y / kCell));
    int best = -1;
    Real bestD2 = 0;
    std::size_t seen = 0;
    for (int64_t ring = 0;; ++ring) {
        for (int64_t dx = -ring; dx <= ring; ++dx)
            for (int64_t dz = -ring; dz <= ring; ++dz) {
                if (std::max(std::abs(dx), std::abs(dz)) != ring) continue;   // (this ring's cells only)
                const std::unordered_map<int64_t, std::vector<int>>::const_iterator it = cells_.find(key(cx + dx, cz + dz));
                if (it == cells_.end()) continue;
                seen += it->second.size();
                for (int i : it->second) {
                    const Real d2 = (nodes[static_cast<std::size_t>(i)] - p).lengthSquared();
                    if (best < 0 || d2 < bestD2 || (d2 == bestD2 && i < best)) {
                        best = i;
                        bestD2 = d2;
                    }
                }
            }
        // a point in a ring further out is at least `ring` cells away (less p's place in its own cell)
        const Real reached = static_cast<Real>(ring) * kCell;
        if (best >= 0 && reached * reached > bestD2) break;   // (strict: an equal one further out could be a lower number)
        if (seen >= nodes.size()) break;   // (every point looked at)
    }
    return best;
}

std::vector<int> WalkPointMap::nearestNodes(Vec2 p, int k) const {
    std::vector<int> idx(static_cast<std::size_t>(nodeCount()));
    for (int i = 0; i < nodeCount(); ++i) idx[static_cast<std::size_t>(i)] = i;
    const std::size_t keep = std::min(idx.size(), static_cast<std::size_t>(std::max(0, k)));
    std::partial_sort(idx.begin(), idx.begin() + static_cast<std::ptrdiff_t>(keep), idx.end(), [&](int a, int b) {
        const Real da = (nodes[static_cast<std::size_t>(a)] - p).lengthSquared();
        const Real db = (nodes[static_cast<std::size_t>(b)] - p).lengthSquared();
        return da != db ? da < db : a < b;
    });
    idx.resize(keep);
    return idx;
}

}  // namespace citysim
