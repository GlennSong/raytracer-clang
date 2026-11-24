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
    at.push_back(p);
    along.push_back(walkDirection(g, p));
    part.push_back(inPart);
    return nodeCount() - 1;
}

int WalkPointMap::nearestNode(Vec2 p) const {
    int best = -1;
    Real bestD2 = 0;
    for (int i = 0; i < nodeCount(); ++i) {
        const Real d2 = (nodes[static_cast<std::size_t>(i)] - p).lengthSquared();
        if (best < 0 || d2 < bestD2) {
            best = i;
            bestD2 = d2;
        }
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
