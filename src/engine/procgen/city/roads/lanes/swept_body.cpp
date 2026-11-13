#include "engine/procgen/city/roads/lanes/swept_body.h"

#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace roads::lanes {

namespace {

// Two points of a line closer than this are one point (metres).
constexpr double kSameLinePoint = 1e-9;

}  // namespace

Vec2 BodyLine::directionAt(double along) const {
    if (xy.size() < 2) return Vec2(1, 0);
    return tangentAtStation(xy, s, std::min(std::max(along, s.front()), s.back()));
}

Vec2 BodyLine::pointAt(double along) const {
    if (xy.empty()) return Vec2();
    if (along < s.front()) return xy.front() + directionAt(s.front()) * (along - s.front());
    if (along > s.back()) return xy.back() + directionAt(s.back()) * (along - s.back());
    return roads::lanes::pointAt(xy, s, along);
}

BodyLine makeBodyLine(const std::vector<Vec2>& before, const std::vector<Vec2>& path, const std::vector<Vec2>& after,
                      double reach) {
    BodyLine line;
    if (path.empty()) return line;
    const std::vector<double> own = stations(path);
    line.length = own.back();
    // back from the path's start through `before`, last point first
    std::vector<Vec2> pre;
    std::vector<double> preS;
    {
        Vec2 q = path.front();
        double d = 0;
        for (std::size_t k = before.size(); k-- > 0 && d < reach;) {
            const double step = distance(before[k], q);
            if (step <= kSameLinePoint) continue;
            d += step;
            q = before[k];
            pre.push_back(q);
            preS.push_back(-d);
        }
    }
    line.xy.assign(pre.rbegin(), pre.rend());
    line.s.assign(preS.rbegin(), preS.rend());
    line.xy.insert(line.xy.end(), path.begin(), path.end());
    line.s.insert(line.s.end(), own.begin(), own.end());
    // on from its end through `after`
    {
        Vec2 q = path.back();
        double d = line.length;
        for (std::size_t k = 0; k < after.size() && d < line.length + reach; ++k) {
            const double step = distance(after[k], q);
            if (step <= kSameLinePoint) continue;
            d += step;
            q = after[k];
            line.xy.push_back(q);
            line.s.push_back(d);
        }
    }
    return line;
}

std::vector<Vec2> pathPoints(const LaneGraph& g, PathId path) {
    if (!path.valid() || path.index >= g.paths.size()) return {};
    const Range& c = g.paths[path.index].curve;
    if (c.end() > g.points.size()) return {};
    return std::vector<Vec2>(g.points.begin() + c.first, g.points.begin() + c.end());
}

PlanRectangle bodyAt(const BodyLine& line, double front, double length, double width) {
    const Vec2 f = line.pointAt(front), r = line.pointAt(front - length);
    // (the way the front travels is read only where the bumpers are one place: it is the dearer read)
    return bodyOnPlan(f, r, distance(f, r) > kShortestBodySpan ? Vec2() : line.directionAt(front), length, width);
}

void sweptPoints(const BodyLine& line, double along, double length, std::vector<Vec2>& out) {
    const std::size_t first = out.size();
    const auto keep = [&](const Vec2& p) {
        for (std::size_t k = first; k < out.size(); ++k)
            if (distance(out[k], p) < kSweptSamePoint) return;
        out.push_back(p);
    };
    if (!(length > 0)) {
        keep(line.pointAt(along));
        return;
    }
    const int fronts = std::max(1, static_cast<int>(std::ceil(length / kSweepFrontStep)));
    for (int k = 0; k <= fronts; ++k) {
        const double share = static_cast<double>(k) / fronts;   // of its length, from `along` back to its rear bumper
        keep(bodyAt(line, along + length * share, length, 0).alongMiddle(1 - share));
    }
}

}  // namespace roads::lanes
}  // namespace engine
