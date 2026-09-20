#include "engine/procgen/city/roads/lanes/polyline_ops.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace engine {
namespace roads::lanes {

void frames(const std::vector<Vec2>& xy, std::vector<Vec2>& tan, std::vector<Vec2>& nrm) {
    size_t n = xy.size(); tan.resize(n); nrm.resize(n);
    for (size_t i = 0; i < n; ++i) { tan[i] = tangentAt(xy, i); nrm[i] = perp(tan[i]); }
}

Vec2 tangentAt(const std::vector<Vec2>& xy, size_t i) {
    if (xy.size() < 2) return {1, 0};
    size_t j = std::min(i + 1, xy.size() - 1), k = i == 0 ? 0 : i - 1; Vec2 d = xy[j] - xy[k];
    return d.length() > 1e-12 ? normalize(d) : Vec2(1, 0);
}

Projection project(const std::vector<Vec2>& xy, const std::vector<double>& s, const Vec2& p) {
    Projection best; best.distance = std::numeric_limits<double>::infinity();
    if (xy.size() == 1) { best.distance = distance(xy[0], p); return best; }
    for (size_t i = 0; i + 1 < xy.size(); ++i) {
        Vec2 a = xy[i], d = xy[i+1] - a; double L2 = d.lengthSquared();
        double t = L2 > 1e-18 ? std::clamp(dot(p - a, d) / L2, 0.0, 1.0) : 0.0;
        double dist = distance(a + d * t, p);
        if (dist < best.distance) { best.distance = dist; best.segment = i; best.t = t; best.station = s[i] + (s[i+1] - s[i]) * t; }
    }
    return best;
}

Vec2 pointAt(const std::vector<Vec2>& xy, const std::vector<double>& s, double station) {
    if (station <= s.front()) return xy.front();
    if (station >= s.back()) return xy.back();
    size_t k = std::upper_bound(s.begin(), s.end(), station) - s.begin();
    double f = (station - s[k-1]) / std::max(s[k] - s[k-1], 1e-12); return lerp(xy[k-1], xy[k], f);
}

Vec2 tangentAtStation(const std::vector<Vec2>& xy, const std::vector<double>& s, double station) {
    Vec2 a = pointAt(xy, s, std::max(station - 0.5, s.front())), b = pointAt(xy, s, std::min(station + 0.5, s.back()));
    Vec2 d = b - a; return d.length() > 1e-12 ? normalize(d) : tangentAt(xy, 0);
}

double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x) {
    if (xs.empty()) return 0.0;
    if (x <= xs.front()) return ys.front();
    if (x >= xs.back()) return ys.back();
    size_t k = std::upper_bound(xs.begin(), xs.end(), x) - xs.begin();
    double f = (x - xs[k-1]) / std::max(xs[k] - xs[k-1], 1e-12); return ys[k-1] + (ys[k] - ys[k-1]) * f;
}

bool pointInRing(const std::vector<Vec2>& ring, const Vec2& p) {
    bool inside = false; size_t n = ring.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2& a = ring[i]; const Vec2& b = ring[j];
        if (((a.y > p.y) != (b.y > p.y)) && (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) inside = !inside;
    }
    return inside;
}

std::vector<Vec2> crossings(const std::vector<Vec2>& a, const std::vector<Vec2>& b) {
    std::vector<Vec2> out;
    auto box = [](const std::vector<Vec2>& p, double& x0, double& x1, double& y0, double& y1) {
        x0 = y0 = 1e300; x1 = y1 = -1e300; for (const Vec2& q : p) { x0 = std::min(x0, q.x); x1 = std::max(x1, q.x); y0 = std::min(y0, q.y); y1 = std::max(y1, q.y); }
    };
    double ax0, ax1, ay0, ay1, bx0, bx1, by0, by1; box(a, ax0, ax1, ay0, ay1); box(b, bx0, bx1, by0, by1);
    if (ax1 < bx0 || bx1 < ax0 || ay1 < by0 || by1 < ay0) return out;
    for (size_t i = 0; i + 1 < a.size(); ++i) {
        Vec2 p = a[i], r = a[i+1] - a[i];
        double sx0 = std::min(p.x, p.x + r.x), sx1 = std::max(p.x, p.x + r.x), sy0 = std::min(p.y, p.y + r.y), sy1 = std::max(p.y, p.y + r.y);
        if (sx1 < bx0 || sx0 > bx1 || sy1 < by0 || sy0 > by1) continue;
        for (size_t j = 0; j + 1 < b.size(); ++j) {
            Vec2 q = b[j], u = b[j+1] - b[j]; double den = cross(r, u);
            if (std::fabs(den) < 1e-12) continue;
            double t = cross(q - p, u) / den, w = cross(q - p, r) / den;
            if (t >= 0 && t <= 1 && w >= 0 && w <= 1) out.push_back(p + r * t);
        }
    }
    return out;
}

SegmentGrid::SegmentGrid(const std::vector<Vec2>& xy, double cell) : pts_(xy), cell_(cell) {
    if (xy.size() < 2) return;
    s_.assign(xy.size(), 0.0); for (size_t i = 1; i < xy.size(); ++i) s_[i] = s_[i-1] + distance(xy[i-1], xy[i]);
    double x1 = -1e300, y1 = -1e300; x0_ = y0_ = 1e300;
    for (const Vec2& p : xy) { x0_ = std::min(x0_, p.x); y0_ = std::min(y0_, p.y); x1 = std::max(x1, p.x); y1 = std::max(y1, p.y); }
    nx_ = static_cast<int>((x1 - x0_) / cell_) + 1; ny_ = static_cast<int>((y1 - y0_) / cell_) + 1; cells_.assign(static_cast<size_t>(nx_) * ny_, {});
    for (size_t i = 0; i + 1 < xy.size(); ++i) {
        int i0 = static_cast<int>((std::min(xy[i].x, xy[i+1].x) - x0_) / cell_), i1 = static_cast<int>((std::max(xy[i].x, xy[i+1].x) - x0_) / cell_);
        int j0 = static_cast<int>((std::min(xy[i].y, xy[i+1].y) - y0_) / cell_), j1 = static_cast<int>((std::max(xy[i].y, xy[i+1].y) - y0_) / cell_);
        for (int j = j0; j <= j1; ++j) for (int ii = i0; ii <= i1; ++ii) cells_[cellIndex(ii, j)].push_back(static_cast<int>(i));
    }
}

double SegmentGrid::distanceWithin(const Vec2& p, double radius, double* station, double* side) const {
    double best = std::numeric_limits<double>::infinity();
    if (pts_.size() < 2) return best;
    int i0 = static_cast<int>(std::floor((p.x - radius - x0_) / cell_)), i1 = static_cast<int>(std::floor((p.x + radius - x0_) / cell_));
    int j0 = static_cast<int>(std::floor((p.y - radius - y0_) / cell_)), j1 = static_cast<int>(std::floor((p.y + radius - y0_) / cell_));
    if (i1 < 0 || j1 < 0 || i0 >= nx_ || j0 >= ny_) return best;
    i0 = std::max(i0, 0); j0 = std::max(j0, 0); i1 = std::min(i1, nx_ - 1); j1 = std::min(j1, ny_ - 1);
    for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) {
        for (int seg : cells_[cellIndex(i, j)]) {
            Vec2 a = pts_[seg], d = pts_[seg+1] - a; double L2 = d.lengthSquared();
            double t = L2 > 1e-18 ? std::clamp(dot(p - a, d) / L2, 0.0, 1.0) : 0.0; double dist = distance(a + d * t, p);
            if (dist < best) { best = dist; if (station) *station = s_[seg] + (s_[seg+1] - s_[seg]) * t; if (side) *side = cross(d, p - a); }
        }
    }
    return best <= radius ? best : std::numeric_limits<double>::infinity();
}

}  // namespace roads::lanes
}  // namespace engine
