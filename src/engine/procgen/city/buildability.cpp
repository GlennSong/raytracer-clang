#include "buildability.h"

#include <algorithm>
#include <cmath>

namespace engine {
namespace {

double distToRivers(const BuildabilityConfig& cfg, double x, double z) {
    double best = 1e30;
    Vec2 p(x, z);
    for (const std::vector<Vec2>& r : cfg.rivers) {
        for (std::size_t i = 0; i + 1 < r.size(); ++i) {
            Vec2 a = r[i], ab = r[i + 1] - a;
            double L2 = ab.lengthSquared();
            double t = L2 < 1e-9 ? 0.0 : std::max(0.0, std::min(1.0, dot(p - a, ab) / L2));
            best = std::min(best, (p - (a + ab * t)).length());
        }
    }
    return best;
}

bool waterAt(const BuildabilityConfig& cfg, const HeightSampler& h, double x, double z) {
    if (h(x, z) < cfg.seaLevel) return true;
    if (!cfg.rivers.empty() && distToRivers(cfg, x, z) < cfg.riverWidth) return true;
    return false;
}

// Nearest water within beachReach (sampled rings); returns a big number if none.
double nearWater(const BuildabilityConfig& cfg, const HeightSampler& h, double x, double z) {
    const double step = std::max(6.0, cfg.beachReach / 4.0);
    for (double r = step; r <= cfg.beachReach; r += step)
        for (int k = 0; k < 8; ++k) {
            double t = 2.0 * 3.14159265358979323846 * k / 8.0;
            if (waterAt(cfg, h, x + std::cos(t) * r, z + std::sin(t) * r)) return r;
        }
    return 1e9;
}

double slopeAt(const HeightSampler& h, double x, double z) {
    const double e = 4.0;
    double gx = (h(x + e, z) - h(x - e, z)) / (2 * e);
    double gz = (h(x, z + e) - h(x, z - e)) / (2 * e);
    return std::sqrt(gx * gx + gz * gz);
}

}  // namespace

LandClass classifyLand(const BuildabilityConfig& cfg, const HeightSampler& height,
                       double x, double z) {
    if (waterAt(cfg, height, x, z)) return LandClass::Water;
    double h = height(x, z);
    // A beach is low, gentle land right at the water's edge — reserved (not built on).
    if (cfg.seaLevel > -1e29 && h < cfg.seaLevel + cfg.beachRise &&
        nearWater(cfg, height, x, z) < cfg.beachReach)
        return LandClass::Beach;
    if (slopeAt(height, x, z) > cfg.maxSlope) return LandClass::Steep;
    return LandClass::Buildable;
}

bool crossesWater(const BuildabilityConfig& cfg, const HeightSampler& height,
                  const Vec2& a, const Vec2& b, double step) {
    int n = std::max(2, static_cast<int>((b - a).length() / std::max(1.0, step)));
    for (int i = 0; i <= n; ++i) {
        double t = static_cast<double>(i) / n;
        Vec2 p = a + (b - a) * t;
        if (waterAt(cfg, height, p.x, p.y)) return true;
    }
    return false;
}

}  // namespace engine
