#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_BUILDABILITY_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_BUILDABILITY_H

#include "polygon.h"        // Vec2
#include <functional>
#include <vector>

namespace engine {

// Where can a city actually go? A buildability field classifies the ground so
// generation can respect it instead of marching over water and up mountains
// (ADR: terrain-aware layout). It reads a terrain height sampler and a little
// water/slope config, and answers per point:
//   Water     — below sea level, or inside a river channel.
//   Beach     — low, gentle land right at the water's edge (a reserved fringe).
//   Steep     — land too steep to build on (mountain flanks, cliffs).
//   Buildable — everything else: the land the city may occupy.
using HeightSampler = std::function<double(double, double)>;

struct BuildabilityConfig {
    double seaLevel   = -1e30;   // terrain below this is ocean; -inf disables water
    double maxSlope   = 0.32;    // rise/run above which land is unbuildable (~18°)
    double beachRise  = 2.5;     // land within this height of the sea, near water, is beach
    double beachReach = 70.0;    // how close to water (m) counts as "at the water's edge"
    std::vector<std::vector<Vec2>> rivers;   // river centrelines, treated as water channels
    double riverWidth = 20.0;    // half-width of the river channel (m)
};

enum class LandClass { Water, Beach, Steep, Buildable };

LandClass classifyLand(const BuildabilityConfig& cfg, const HeightSampler& height,
                       double x, double z);

inline bool isBuildable(const BuildabilityConfig& cfg, const HeightSampler& height,
                        double x, double z) {
    return classifyLand(cfg, height, x, z) == LandClass::Buildable;
}

// True if any point along the segment a->b sits over water — i.e. the road would
// need a bridge. Sampled at ~stepped intervals.
bool crossesWater(const BuildabilityConfig& cfg, const HeightSampler& height,
                  const Vec2& a, const Vec2& b, double step = 8.0);

}  // namespace engine

#endif
