#ifndef RAYTRACER_ENGINE_WIND_H
#define RAYTRACER_ENGINE_WIND_H

// THE WIND (ADR-0337; #187, Glenn: "if this weather map has wind, can we use that to also manipulate things like
// players ... the parachute and wing suit ... grass or trees ... the waves in the sea"). ONE wind for everything that
// moves with the air: the weather state sets its speed and gustiness (weather_cycle.h, eased with the rest), its
// direction veers slowly through the day, and everything asks `windAt` -- the clouds' drift, the grass and trees'
// sway, the sea's wave trains, a canopy or a suit in flight. Gusts are patches of stronger air carried downwind, so a
// gust seen crossing a field is the one a glider then meets.
//
// Pure and window-free: DayNightSystem owns the live state and publishes it as the world's `Wind` component each
// fixed step; the renderer gets the same numbers for its shaders (Renderer::wind). Phase 3's weather simulation will
// make it vary by place (the lee of the mountains, the ridges, the open sea); until then it is one airmass.

#include "../rt_math.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine {

struct Wind {
    Vec3 dir{0.94, 0.0, 0.34};   // where the air is going, unit, horizontal
    Real speed = 6.0;            // m/s at 10 m above the ground
    Real gust = 0.35;            // 0 steady .. 1 squally: how much stronger a gust is than the mean
    Real time = 0.0;             // seconds: what carries the gust patches downwind
};

namespace windDetail {
inline Real hash2(int64_t i, int64_t j) {
    uint64_t h = static_cast<uint64_t>(i) * 0x9E3779B97F4A7C15ull ^ static_cast<uint64_t>(j) * 0xC2B2AE3D27D4EB4Full;
    h ^= h >> 31; h *= 0xBF58476D1CE4E5B9ull; h ^= h >> 29;
    return static_cast<Real>(h >> 11) * (1.0 / 9007199254740992.0);
}
inline Real valueNoise(Real x, Real z) {
    const Real fx = std::floor(x), fz = std::floor(z);
    const int64_t i = static_cast<int64_t>(fx), j = static_cast<int64_t>(fz);
    Real u = x - fx, v = z - fz;
    u = u * u * (3 - 2 * u); v = v * v * (3 - 2 * v);
    const Real a = hash2(i, j), b = hash2(i + 1, j), c = hash2(i, j + 1), d = hash2(i + 1, j + 1);
    return (a + (b - a) * u) + ((c + (d - c) * u) - (a + (b - a) * u)) * v;
}
}  // namespace windDetail

// The gust factor at a point: 1 - gust/2 .. 1 + gust, patches ~120 m across travelling with the wind
inline Real windGustFactor(const Wind& w, const Vec3& p) {
    const Real s = 1.0 / 120.0;
    const Vec3 q = p - w.dir * (w.speed * w.time);   // the air that is here now was upwind of it
    const Real n = 0.65 * windDetail::valueNoise(q.x * s, q.z * s) + 0.35 * windDetail::valueNoise(q.x * s * 2.7 + 5.3, q.z * s * 2.7 - 1.9);
    return 1.0 + w.gust * (1.5 * n - 0.5);
}

// The wind at a point, m/s, horizontal: the mean along `dir`, stronger with height (the power law, a = 0.14, held
// within 0.6 .. 1.8 of the 10 m speed) and gusting
inline Vec3 windAt(const Wind& w, const Vec3& p, Real groundY = 0.0) {
    const Real h = std::max(p.y - groundY, 2.0);
    const Real prof = std::clamp(std::pow(h / 10.0, 0.14), 0.6, 1.8);
    return w.dir * (w.speed * prof * windGustFactor(w, p));
}

}  // namespace engine

#endif
