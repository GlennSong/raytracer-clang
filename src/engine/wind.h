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
#include "procgen/noise.h"   // valueNoise2: the gust patches

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine {

struct Wind {
    Vec3 dir{0.94, 0.0, 0.34};   // where the air is going, unit, horizontal
    Real speed = 6.0;            // m/s at 10 m above the ground
    Real gust = 0.35;            // 0 steady .. 1 squally: how much stronger a gust is than the mean
    Vec3 drift{0.0, 0.0, 0.0};   // m: how far the air has gone, the wind summed step by step -- what carries the gust
                                 // patches downwind (summed, not dir * speed * time: a veering wind turns the patches'
                                 // path, it does not swing the whole field round the origin)
};


// The gust factor at a point: 1 - gust/2 .. 1 + gust, patches ~120 m across travelling with the wind. The grass and
// trees bow to the same patches (mesh.vert, windGustFactor in hash.glsl: keep the two alike).
inline Real windGustFactor(const Wind& w, const Vec3& p) {
    const Real s = 1.0 / 120.0;
    const Vec3 q = p - w.drift;   // the air that is here now was upwind of it
    const Real n = 0.65 * valueNoise2(q.x * s, q.z * s, 0x57u) + 0.35 * valueNoise2(q.x * s * 2.7 + 5.3, q.z * s * 2.7 - 1.9, 0x57u);
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
