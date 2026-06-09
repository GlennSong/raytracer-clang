#include "day_night_cycle.h"

#include <cmath>

namespace engine {

namespace {

// Hermite smoothstep, clamped — the standard 0→1 ease used throughout.
double smoothstep(double edge0, double edge1, double x) {
    double t = (x - edge0) / (edge1 - edge0);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t * t * (3.0 - 2.0 * t);
}

double mix(double a, double b, double t) { return a + (b - a) * t; }

}  // namespace

void DayNightCycle::advance(double dt) {
    if (paused) return;
    timeOfDay += speed * dt;
    timeOfDay -= std::floor(timeOfDay);  // wrap into [0, 1)
}

DayNightState DayNightCycle::evaluateAt(double timeOfDay, double axisTilt) {
    const double TWO_PI = 6.283185307179586;

    // Sun arc. angle: 0 at sunrise (east, on the horizon), +pi/2 at noon
    // (overhead), +pi at sunset (west). y < 0 is night (sun below horizon).
    double angle = (timeOfDay - 0.25) * TWO_PI;
    Vec3 sunDir = normalize(Vec3(std::cos(angle), std::sin(angle), axisTilt));
    double elev = sunDir.y;  // -1 (deepest night) .. 1 (highest noon)

    // Blend weights. day: night→day. dusk: warm band hugging the horizon, gated
    // off once the sun drops well below it so deep night stays cold.
    double day  = smoothstep(-0.10, 0.20, elev);
    double dusk = (1.0 - smoothstep(0.0, 0.30, std::abs(elev))) *
                  smoothstep(-0.15, 0.02, elev);

    // Palettes (linear RGB).
    const Vec3 dayZenith(0.25, 0.45, 0.85);
    const Vec3 dayHorizon(0.62, 0.76, 0.92);
    const Vec3 dayGround(0.35, 0.30, 0.25);
    const Vec3 nightZenith(0.015, 0.02, 0.06);
    const Vec3 nightHorizon(0.03, 0.04, 0.10);
    const Vec3 nightGround(0.01, 0.01, 0.02);
    const Vec3 duskHorizon(0.95, 0.45, 0.18);
    const Vec3 duskZenith(0.30, 0.22, 0.45);

    Vec3 zenith  = lerp(nightZenith,  dayZenith,  day);
    Vec3 horizon = lerp(nightHorizon, dayHorizon, day);
    Vec3 ground  = lerp(nightGround,  dayGround,  day);
    horizon = lerp(horizon, duskHorizon, dusk);
    zenith  = lerp(zenith,  duskZenith,  dusk * 0.6);

    // Sun color: warm orange grazing the horizon, neutral high in the sky.
    const Vec3 warm(1.0, 0.55, 0.28);
    const Vec3 midday(1.0, 0.97, 0.92);
    Vec3 sunColor = lerp(warm, midday, smoothstep(0.0, 0.45, elev));

    DayNightState s;
    s.sunDirection     = sunDir;
    s.sunColor         = sunColor;
    s.sunIntensity     = static_cast<float>(1.6 * day);
    s.skyDiscIntensity = static_cast<float>(smoothstep(-0.04, 0.10, elev));
    s.zenithColor      = zenith;
    s.horizonColor     = horizon;
    s.groundColor      = ground;
    s.ambient          = static_cast<float>(mix(0.04, 0.34, day));
    return s;
}

}  // namespace engine
