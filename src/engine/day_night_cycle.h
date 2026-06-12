#ifndef RAYTRACER_ENGINE_DAY_NIGHT_CYCLE_H
#define RAYTRACER_ENGINE_DAY_NIGHT_CYCLE_H

#include "../rt_math.h"

namespace engine {

// Lighting + sky colors produced by the day/night model at a moment in time.
// Consumed by the renderer's procedural sky (ADR-0016) and the scene's
// directional sun light, so the sky and the shading that lights the world stay
// locked together as the cycle advances.
struct DayNightState {
    Vec3  sunDirection;     // normalized, toward the sun (world space)
    Vec3  sunColor;         // directional-light + sky-disc tint (linear)
    float sunIntensity;     // directional-light intensity (≈0 at night)
    float skyDiscIntensity; // procedural sun disc brightness (fades through dusk)
    Vec3  zenithColor;      // sky dome at the top
    Vec3  horizonColor;     // sky at the horizon
    Vec3  groundColor;      // below-horizon tint
    float ambient;          // ambient multiplier (cool + low at night)
};

// Maps a normalized time-of-day to a sun arc and a graded sky/light palette.
// Time runs in [0, 1): 0.0 = midnight, 0.25 = sunrise, 0.5 = noon, 0.75 =
// sunset. Pure and window-free — the visual curve is unit-tested headless; the
// renderer-side shader only interpolates the colors this produces.
class DayNightCycle {
public:
    double timeOfDay = 0.35;   // start mid-morning
    double speed     = 0.02;   // fraction of a full day per second (~50s/day)
    double axisTilt  = 0.35;   // sun-arc tilt out of the east-up plane
    bool   paused    = false;

    // Advance time by dt seconds (no-op when paused), wrapping into [0, 1).
    void advance(double dt);

    DayNightState evaluate() const { return evaluateAt(timeOfDay, axisTilt); }

    // Stateless evaluation, exposed for testing and reuse.
    static DayNightState evaluateAt(double timeOfDay, double axisTilt);
};

}  // namespace engine

#endif
