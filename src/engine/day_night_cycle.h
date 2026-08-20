#ifndef RAYTRACER_ENGINE_DAY_NIGHT_CYCLE_H
#define RAYTRACER_ENGINE_DAY_NIGHT_CYCLE_H

#include "../rt_math.h"

namespace engine {

// Lighting + sky colors produced by the day/night model at a moment in time.
// Consumed by the renderer's procedural sky (ADR-0016) and the scene's
// directional sun light, so the sky and the shading that lights the world stay
// locked together as the cycle advances.
// The curve's NOON anchors — what sunIntensity/ambient evaluate to at full
// day. DayNightSystem NORMALIZES against these so the cycle acts as a
// day-shape MULTIPLIER on the level's AUTHORED sun/ambient instead of
// replacing them (metro authors sun 8.0 + ambient 0.6; the raw curve values
// 5.0 + 0.34 silently overrode both — "certain values are being ignored").
constexpr float kCycleNoonSunIntensity = 5.0f;
constexpr float kCycleNoonAmbient = 0.34f;

struct DayNightState {
    Vec3  sunDirection;     // normalized, toward the sun (world space)
    Vec3  sunColor;         // directional-light + sky-disc tint (linear)
    float sunIntensity;     // directional-light intensity (≈0 at night)
    float skyDiscIntensity; // procedural sun disc brightness (fades through dusk)
    Vec3  zenithColor;      // sky dome at the top
    Vec3  horizonColor;     // sky at the horizon
    Vec3  groundColor;      // below-horizon tint
    float ambient;          // ambient multiplier (cool + low at night)

    // The ACTIVE light — what the renderer should drive the sky and shading
    // with. Sun by day; the MOON by night (WS2, "night is way too dark"): a
    // moonlit sky is a blue-shifted day sky a few hundred times dimmer, so
    // driving the same scattering pipeline with a dim cool moon buys sky,
    // IBL ambient, and aerial haze at night for free. The handoff is a hard
    // switch at the intensity crossover — both lights are near-black there,
    // so nothing visibly pops (the sky bake re-keys on direction+intensity).
    Vec3  lightDirection;   // normalized, toward the active light
    Vec3  lightColor;
    float lightIntensity;
    // The sun's TRUTH regardless of which light is active: dusk predicates
    // (vehicle lamps, street lights) must key on solar elevation, or the
    // moon rising would read as daylight and switch every lamp off.
    float solarElevation;
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
    // 0 = new moon (nights stay truly dark), 1 = full moon (the default —
    // moonlit-realistic nights). Scales the moon's intensity only; the disc
    // rendering for visible phases is deferred.
    double moonPhase = 1.0;
    bool   paused    = false;

    // Advance time by dt seconds (no-op when paused), wrapping into [0, 1).
    void advance(double dt);

    DayNightState evaluate() const {
        return evaluateAt(timeOfDay, axisTilt, moonPhase);
    }

    // Stateless evaluation, exposed for testing and reuse.
    static DayNightState evaluateAt(double timeOfDay, double axisTilt,
                                    double moonPhase = 1.0);
};

}  // namespace engine

#endif
