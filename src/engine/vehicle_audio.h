#ifndef RAYTRACER_ENGINE_VEHICLE_AUDIO_H
#define RAYTRACER_ENGINE_VEHICLE_AUDIO_H

#include "../rt_math.h"

#include <algorithm>
#include <cmath>

namespace engine {

// Engine-sound DECISION CORE for every vehicle — the player's car and the
// city's traffic — the same split that keeps lamps honest (vehicle_lamps.h):
// one pure predicate here, thin adapters at each caller. Both sides then rev
// alike, because they ARE alike; the sound is a function of what the car is
// doing, not of which system happens to own it.
//
// Pure: no ECS, no audio engine, no clock. Callers pass what the car is doing
// and get back how its one looping engine sample should be played — the
// playback-rate multiplier on sfx::engine (baked at sfx::kEngineRefHz) and a
// voice gain. Spatialization, distance falloff and ramping stay with the
// caller's voice.

struct EngineSound {
    float pitch = 1.0f;   // playback-rate multiplier on the engine loop
    float gain = 0.0f;    // voice volume [0,1]; 0 = silent (engine off)
};

// GEARS are what make an engine sound like an engine: revs climb through a
// ratio, drop as it shifts, climb again. Without this a car is a siren whose
// pitch tracks the speedometer. Tops are the speeds (m/s) at which each gear
// hands over — roughly 40/72/108/151/216 km/h.
constexpr Real kEngineGearTops[] = {11.0, 20.0, 30.0, 42.0, 60.0};
constexpr int kEngineGearCount =
    static_cast<int>(sizeof(kEngineGearTops) / sizeof(kEngineGearTops[0]));

// The rev band, as a fraction of redline: what a gear starts and ends at, and
// where a warm idle sits.
constexpr Real kEngineIdleRev = 0.16;
constexpr Real kEngineGearLowRev = 0.32;
constexpr Real kEngineRedlineRev = 1.0;

// Pitch multipliers at idle and redline. 0.55 x 56 Hz is a ~925 rpm idle;
// 2.35 x is a ~3950 rpm redline — a modest family-car band, and a shift range
// a single sample survives without artefacts.
constexpr Real kEnginePitchIdle = 0.55;
constexpr Real kEnginePitchMax = 2.35;

// Below this the car is stopped or creeping, and revs blend back to idle so
// pulling away eases in instead of jumping to the bottom of first gear.
constexpr Real kEngineCreepSpeed = 2.0;

// Revs as a fraction of redline, from road speed alone.
inline Real engineRevFraction(Real speedMps) {
    const Real v = std::fabs(speedMps);
    int gear = kEngineGearCount - 1;
    for (int g = 0; g < kEngineGearCount; ++g) {
        if (v < kEngineGearTops[g]) { gear = g; break; }
    }
    const Real low = gear == 0 ? Real(0) : kEngineGearTops[gear - 1];
    const Real span = std::max(Real(0.1), kEngineGearTops[gear] - low);
    const Real through = std::min(Real(1), (v - low) / span);
    const Real geared =
        kEngineGearLowRev + (kEngineRedlineRev - kEngineGearLowRev) * through;
    if (v >= kEngineCreepSpeed) return geared;
    // Creep: idle -> the bottom of first gear.
    const Real t = std::max(Real(0), v) / kEngineCreepSpeed;
    return kEngineIdleRev + (geared - kEngineIdleRev) * t;
}

// `running`: the engine is on — a car with someone (or some brain) at the
// wheel. A parked, driverless car is silent, which is what makes a street of
// stopped traffic sound different from an empty one.
// `throttle01` is the pedal, magnitude only (reverse revs like forward).
// A running engine never holds a steady note: revs hunt, load flutters, the
// idle lopes. Without this even a lumpy sample reads as a hum, because it is
// the SAME lump every loop (device: "it just sounds like a constant hum").
// Two incommensurate rates so the pattern never audibly repeats; deeper at
// idle (a lumpy tickover) than at revs, where inertia smooths it. `phase`
// separates cars so a street doesn't wobble in unison.
inline float engineWobble(Real seconds, Real phase, Real revFraction) {
    const Real depth =
        Real(0.035) - Real(0.022) * std::min(Real(1), std::max(Real(0), revFraction));
    const Real tau = Real(6.283185307179586);
    const Real a = std::sin((seconds * Real(7.3) + phase) * tau);
    const Real b = std::sin((seconds * Real(2.7) + phase * Real(1.7)) * tau);
    return static_cast<float>(Real(1) + depth * (Real(0.62) * a + Real(0.38) * b));
}

inline EngineSound engineSoundFor(Real speedMps, Real throttle01, bool running) {
    EngineSound out;
    if (!running) return out;
    const Real rev = engineRevFraction(speedMps);
    const Real pedal = std::min(Real(1), std::fabs(throttle01));
    out.pitch = static_cast<float>(
        kEnginePitchIdle + (kEnginePitchMax - kEnginePitchIdle) * rev);
    // Load: the same revs are louder under power than trailing throttle.
    const Real body = Real(0.20) + Real(0.62) * rev;
    const Real load = Real(0.85) + Real(0.30) * pedal;
    out.gain = static_cast<float>(std::min(Real(1), body * load));
    return out;
}

}  // namespace engine

#endif
