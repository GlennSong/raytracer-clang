#ifndef RAYTRACER_ENGINE_VEHICLE_LAMPS_H
#define RAYTRACER_ENGINE_VEHICLE_LAMPS_H

#include "../rt_math.h"

#include <cmath>
#include <cstdint>

namespace engine {

// Lamp DECISION CORE for every vehicle — the player's car and the city's traffic.
//
// This started life as citysim's carLampState, typed against `Agent::State` and
// the sim's own clock, which meant the player's car could not use it: it had a
// manual headlight toggle, no brake lights and no indicators. The predicate is
// the same question in both cases, so it lives here and citysim adapts to it
// (see apps/citysim/car_lamps.h) rather than owning it.
//
// Pure: no ECS, no rendering, no Lua, no clock of its own. Callers supply the
// two things that differ between them — whether it is dark, and what the vehicle
// is doing — and get back which lamps are lit. The turn-signal BLINK is a render
// concern layered on top; `left`/`right` only say which side is SIGNALLING.

// What the vehicle is doing, coarsely. citysim maps its Agent::State onto this;
// the player's car derives it from brake input and steering.
enum class LampMotion : std::uint8_t {
    Rolling,   // moving or stopped, nothing special
    Holding,   // waiting at a light / yielding / braking to a halt
    Turning,   // committed to a turn, so the indicator is on
};

// A car sheds speed hard while already slow -> brake lights, so a car rolling to
// a stop lights up before it has actually stopped.
constexpr Real kLampBrakeDecel = 0.20;      // m/s drop per step that reads as braking
constexpr Real kLampBrakeSlowSpeed = 7.0;   // ...only below this speed

// Headlights follow the SUN, not a wall clock: `sunDirY` is the y component of
// the direction toward the sun, so <= 0 is below the horizon. Using the sun means
// a level with a fixed sun behaves correctly too, instead of depending on a
// simulated clock that such a level never advances.
constexpr Real kLampDuskSunY = 0.06;

// ~1.5 Hz at 50% duty.
constexpr Real kLampBlinkHz = 1.5;

struct VehicleLamps {
    bool head = false;    // headlights (white, forward)
    bool brake = false;   // brake lights (red, rear)
    bool left = false;    // left indicator is signalling (amber)
    bool right = false;   // right indicator is signalling (amber)
};

// `turnDir`: -1 left, +1 right, 0 none. `headOverride` forces headlights on
// regardless of daylight — the player's manual switch, which must be able to turn
// lights ON in daylight but should never fight the automatic dusk behaviour.
inline VehicleLamps vehicleLampState(LampMotion motion, Real speed, Real prevSpeed,
                                     bool dark, int turnDir,
                                     bool headOverride = false) {
    VehicleLamps lamps;
    lamps.head = dark || headOverride;
    const bool decelHard =
        speed < prevSpeed - kLampBrakeDecel && speed < kLampBrakeSlowSpeed;
    lamps.brake = motion == LampMotion::Holding || decelHard;
    if (motion == LampMotion::Turning) {
        lamps.left = turnDir < 0;
        lamps.right = turnDir > 0;
    }
    return lamps;
}

// Is it dark enough for headlights, from the SUN's elevation (the y of the
// direction toward the sun)? Takes the scalar, not the light direction: at
// night the active light in slot 0 is the MOON (WS2), whose elevation must
// never gate the lamps — callers pass lighting.solarElevation.
inline bool lampsDark(Real solarElevation) {
    return solarElevation < kLampDuskSunY;
}

// A smooth 0..1 dusk ramp around the same boundary, for lights that should
// fade in rather than pop (street lamps, window glow): 0 in daylight, 1 once
// the sun is a few degrees under. Shares kLampDuskSunY so every night light
// in the world agrees on when evening starts.
inline Real duskRamp(Real solarElevation) {
    const Real hi = kLampDuskSunY;          // fully off above this
    const Real lo = kLampDuskSunY - 0.10;   // fully on below this
    Real t = (hi - solarElevation) / (hi - lo);
    if (t < 0) t = 0;
    if (t > 1) t = 1;
    return t * t * (3.0 - 2.0 * t);
}

// Blink phase, shared so the player's indicators and the city's flash together.
// Driven by a sim clock, so it stays deterministic.
inline bool lampBlinkOn(Real seconds) {
    return std::fmod(seconds * kLampBlinkHz, Real(1)) < Real(0.5);
}

}  // namespace engine

#endif
