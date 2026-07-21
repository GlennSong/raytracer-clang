#ifndef RAYTRACER_APPS_CITYSIM_CAR_LAMPS_H
#define RAYTRACER_APPS_CITYSIM_CAR_LAMPS_H

#include "../../engine/ai/nav_graph.h"   // engine::Vec2 (turn-side helper)
#include "../../engine/vehicle_lamps.h"  // the shared decision core
#include "city_sim.h"                     // Agent::State, Real

namespace citysim {

// Car lamps for CITY TRAFFIC — an adapter over engine::vehicleLampState.
//
// The predicate used to live here, typed against Agent::State and the sim clock,
// which is exactly why the player's car could not use it and ended up with a
// manual headlight toggle and no brake lights or indicators at all. The rule is
// the same question for both, so it moved to engine/vehicle_lamps.h and this
// header now just translates the sim's vocabulary into it. Behaviour is
// unchanged; city_render.cpp still reads it each step and pushes an emissive
// quad at the matching marker.

// Headlights follow the sim's own deterministic clock (CitySim::timeOfDay, 0..24
// in-world hours): on before dawn / after dusk, off through the day. Named so the
// dusk/dawn boundary is one obvious knob, not a magic number in the predicate.
constexpr Real kHeadlightDawn = 6.5;    // headlights go OFF at/after this hour
constexpr Real kHeadlightDusk = 19.0;   // headlights come ON after this hour

// Brake thresholds now live with the predicate; aliased here so the existing
// citysim names keep working.
constexpr Real kBrakeDecel = engine::kLampBrakeDecel;
constexpr Real kBrakeSlowSpeed = engine::kLampBrakeSlowSpeed;
constexpr Real kTurnBlinkHz = engine::kLampBlinkHz;

using CarLamps = engine::VehicleLamps;

// The sim runs on a deterministic in-world clock rather than a sun, so darkness
// is decided here and handed to the shared predicate as a plain bool.
inline bool cityIsDark(Real clockHours) {
    return clockHours < kHeadlightDawn || clockHours > kHeadlightDusk;
}

inline engine::LampMotion cityLampMotion(Agent::State state) {
    if (state == Agent::State::Waiting || state == Agent::State::Yielding)
        return engine::LampMotion::Holding;
    if (state == Agent::State::Turning) return engine::LampMotion::Turning;
    return engine::LampMotion::Rolling;
}

// The lamp state for one car this step. `turnDir`: -1 = turning left, +1 = turning
// right, 0 = none (the route bend, via carTurnSide). `prevSpeed` is last step's
// speed so a hard deceleration lights the brakes.
inline CarLamps carLampState(Agent::State state, Real speed, Real prevSpeed,
                             Real clockHours, int turnDir) {
    return engine::vehicleLampState(cityLampMotion(state), speed, prevSpeed,
                                    cityIsDark(clockHours), turnDir);
}

// The SIDE a car turns as its path bends from `fromDir` into `toDir` (both unit XZ
// travel directions): -1 left, +1 right, 0 straight/undetermined. The sim's right
// vector is (d.y, -d.x) (see city_render signalSite), so a bend TOWARD the right
// yields a negative cross product — this maps that to +1. Pure geometry, so the
// renderer feeds it consecutive route-leg directions (nav.direction).
inline int carTurnSide(engine::Vec2 fromDir, engine::Vec2 toDir,
                       Real minBend = 0.02) {
    const Real cross = fromDir.x * toDir.y - fromDir.y * toDir.x;
    if (cross < -minBend) return +1;   // toDir swings toward the right vector
    if (cross > minBend) return -1;    // ...toward the left
    return 0;
}

}  // namespace citysim

#endif
