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
// travel directions): -1 left, +1 right, 0 straight/undetermined. PHYSICAL side,
// in the world's right-handed frame: facing +z, the right hand points toward -x
// (the driving lab's steer-sign probe measured "+steer turns toward -x", and
// +steer is the right turn — see driver_control.h). A bend toward -x from +z
// gives a POSITIVE cross product, so cross > 0 maps to +1. This used to be
// mirrored ("the sim's right is (d.y, -d.x)"), which cancelled against the
// then-swapped lamp marker tags; both are now physical, so the pair of flips
// that kept city signals looking right is gone rather than doubled.
inline int carTurnSide(engine::Vec2 fromDir, engine::Vec2 toDir,
                       Real minBend = 0.02) {
    const Real cross = fromDir.x * toDir.y - fromDir.y * toDir.x;
    if (cross > minBend) return +1;    // toDir swings toward the right (-x side)
    if (cross < -minBend) return -1;   // ...toward the left (+x side)
    return 0;
}

}  // namespace citysim

#endif
