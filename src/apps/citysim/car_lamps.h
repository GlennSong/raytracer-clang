#ifndef RAYTRACER_APPS_CITYSIM_CAR_LAMPS_H
#define RAYTRACER_APPS_CITYSIM_CAR_LAMPS_H

#include "../../engine/ai/nav_graph.h"   // engine::Vec2 (turn-side helper)
#include "city_sim.h"                     // Agent::State, Real

namespace citysim {

// Car lamp DECISION CORE (ADR-0065 follow-up: the light attachment markers become
// emissive lamps). A pure predicate over an agent's sim state — no ECS, no
// rendering, no Lua — so the whole "when does a lamp light" question is decided in
// one headless-testable place. city_render.cpp reads it each step and pushes an
// emissive quad at the matching marker's world position; the turn-signal BLINK is
// a render concern layered on top (this predicate only says which side is
// *signalling*, not whether it is lit THIS instant).

// Headlights follow the sim's own deterministic clock (CitySim::timeOfDay, 0..24
// in-world hours): on before dawn / after dusk, off through the day. Named so the
// dusk/dawn boundary is one obvious knob, not a magic number in the predicate.
constexpr Real kHeadlightDawn = 6.5;    // headlights go OFF at/after this hour
constexpr Real kHeadlightDusk = 19.0;   // headlights come ON after this hour

// Brake lights. A car is braking when it is holding for a signal / yielding
// (Waiting / Yielding), OR shedding speed hard while already slow — a car rolling
// to a stop lights its brakes before it fully stops.
constexpr Real kBrakeDecel = 0.20;      // speed drop per step (m/s) that reads as braking
constexpr Real kBrakeSlowSpeed = 7.0;   // ...only counts as braking below this speed (m/s)

// Turn-signal blink cadence (render concern; documented here so the constant has
// a home next to the predicate). ~1.5 Hz at 50% duty.
constexpr Real kTurnBlinkHz = 1.5;

// Which of a car's lamps are lit THIS step. `left`/`right` mean "the turn signal
// on this side is signalling" — the render layer blinks it; the predicate itself
// carries no blink phase.
struct CarLamps {
    bool head = false;    // headlights (white, forward-facing)
    bool brake = false;   // brake lights (red, rear)
    bool left = false;    // left turn signal is signalling (amber)
    bool right = false;   // right turn signal is signalling (amber)
};

// The lamp state for one car this step. `turnDir`: -1 = turning left, +1 = turning
// right, 0 = none (the route bend, via carTurnSide). `prevSpeed` is last step's
// speed so a hard deceleration lights the brakes.
inline CarLamps carLampState(Agent::State state, Real speed, Real prevSpeed,
                             Real clockHours, int turnDir) {
    CarLamps lamps;
    lamps.head = clockHours < kHeadlightDawn || clockHours > kHeadlightDusk;
    const bool holding =
        state == Agent::State::Waiting || state == Agent::State::Yielding;
    const bool decelHard = speed < prevSpeed - kBrakeDecel && speed < kBrakeSlowSpeed;
    lamps.brake = holding || decelHard;
    if (state == Agent::State::Turning) {
        lamps.left = turnDir < 0;
        lamps.right = turnDir > 0;
    }
    return lamps;
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
