#include "test_framework.h"

#include "../src/apps/citysim/car_lamps.h"

using namespace engine;
using namespace citysim;

// The car-lamp decision core (ADR-0065 follow-up). carLampState is a PURE
// predicate over an agent's sim state, so every case here is fully headless — no
// World, no AssetManager, no Lua. The instanced RENDER of these lamps (emissive
// quads at the body markers) is viewer-gated; a couple of headless bake checks
// live in test_city_render.cpp, but the "when does a lamp light" logic is all
// decided (and tested) here.

namespace {
// A steady daytime cruise with no bend: the baseline "all lamps off" input.
CarLamps cruising(Real clock) {
    return carLampState(Agent::State::Cruising, 10.0, 10.0, clock, 0);
}
}  // namespace

TEST_CASE(car_lamps_headlights_off_in_daylight) {
    CarLamps noon = cruising(12.0);
    CHECK(!noon.head);
    CHECK(!noon.brake);
    CHECK(!noon.left);
    CHECK(!noon.right);   // a plain daylight cruise lights nothing
}

TEST_CASE(car_lamps_headlights_on_at_night) {
    CHECK(cruising(21.0).head);    // well after dusk
    CHECK(cruising(2.0).head);     // deep night
    CHECK(cruising(5.0).head);     // before dawn
}

TEST_CASE(car_lamps_headlight_dawn_dusk_boundaries) {
    // head = clock < kHeadlightDawn (6.5) || clock > kHeadlightDusk (19.0).
    CHECK(cruising(6.0).head);                    // before dawn: on
    CHECK(!cruising(kHeadlightDawn).head);        // AT dawn: off (not < dawn)
    CHECK(!cruising(6.6).head);                   // just past dawn: off
    CHECK(!cruising(kHeadlightDusk).head);        // AT dusk: off (not > dusk)
    CHECK(!cruising(18.9).head);                  // just before dusk: off
    CHECK(cruising(19.1).head);                   // just past dusk: on
    // Sanity: the two thresholds bracket a lights-off daytime band.
    CHECK(kHeadlightDawn < kHeadlightDusk);
}

TEST_CASE(car_lamps_brake_when_waiting_or_yielding) {
    // Holding for a red / yielding at a junction lights the brakes regardless of
    // clock or speed detail.
    CHECK(carLampState(Agent::State::Waiting, 0.0, 0.0, 12.0, 0).brake);
    CHECK(carLampState(Agent::State::Yielding, 4.0, 5.0, 12.0, 0).brake);
    // Even at night the brake predicate is independent of the headlight one.
    CarLamps night = carLampState(Agent::State::Waiting, 0.0, 0.0, 23.0, 0);
    CHECK(night.brake);
    CHECK(night.head);
}

TEST_CASE(car_lamps_brake_on_hard_deceleration_while_slow) {
    // Shedding speed hard while already slow (rolling to a stop) lights the brakes
    // even in a non-holding FSM state.
    CHECK(carLampState(Agent::State::Following, 3.0, 5.0, 12.0, 0).brake);
    CHECK(carLampState(Agent::State::Cruising, 1.0, 4.0, 12.0, 0).brake);
}

TEST_CASE(car_lamps_no_brake_on_gentle_or_fast_deceleration) {
    // A gentle slow-down (below the decel threshold) is not braking.
    CHECK(!carLampState(Agent::State::Cruising, 6.9, 7.0, 12.0, 0).brake);
    // A big lift-off at HIGH speed (still above the slow-speed cap) is not the
    // brake-to-a-stop the light is for.
    CHECK(!carLampState(Agent::State::Cruising, 20.0, 25.0, 12.0, 0).brake);
    // Steady cruise: no decel at all.
    CHECK(!carLampState(Agent::State::Cruising, 10.0, 10.0, 12.0, 0).brake);
    // Accelerating certainly isn't braking.
    CHECK(!carLampState(Agent::State::Cruising, 8.0, 5.0, 12.0, 0).brake);
}

TEST_CASE(car_lamps_turn_signal_side_selection) {
    CarLamps left = carLampState(Agent::State::Turning, 6.0, 6.0, 12.0, -1);
    CHECK(left.left);
    CHECK(!left.right);

    CarLamps right = carLampState(Agent::State::Turning, 6.0, 6.0, 12.0, +1);
    CHECK(right.right);
    CHECK(!right.left);

    CarLamps straight = carLampState(Agent::State::Turning, 6.0, 6.0, 12.0, 0);
    CHECK(!straight.left);
    CHECK(!straight.right);   // Turning but no bend side: neither indicator
}

TEST_CASE(car_lamps_turn_signal_only_while_turning) {
    // A turnDir carried in a non-Turning state must NOT flash an indicator — the
    // side comes from the route, but the DECISION to signal is the Turning state.
    CarLamps cruise = carLampState(Agent::State::Cruising, 10.0, 10.0, 12.0, -1);
    CHECK(!cruise.left);
    CHECK(!cruise.right);
    CarLamps follow = carLampState(Agent::State::Following, 5.0, 5.0, 12.0, +1);
    CHECK(!follow.left);
    CHECK(!follow.right);
}

TEST_CASE(car_lamps_night_turn_combines_head_and_signal) {
    // Lamps are independent: a car turning left at night has BOTH headlights and
    // the left indicator (and no brakes if it isn't slowing).
    CarLamps lamps = carLampState(Agent::State::Turning, 8.0, 8.0, 22.0, -1);
    CHECK(lamps.head);
    CHECK(lamps.left);
    CHECK(!lamps.right);
    CHECK(!lamps.brake);
}

TEST_CASE(car_turn_side_geometry) {
    // Straight through: no side.
    CHECK(carTurnSide(Vec2(0, 1), Vec2(0, 1)) == 0);
    CHECK(carTurnSide(Vec2(1, 0), Vec2(1, 0)) == 0);
    // A right turn: travelling +Z then swinging to +X (the sim's right vector).
    CHECK(carTurnSide(Vec2(0, 1), Vec2(1, 0)) == +1);
    // A left turn: +Z swinging to -X.
    CHECK(carTurnSide(Vec2(0, 1), Vec2(-1, 0)) == -1);
    // Consistent from another heading: travelling +X, turning toward -Z is a right.
    CHECK(carTurnSide(Vec2(1, 0), Vec2(0, -1)) == +1);
    // And +X turning toward +Z is a left.
    CHECK(carTurnSide(Vec2(1, 0), Vec2(0, 1)) == -1);
}

TEST_CASE(car_turn_side_ignores_tiny_bends) {
    // A hair of lane wobble (below minBend) is not a turn — the straight-ahead
    // navgraph is a chain of nearly-collinear links.
    CHECK(carTurnSide(Vec2(0, 1), Vec2(0.01, 0.9999)) == 0);
    CHECK(carTurnSide(Vec2(0, 1), Vec2(-0.01, 0.9999)) == 0);
    // But a clear bend past the threshold registers.
    CHECK(carTurnSide(Vec2(0, 1), Vec2(0.3, 0.95)) == +1);
    CHECK(carTurnSide(Vec2(0, 1), Vec2(-0.3, 0.95)) == -1);
}
