#include "test_framework.h"

#include "../src/engine/vehicle_lamps.h"

using namespace engine;

// The SHARED lamp predicate. It used to live in citysim typed against
// Agent::State, which is exactly why the player's car had only a manual headlight
// switch — no brake lights, no indicators. These pin the behaviour both callers
// now depend on; citysim's own adapter is covered by tests/test_car_lamps.cpp.

namespace {
VehicleLamps roll(Real speed, Real prev, bool dark = false) {
    return vehicleLampState(LampMotion::Rolling, speed, prev, dark, 0);
}
}  // namespace

TEST_CASE(vehicle_lamps_headlights_follow_darkness) {
    CHECK(!roll(10, 10, /*dark=*/false).head);
    CHECK(roll(10, 10, /*dark=*/true).head);
}

// The manual switch can only ADD light. A driver who flicks the lights on at noon
// gets them; a driver who never touches the switch still gets them at dusk.
TEST_CASE(vehicle_lamps_manual_switch_forces_on_but_never_off) {
    CHECK(vehicleLampState(LampMotion::Rolling, 10, 10, false, 0, true).head);
    CHECK(vehicleLampState(LampMotion::Rolling, 10, 10, true, 0, false).head);
}

TEST_CASE(vehicle_lamps_brake_when_holding) {
    CHECK(vehicleLampState(LampMotion::Holding, 0, 0, false, 0).brake);
    CHECK(!vehicleLampState(LampMotion::Rolling, 10, 10, false, 0).brake);
}

// A car rolling to a stop should light up BEFORE it has stopped.
TEST_CASE(vehicle_lamps_brake_on_hard_decel_while_slow) {
    CHECK(roll(4.0, 4.0 + kLampBrakeDecel + 0.01).brake);
    // Gentle lift-off is not braking.
    CHECK(!roll(4.0, 4.0 + kLampBrakeDecel - 0.01).brake);
    // The same deceleration at motorway speed is not braking either — that is
    // ordinary drag, and lighting the brakes for it would strobe on every car.
    CHECK(!roll(kLampBrakeSlowSpeed + 5.0, kLampBrakeSlowSpeed + 5.0 + 1.0).brake);
}

TEST_CASE(vehicle_lamps_indicators_need_a_turn) {
    const VehicleLamps left = vehicleLampState(LampMotion::Turning, 5, 5, false, -1);
    CHECK(left.left);
    CHECK(!left.right);

    const VehicleLamps right = vehicleLampState(LampMotion::Turning, 5, 5, false, +1);
    CHECK(right.right);
    CHECK(!right.left);

    // A turn direction with no committed turn signals nothing — otherwise every
    // steering correction would flash the indicators.
    const VehicleLamps straight = vehicleLampState(LampMotion::Rolling, 5, 5, false, -1);
    CHECK(!straight.left);
    CHECK(!straight.right);
}

// Darkness comes from the SUN, not a clock, so a level with a fixed sun behaves
// correctly instead of waiting on a simulated clock it never advances.
TEST_CASE(vehicle_lamps_darkness_from_sun_elevation) {
    CHECK(!lampsDark(Vec3(0.3, 0.9, 0.2)));    // sun high
    CHECK(lampsDark(Vec3(0.9, -0.1, 0.2)));    // below the horizon
    CHECK(lampsDark(Vec3(0.9, 0.0, 0.2)));     // on the horizon counts as dusk
}

TEST_CASE(vehicle_lamps_blink_is_periodic_and_even) {
    const Real period = 1.0 / kLampBlinkHz;
    CHECK(lampBlinkOn(0.0));
    CHECK(!lampBlinkOn(period * 0.75));
    // Same phase one period later.
    CHECK(lampBlinkOn(period) == lampBlinkOn(0.0));
    CHECK(lampBlinkOn(period * 1.75) == lampBlinkOn(period * 0.75));
}
