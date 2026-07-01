#include "test_framework.h"

#include "../src/engine/ai/driver_agent.h"

#include <cmath>

using namespace engine;

// The unified driver controller (ADR-0061): the SAME {throttle, steer, brake}
// the player produces, computed by an AI from what its brain wants. These pin the
// steering sign convention and the throttle/brake behaviour so player and AI cars
// drive identically.

namespace {
DriverState facing(Vec2 fwd, Real speed) { DriverState s; s.forward = fwd; s.speed = speed; return s; }
DriverCommand want(Vec2 head, Real spd) { DriverCommand c; c.desiredHeading = head; c.desiredSpeed = spd; return c; }
}  // namespace

TEST_CASE(driver_throttles_toward_target_speed_when_aligned) {
    // Facing +Z, going slow, told to go straight ahead fast → throttle, no brake,
    // (near) no steer.
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 2.0), want(Vec2(0, 1), 12.0));
    CHECK(in.throttle > 0.0);
    CHECK(in.brake == 0.0);
    CHECK(std::fabs(in.steer) < 1e-6);
}

TEST_CASE(driver_steers_right_toward_a_heading_on_its_right) {
    // Facing +Z; the car's right is +X. A desired heading of +X must steer +
    // (right), matching PhysicsWorld::setVehicleInput's `right` sign.
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 5.0), want(Vec2(1, 0), 5.0));
    CHECK(in.steer > 0.0);
}

TEST_CASE(driver_steers_left_toward_a_heading_on_its_left) {
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 5.0), want(Vec2(-1, 0), 5.0));
    CHECK(in.steer < 0.0);
}

TEST_CASE(driver_brakes_to_a_stop_when_asked_for_zero_speed) {
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 6.0), want(Vec2(0, 1), 0.0));
    CHECK(in.brake == 1.0);
    CHECK(in.throttle == 0.0);
}

TEST_CASE(driver_brakes_when_overspeed) {
    // Going faster than the target (but target > stopSpeed) → brake, no throttle.
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 15.0), want(Vec2(0, 1), 6.0));
    CHECK(in.brake > 0.0);
    CHECK(in.throttle == 0.0);
}

TEST_CASE(driver_steer_and_pedals_stay_in_range) {
    // A hard error (told to reverse direction, wildly overspeed) still yields legal
    // inputs — steer clamped to [-1,1], brake to [0,1].
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 40.0), want(Vec2(0, -1), 1.0));
    CHECK(in.steer >= -1.0 && in.steer <= 1.0);
    CHECK(in.brake >= 0.0 && in.brake <= 1.0);
    CHECK(in.throttle >= -1.0 && in.throttle <= 1.0);
    CHECK(std::fabs(in.steer) == 1.0);   // saturated on a 180° error
}
