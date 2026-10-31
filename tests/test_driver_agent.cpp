#include "test_framework.h"

#include "../src/engine/ai/driver_agent.h"

#include <cmath>

using namespace engine;

// The unified driver controller (ADR-0062): the SAME {throttle, steer, brake}
// the player produces, computed by an AI from what its brain wants. These pin the
// steering sign convention and the throttle/brake behaviour so player and AI cars
// drive identically.

namespace {
DriverState facing(Vec2 fwd, Real speed) { DriverState s; s.forward = fwd; s.speed = speed; return s; }
// Pure pursuit from the car at the origin: a lane frame toward `head`.
DriverCommand want(Vec2 head, Real spd) { return DriverCommand::toward(Vec2(0, 0), head, spd); }
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
    // Facing +Z; the car's right is -X — the lab-measured chirality ("+steer
    // turns toward -x", the driving lab's steer-sign probe). A desired heading of -X must steer +
    // (right), matching PhysicsWorld::setVehicleInput's `right` sign. These two
    // used to pin the mirrored belief (right = +X), which Jolt refutes.
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 5.0), want(Vec2(-1, 0), 5.0));
    CHECK(in.steer > 0.0);
}

TEST_CASE(driver_steers_left_toward_a_heading_on_its_left) {
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 5.0), want(Vec2(1, 0), 5.0));
    CHECK(in.steer < 0.0);
}

TEST_CASE(driver_brakes_to_a_stop_when_asked_for_zero_speed) {
    DriverInput in = computeDriverInput(facing(Vec2(0, 1), 6.0), want(Vec2(0, 1), 0.0));
    CHECK(in.brake == 1.0);
    CHECK(in.throttle == 0.0);
}

TEST_CASE(driver_brakes_when_overspeed) {
    // Going faster than the target → brake, no throttle.
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

// THE CROSS-TRACK TERM (Stanley; item 24, #117: one controller for every AI driver): a car pointed straight down
// its lane but sitting beside it steers back onto it -- toward -x (right, facing +z) when it sits at +x of the lane,
// and harder the further out -- where a heading-only controller would have driven on parallel forever.
TEST_CASE(driver_steers_back_onto_its_lane_from_beside_it) {
    DriverCommand lane;
    lane.laneA = Vec2(0, 0);
    lane.laneB = Vec2(0, 20);
    lane.desiredSpeed = 8.0;
    DriverState s = facing(Vec2(0, 1), 8.0);
    s.pos = Vec2(1.0, 5.0);   // a metre to the lane's left (+x facing +z)
    const Real near = computeDriverInput(s, lane).steer;
    s.pos = Vec2(2.5, 5.0);
    const Real far = computeDriverInput(s, lane).steer;
    CHECK(near > 0.0);          // right, back toward the lane
    CHECK(far > near);          // harder the further out
    s.pos = Vec2(-1.0, 5.0);
    CHECK(computeDriverInput(s, lane).steer < 0.0);
    s.pos = Vec2(0.0, 5.0);
    CHECK(std::fabs(computeDriverInput(s, lane).steer) < 1e-9);   // on the lane, aligned: straight
}

// A stop is HELD: told to stop at a crawl, the brake does not ease off to a creep.
TEST_CASE(driver_holds_the_brake_at_a_stop) {
    const DriverInput in = computeDriverInput(facing(Vec2(0, 1), 0.1), want(Vec2(0, 1), 0.0));
    CHECK(in.brake >= 0.6);
    CHECK(in.throttle == 0.0);
}
