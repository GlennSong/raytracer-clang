#include "test_framework.h"

#include "../src/engine/ai/lane_follow.h"

#include <cmath>
#include <vector>

using namespace engine;

// Closed-loop driving quality (ADR-0061): a kinematic BICYCLE-MODEL car driven
// by the real controller stack — LaneFollower (pure pursuit) -> DriverCommand ->
// computeDriverInput -> vehicle. This is the headless stand-in for the Jolt car:
// same controller, same sign conventions, a simple but honest plant (speed-
// proportional yaw rate, bounded wheel angle, throttle/brake accel). If the car
// tracks lanes, corners, and stops HERE, the on-device work is tuning, not
// debugging the control law.

namespace {

// Kinematic bicycle: positive steer turns RIGHT (matching setVehicleInput's
// `right` convention); yaw rate = v / wheelbase * tan(wheelAngle).
struct BikeCar {
    Vec2 pos{0, 0};
    Vec2 fwd{0, 1};
    Real speed = 0;

    void step(const DriverInput& in, Real dt) {
        Real accel = in.throttle * 4.0 - in.brake * 7.0;
        speed = std::max(Real(0), std::min(Real(30), speed + accel * dt));
        Real wheel = in.steer * 0.55;                     // rad, + = right
        Real yaw = speed / 2.8 * std::tan(wheel) * dt;    // turned this tick
        Vec2 right(fwd.y, -fwd.x);
        Real ca = std::cos(yaw), sa = std::sin(yaw);
        fwd = Vec2(fwd.x * ca + right.x * sa, fwd.y * ca + right.y * sa);
        Real l = std::sqrt(fwd.x * fwd.x + fwd.y * fwd.y);
        fwd = Vec2(fwd.x / l, fwd.y / l);
        pos = pos + fwd * (speed * dt);
    }
};

// One control tick: real progress -> pursuit heading -> pedal/wheel.
DriverInput controlTick(LaneFollower& lf, const BikeCar& car, Real desiredSpeed,
                        Real* latOut = nullptr) {
    Real lat = lf.update(car.pos);
    if (latOut) *latOut = lat;
    DriverCommand cmd = pursuitCommand(lf, car.pos, desiredSpeed,
                                       pursuitLookahead(car.speed));
    return computeDriverInput(DriverState{car.fwd, car.speed}, cmd);
}

std::vector<Vec2> straightPath(Real len, Real step = 5.0) {
    std::vector<Vec2> p;
    for (Real z = 0; z <= len; z += step) p.push_back(Vec2(0, z));
    return p;
}

}  // namespace

TEST_CASE(pursuit_converges_onto_a_straight_lane_from_an_offset) {
    LaneFollower lf;
    lf.setPath(straightPath(400.0));
    BikeCar car;
    car.pos = Vec2(3.0, 0);      // 3 m off the lane centre
    car.fwd = Vec2(0, 1);

    const Real dt = 0.05;
    Real maxLateLat = 0;         // worst offset after the convergence window
    for (int i = 0; i < 800; ++i) {                       // 40 s
        Real lat = 0;
        DriverInput in = controlTick(lf, car, 9.0, &lat);
        car.step(in, dt);
        CHECK(lat < 5.0);                                 // never diverges
        if (i > 300) maxLateLat = std::max(maxLateLat, lat);
    }
    CHECK(maxLateLat < 1.0);     // converged onto the lane and stayed there
    CHECK(car.speed > 7.0);      // while actually driving, not crawling
}

TEST_CASE(pursuit_carries_the_car_through_a_right_angle_corner) {
    // An L: north 60 m, then east 60 m. The old open-loop ghost chase cut or
    // overshot corners; pursuit must track around within lane-ish error.
    std::vector<Vec2> path;
    for (Real z = 0; z <= 60; z += 3) path.push_back(Vec2(0, z));
    for (Real x = 3; x <= 60; x += 3) path.push_back(Vec2(x, 60));
    LaneFollower lf;
    lf.setPath(path);
    BikeCar car;
    car.pos = Vec2(0, 0);
    car.fwd = Vec2(0, 1);

    const Real dt = 0.05;
    Real worst = 0;
    bool reachedEnd = false;
    for (int i = 0; i < 1200 && !reachedEnd; ++i) {       // up to 60 s
        Real lat = 0;
        DriverInput in = controlTick(lf, car, 6.0, &lat);
        car.step(in, dt);
        worst = std::max(worst, lat);
        Real dx = car.pos.x - 60.0, dy = car.pos.y - 60.0;
        if (std::sqrt(dx * dx + dy * dy) < 4.0) reachedEnd = true;
    }
    CHECK(reachedEnd);           // the corner was completed, not orbited
    CHECK(worst < 3.5);          // stayed within a road half-width of the lane
}

TEST_CASE(pursuit_station_is_monotonic_through_a_hairpin) {
    // Two parallel legs joined by a half-circle (radius 7 m > the bike's ~4.5 m
    // minimum turn radius). The legs run close and OPPOSITE — a naive global
    // closest-point would snap progress onto the return leg; the windowed,
    // monotonic follower must not.
    std::vector<Vec2> path;
    for (Real z = 0; z <= 40; z += 3) path.push_back(Vec2(0, z));
    for (int k = 1; k < 12; ++k) {                        // half-circle, centre (7,40)
        Real a = 3.14159265358979 * k / 12.0;
        path.push_back(Vec2(7 - 7 * std::cos(a), 40 + 7 * std::sin(a)));
    }
    for (Real z = 40; z >= 0; z -= 3) path.push_back(Vec2(14, z));
    LaneFollower lf;
    lf.setPath(path);
    BikeCar car;
    car.pos = Vec2(0, 0);
    car.fwd = Vec2(0, 1);

    // Cruise commanded the whole way: pursuitCommand's built-in end guard (speed
    // capped by remaining path) is what must park the car at the far end instead
    // of letting it orbit the last point at full lock.
    const Real dt = 0.05;
    Real prevStation = 0, worst = 0;
    for (int i = 0; i < 1600; ++i) {                      // 80 s at 4 m/s
        Real lat = 0;
        DriverInput in = controlTick(lf, car, 4.0, &lat);
        car.step(in, dt);
        CHECK(lf.station() >= prevStation);               // progress never jumps back
        prevStation = lf.station();
        worst = std::max(worst, lat);
    }
    CHECK(worst < 6.0);                                   // never wandered to the other leg
    CHECK(car.speed < 0.1);                               // parked at the end, not orbiting it
    Real dx = car.pos.x - 14.0, dy = car.pos.y - 0.0;
    CHECK(std::sqrt(dx * dx + dy * dy) < 6.0);            // traversed to the far end
}

TEST_CASE(pursuit_car_stops_at_the_end_of_its_path) {
    LaneFollower lf;
    lf.setPath(straightPath(80.0));
    BikeCar car;
    car.fwd = Vec2(0, 1);

    const Real dt = 0.05;
    for (int i = 0; i < 1200; ++i) {
        // The caller's speed plan: cruise, then ease down approaching the end
        // (<= the controller's stopSpeed near the end -> full brake to a halt).
        Real desired = std::min(Real(8.0), lf.remaining() * 0.3);
        DriverInput in = controlTick(lf, car, desired);
        car.step(in, dt);
    }
    CHECK(car.speed < 0.1);           // came to rest
    CHECK(lf.station() > 70.0);       // ...near the end of the path
    CHECK(car.pos.y < 84.0);          // ...without sailing past it
}

TEST_CASE(pursuit_is_deterministic) {
    auto run = [] {
        LaneFollower lf;
        lf.setPath(straightPath(200.0));
        BikeCar car;
        car.pos = Vec2(2.0, 0);
        car.fwd = Vec2(0, 1);
        for (int i = 0; i < 500; ++i) car.step(controlTick(lf, car, 7.0), 0.05);
        return car.pos;
    };
    Vec2 a = run(), b = run();
    CHECK(a.x == b.x);
    CHECK(a.y == b.y);
}
