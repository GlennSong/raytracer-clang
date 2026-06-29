#include "test_framework.h"

#include "../src/apps/citysim/traffic_rules.h"

using namespace engine;
using namespace citysim;

TEST_CASE(approach_stop_eases_to_zero) {
    // At the stop point, speed 0; farther away, faster; capped at maxSpeed.
    CHECK_APPROX(approachStop(0.0, 4.0, 20.0), 0.0, 1e-9);
    CHECK(approachStop(2.0, 4.0, 20.0) > 0.0);
    CHECK(approachStop(2.0, 4.0, 20.0) < approachStop(20.0, 4.0, 20.0));
    CHECK_APPROX(approachStop(1000.0, 4.0, 20.0), 20.0, 1e-9);   // capped
}

TEST_CASE(signal_cap_green_is_free_red_stops) {
    CHECK_APPROX(signalSpeedCap(SignalState::Green, 5.0, 16.0, 4.0), 16.0, 1e-9);
    // Red close ahead caps well below free speed (easing to a stop).
    CHECK(signalSpeedCap(SignalState::Red, 3.0, 16.0, 4.0) < 16.0);
    CHECK_APPROX(signalSpeedCap(SignalState::Red, 0.0, 16.0, 4.0), 0.0, 1e-9);
}

TEST_CASE(nearest_obstacle_ahead_in_cone) {
    VisionCone c;
    c.origin = Vec2(0, 0);
    c.forward = Vec2(1, 0);
    c.range = 30;
    c.halfAngleRad = 0.6;
    std::vector<Vec2> obstacles = {
        Vec2(10, 0),    // ahead, 10 m
        Vec2(5, 0),     // ahead, 5 m (nearest)
        Vec2(-8, 0),    // behind — ignored
        Vec2(2, 9),     // off to the side, outside the cone — ignored
    };
    CHECK_APPROX(nearestObstacleAhead(c, obstacles), 5.0, 1e-6);
}

TEST_CASE(nearest_obstacle_clear_cone) {
    VisionCone c;
    c.forward = Vec2(1, 0);
    c.range = 30;
    std::vector<Vec2> obstacles = { Vec2(-5, 0), Vec2(0, 20) };   // behind / to the side
    CHECK(nearestObstacleAhead(c, obstacles, 1e9) >= 1e9);        // clear road
}
