#include "test_framework.h"

#include "../src/engine/ai/perception.h"

using namespace engine;

TEST_CASE(cone_sees_in_front_not_behind) {
    VisionCone c;
    c.origin = Vec2(0, 0);
    c.forward = Vec2(1, 0);
    c.range = 30;
    c.halfAngleRad = 0.6;
    CHECK(sees(c, Vec2(10, 0)));      // straight ahead
    CHECK(!sees(c, Vec2(-10, 0)));    // directly behind
}

TEST_CASE(cone_respects_range) {
    VisionCone c;
    c.forward = Vec2(1, 0);
    c.range = 20;
    CHECK(sees(c, Vec2(19, 0)));
    CHECK(!sees(c, Vec2(21, 0)));     // beyond range
}

TEST_CASE(cone_respects_half_angle) {
    VisionCone c;
    c.forward = Vec2(1, 0);
    c.range = 50;
    c.halfAngleRad = 0.5;             // ~28.6 deg
    CHECK(sees(c, Vec2(10, 1)));      // ~5.7 deg off axis — inside
    CHECK(!sees(c, Vec2(1, 10)));     // ~84 deg off axis — outside
}

TEST_CASE(cone_forward_distance_sign) {
    VisionCone c;
    c.forward = Vec2(0, 1);           // looking +Z
    CHECK_APPROX(forwardDistance(c, Vec2(0, 12)), 12.0, 1e-9);   // ahead
    CHECK(forwardDistance(c, Vec2(0, -5)) < 0);                  // behind
}
