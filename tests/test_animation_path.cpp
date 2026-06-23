#include "test_framework.h"

#include "../src/engine/animation_path.h"
#include <cmath>

using namespace engine;

namespace {
double dist(const Vec3& a, const Vec3& b) { return std::sqrt((a - b).lengthSquared()); }

AnimationPath straightPath() {
    AnimationPath p;
    p.curve.addKnot(Vec3(0, 0, 0));
    p.curve.addKnot(Vec3(20, 0, 0));    // 20 m along +X
    p.duration = 4.0;
    return p;
}
}

TEST_CASE(anim_path_constant_speed_midpoint) {
    AnimationPath p = straightPath();
    p.loop = LoopMode::Once;
    PathPose half = animationPose(p, 2.0);              // halfway in time
    CHECK_APPROX(half.u, 0.5, 1e-9);
    CHECK(dist(half.position, Vec3(10, 0, 0)) < 1e-2);  // halfway in distance
}

TEST_CASE(anim_path_constant_speed_on_a_curve) {
    AnimationPath p;
    p.curve.addKnot(Vec3(0, 0, 0));
    p.curve.addKnot(Vec3(10, 0, 10));
    p.curve.addKnot(Vec3(20, 0, 0));
    p.duration = 2.0;
    double L = p.curve.length();
    PathPose mid = animationPose(p, 1.0);
    CHECK(dist(mid.position, p.curve.atDistance(L * 0.5)) < 1e-2);   // arc-length, not param
}

TEST_CASE(anim_path_loop_modes) {
    AnimationPath p = straightPath();                  // duration 4
    p.loop = LoopMode::Loop;
    CHECK_APPROX(animationPose(p, 6.0).u, 0.5, 1e-9);  // 1.5 cycles -> 0.5
    p.loop = LoopMode::PingPong;
    CHECK_APPROX(animationPose(p, 6.0).u, 0.5, 1e-9);  // 1.5 -> back down to 0.5
    CHECK_APPROX(animationPose(p, 4.0).u, 1.0, 1e-9);  // 1.0 -> the far end
    p.loop = LoopMode::Once;
    CHECK_APPROX(animationPose(p, 20.0).u, 1.0, 1e-9); // clamps at the end
}

TEST_CASE(anim_path_faces_along_travel) {
    AnimationPath p = straightPath();                  // travels +X
    p.faceTangent = true;
    PathPose pose = animationPose(p, 2.0);
    Vec3 fwd = pose.orientation.rotate(Vec3(0, 0, 1)); // local forward -> world
    CHECK(dist(fwd, Vec3(1, 0, 0)) < 1e-4);            // forward points along +X
    Vec3 up = pose.orientation.rotate(Vec3(0, 1, 0));
    CHECK(up.y > 0.9);                                  // up stays roughly world-up
}

TEST_CASE(anim_path_no_facing_is_identity) {
    AnimationPath p = straightPath();
    p.faceTangent = false;
    PathPose pose = animationPose(p, 1.0);
    Vec3 fwd = pose.orientation.rotate(Vec3(0, 0, 1));
    CHECK(dist(fwd, Vec3(0, 0, 1)) < 1e-9);            // unrotated
}
