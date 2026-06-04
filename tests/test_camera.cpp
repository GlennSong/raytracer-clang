#include "test_framework.h"

#include "../src/engine/camera/fly_camera_controller.h"
#include "../src/engine/camera/orbit_camera_controller.h"

namespace {
constexpr Real EPS = 1e-6;

CameraInput move(Real forward, Real right, Real up) {
    CameraInput in;
    in.moveForward = forward;
    in.moveRight = right;
    in.moveUp = up;
    return in;
}

CameraInput look(Real yawDelta, Real pitchDelta) {
    CameraInput in;
    in.lookYawDelta = yawDelta;
    in.lookPitchDelta = pitchDelta;
    return in;
}
}  // namespace

TEST_CASE(fly_default_faces_negative_z) {
    FlyCameraController fly;
    Vec3 f = fly.forward();
    CHECK_APPROX(f.x, 0, EPS);
    CHECK_APPROX(f.y, 0, EPS);
    CHECK_APPROX(f.z, -1, EPS);
}

TEST_CASE(fly_moves_along_facing) {
    FlyCameraController fly;
    fly.eye = Vec3(0, 0, 0);
    fly.moveSpeed = 5.0;

    fly.update(move(1, 0, 0), 1.0);   // forward 1s at speed 5 -> -Z by 5
    CHECK_APPROX(fly.eye.z, -5, EPS);

    fly.update(move(0, 1, 0), 1.0);   // strafe right -> +X by 5
    CHECK_APPROX(fly.eye.x, 5, EPS);

    fly.update(move(0, 0, 1), 1.0);   // up -> +Y by 5
    CHECK_APPROX(fly.eye.y, 5, EPS);
}

TEST_CASE(fly_boost_multiplies_speed) {
    FlyCameraController fly;
    fly.eye = Vec3(0, 0, 0);
    fly.moveSpeed = 5.0;
    fly.boostMultiplier = 4.0;

    CameraInput in = move(1, 0, 0);
    in.boost = true;
    fly.update(in, 1.0);              // 5 * 4 = 20 along -Z
    CHECK_APPROX(fly.eye.z, -20, EPS);
}

TEST_CASE(fly_yaw_rotates_facing) {
    FlyCameraController fly;
    fly.update(look(90, 0), 1.0);     // yaw 90 deg -> face +X
    Vec3 f = fly.forward();
    CHECK_APPROX(f.x, 1, EPS);
    CHECK_APPROX(f.z, 0, EPS);

    fly.eye = Vec3(0, 0, 0);
    fly.moveSpeed = 3.0;
    fly.update(move(1, 0, 0), 1.0);   // forward now goes +X
    CHECK_APPROX(fly.eye.x, 3, EPS);
}

TEST_CASE(fly_pitch_is_clamped) {
    FlyCameraController fly;
    fly.update(look(0, 200), 1.0);    // way past vertical
    CHECK_APPROX(fly.pitch, 89, EPS);
    fly.update(look(0, -400), 1.0);
    CHECK_APPROX(fly.pitch, -89, EPS);
}

TEST_CASE(fly_camera_state_target_is_ahead) {
    FlyCameraController fly;
    fly.eye = Vec3(1, 2, 3);
    CameraState s = fly.cameraState(1.6f);
    CHECK_APPROX(s.position.x, 1, EPS);
    CHECK_APPROX(s.position.y, 2, EPS);
    CHECK_APPROX(s.position.z, 3, EPS);
    // target = eye + forward(-Z)
    CHECK_APPROX(s.target.z, 2, EPS);
    CHECK(s.projection == CameraProjection::Perspective);

    fly.setOrthographic(true);
    CHECK(fly.cameraState(1.6f).projection == CameraProjection::Orthographic);
}

TEST_CASE(orbit_zoom_changes_distance) {
    OrbitCameraController orbit;
    orbit.target = Vec3(0, 0, 0);
    orbit.distance = 8.0;
    orbit.zoomSpeed = 1.0;

    CameraInput in;
    in.zoomDelta = 1.0;
    orbit.update(in, 1.0);            // distance 8 - 1 = 7
    Vec3 p = orbit.position();
    CHECK_APPROX(p.length(), 7, EPS);  // |position - target(origin)| == distance
}

TEST_CASE(orbit_zoom_clamped_to_minimum) {
    OrbitCameraController orbit;
    orbit.target = Vec3(0, 0, 0);
    orbit.distance = 2.0;

    CameraInput in;
    in.zoomDelta = 1000.0;           // way past the target
    orbit.update(in, 1.0);
    CHECK(orbit.distance >= 0.5);
    CHECK_APPROX(orbit.distance, 0.5, EPS);
}

TEST_CASE(orbit_move_pans_target) {
    OrbitCameraController orbit;
    orbit.target = Vec3(0, 0, 0);
    orbit.yaw = 0.0;
    orbit.panSpeed = 3.0;

    orbit.update(move(1, 0, 0), 1.0);  // forward-ground at yaw 0 is -Z
    CHECK_APPROX(orbit.target.z, -3, EPS);

    orbit.update(move(0, 0, 1), 1.0);  // up pans target.y
    CHECK_APPROX(orbit.target.y, 3, EPS);
}

TEST_CASE(orbit_projection_toggle) {
    OrbitCameraController orbit;
    CHECK(orbit.cameraState(1.0f).projection == CameraProjection::Perspective);
    orbit.setOrthographic(true);
    CameraState s = orbit.cameraState(1.0f);
    CHECK(s.projection == CameraProjection::Orthographic);
    // ortho height tracks the orbit distance for matched framing.
    CHECK(s.orthoHeight > 0.0f);
}
