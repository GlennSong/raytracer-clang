#include "test_framework.h"

#include "../src/engine/camera/follow_camera_controller.h"

using namespace engine;

namespace {
constexpr Real EPS = 1e-6;

CameraInput look(Real yawDelta, Real pitchDelta) {
    CameraInput in;
    in.lookYawDelta = yawDelta;
    in.lookPitchDelta = pitchDelta;
    return in;
}
}  // namespace

TEST_CASE(follow_sits_behind_and_above_target) {
    FollowCameraController cam;
    cam.setTarget(Vec3(0, 0, 0), 0.0);   // target at origin, facing -Z
    CameraState s = cam.cameraState(1.0f);
    // The aim point is the target plus a small height.
    CHECK_APPROX(s.target.x, 0.0, EPS);
    CHECK_APPROX(s.target.y, cam.targetHeight, EPS);
    // Camera rides behind (+Z, since the target faces -Z) and above the aim.
    CHECK(s.position.z > 0.0);
    CHECK(s.position.y > s.target.y);
}

TEST_CASE(follow_tracks_target_position) {
    FollowCameraController cam;
    cam.setTarget(Vec3(0, 0, 0), 0.0);
    Vec3 a = cam.cameraState(1.0f).position;
    cam.setTarget(Vec3(100, 0, 0), 0.0);
    Vec3 b = cam.cameraState(1.0f).position;
    // The whole rig shifts with the target (x advances by ~100).
    CHECK_APPROX(b.x - a.x, 100.0, 1e-3);
}

TEST_CASE(follow_orbits_with_look_input) {
    FollowCameraController cam;
    cam.setTarget(Vec3(0, 0, 0), 0.0);
    Vec3 before = cam.cameraState(1.0f).position;
    cam.update(look(90.0, 0.0), 1.0);    // orbit a quarter turn
    Vec3 after = cam.cameraState(1.0f).position;
    // Orbiting 90 deg swings the camera off the +Z axis onto the X axis.
    CHECK(std::fabs(after.x - before.x) > 1.0);
}

TEST_CASE(follow_pitch_is_clamped) {
    FollowCameraController cam;
    cam.update(look(0.0, -1000.0), 1.0);   // slam the pitch down
    CHECK(cam.orbitPitch >= -80.0 - EPS);
    cam.update(look(0.0, 2000.0), 1.0);    // and up
    CHECK(cam.orbitPitch <= 60.0 + EPS);
}

TEST_CASE(follow_zoom_clamps_distance) {
    FollowCameraController cam;
    CameraInput zoomIn;
    zoomIn.zoomDelta = 1000.0;            // dolly hard in
    cam.update(zoomIn, 1.0);
    CHECK(cam.distance >= cam.minDistance - EPS);
    CameraInput zoomOut;
    zoomOut.zoomDelta = -1000.0;          // and hard out
    cam.update(zoomOut, 1.0);
    CHECK(cam.distance <= cam.maxDistance + EPS);
}

TEST_CASE(shoulder_preset_offsets_in_the_follow_frame) {
    // The over-the-shoulder preset (third person ON FOOT): the aim point rides
    // a bit above head height and a little to the target's RIGHT, and that
    // offset lives in the follow frame — it turns with the heading.
    FollowCameraController cam;
    cam.applyShoulderPreset();
    cam.setTarget(Vec3(0, 0, 0), 0.0);           // facing -Z: right = +X
    CameraState s = cam.cameraState(1.0f);
    CHECK_APPROX(s.target.x, FollowCameraController::SHOULDER_SIDE, EPS);
    CHECK_APPROX(s.target.y, FollowCameraController::SHOULDER_HEIGHT, EPS);
    CHECK_APPROX(s.target.z, 0.0, EPS);
    CHECK(s.position.z > 0.0);                   // behind a -Z-facing target

    cam.setTarget(Vec3(0, 0, 0), 90.0);          // facing +X: right = +Z
    s = cam.cameraState(1.0f);
    CHECK_APPROX(s.target.x, 0.0, EPS);
    CHECK_APPROX(s.target.z, FollowCameraController::SHOULDER_SIDE, EPS);
}

TEST_CASE(shoulder_preset_arm_is_short_and_respected) {
    // The shoulder arm is the eye-to-aim distance exactly, and it is much
    // shorter than the vehicle chase default.
    FollowCameraController car;                  // vehicle chase defaults
    FollowCameraController onFoot;
    onFoot.applyShoulderPreset();
    CHECK(onFoot.distance < car.distance);
    onFoot.setTarget(Vec3(3, 1, -7), 30.0);
    CameraState s = onFoot.cameraState(1.0f);
    CHECK_APPROX((s.position - s.target).length(),
                 FollowCameraController::SHOULDER_ARM, 1e-6);
    // Zoom clamps re-anchor to the preset's shorter range.
    CameraInput zoomOut;
    zoomOut.zoomDelta = -1000.0;
    onFoot.update(zoomOut, 1.0);
    CHECK(onFoot.distance <= FollowCameraController::SHOULDER_ARM_MAX + EPS);
}

TEST_CASE(follow_heading_rotates_rig) {
    FollowCameraController cam;
    cam.setTarget(Vec3(0, 0, 0), 0.0);
    Vec3 facingNorth = cam.cameraState(1.0f).position;   // behind a -Z-facing target
    cam.setTarget(Vec3(0, 0, 0), 90.0);                  // target now faces +X-ish
    Vec3 facingEast = cam.cameraState(1.0f).position;
    // Behind-ness follows the heading: the camera moved to a different quadrant.
    CHECK((facingEast - facingNorth).length() > 1.0);
}
