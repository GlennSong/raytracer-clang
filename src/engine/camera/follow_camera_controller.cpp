#include "follow_camera_controller.h"

#include <algorithm>

namespace engine {

Vec3 FollowCameraController::forward() const {
    Real yawRad = degreesToRadians(targetYaw + orbitYaw);
    Real pitchRad = degreesToRadians(orbitPitch);
    Real cp = std::cos(pitchRad);
    return Vec3(std::sin(yawRad) * cp,
                std::sin(pitchRad),
                -std::cos(yawRad) * cp);
}

void FollowCameraController::applyShoulderPreset() {
    distance = SHOULDER_ARM;
    minDistance = SHOULDER_ARM_MIN;
    maxDistance = SHOULDER_ARM_MAX;
    lateralOffset = SHOULDER_SIDE;
    targetHeight = SHOULDER_HEIGHT;
    orbitPitch = SHOULDER_PITCH;
}

void FollowCameraController::update(const CameraInput& input, Real /*dt*/) {
    // Orbit the camera around the target; pitch is clamped so it can't flip over
    // or bury itself in the ground.
    orbitYaw += input.lookYawDelta;
    orbitPitch = std::clamp(orbitPitch + input.lookPitchDelta, PITCH_MIN, PITCH_MAX);
    // Scroll dollies the chase distance (negative zoomDelta pushes out).
    distance = std::clamp(distance - input.zoomDelta * 0.8, minDistance, maxDistance);
}

CameraState FollowCameraController::cameraState(float aspect) const {
    // Aim above the target origin, shifted sideways in the follow frame (the
    // full yaw: heading + orbit) — the over-the-shoulder offset turns with it.
    Real yawRad = degreesToRadians(targetYaw + orbitYaw);
    Vec3 right(std::cos(yawRad), 0, std::sin(yawRad));
    Vec3 aim = targetPos + Vec3(0, targetHeight, 0) + right * lateralOffset;
    Vec3 dir = forward();

    CameraState state;
    state.position = aim - dir * distance;   // sit `distance` back along the look dir
    state.target = aim;
    state.up = Vec3(0, 1, 0);
    state.projection = orthoEnabled ? CameraProjection::Orthographic
                                    : CameraProjection::Perspective;
    state.fovDegrees = static_cast<float>(fovDegrees);
    Real fovRad = degreesToRadians(fovDegrees);
    state.orthoHeight = static_cast<float>(2.0 * distance * std::tan(fovRad * 0.5));
    state.aspectRatio = aspect;
    state.nearPlane = static_cast<float>(nearPlane);
    state.farPlane = static_cast<float>(farPlane);
    return state;
}

}  // namespace engine
