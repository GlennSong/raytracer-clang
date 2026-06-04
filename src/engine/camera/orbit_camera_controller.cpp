#include "orbit_camera_controller.h"

#include <algorithm>

void OrbitCameraController::update(const CameraInput& input, Real dt) {
    yaw += input.lookYawDelta;
    pitch += input.lookPitchDelta;
    pitch = std::clamp(pitch, Real(-89.0), Real(89.0));

    // Pan the target across the ground plane relative to the current heading.
    Real yawRad = degreesToRadians(yaw);
    Vec3 forwardGround(-std::sin(yawRad), 0.0, -std::cos(yawRad));
    Vec3 rightGround(std::cos(yawRad), 0.0, -std::sin(yawRad));

    Real pan = panSpeed * dt;
    target += forwardGround * (input.moveForward * pan);
    target += rightGround * (input.moveRight * pan);
    target.y += input.moveUp * pan;

    distance -= input.zoomDelta * zoomSpeed;
    distance = std::max(Real(0.5), distance);
}

Vec3 OrbitCameraController::position() const {
    Real yawRad = degreesToRadians(yaw);
    Real pitchRad = degreesToRadians(pitch);
    return Vec3(
        target.x + distance * std::cos(pitchRad) * std::sin(yawRad),
        target.y + distance * std::sin(pitchRad),
        target.z + distance * std::cos(pitchRad) * std::cos(yawRad));
}

CameraState OrbitCameraController::cameraState(float aspect) const {
    CameraState state;
    state.position = position();
    state.target = target;
    state.up = Vec3(0, 1, 0);
    state.projection = orthoEnabled ? CameraProjection::Orthographic
                                    : CameraProjection::Perspective;
    state.fovDegrees = static_cast<float>(fovDegrees);
    // Match perspective framing at the orbit distance so the subject stays the
    // same size when toggling projection, and zoom keeps working in ortho.
    Real fovRad = degreesToRadians(fovDegrees);
    state.orthoHeight = static_cast<float>(2.0 * distance * std::tan(fovRad * 0.5));
    state.aspectRatio = aspect;
    state.nearPlane = 0.1f;
    state.farPlane = 1000.0f;
    return state;
}
