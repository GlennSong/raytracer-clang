#include "fly_camera_controller.h"

#include <algorithm>

namespace engine {

// Convention: yaw rotates about world Y, pitch about the camera's right axis.
// At yaw = pitch = 0 the camera looks down -Z and right is +X.
Vec3 FlyCameraController::forward() const {
    Real yawRad = degreesToRadians(yaw);
    Real pitchRad = degreesToRadians(pitch);
    Real cp = std::cos(pitchRad);
    return Vec3(std::sin(yawRad) * cp,
                std::sin(pitchRad),
                -std::cos(yawRad) * cp);
}

Vec3 FlyCameraController::right() const {
    Real yawRad = degreesToRadians(yaw);
    return Vec3(std::cos(yawRad), 0.0, std::sin(yawRad));
}

void FlyCameraController::update(const CameraInput& input, Real dt) {
    yaw += input.lookYawDelta;
    pitch += input.lookPitchDelta;
    pitch = std::clamp(pitch, -PITCH_LIMIT, PITCH_LIMIT);

    Real speed = moveSpeed * dt * (input.boost ? boostMultiplier : Real(1.0));
    Vec3 f = forward();
    Vec3 r = right();
    Vec3 worldUp(0, 1, 0);

    eye += f * (input.moveForward * speed);
    eye += r * (input.moveRight * speed);
    eye += worldUp * (input.moveUp * speed);
}

CameraState FlyCameraController::cameraState(float aspect) const {
    CameraState state;
    state.position = eye;
    state.target = eye + forward();
    state.up = Vec3(0, 1, 0);
    state.projection = orthoEnabled ? CameraProjection::Orthographic
                                    : CameraProjection::Perspective;
    state.fovDegrees = static_cast<float>(fovDegrees);
    // Nominal framing for the ortho case (fly is normally perspective).
    Real fovRad = degreesToRadians(fovDegrees);
    state.orthoHeight = static_cast<float>(2.0 * 10.0 * std::tan(fovRad * 0.5));
    state.aspectRatio = aspect;
    state.nearPlane = 0.1f;
    state.farPlane = 1000.0f;
    return state;
}

}  // namespace engine

