#include "orbit_camera.h"
#include <cmath>
#include <algorithm>

OrbitCamera::OrbitCamera()
    : target(0, 0, 0), distance(5.0f), yaw(0.0f), pitch(20.0f),
      fovDegrees(60.0f), moveSpeed(3.0f), rotateSpeed(0.3f), zoomSpeed(1.0f),
      orthographic(false) {}

void OrbitCamera::update(const InputState& input, double deltaTime) {
    if (input.mouseLeftDown) {
        yaw -= static_cast<float>(input.mouseDeltaX) * rotateSpeed;
        pitch -= static_cast<float>(input.mouseDeltaY) * rotateSpeed;
        pitch = std::clamp(pitch, -89.0f, 89.0f);
    }

    if (input.mouseRightDown) {
        float panScale = distance * 0.002f;
        float yawRad = static_cast<float>(degreesToRadians(yaw));
        float rightX = std::cos(yawRad);
        float rightZ = -std::sin(yawRad);

        target.x -= rightX * static_cast<float>(input.mouseDeltaX) * panScale;
        target.z -= rightZ * static_cast<float>(input.mouseDeltaX) * panScale;
        target.y += static_cast<float>(input.mouseDeltaY) * panScale;
    }

    distance -= static_cast<float>(input.scrollDelta) * zoomSpeed;
    distance = std::max(0.5f, distance);

    float speed = moveSpeed * static_cast<float>(deltaTime);
    if (input.keyShift) speed *= 3.0f;

    float yawRad = static_cast<float>(degreesToRadians(yaw));
    float forwardX = -std::sin(yawRad);
    float forwardZ = -std::cos(yawRad);
    float rightX = std::cos(yawRad);
    float rightZ = -std::sin(yawRad);

    if (input.keyW) { target.x += forwardX * speed; target.z += forwardZ * speed; }
    if (input.keyS) { target.x -= forwardX * speed; target.z -= forwardZ * speed; }
    if (input.keyA) { target.x -= rightX * speed; target.z -= rightZ * speed; }
    if (input.keyD) { target.x += rightX * speed; target.z += rightZ * speed; }
    if (input.keyQ) { target.y -= speed; }
    if (input.keyE) { target.y += speed; }
}

Vec3 OrbitCamera::getPosition() const {
    float yawRad = static_cast<float>(degreesToRadians(yaw));
    float pitchRad = static_cast<float>(degreesToRadians(pitch));

    float x = target.x + distance * std::cos(pitchRad) * std::sin(yawRad);
    float y = target.y + distance * std::sin(pitchRad);
    float z = target.z + distance * std::cos(pitchRad) * std::cos(yawRad);

    return Vec3(x, y, z);
}

CameraState OrbitCamera::getCameraState(float aspectRatio) const {
    CameraState state;
    state.position = getPosition();
    state.target = target;
    state.up = Vec3(0, 1, 0);
    state.projection = orthographic ? CameraProjection::Orthographic
                                    : CameraProjection::Perspective;
    state.fovDegrees = fovDegrees;
    // Match the perspective framing at the orbit distance so toggling keeps the
    // subject the same size, and zoom (distance) keeps working in ortho.
    float fovRad = static_cast<float>(degreesToRadians(fovDegrees));
    state.orthoHeight = 2.0f * distance * std::tan(fovRad * 0.5f);
    state.aspectRatio = aspectRatio;
    state.nearPlane = 0.1f;
    state.farPlane = 1000.0f;
    return state;
}
