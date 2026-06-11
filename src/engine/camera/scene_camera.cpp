#include "scene_camera.h"

#include <algorithm>
#include <cmath>

namespace engine {

Real LensParams::verticalFovDegrees() const {
    if (focalLength <= 0.0) return 60.0;  // degenerate lens; nominal default
    return radiansToDegrees(2.0 * std::atan(sensorHeight / (2.0 * focalLength)));
}

Real LensParams::apertureDiameter() const {
    if (fStop <= 0.0) return 0.0;
    return (focalLength / fStop) / 1000.0;  // mm -> world units (meters)
}

CameraState sceneCameraState(const Transform& transform, const SceneCamera& camera,
                             float aspect) {
    CameraState state;
    state.position = transform.position;
    state.target = transform.position + transform.orientation.rotate(Vec3(0, 0, -1));
    state.up = transform.orientation.rotate(Vec3(0, 1, 0));
    state.projection = CameraProjection::Perspective;
    state.fovDegrees = static_cast<float>(camera.lens.verticalFovDegrees());
    state.aspectRatio = aspect;
    state.nearPlane = static_cast<float>(camera.lens.nearPlane);
    state.farPlane = static_cast<float>(camera.lens.farPlane);
    return state;
}

Quat orientationFromYawPitch(Real yawDegrees, Real pitchDegrees) {
    // Fly convention: +yaw turns forward toward +X, which is a negative
    // rotation about world Y; pitch is then applied about the local right axis.
    return Quat::fromAxisAngle(Vec3(0, 1, 0), -degreesToRadians(yawDegrees)) *
           Quat::fromAxisAngle(Vec3(1, 0, 0), degreesToRadians(pitchDegrees));
}

Quat orientationFromForward(const Vec3& forward) {
    if (forward.lengthSquared() <= 0.0) return Quat::identity();
    Vec3 f = normalize(forward);
    Real yaw = radiansToDegrees(std::atan2(f.x, -f.z));
    Real pitch = radiansToDegrees(std::asin(std::clamp(f.y, Real(-1.0), Real(1.0))));
    return orientationFromYawPitch(yaw, pitch);
}

std::vector<Entity> collectCameras(World& world) {
    std::vector<Entity> cameras;
    world.each<Transform, SceneCamera>(
        [&](Entity entity, Transform&, SceneCamera&) { cameras.push_back(entity); });
    std::sort(cameras.begin(), cameras.end());
    return cameras;
}

Entity cycleCamera(const std::vector<Entity>& cameras, Entity current,
                   int direction) {
    if (cameras.empty()) return Entity{};
    if (!current.valid())
        return direction >= 0 ? cameras.front() : cameras.back();

    auto it = std::find(cameras.begin(), cameras.end(), current);
    if (it == cameras.end()) return Entity{};

    if (direction >= 0)
        return (it + 1 == cameras.end()) ? Entity{} : *(it + 1);
    return (it == cameras.begin()) ? Entity{} : *(it - 1);
}

}  // namespace engine
