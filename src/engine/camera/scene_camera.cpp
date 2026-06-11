#include "scene_camera.h"

#include <algorithm>
#include <cmath>

namespace engine {

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
    state.lens = camera.lens;
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
    Real yaw, pitch;
    yawPitchFromForward(forward, yaw, pitch);
    return orientationFromYawPitch(yaw, pitch);
}

void yawPitchFromForward(const Vec3& forward, Real& yawDegrees, Real& pitchDegrees) {
    if (forward.lengthSquared() <= 0.0) {
        yawDegrees = 0.0;
        pitchDegrees = 0.0;
        return;
    }
    Vec3 f = normalize(forward);
    yawDegrees = radiansToDegrees(std::atan2(f.x, -f.z));
    pitchDegrees = radiansToDegrees(std::asin(std::clamp(f.y, Real(-1.0), Real(1.0))));
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
