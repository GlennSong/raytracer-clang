#ifndef RAYTRACER_ENGINE_FLY_CAMERA_CONTROLLER_H
#define RAYTRACER_ENGINE_FLY_CAMERA_CONTROLLER_H

#include "camera_controller.h"

namespace engine {

// Free-fly / FPS camera: the camera is a free agent in space. Look deltas rotate
// the view (yaw/pitch); move axes translate along the facing direction (forward/
// strafe) and world up. The camera for traversing large/generated worlds
// (ROADMAP 2.2).
class FlyCameraController : public CameraController {
public:
    Vec3 eye{0, 1.5, 8};
    Real yaw = 0.0;     // degrees; 0 looks down -Z
    Real pitch = 0.0;   // degrees
    Real fovDegrees = 60.0;
    Real moveSpeed = 14.0;   // scroll wheel throttles this live
    Real boostMultiplier = 6.0;
    Real nearPlane = 0.1;
    Real farPlane = 8000.0;   // large enough for distant LOD terrain (view distance)
    bool positionLocked = false;
    // FPS walk: move on the horizontal plane (forward ignores pitch) at a fixed
    // eye height, no vertical axis. Off = free 6-DOF fly. Look is free either way.
    bool grounded = false;

    void update(const CameraInput& input, Real dt) override;
    CameraState cameraState(float aspect) const override;

    Vec3 forward() const;
    Vec3 right() const;

private:
    static constexpr Real PITCH_LIMIT = 89.0;
};


}  // namespace engine

#endif
