#ifndef RAYTRACER_ENGINE_FOLLOW_CAMERA_CONTROLLER_H
#define RAYTRACER_ENGINE_FOLLOW_CAMERA_CONTROLLER_H

#include "camera_controller.h"

namespace engine {

// Third-person chase camera (ADR-0057): sits behind and above a tracked target
// (a driven vehicle) and looks at it. The owning system feeds the target's pose
// each frame via setTarget(); look input orbits the camera around the target
// (right stick / mouse) and zoom dollies in/out. Pure and window-free like the
// fly/orbit controllers, so it is unit-tested without a window.
//
// Convention matches FlyCameraController: yaw about world Y (0 looks down -Z),
// pitch about the camera's right axis. A negative pitch tips the look down, which
// places the eye ABOVE the target — the natural chase framing.
class FollowCameraController : public CameraController {
public:
    // Target pose (set by the system from the vehicle's Transform each frame).
    Vec3 targetPos{0, 0, 0};
    Real targetYaw = 0.0;        // vehicle heading, degrees (FlyCamera convention)

    // Orbit offset the player controls, relative to the target's heading.
    Real orbitYaw = 0.0;         // degrees around the target
    Real orbitPitch = -12.0;     // degrees; negative tips down -> eye rides high

    Real distance = 8.0;         // metres behind the target
    Real minDistance = 3.0;
    Real maxDistance = 20.0;
    Real targetHeight = 1.2;     // aim point above the target origin (m)
    Real fovDegrees = 65.0;
    Real nearPlane = 0.1;
    Real farPlane = 8000.0;

    // Feed the tracked pose. `headingDegrees` uses the FlyCamera yaw convention.
    void setTarget(const Vec3& position, Real headingDegrees) {
        targetPos = position;
        targetYaw = headingDegrees;
    }

    void update(const CameraInput& input, Real dt) override;
    CameraState cameraState(float aspect) const override;

    // The unit direction the camera looks (target heading + orbit, with pitch).
    Vec3 forward() const;

private:
    static constexpr Real PITCH_MIN = -80.0;
    static constexpr Real PITCH_MAX = 60.0;
};

}  // namespace engine

#endif
