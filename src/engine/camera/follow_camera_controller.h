#ifndef RAYTRACER_ENGINE_FOLLOW_CAMERA_CONTROLLER_H
#define RAYTRACER_ENGINE_FOLLOW_CAMERA_CONTROLLER_H

#include "camera_controller.h"

namespace engine {

// Third-person chase camera (ADR-0059): sits behind and above a tracked target
// (a driven vehicle, or the on-foot player over the shoulder) and looks at it.
// The owning system feeds the target's pose each frame via setTarget(); look
// input orbits the camera around the target (right stick / mouse) and zoom
// dollies in/out. Pure and window-free like the fly/orbit controllers, so it
// is unit-tested without a window.
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
    // Sideways shift of the aim point IN THE FOLLOW FRAME (heading + orbit yaw),
    // metres, + = the target's right — the over-the-shoulder framing. It turns
    // with the target, so the offset stays glued to the same shoulder.
    Real lateralOffset = 0.0;
    Real fovDegrees = 65.0;
    Real nearPlane = 0.1;
    Real farPlane = 8000.0;

    // Over-the-shoulder preset: third-person ON FOOT. A much shorter arm than
    // the vehicle chase default, the aim a bit above head height (the target
    // origin is the capsule CENTRE, ~mid-body), and a slight right-shoulder
    // offset so the player frames just left of centre.
    static constexpr Real SHOULDER_ARM = 3.0;        // metres behind the aim
    static constexpr Real SHOULDER_ARM_MIN = 1.5;    // zoom clamps
    static constexpr Real SHOULDER_ARM_MAX = 6.0;
    static constexpr Real SHOULDER_SIDE = 0.45;      // right-shoulder offset (m)
    static constexpr Real SHOULDER_HEIGHT = 0.95;    // aim above the target origin (m)
    static constexpr Real SHOULDER_PITCH = -10.0;    // default tilt (degrees)
    void applyShoulderPreset();

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
