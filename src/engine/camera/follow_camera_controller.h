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
    // The first pose snaps (nothing to glide from), a disabled spring
    // (smoothTime <= 0) tracks exactly, and a teleport-sized jump snaps right
    // here — so a caller that never ticks update() (retarget + immediate
    // cameraState) still cuts to the new pose instead of framing stale ground.
    void setTarget(const Vec3& position, Real headingDegrees) {
        targetPos = position;
        targetYaw = headingDegrees;
        const Vec3 jump = position - smoothedPos;
        const bool teleport =
            (jump.x * jump.x + jump.y * jump.y + jump.z * jump.z) >
            snapDistance * snapDistance;
        if (!hasPose || teleport) {
            smoothedPos = position;
            smoothedYaw = headingDegrees;
        }
        if (posSmoothTime <= 0.0) smoothedPos = position;
        if (yawSmoothTime <= 0.0) smoothedYaw = headingDegrees;
        hasPose = true;
    }

    // SPRING (Glenn: "maybe we need to put it on a spring?"): the rig follows a
    // critically-damped smoothed copy of the target pose, not the raw feed. The
    // raw feed is quantized by the fixed physics step AND phase-lagged a frame
    // (camera writers run in the Update phase, renderables interpolate in the
    // render phase), so a stiff camera judders at speed no matter how good the
    // feed is — the spring low-passes all of it into a small, steady trail.
    // Critically damped = no overshoot; a car pulling away doesn't bounce the
    // camera. 0 disables (tests want exactness; editor rigs never move fast).
    Real posSmoothTime = 0.12;   // seconds to close ~63% of a position error
    Real yawSmoothTime = 0.22;   // heading eases slower — reads as cinematic
    // A target jump past this snaps instead of gliding (enter/exit, respawn,
    // spectate retarget — a camera sweeping across the city is worse than a cut).
    Real snapDistance = 6.0;

    void update(const CameraInput& input, Real dt) override;
    CameraState cameraState(float aspect) const override;

    // The unit direction the camera looks (smoothed heading + orbit, pitch).
    Vec3 forward() const;

    // The smoothed pose the rig actually frames (tests + debug overlays).
    const Vec3& followedPos() const { return smoothedPos; }
    Real followedYaw() const { return smoothedYaw; }

private:
    static constexpr Real PITCH_MIN = -80.0;
    static constexpr Real PITCH_MAX = 60.0;

    Vec3 smoothedPos{0, 0, 0};
    Vec3 posVelocity{0, 0, 0};
    Real smoothedYaw = 0.0;
    Real yawVelocity = 0.0;
    bool hasPose = false;   // first setTarget/update pair snaps
};

}  // namespace engine

#endif
