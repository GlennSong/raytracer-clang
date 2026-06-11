#ifndef RAYTRACER_ENGINE_SCENE_CAMERA_H
#define RAYTRACER_ENGINE_SCENE_CAMERA_H

#include "../components.h"
#include "../world.h"
#include <string>
#include <vector>

namespace engine {

// Physical lens parameterization for a placeable camera
// (docs/virtual-camera-plan.md). Framing is expressed the way a real camera is
// set up — focal length + sensor size — and FOV is derived, never stored. The
// aberration fields are carried from the start so both render paths read one
// parameter set; they take effect in later phases (thin-lens sampling in the
// offline tracer, post-process passes in the viewer).
struct LensParams {
    Real focalLength = 50.0;    // mm
    Real sensorHeight = 24.0;   // mm (full-frame 36x24 default)
    Real fStop = 8.0;           // aperture f-number
    Real focusDistance = 10.0;  // world units
    Real nearPlane = 0.1;
    Real farPlane = 1000.0;

    // Image-forming aberrations; 0 = ideal lens.
    Real distortionK1 = 0.0;          // Brown radial terms
    Real distortionK2 = 0.0;
    Real chromaticAberration = 0.0;   // lateral CA strength
    Real vignette = 0.0;

    Real verticalFovDegrees() const;
    Real apertureDiameter() const;    // world units; 0 = pinhole (fStop <= 0)
};

// A placeable, stationary camera. Pose comes from the entity's Transform:
// forward is the orientation's -Z, up its +Y (the engine's right-handed
// convention, ADR-0009). Placed cameras render from wherever their Transform
// says — no controller drives them.
struct SceneCamera {
    LensParams lens;
    std::string name;
    bool showGizmo = true;   // draw the camera-body gizmo when not looked through
};

// View state for rendering through a placed camera.
CameraState sceneCameraState(const Transform& transform, const SceneCamera& camera,
                             float aspect);

// Orientation from fly-camera-style angles: yaw about world Y (0 looks down
// -Z), pitch about the camera's right axis. Matches FlyCameraController's
// forward() convention, so a camera placed at the editor view sees the same
// framing.
Quat orientationFromYawPitch(Real yawDegrees, Real pitchDegrees);

// Roll-free orientation whose -Z points along `forward` (need not be
// normalized). Zero-length input yields identity.
Quat orientationFromForward(const Vec3& forward);

// All entities carrying a Transform + SceneCamera, sorted by handle so the
// cycling order is stable regardless of pool order.
std::vector<Entity> collectCameras(World& world);

// The next view in the cycle editor -> cameras[0] -> ... -> editor. An invalid
// entity stands for the editor view; `direction` is +1 / -1. A `current` that
// is not in the list (e.g. just destroyed) returns to the editor.
Entity cycleCamera(const std::vector<Entity>& cameras, Entity current,
                   int direction);

}  // namespace engine

#endif
