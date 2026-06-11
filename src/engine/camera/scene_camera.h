#ifndef RAYTRACER_ENGINE_SCENE_CAMERA_H
#define RAYTRACER_ENGINE_SCENE_CAMERA_H

#include "../components.h"
#include "../world.h"
#include "../../lens_params.h"
#include <string>
#include <vector>

namespace engine {

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
