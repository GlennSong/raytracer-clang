#ifndef RAYTRACER_ENGINE_CAMERA_SYSTEM_H
#define RAYTRACER_ENGINE_CAMERA_SYSTEM_H

#include "../system.h"
#include "../camera/orbit_camera_controller.h"
#include "../camera/fly_camera_controller.h"
#include "../camera/follow_camera_controller.h"
#include "../camera/scene_camera.h"
#include "../properties.h"

namespace engine {

// CameraPoseRequest (ADR-0253): put the free camera at a pose and detach it from the player (it then wins even in
// play mode). The socket's `camera` and a teleport with no player publish it.
struct CameraPoseRequest {
    Vec3 eye;
    Real pitch = 0, yaw = 0;   // degrees; yaw 0 looks toward -Z
};
inline void describeProperties(CameraPoseRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("eye").units("m"), r.eye);
    v.field(FieldMeta("pitch").units("deg"), r.pitch);
    v.field(FieldMeta("yaw").units("deg"), r.yaw);
}
// CameraDetachRequest: the F key's toggle, from outside. mode 1 = detach, 0 = re-attach (the player comes to the
// camera), 2 = toggle.
struct CameraDetachRequest {
    int mode = 2;
};
inline void describeProperties(CameraDetachRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("mode").range(0, 2), r.mode);
}

// Drives the active camera controller from the action layer (keyboard, mouse,
// and gamepad) each frame and publishes the resulting view into the RenderView.
// Holds both an orbit and a fly controller and toggles between them at runtime
// (ROADMAP 2.2). Loads/saves pose and mode via Settings.
//
// Also the view selector for placed SceneCamera entities
// (docs/virtual-camera-plan.md): C spawns a camera at the current editor view,
// V/B cycle editor -> cam1 -> ... -> editor, Backspace (or Tab) returns to the
// editor. While a placed camera is active the controllers receive no input —
// placed cameras are stationary.
class CameraSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;   // the chase rig, after the fixed steps
    void onStop(FrameContext& ctx) override;

    FlyCameraController& flyController() { return fly; }
    // City Planner presets (P7.3): the ORBIT controller is the ortho-capable
    // plan-view camera (its orthoHeight scales with distance; fly-ortho is a
    // fixed ~10 m and unusable for plan views), and setActiveController is
    // the programmatic twin of the Tab toggle so presets can switch between
    // orbit plan views and the free fly camera. Inline, like everything the
    // bridge calls.
    OrbitCameraController& orbitController() { return orbit; }
    void setActiveController(bool flyOn) {
        flyActive = flyOn;
        active = flyOn ? static_cast<CameraController*>(&fly)
                       : static_cast<CameraController*>(&orbit);
    }
    bool flyControllerActive() const { return flyActive; }
    Entity activeSceneCamera() const { return activeCamera; }

    // Chase-camera control (ADR-0059): while a target is set, the view follows it
    // (third-person), overriding the editor controllers and placed cameras. Used
    // by VehicleSystem on vehicle enter/exit.
    void setFollowTarget(Entity target) { followTarget = target; }
    void clearFollowTarget() { followTarget = Entity{}; }
    // How far behind and how high the chase sits: a car's 8 m / 1.2 m, a bike's closer (ADR-0235).
    void setFollowFraming(Real distance, Real aimHeight) {
        follow.distance = distance;
        follow.targetHeight = aimHeight;
    }
    Entity followingEntity() const { return followTarget; }

    // For the camera panel (and future pose sources): request a view switch
    // (validity is re-checked next update; an invalid handle means the editor)
    // and place a camera at the current editor framing.
    void setActiveSceneCamera(Entity camera) { activeCamera = camera; }
    Entity placeCameraAtView(FrameContext& ctx);

private:
    EventBus::Subscription onPose_, onDetach_;
    bool posePending_ = false;
    CameraPoseRequest pose_;
    int detachPending_ = -1;   // a CameraDetachRequest's mode, until update takes it
    void registerBindings(InputMap& actions) const;
    CameraInput gatherInput(FrameContext& ctx) const;
    Entity placeCamera(FrameContext& ctx, float aspect);
    void ensureGizmos(FrameContext& ctx);

    OrbitCameraController orbit;
    FlyCameraController fly;
    FollowCameraController follow;
    Entity followTarget;        // valid => chase this entity (vehicle), overrides all
    CameraController* active = &orbit;
    bool flyActive = false;
    bool freeLook = false;      // detached freecam: mouse looks without a button
    bool detachEnabled = true;  // editor states repurpose F (frame selected)

    Entity activeCamera;        // invalid => the editor controllers drive the view
    MeshHandle gizmoMesh;       // camera-body mesh, uploaded on first placement
    int placedCount = 0;        // for default names ("Camera 1", ...)
    std::string storePath;      // camera persistence sidecar; empty = no store

    Real mouseSensitivity = 0.3;   // degrees per pixel
    Real stickLookSpeed = 120.0;   // degrees/sec at full stick deflection
};


}  // namespace engine

#endif
