#ifndef RAYTRACER_ENGINE_CAMERA_SYSTEM_H
#define RAYTRACER_ENGINE_CAMERA_SYSTEM_H

#include "../system.h"
#include "../camera/orbit_camera_controller.h"
#include "../camera/fly_camera_controller.h"
#include "../camera/scene_camera.h"

namespace engine {

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
    void onStop(FrameContext& ctx) override;

    FlyCameraController& flyController() { return fly; }
    Entity activeSceneCamera() const { return activeCamera; }

    // For the camera panel (and future pose sources): request a view switch
    // (validity is re-checked next update; an invalid handle means the editor)
    // and place a camera at the current editor framing.
    void setActiveSceneCamera(Entity camera) { activeCamera = camera; }
    Entity placeCameraAtView(FrameContext& ctx);

private:
    void registerBindings(InputMap& actions) const;
    CameraInput gatherInput(FrameContext& ctx) const;
    Entity placeCamera(FrameContext& ctx, float aspect);
    void ensureGizmos(FrameContext& ctx);

    OrbitCameraController orbit;
    FlyCameraController fly;
    CameraController* active = &orbit;
    bool flyActive = false;

    Entity activeCamera;        // invalid => the editor controllers drive the view
    MeshHandle gizmoMesh;       // camera-body mesh, uploaded on first placement
    int placedCount = 0;        // for default names ("Camera 1", ...)
    std::string storePath;      // camera persistence sidecar; empty = no store

    Real mouseSensitivity = 0.3;   // degrees per pixel
    Real stickLookSpeed = 120.0;   // degrees/sec at full stick deflection
};


}  // namespace engine

#endif
