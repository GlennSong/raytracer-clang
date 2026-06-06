#ifndef RAYTRACER_ENGINE_CAMERA_SYSTEM_H
#define RAYTRACER_ENGINE_CAMERA_SYSTEM_H

#include "../system.h"
#include "../camera/orbit_camera_controller.h"
#include "../camera/fly_camera_controller.h"

namespace engine {

// Drives the active camera controller from the action layer (keyboard, mouse,
// and gamepad) each frame and publishes the resulting view into the RenderView.
// Holds both an orbit and a fly controller and toggles between them at runtime
// (ROADMAP 2.2). Loads/saves pose and mode via Settings.
class CameraSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    FlyCameraController& flyController() { return fly; }

private:
    void registerBindings(InputMap& actions) const;
    CameraInput gatherInput(FrameContext& ctx) const;

    OrbitCameraController orbit;
    FlyCameraController fly;
    CameraController* active = &orbit;
    bool flyActive = false;

    Real mouseSensitivity = 0.3;   // degrees per pixel
    Real stickLookSpeed = 120.0;   // degrees/sec at full stick deflection
};


}  // namespace engine

#endif
