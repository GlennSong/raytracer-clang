#ifndef RAYTRACER_ENGINE_CAMERA_SYSTEM_H
#define RAYTRACER_ENGINE_CAMERA_SYSTEM_H

#include "../system.h"
#include "../../renderer/orbit_camera.h"

// Drives an orbit camera from input each frame and publishes the resulting
// view into the frame's RenderView. Loads/saves its pose via Settings.
class CameraSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    OrbitCamera camera;
};

#endif
