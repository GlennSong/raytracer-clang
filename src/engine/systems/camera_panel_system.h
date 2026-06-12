#ifndef RAYTRACER_ENGINE_CAMERA_PANEL_SYSTEM_H
#define RAYTRACER_ENGINE_CAMERA_PANEL_SYSTEM_H

#include "../system.h"
#include "camera_system.h"

namespace engine {

// ImGui authoring panel for placed cameras (docs/virtual-camera-plan.md,
// Phase 2): list/select/look-through/delete, transform + look-at editing, and
// the physical lens controls (focal length, f-stop, focus, aberrations). Also
// draws framing overlays (rule-of-thirds, cinematic letterbox) for virtual
// filming. Inert without RT_ENABLE_IMGUI, like DebugOverlaySystem (ADR-0011).
class CameraPanelSystem : public System {
public:
    explicit CameraPanelSystem(CameraSystem& cameras) : cameras(cameras) {}

    void render(FrameContext& ctx) override;

private:
    void drawFramingOverlay() const;

    CameraSystem& cameras;
    Entity selected;
    bool preview = true;   // selecting a camera looks through it (live editing)
    bool showThirds = false;
    int letterbox = 0;   // 0 = off, 1 = 2.39:1, 2 = 1.85:1, 3 = 16:9
};

}  // namespace engine

#endif
