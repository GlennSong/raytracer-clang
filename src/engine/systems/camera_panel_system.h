#ifndef RAYTRACER_ENGINE_CAMERA_PANEL_SYSTEM_H
#define RAYTRACER_ENGINE_CAMERA_PANEL_SYSTEM_H

#include "../system.h"
#include "camera_system.h"
#include <string>

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

    // Offline render progress: the tracer runs as a detached subprocess that
    // writes its percentage to a sidecar file; the panel polls it each frame
    // and shows a bar (the tracer's --progress flag).
    bool offlineRendering = false;
    float offlineProgress = 0.0f;        // [0,1], from the sidecar
    std::string offlineOutput;           // PNG the subprocess is writing
    std::string offlineProgressPath;     // sidecar being polled
};

}  // namespace engine

#endif
