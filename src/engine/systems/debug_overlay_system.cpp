#include "debug_overlay_system.h"

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DebugOverlaySystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    ImGui::Begin("Debug");
    double fps = ctx.frameDelta > 0.0 ? 1.0 / ctx.frameDelta : 0.0;
    ImGui::Text("FPS: %.1f (%.2f ms)", fps, ctx.frameDelta * 1000.0);
    ImGui::Text("Entities: %zu", ctx.world.entityCount());

    const CameraState& cam = ctx.view.camera;
    ImGui::Text("Camera pos: %.2f, %.2f, %.2f",
                cam.position.x, cam.position.y, cam.position.z);
    ImGui::Text("Projection: %s",
                cam.projection == CameraProjection::Orthographic ? "Orthographic"
                                                                 : "Perspective");
    ImGui::End();
#else
    (void)ctx;  // inert without ImGui (ADR-0011)
#endif
}

}  // namespace engine
