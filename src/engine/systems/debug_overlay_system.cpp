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

    RenderStats rs = ctx.renderer.getRenderStats();
    uint32_t culled = static_cast<uint32_t>(ctx.world.entityCount()) - rs.entitiesSubmitted;
    ImGui::Separator();
    ImGui::Text("Visible: %u  Culled: %u", rs.entitiesSubmitted, culled);
    ImGui::Text("Draw calls: %u (instanced: %u)", rs.drawCalls, rs.instancedDrawCalls);
    ImGui::Text("Instances: %u", rs.totalInstances);

    ImGui::Separator();
    ImGui::Text("Post-Processing");
    ImGui::Checkbox("SSAO", &ctx.renderer.ssaoEnabled);
    ImGui::Checkbox("SSR", &ctx.renderer.ssrEnabled);
    ImGui::Checkbox("Reflection Probes", &ctx.renderer.reflectionProbesEnabled);

    ImGui::Separator();
    const char* viewNames[] = {"Normal", "AO Only", "SSR Only", "Depth"};
    ImGui::Combo("View", &ctx.renderer.debugView, viewNames, 4);
    ImGui::End();
#else
    (void)ctx;  // inert without ImGui (ADR-0011)
#endif
}

}  // namespace engine
