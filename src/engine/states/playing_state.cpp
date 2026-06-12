#include "playing_state.h"
#include "../systems/debug_overlay_system.h"
#include "../../renderer/window.h"

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

PlayingState::PlayingState(Window& window) : window(window) {}

void PlayingState::onEnter(FrameContext& ctx) {
    window.setCursorMode(CursorMode::Disabled);
    window.resetMouseDelta();
    for (auto& system : systems) system->onStart(ctx);
    DebugOverlaySystem::loadSettings(ctx);
}

void PlayingState::onExit(FrameContext& ctx) {
    for (auto& system : systems) system->onStop(ctx);
}

void PlayingState::onResume(FrameContext&) {
    window.setCursorMode(CursorMode::Disabled);
    window.resetMouseDelta();
}

void PlayingState::onEvent(const Event& event, FrameContext& ctx) {
    for (auto& system : systems) system->onEvent(event, ctx);
}

void PlayingState::update(FrameContext& ctx) {
    for (auto& system : systems) system->update(ctx);
}

void PlayingState::fixedUpdate(FrameContext& ctx) {
    for (auto& system : systems) system->fixedUpdate(ctx);
}

void PlayingState::render(FrameContext& ctx) {
    for (auto& system : systems) system->render(ctx);

#ifdef RT_ENABLE_IMGUI
    if (ctx.renderer.showHud) {
        ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_FirstUseEver);
        ImGui::SetNextWindowBgAlpha(0.4f);
        ImGui::Begin("##hud", nullptr,
            ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize
            | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav
            | ImGuiWindowFlags_NoInputs);

        double fps = ctx.frameDelta > 0.0 ? 1.0 / ctx.frameDelta : 0.0;
        ImGui::Text("FPS: %.0f (%.2f ms)", fps, ctx.frameDelta * 1000.0);

        ImGui::Text("Entities: %zu", ctx.world.entityCount());

        const CameraState& cam = ctx.view.camera;
        ImGui::Text("Pos: %.1f, %.1f, %.1f",
                    cam.position.x, cam.position.y, cam.position.z);

        RenderStats rs = ctx.renderer.getRenderStats();
        uint32_t culled = static_cast<uint32_t>(ctx.world.entityCount()) - rs.entitiesSubmitted;
        ImGui::Text("Visible: %u  Culled: %u", rs.entitiesSubmitted, culled);
        ImGui::Text("Draws: %u (instanced: %u)", rs.drawCalls, rs.instancedDrawCalls);

        ImGui::End();
    }
#endif
}

}  // namespace engine
