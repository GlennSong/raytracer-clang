#include "playing_state.h"
#include "../systems/debug_overlay_system.h"
#include "../../renderer/window.h"
#include "../../log.h"

#include <chrono>
#include <cstdlib>
#include <typeinfo>
#include <vector>

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
    static const bool dumpStats = std::getenv("RT_DUMP_STATS") != nullptr;
    if (!dumpStats) {
        for (auto& system : systems) system->update(ctx);
        return;
    }
    static std::vector<double> ms;
    static int calls = 0;
    ms.resize(systems.size(), 0.0);
    for (std::size_t i = 0; i < systems.size(); ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        systems[i]->update(ctx);
        ms[i] += std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - t0).count();
    }
    if (++calls % 300 == 0) {
        for (std::size_t i = 0; i < systems.size(); ++i)
            if (ms[i] / calls > 0.05) {
                // Name the system through a reference, not `*systems[i]`:
                // typeid EVALUATES a polymorphic operand (it reads the vtable),
                // and a unique_ptr deref inside it reads as a side-effecting
                // expression. Same lookup, no warning.
                const System& sys = *systems[i];
                LOG_INFO << "[stats] update " << (ms[i] / calls) << " ms  "
                         << typeid(sys).name();
            }
        std::fill(ms.begin(), ms.end(), 0.0);
        calls = 0;
    }
}

void PlayingState::fixedUpdate(FrameContext& ctx) {
    // RT_DUMP_STATS: per-system fixed-step bill (typeid names), printed every
    // ~300 steps — the instrument that ends bottleneck guessing.
    static const bool dumpStats = std::getenv("RT_DUMP_STATS") != nullptr;
    if (!dumpStats) {
        for (auto& system : systems) system->fixedUpdate(ctx);
        return;
    }
    static std::vector<double> ms;
    static int calls = 0;
    ms.resize(systems.size(), 0.0);
    for (std::size_t i = 0; i < systems.size(); ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        systems[i]->fixedUpdate(ctx);
        ms[i] += std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - t0).count();
    }
    if (++calls % 300 == 0) {
        for (std::size_t i = 0; i < systems.size(); ++i)
            if (ms[i] / calls > 0.05) {
                const System& sys = *systems[i];   // see the update() note above
                LOG_INFO << "[stats] fixed " << (ms[i] / calls) << " ms  "
                         << typeid(sys).name();
            }
        std::fill(ms.begin(), ms.end(), 0.0);
        calls = 0;
    }
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
        // entitiesSubmitted counts submitted draws (instances included) and can
        // exceed the entity count — clamp so this never underflows to billions.
        const uint32_t totalEnts = static_cast<uint32_t>(ctx.world.entityCount());
        uint32_t culled =
            rs.entitiesSubmitted <= totalEnts ? totalEnts - rs.entitiesSubmitted : 0;
        ImGui::Text("Visible: %u  Culled: %u", rs.entitiesSubmitted, culled);
        ImGui::Text("Draws: %u (instanced: %u)", rs.drawCalls, rs.instancedDrawCalls);

        ImGui::End();
    }
#endif
}

}  // namespace engine
