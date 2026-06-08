#include "debug_overlay_state.h"
#include "../../renderer/window.h"

namespace engine {

DebugOverlayState::DebugOverlayState(Window& window) : window(window) {}

void DebugOverlayState::onEnter(FrameContext& ctx) {
    window.setCursorMode(CursorMode::Normal);
    if (!started) {
        overlay.onStart(ctx);
        started = true;
    }
}

void DebugOverlayState::onExit(FrameContext& ctx) {
    overlay.onStop(ctx);
    started = false;
}

void DebugOverlayState::onEvent(const Event& event, FrameContext& ctx) {
    overlay.onEvent(event, ctx);
}

void DebugOverlayState::update(FrameContext& ctx) {
    overlay.update(ctx);
}

void DebugOverlayState::render(FrameContext& ctx) {
    overlay.render(ctx);
}

}  // namespace engine
