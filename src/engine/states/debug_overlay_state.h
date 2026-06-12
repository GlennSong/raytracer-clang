#ifndef RAYTRACER_ENGINE_STATES_DEBUG_OVERLAY_STATE_H
#define RAYTRACER_ENGINE_STATES_DEBUG_OVERLAY_STATE_H

#include "../app_state.h"
#include "../systems/debug_overlay_system.h"
#include <memory>

namespace engine {

class Window;

class DebugOverlayState : public AppState {
public:
    explicit DebugOverlayState(Window& window);

    bool isTransparent() const override { return true; }
    bool blocksInput() const override { return true; }

    void onEnter(FrameContext& ctx) override;
    void onExit(FrameContext& ctx) override;
    void onEvent(const Event& event, FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

private:
    Window& window;
    DebugOverlaySystem overlay;
};

}  // namespace engine

#endif
