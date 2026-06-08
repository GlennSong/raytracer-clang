#ifndef RAYTRACER_ENGINE_DEBUG_OVERLAY_SYSTEM_H
#define RAYTRACER_ENGINE_DEBUG_OVERLAY_SYSTEM_H

#include "../system.h"

namespace engine {

// The agreed home for engine debug UI (ADR-0011). Draws a base ImGui overlay
// (FPS, entity count, camera) in its render hook. Without RT_ENABLE_IMGUI it is
// an inert System, so it can stay registered in all builds. Other systems may
// also emit ImGui directly in their own render().
class DebugOverlaySystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
};


}  // namespace engine

#endif
