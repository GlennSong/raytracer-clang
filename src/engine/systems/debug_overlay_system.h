#ifndef RAYTRACER_ENGINE_DEBUG_OVERLAY_SYSTEM_H
#define RAYTRACER_ENGINE_DEBUG_OVERLAY_SYSTEM_H

#include "../system.h"
#include "../pass_cost.h"

namespace engine {

// The agreed home for engine debug UI (ADR-0011). Draws a base ImGui overlay
// (FPS, entity count, camera) in its render hook. Without RT_ENABLE_IMGUI it is
// an inert System, so it can stay registered in all builds. Other systems may
// also emit ImGui directly in their own render().
class DebugOverlaySystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;

    // The settings.json <-> renderer mapping for the post/render knobs
    // (ssao.*, ssr.*, shadow.*, bloom.*, tonemap.op, grade.*, plus the
    // bloom/ssao/ssr enable toggles). Split from FrameContext so hosts
    // without a frame in flight — the visionOS settings panel — can apply
    // and store the same keys the ImGui overlay uses.
    static void loadSettings(Settings& s, Renderer& r);
    static void saveSettings(Settings& s, Renderer& r);

    static void loadSettings(FrameContext& ctx);
    static void saveSettings(FrameContext& ctx);
    static void resetDefaults(FrameContext& ctx);

private:
    // Owned by the system, NOT a static local inside render(): the probe
    // mutates renderer state (it disables passes to time them) and must be
    // able to put it back on shutdown. As a function-local static it could be
    // left mid-run — which persisted a disabled pass into settings.json and
    // made bloom silently vanish on the next launch.
    PassCost passCost;
};


}  // namespace engine

#endif
