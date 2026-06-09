#ifndef RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H

#include "../system.h"
#include "../day_night_cycle.h"

namespace engine {

// Advances a DayNightCycle each frame and writes its state into the RenderView's
// lighting: the procedural sky (skybox + IBL) and the directional sun light stay
// locked to one time-of-day, so shadows and shading track the sky (ADR-0016).
// Settings persist time/speed/enabled; ImGui exposes them under "Day / Night".
class DayNightSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    void apply(FrameContext& ctx);

    DayNightCycle cycle;
    bool enabled = true;   // when off, the level's static sun/sky is left alone
};

}  // namespace engine

#endif
