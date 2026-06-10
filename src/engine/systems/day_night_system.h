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
    void applyLighting(FrameContext& ctx);  // sun + sky colors from the cycle
    void applyClouds(FrameContext& ctx);     // cloud params + drift phase

    // True when an HDR environment is bound — it owns the lighting, so the cycle
    // must not drive the sun/sky/ambient (see definition in the .cpp).
    bool hdrEnvironmentActive(FrameContext& ctx) const;

    DayNightCycle cycle;
    bool enabled = true;   // when off, the level's static sun/sky is left alone

    // Procedural clouds (ADR-0016 step 3) — independent of the day/night toggle.
    bool   cloudsEnabled  = true;
    float  cloudCoverage  = 0.5f;
    float  cloudDensity   = 1.0f;
    float  cloudScale     = 1.5f;
    float  cloudWindSpeed = 1.0f;
    double cloudPhase     = 0.0;   // accumulated drift, seconds
};

}  // namespace engine

#endif
