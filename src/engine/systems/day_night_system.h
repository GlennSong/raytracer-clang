#ifndef RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H

#include "../system.h"
#include "../day_night_cycle.h"
#include "../weather_cycle.h"

#ifdef __EMSCRIPTEN__
#include <limits>
#endif

namespace engine {

// Advances a DayNightCycle on SIMULATION time and writes its state into the
// RenderView's lighting: the procedural sky (skybox + IBL) and the directional
// sun light stay locked to one time-of-day, so shadows and shading track the
// sky (ADR-0016). Sun motion and cloud drift integrate in fixedUpdate, so they
// pause, slow-mo, and single-step with the clock like everything simulated;
// the panel's edits still apply instantly while paused (update pushes the
// current state into the view every frame). Settings persist
// time/speed/enabled; ImGui exposes them under "Day / Night" in debug mode.
class DayNightSystem : public System {
public:
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    void applyLighting(FrameContext& ctx);  // sun + sky colors from the cycle
    void applyClouds(FrameContext& ctx);     // cloud params + drift phase

    // True when an HDR environment is bound — it owns the lighting, so the cycle
    // must not drive the sun/sky/ambient (see definition in the .cpp).
    bool hdrEnvironmentActive(FrameContext& ctx) const;
    bool configSeeded_ = false;   // DayNightConfig time/speed applied once

    DayNightCycle cycle;
    bool enabled = true;   // when off, the level's static sun/sky is left alone

    // Procedural clouds (ADR-0016 step 3) — independent of the day/night toggle.
    // Defaults are the FAIR-weather deck: the old 0.5/1.0 pair was the
    // cinematic-sky tuning, which reads as an incoming storm at midday — that
    // sky is now WeatherKind::Storm, opted into rather than shipped as noon.
    bool   cloudsEnabled  = true;
    float  cloudCoverage  = 0.35f;
    float  cloudDensity   = 0.55f;
    float  cloudScale     = 1.2f;
    float  cloudWindSpeed = 1.0f;
    double cloudPhase     = 0.0;   // accumulated drift, seconds

    // Weather states over the deck (weather_cycle.h): while active it OWNS the
    // cloud knobs above and dims the sun per state; `weather off` hands the
    // knobs back to the panel/clouds.apply. Auto mode rolls a seeded neighbor
    // walk every few in-world hours.
    WeatherCycle weather;
    bool weatherActive = false;

#ifdef __EMSCRIPTEN__
    // Last time-of-day the web panel pushed through settings; lets update()
    // distinguish a fresh slider drag (jump there) from the cycle's own
    // animation (leave alone). NaN so the first read always applies.
    double webLastTimeOfDay_ = std::numeric_limits<double>::quiet_NaN();
#endif
};

}  // namespace engine

#endif
