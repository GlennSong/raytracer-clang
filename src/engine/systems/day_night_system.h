#ifndef RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H

#include "../system.h"
#include "../day_night_cycle.h"
#include "../sky_chart.h"
#include "../weather_cycle.h"

#ifdef __EMSCRIPTEN__
#include <limits>
#endif

namespace engine {

struct DayNightConfig;

// Advances a DayNightCycle on SIMULATION time and writes its state into the
// RenderView's lighting: the procedural sky (skybox + IBL) and the directional
// sun light stay locked to one time-of-day, so shadows and shading track the
// sky (ADR-0016). Sun motion and cloud drift integrate in fixedUpdate, so they
// pause, slow-mo, and single-step with the clock like everything simulated;
// the panel's edits still apply instantly while paused (update pushes the
// current state into the view every frame). Settings persist
// time/dayMinutes/latitude/dayOfYear/enabled; ImGui exposes them under
// "Day / Night" in debug mode.
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
    bool configSeeded_ = false;   // DayNightConfig seeds applied once
    void seedFromConfig(const DayNightConfig& c);
    // `daynight?` on the control channel reads the clock as numbers from
    // settings ("daynight.status", the weather.status idiom) — the loop's
    // rate is MEASURED, not trusted: two reads a real minute apart.
    void publishStatus(FrameContext& ctx, bool active);
    // THE SKY HUD (device: "sun and moon positions in the sky"): a compass
    // strip with sun/moon markers at their bearings relative to the camera,
    // and the day's polar chart (sky_chart.h) — the same numbers the SVG
    // instrument draws. Debug-overlay only; persisted as daynight.hud.
    bool skyHud_ = false;
    SkyChart hudChart_;
    long hudChartKey_ = -1;   // (day, year, lock, latitude) the cached chart is for
    void drawSkyHud(FrameContext& ctx);
    void writeSkyChart(const std::string& path);

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

    // Night exposure adaptation (WS2): the eye the renderer doesn't have. A
    // moonlit world is readable because pupils dilate — without adaptation the
    // physically-scaled moon lights the sky but the ground tonemaps to black.
    // The level's authored exposure is captured at start and scaled up as the
    // sun sinks (smooth, keyed on solar elevation); lamps brightening with it
    // is what adapted eyes actually see. -1 = not captured yet.
    float baseExposure_ = -1.0f;
    static constexpr float kNightAdapt = 6.0f;   // midnight exposure multiplier
    // The level's AUTHORED lighting, captured lazily like baseExposure_: the
    // cycle multiplies a day-shape onto these instead of replacing them.
    float baseSunIntensity_ = -1.0f;
    float baseAmbient_ = -1.0f;
    ShadowArtistic baseShadow_;
    bool shadowCaptured_ = false;

    // NightGlow pass (WS3): scales tagged emissive materials on the dusk ramp
    // every frame (skipped while the ramp is unchanged). -1 forces the first
    // application even at noon (spawn emission is 0 anyway).
    void applyNightGlow(FrameContext& ctx);
    Real lastGlowRamp_ = -1.0;

    // The level's authored volumetric deck, captured on first sight: the
    // artistic knobs write ABSOLUTE values derived from this base each frame
    // (a relative per-frame mutation would compound).
    VolumetricCloudParams cloudBase_;
    bool cloudBaseCaptured_ = false;

#ifdef __EMSCRIPTEN__
    // Last time-of-day the web panel pushed through settings; lets update()
    // distinguish a fresh slider drag (jump there) from the cycle's own
    // animation (leave alone). NaN so the first read always applies.
    double webLastTimeOfDay_ = std::numeric_limits<double>::quiet_NaN();
#endif
};

}  // namespace engine

#endif
