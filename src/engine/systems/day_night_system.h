#ifndef RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H
#define RAYTRACER_ENGINE_DAY_NIGHT_SYSTEM_H

#include "../system.h"
#include "../day_night_cycle.h"
#include "../sky_chart.h"
#include "../weather_cycle.h"
#include "../procgen/terrain_horizon.h"
#include "../properties.h"
#include "../control/control_commands.h"

#include <string>
#include <vector>

#ifdef __EMSCRIPTEN__
#include <limits>
#endif

namespace engine {

struct DayNightConfig;

// DayNightRequest (ADR-0253): drive the cycle from outside -- the socket's `daynight`, Lua. Each field left at its
// "unset" value is left alone: hour < 0, hold/terrain/hud < 0 (else 0 off, 1 on), minutes < 0, day 0, chart "",
// moon < 0 unless moonAuto. Setting the hour, the day or the moon turns the cycle on.
struct DayNightRequest {
    Real hour = -1;        // 0-24
    int hold = -1;         // 1 holds the light (not the sim clock), 0 runs it
    Real minutes = -1;     // real minutes per day; 0 freezes the sun
    int day = 0;           // 1-365: the calendar day (season, moon)
    Real moon = -1;        // the moon's age in days, held
    bool moonAuto = false; // the moon follows the calendar again
    int terrain = -1;      // terrain horizon shadows
    int hud = -1;          // the sky HUD
    std::string chart;     // write the day's sky chart (SVG) here
};
inline void describeProperties(DayNightRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("hour").range(0, 24), r.hour);
    v.field(FieldMeta("hold"), r.hold);
    v.field(FieldMeta("minutes"), r.minutes);
    v.field(FieldMeta("day").range(1, 365), r.day);
    v.field(FieldMeta("moon").units("days"), r.moon);
    v.field(FieldMeta("moonAuto"), r.moonAuto);
    v.field(FieldMeta("terrain"), r.terrain);
    v.field(FieldMeta("hud"), r.hud);
    v.field(FieldMeta("chart"), r.chart);
}
// WeatherRequest: clear, fair, overcast, storm (eased in over ~40 s, or snapped with `now`), auto (the seeded
// walk), or off (the cloud knobs back to the panel).
struct WeatherRequest {
    std::string state;
    bool now = false;
};
inline void describeProperties(WeatherRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("state").options({"clear", "fair", "overcast", "storm", "auto", "off"}), r.state);
    v.field(FieldMeta("now"), r.now);
}
// CloudsRequest: set the cloud deck's knobs live (they are otherwise read at state entry). Each field left below
// zero is left alone; enabled 0 off, 1 on. The socket's `clouds`; the values persist with the rest of the deck.
struct CloudsRequest {
    int enabled = -1;
    Real coverage = -1;
    Real density = -1;
    Real scale = -1;
    Real wind = -1;
};
inline void describeProperties(CloudsRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("enabled"), r.enabled);
    v.field(FieldMeta("coverage").range(0, 1), r.coverage);
    v.field(FieldMeta("density"), r.density);
    v.field(FieldMeta("scale"), r.scale);
    v.field(FieldMeta("wind").units("m/s"), r.wind);
}
// SimHourQuery: the city sim's own hour, for `daynight?` beside the sky's (one clock). A question on the bus: the
// sim, if there is one, writes it into `hours` (left at -1 otherwise). Not named: it carries a pointer.
struct SimHourQuery {
    double* hours = nullptr;
};

// Advances a DayNightCycle on SIMULATION time and writes its state into the
// RenderView's lighting: the procedural sky (skybox + IBL) and the directional
// sun light stay locked to one time-of-day, so shadows and shading track the
// sky (ADR-0016). Sun motion and cloud drift integrate in fixedUpdate, so they
// pause, slow-mo, and single-step with the clock like everything simulated;
// the panel's edits still apply instantly while paused (update pushes the
// current state into the view every frame). Settings persist
// time/dayMinutes/latitude/dayOfYear/enabled; ImGui exposes them under
// "Day / Night" in debug mode.
// Write every NightGlow entity's emission for `ramp` (0 day .. 1 night) -- each entity once per ramp value, so one
// that streamed in since the last change is lit too.
void applyNightGlowRamp(World& world, Real ramp);

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
    // `daynight?` on the control channel (bound by this system) answers the
    // clock as numbers, measured here each frame into status_ — the loop's
    // rate is MEASURED, not trusted: two reads a real minute apart.
    void publishStatus(FrameContext& ctx, bool active);
    std::string status_ = "no cycle";   // what publishStatus last measured
    std::vector<DayNightRequest> dayRequests_;   // heard since the last update, applied in order
    bool weatherPending_ = false;
    WeatherRequest weatherRequest_;
    void applyRequests(FrameContext& ctx);
    std::vector<CloudsRequest> cloudRequests_;
    EventBus::Subscription onDay_, onWeather_, onClouds_;
    ControlCommands::Binding answerClock_, answerWeather_;
    // THE SKY HUD (device: "sun and moon positions in the sky"): a compass
    // strip with sun/moon markers at their bearings relative to the camera,
    // and the day's polar chart (sky_chart.h) — the same numbers the SVG
    // instrument draws. Debug-overlay only; persisted as daynight.hud.
    bool skyHud_ = false;
    // The star field (persisted daynight.stars / daynight.milkyWay).
    bool  stars_ = true;
    float milkyWay_ = 1.0f;
    // LIGHT POLLUTION (device: "what if you're away from the city?"): the
    // city's footprint (its road graph's bounds, found once) and how far past
    // it the sky goes dark. Evaluated at the camera every frame.
    float lightPollution_ = 0.7f;
    float pollutionFalloff_ = 1500.0f;
    bool  cityBoundsKnown_ = false;
    double cityCentreX_ = 0.0, cityCentreZ_ = 0.0, cityRadius_ = 0.0;
    void findCityBounds(FrameContext& ctx);

    // TERRAIN HORIZON SHADOWS (device: "when the sun set behind the
    // mountains ... I thought I might start seeing more darkness right
    // away, or at least some shadow casting from the mountain"). The height
    // raster is built once from the level's terrain; the horizon map toward
    // the active light is re-marched whenever it has moved a couple of
    // degrees (a few ms across the job pool) and uploaded; the camera's
    // own texel tells whether the disc is behind the ridge.
    bool terrainShadows_ = true;
    HeightRaster raster_;
    HorizonMap horizon_;
    TextureHandle horizonTex_;
    bool horizonTexValid_ = false;
    double horizonAzimuth_ = 1e9;
    float ridgeAtCamera_ = -1.0f;   // sin(horizon elevation) toward the light, at the camera
    float lightBehindRidge_ = 0.0f;
    void buildHeightRaster(FrameContext& ctx);
    void refreshHorizon(FrameContext& ctx, const DayNightState& st);
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
    // knobs back to the panel and `clouds`. Auto mode rolls a seeded neighbor
    // walk every few in-world hours.
    WeatherCycle weather;
    bool weatherActive = false;
    bool weatherFromSettings_ = false;   // a persisted state was restored: the level's policy yields
    bool weatherDefaulted_ = false;      // the level policy / volumetric default applied once

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
    // Beacon flash pass (skyscrapers v2 M4): entities tagged NightGlow +
    // BeaconBlink get their ramped emission gated on the flash cycle every
    // frame. Wall-clock driven (beacons keep flashing while the sim is paused).
    void applyBeaconBlink(FrameContext& ctx);
    double blinkSeconds_ = 0.0;
    float nightAdapt_ = 1.0f;   // exposure / authored exposure this frame (1 by day, kNightAdapt at midnight)

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
