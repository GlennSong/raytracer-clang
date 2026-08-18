#include "day_night_system.h"
#include "../components.h"

#include "../../log.h"

#include <cmath>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DayNightSystem::onStart(FrameContext& ctx) {
    auto& s = ctx.settings;
    enabled         = s.getBool("daynight.enabled", enabled);
    cycle.timeOfDay = s.getDouble("daynight.timeOfDay", cycle.timeOfDay);
    cycle.speed     = s.getDouble("daynight.speed", cycle.speed);
    cycle.paused    = s.getBool("daynight.paused", cycle.paused);

    cloudsEnabled  = s.getBool("clouds.enabled", cloudsEnabled);
    cloudCoverage  = static_cast<float>(s.getDouble("clouds.coverage", cloudCoverage));
    cloudDensity   = static_cast<float>(s.getDouble("clouds.density", cloudDensity));
    cloudScale     = static_cast<float>(s.getDouble("clouds.scale", cloudScale));
    cloudWindSpeed = static_cast<float>(s.getDouble("clouds.windSpeed", cloudWindSpeed));

    // Same level-policy gate as update(): a DayNightConfig with enabled=false
    // means the level's authored sun is the truth — without this, the settings
    // state restored above stomped it once here and the update() gate then
    // preserved the stomp forever.
    bool levelEnabled = true;
    ctx.world.each<DayNightConfig>([&](Entity, DayNightConfig& c) {
        levelEnabled = c.enabled;
        if (!configSeeded_) {
            if (c.timeOfDay >= 0.0f) cycle.timeOfDay = c.timeOfDay;
            if (c.speed >= 0.0f) cycle.speed = c.speed;
            configSeeded_ = true;
        }
    });

    // One boot line so a headset/simulator console shows the resolved state —
    // "why is it dusk" is unanswerable from the picture alone. Printed after the
    // level gate is read, so it reports what actually takes effect.
    LOG_INFO << "DayNight onStart: enabled=" << enabled
             << " levelEnabled=" << levelEnabled
             << " timeOfDay=" << cycle.timeOfDay
             << " paused=" << cycle.paused
             << " speed=" << cycle.speed
             << " hdrActive=" << hdrEnvironmentActive(ctx);

    if (enabled && levelEnabled && !hdrEnvironmentActive(ctx)) applyLighting(ctx);
    applyClouds(ctx);
}

void DayNightSystem::onStop(FrameContext& ctx) {
    auto& s = ctx.settings;
    s.setBool("daynight.enabled", enabled);
    s.setDouble("daynight.timeOfDay", cycle.timeOfDay);
    s.setDouble("daynight.speed", cycle.speed);
    s.setBool("daynight.paused", cycle.paused);

    s.setBool("clouds.enabled", cloudsEnabled);
    s.setDouble("clouds.coverage", cloudCoverage);
    s.setDouble("clouds.density", cloudDensity);
    s.setDouble("clouds.scale", cloudScale);
    s.setDouble("clouds.windSpeed", cloudWindSpeed);

    s.save("settings.json");
}

// A bound HDR environment owns the sun/sky/ambient (its sun is baked into the
// image at a fixed spot), so the day/night cycle must not drive lighting while
// one is active — otherwise an animated analytic sun fights the fixed HDR.
bool DayNightSystem::hdrEnvironmentActive(FrameContext& ctx) const {
    return ctx.renderer.environmentAvgLuminance > 0.0f;
}

void DayNightSystem::fixedUpdate(FrameContext& ctx) {
    // Simulation time, not wall time: pausing the clock freezes the sun and
    // the clouds, Step advances them one tick with the rest of the world,
    // and slow-mo slows them. (The cycle's own Pause checkbox remains the
    // artistic "hold this time of day while the game runs" control.)
    if (enabled && !hdrEnvironmentActive(ctx)) cycle.advance(ctx.clock.fixedStep());
    if (cloudsEnabled) cloudPhase += ctx.clock.fixedStep() * cloudWindSpeed;
}

void DayNightSystem::update(FrameContext& ctx) {
#ifdef __EMSCRIPTEN__
    // The web build has no Dear ImGui, so the page's debug panel drives the
    // cycle through settings instead (rt_web_env in web_main.cpp). Re-read them
    // each frame so the sliders are live. Time-of-day is special: the page
    // writes it only when the user drags the slider, so honour a *changed*
    // value (jump there) but otherwise leave cycle.timeOfDay alone — that lets
    // a non-zero Speed keep animating between drags instead of being pinned.
    auto& ws = ctx.settings;
    enabled      = ws.getBool("daynight.enabled", enabled);
    cycle.speed  = ws.getDouble("daynight.speed", cycle.speed);
    cycle.paused = ws.getBool("daynight.paused", cycle.paused);
    double tod = ws.getDouble("daynight.timeOfDay", cycle.timeOfDay);
    if (tod != webLastTimeOfDay_) { cycle.timeOfDay = tod; webLastTimeOfDay_ = tod; }
#endif
    // Control channel one-shots (ADR-0078): `daynight <hour>` / `daynight
    // hold|run` write these; consume-and-clear (the citysim.* idiom) so the
    // ImGui panel and the cycle's own animation stay the owners between
    // pokes. Unlike the web block above this does NOT re-read the persistent
    // keys every frame — that would stomp live panel edits natively.
    {
        auto& cs = ctx.settings;
        const double setTod = cs.getDouble("daynight.set", -1.0);
        if (setTod >= 0.0) {
            cycle.timeOfDay = std::fmod(setTod, 24.0);
            cs.setDouble("daynight.set", -1.0);
        }
        const double setHold = cs.getDouble("daynight.setPaused", -1.0);
        if (setHold >= 0.0) {
            cycle.paused = setHold > 0.5;   // "artistic hold", not the sim clock
            cs.setDouble("daynight.setPaused", -1.0);
        }
        // `set clouds.<knob> <v>` then `clouds.apply 1`: re-read the cloud
        // deck live (coverage/density/scale/wind/enabled are otherwise
        // state-entry-only, which made "why is the sky so heavy" answerable
        // only by a reload).
        if (cs.getDouble("clouds.apply", 0.0) > 0.5) {
            cs.setDouble("clouds.apply", 0.0);
            cloudsEnabled = cs.getBool("clouds.enabled", cloudsEnabled);
            cloudCoverage =
                static_cast<float>(cs.getDouble("clouds.coverage", cloudCoverage));
            cloudDensity =
                static_cast<float>(cs.getDouble("clouds.density", cloudDensity));
            cloudScale =
                static_cast<float>(cs.getDouble("clouds.scale", cloudScale));
            cloudWindSpeed = static_cast<float>(
                cs.getDouble("clouds.windSpeed", cloudWindSpeed));
        }
    }

    // Pushing current state into the view every frame (cheap) keeps panel
    // Level policy (DayNightConfig): a level that authors a static sun turns
    // the cycle off — the same yield rule as a bound HDR environment. Seeds
    // (time/speed) apply once.
    bool levelEnabled = true;
    ctx.world.each<DayNightConfig>([&](Entity, DayNightConfig& c) {
        levelEnabled = c.enabled;
        if (!configSeeded_) {
            if (c.timeOfDay >= 0.0f) cycle.timeOfDay = c.timeOfDay;
            if (c.speed >= 0.0f) cycle.speed = c.speed;
            configSeeded_ = true;
        }
    });
    // edits live even while the simulation is paused.
    if (enabled && levelEnabled && !hdrEnvironmentActive(ctx)) applyLighting(ctx);
    applyClouds(ctx);
}

// Push the current cycle state into both the procedural sky colors and the
// directional sun so the rendered sky and the lighting agree.
void DayNightSystem::applyLighting(FrameContext& ctx) {
    DayNightState st = cycle.evaluate();
    auto& lit = ctx.view.lighting;

    lit.sun.direction = st.sunDirection;
    lit.sun.color     = st.sunColor;
    lit.sun.intensity = st.sunIntensity;

    lit.sky.sunDirection     = st.sunDirection;
    lit.sky.sunColor         = st.sunColor;
    lit.sky.sunDiscIntensity = st.skyDiscIntensity;
    lit.sky.zenithColor      = st.zenithColor;
    lit.sky.horizonColor     = st.horizonColor;
    lit.sky.groundColor      = st.groundColor;

    lit.ambientMultiplier = st.ambient;
}

// Cloud parameters are independent of the day/night toggle; the shader shades
// them against whatever sun state is active, so they work over a static sky too.
void DayNightSystem::applyClouds(FrameContext& ctx) {
    auto& sky = ctx.view.lighting.sky;
    sky.cloudsEnabled = cloudsEnabled;
    sky.cloudCoverage = cloudCoverage;
    sky.cloudDensity  = cloudDensity;
    sky.cloudScale    = cloudScale;
    sky.cloudTime     = static_cast<float>(cloudPhase);
}

void DayNightSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    // Debug-mode only (backtick), and into the shared "Debug" window — the
    // headers used to land in ImGui's implicit fallback window, which is why
    // they floated over plain play.
    if (!ctx.debugOverlayActive) return;
    ImGui::Begin("Debug");
    if (ImGui::CollapsingHeader("Day / Night")) {
        bool hdrActive = hdrEnvironmentActive(ctx);
        if (hdrActive) {
            ImGui::TextDisabled("HDR environment active - cycle overridden");
        }
        ImGui::BeginDisabled(hdrActive);
        ImGui::Checkbox("Enabled##daynight", &enabled);
        float t = static_cast<float>(cycle.timeOfDay);
        if (ImGui::SliderFloat("Time of Day", &t, 0.0f, 1.0f, "%.3f")) {
            cycle.timeOfDay = t;
            if (enabled) applyLighting(ctx);
        }
        ImGui::SameLine();
        ImGui::Checkbox("Pause", &cycle.paused);
        float speed = static_cast<float>(cycle.speed);
        if (ImGui::SliderFloat("Speed (days/sec)", &speed, 0.0f, 0.2f, "%.3f")) {
            cycle.speed = speed;
        }
        // Readout: 24h clock derived from time-of-day (0.0 == midnight).
        int totalMin = static_cast<int>(cycle.timeOfDay * 24.0 * 60.0) % (24 * 60);
        ImGui::Text("Clock: %02d:%02d", totalMin / 60, totalMin % 60);
        ImGui::EndDisabled();
    }

    // CLOUDS — there are two independent cloud systems and only one runs at a
    // time: the 2D FBM sky overlay, and the volumetric slab that REPLACES it
    // (post.metal switches the overlay off whenever the slab is active). This
    // header used to edit the FBM fields unconditionally, so on a level with a
    // volumetric deck every slider did nothing while still writing itself to
    // settings.json as though it were authoritative. Show the controls for the
    // deck that is actually live, and say which one that is.
    {
        VolumetricCloudParams& vc = ctx.view.lighting.volumetricClouds;
        const bool volumetric = vc.enabled;
        if (ImGui::CollapsingHeader(volumetric ? "Clouds (volumetric)"
                                               : "Clouds (2D overlay)")) {
            if (volumetric) {
                // Bound STRAIGHT to the live params: the level loader writes
                // them once at load and the renderer reads them fresh each
                // frame, with nothing re-applying in between — so an edit here
                // is the value the next frame marches.
                ImGui::Checkbox("Enabled##vclouds",
                                &ctx.renderer.volumetricCloudsEnabled);
                ImGui::SliderFloat("Coverage##v", &vc.coverage, 0.0f, 1.0f);
                ImGui::SliderFloat("Density##v", &vc.density, 0.0f, 1.0f);
                ImGui::SliderFloat("Ambient##v", &vc.ambient, 0.0f, 2.0f);
                ImGui::SliderFloat("Detail erosion##v", &vc.detailStrength,
                                   0.0f, 1.0f);
                ImGui::SliderFloat("Phase g##v", &vc.phaseG, -0.9f, 0.9f);
                ImGui::SliderFloat("Wind (m/s)##v", &vc.wind, 0.0f, 60.0f);
                ImGui::SliderFloat("Noise scale##v", &vc.noiseScale,
                                   0.0002f, 0.005f, "%.4f");
                ImGui::DragFloatRange2("Slab bottom / top##v", &vc.bottom,
                                       &vc.top, 10.0f, 100.0f, 6000.0f,
                                       "%.0f m", "%.0f m");
                ImGui::SliderInt("Light steps##v", &vc.lightSteps, 1, 16);
                // The step-count override lives on the renderer, not the params,
                // so it belongs in the same panel rather than beside it.
                ImGui::SliderInt("View steps (0 = level)##v",
                                 &ctx.renderer.cloudStepsOverride, 0, 96);
                ImGui::TextDisabled("Authored per level in environment.clouds;");
                ImGui::TextDisabled("edits here are session-only, not saved.");
            } else {
                ImGui::Checkbox("Enabled##clouds", &cloudsEnabled);
                ImGui::SliderFloat("Coverage", &cloudCoverage, 0.0f, 1.0f);
                ImGui::SliderFloat("Density", &cloudDensity, 0.0f, 1.0f);
                ImGui::SliderFloat("Scale", &cloudScale, 0.2f, 5.0f);
                ImGui::SliderFloat("Wind Speed", &cloudWindSpeed, 0.0f, 5.0f);
                applyClouds(ctx);  // reflect edits immediately, even while paused
            }
        }
    }
    ImGui::End();
#else
    (void)ctx;
#endif
}

}  // namespace engine
