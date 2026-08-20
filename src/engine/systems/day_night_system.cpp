#include "day_night_system.h"
#include "../components.h"
#include "../vehicle_lamps.h"   // duskRamp: night glow shares the lamp boundary

#include "../../log.h"

#include <algorithm>
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
    // 1 = full moon (moonlit-realistic nights), 0 = new moon (true dark).
    cycle.moonPhase = s.getDouble("daynight.moonPhase", cycle.moonPhase);

    cloudsEnabled  = s.getBool("clouds.enabled", cloudsEnabled);
    cloudCoverage  = static_cast<float>(s.getDouble("clouds.coverage", cloudCoverage));
    cloudDensity   = static_cast<float>(s.getDouble("clouds.density", cloudDensity));
    cloudScale     = static_cast<float>(s.getDouble("clouds.scale", cloudScale));
    cloudWindSpeed = static_cast<float>(s.getDouble("clouds.windSpeed", cloudWindSpeed));

    // Weather state survives the session (yesterday's storm resumes AS a
    // storm — snap, don't fade in from the boot deck).
    const std::string ws = s.getString("weather.state", "");
    if (!ws.empty() && ws != "off") {
        WeatherKind kind;
        if (weatherKindFromName(ws, kind)) {
            weatherActive = true;
            weather.state = kind;
            weather.autoMode = s.getBool("weather.auto", false);
            weather.snap();
        }
    }

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
    s.setDouble("daynight.moonPhase", cycle.moonPhase);

    s.setBool("clouds.enabled", cloudsEnabled);
    s.setDouble("clouds.coverage", cloudCoverage);
    s.setDouble("clouds.density", cloudDensity);
    s.setDouble("clouds.scale", cloudScale);
    s.setDouble("clouds.windSpeed", cloudWindSpeed);

    s.setString("weather.state",
                weatherActive ? weatherKindName(weather.state) : "off");
    s.setBool("weather.auto", weather.autoMode);

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

    // Weather: the auto walk rides the CYCLE's clock (cycle.speed is a day
    // fraction per second, so hours = dt * speed * 24) and holds with the
    // cycle's artistic pause; the EASING keeps running regardless, so an
    // in-flight front always finishes arriving. While active, weather owns
    // the cloud knobs — the ImGui sliders and clouds.apply rule again after
    // `weather off`.
    if (weatherActive) {
        const double dt = ctx.clock.fixedStep();
        if (!cycle.paused)
            weather.advanceAuto(dt * cycle.speed * 24.0,
                                static_cast<uint32_t>(
                                    ctx.settings.getDouble("weather.seed", 7.0)));
        weather.ease(dt);
        cloudCoverage = weather.coverage;
        cloudDensity  = weather.density;
        cloudScale    = weather.scale;
    }
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
            // HOURS on the wire, FRACTION in the cycle (0.5 = noon). The first
            // cut dropped hours straight in; the sun trig wraps modulo one DAY
            // FRACTION, so `daynight 12` landed on 12 mod 1 = midnight-and-
            // three-quarters — sunset. Every "golden hour" was found by eye,
            // one wrong number away.
            cycle.timeOfDay = std::fmod(setTod, 24.0) / 24.0;
            cs.setDouble("daynight.set", -1.0);
            // Asking for an hour IS asking for the cycle's lighting: a level
            // that booted with the cycle off (static authored sun) would
            // otherwise consume the time and change nothing — the papercut
            // that ate the first moonlit-midnight attempt.
            enabled = true;
        }
        // Weather one-shot: `weather clear|fair|overcast|storm|auto|off`.
        const std::string setWeather = cs.getString("weather.set", "");
        if (!setWeather.empty()) {
            cs.setString("weather.set", "");
            WeatherKind kind;
            if (setWeather == "off") {
                weatherActive = false;
            } else if (setWeather == "auto") {
                weatherActive = true;
                weather.autoMode = true;
            } else if (weatherKindFromName(setWeather, kind)) {
                weatherActive = true;
                weather.autoMode = false;
                weather.state = kind;
            }
        }
        cs.setString("weather.status",
                     weatherActive
                         ? std::string(weatherKindName(weather.state)) +
                               (weather.autoMode ? " (auto)" : "")
                         : "off");
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
    applyNightGlow(ctx);   // even with the cycle off: static dusk levels glow
}

// Push the current cycle state into both the procedural sky colors and the
// directional sun so the rendered sky and the lighting agree.
void DayNightSystem::applyLighting(FrameContext& ctx) {
    DayNightState st = cycle.evaluate();
    auto& lit = ctx.view.lighting;

    // The ACTIVE light drives slot 0 — sun by day, MOON by night (WS2). The
    // scattering sky, its IBL bake, shadows, and the aerial haze all follow
    // this one light, which is exactly what makes a moonlit night work: the
    // whole daytime pipeline runs, ~200x dimmer and blue.
    lit.sun.direction = st.lightDirection;
    lit.sun.color     = st.lightColor;
    // Weather dims the light (overcast halves it, storm guts it) — dead flat
    // shadows are what sell a heavy deck as WEATHER rather than a weird sky.
    lit.sun.intensity =
        st.lightIntensity * (weatherActive ? weather.sunScale : 1.0f);
    // Dusk predicates (vehicle lamps, street lights) key on the SUN's truth,
    // not the active light — the moon rising must not read as daylight.
    lit.solarElevation = st.solarElevation;

    lit.sky.sunDirection     = st.lightDirection;
    lit.sky.sunColor         = st.lightColor;
    lit.sky.sunDiscIntensity = st.skyDiscIntensity;
    lit.sky.zenithColor      = st.zenithColor;
    lit.sky.horizonColor     = st.horizonColor;
    lit.sky.groundColor      = st.groundColor;

    lit.ambientMultiplier = st.ambient;

    // Night exposure adaptation (see header): scale the authored exposure as
    // the sun sinks. Captured lazily so it reads the level's value after load;
    // the mix keeps daytime EXACTLY the authored exposure.
    if (baseExposure_ < 0.0f) baseExposure_ = lit.exposure;
    const float dayF = static_cast<float>(
        std::min(1.0, std::max(0.0, (st.solarElevation + 0.10) / 0.30)));
    const float dayEase = dayF * dayF * (3.0f - 2.0f * dayF);
    lit.exposure = baseExposure_ * (1.0f + (kNightAdapt - 1.0f) * (1.0f - dayEase));
}

// NIGHT GLOW (WS3): everything the loader tagged — street-lamp glow shells,
// lit building windows — fades in on the shared dusk ramp, so every light in
// the city agrees with the headlights on when evening starts. Runs even when
// the CYCLE is off: a static level authored at dusk still glows, because
// solarElevation carries the authored sun's truth. Emission-only writes — no
// structural mutation inside each() (ADR-0006).
void DayNightSystem::applyNightGlow(FrameContext& ctx) {
    const Real ramp = duskRamp(ctx.view.lighting.solarElevation);
    if (ramp == lastGlowRamp_) return;   // emission writes only on change
    lastGlowRamp_ = ramp;
    ctx.world.each<NightGlow, Renderable>(
        [&](Entity, NightGlow& glow, Renderable& r) {
            r.material.emission = glow.fullEmission * ramp;
        });
    ctx.world.each<NightGlow, InstanceGroup>(
        [&](Entity, NightGlow& glow, InstanceGroup& g) {
            g.material.emission = glow.fullEmission * ramp;
        });
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

    // The VOLUMETRIC deck (cinematic sky) reads lighting.volumetricClouds, not
    // sky.cloud* — the renderer retires the 2D overlay while it is active, so
    // until this block every knob above was DEAD in volumetric levels (the
    // weather round's storm never arrived; measured live, `weather?`=storm
    // over a fair sky). Semantics differ: the deck's density is EXTINCTION
    // PER METRE (0.05 is the hand-tuned value; 0.55 saturated the march in
    // one step — see renderer.h), so the 0-1 artistic knob maps into a narrow
    // physical band around that tuning. The level's authored params are
    // captured once and SEED the knobs (authored look wins at boot; the
    // knobs, clouds.apply, and weather move it live from there).
    auto& vc = ctx.view.lighting.volumetricClouds;
    if (vc.enabled) {
        if (!cloudBaseCaptured_) {
            cloudBase_ = vc;
            cloudBaseCaptured_ = true;
            cloudCoverage = vc.coverage;
            cloudDensity = std::clamp((vc.density - 0.02f) / 0.065f, 0.0f, 1.0f);
            cloudScale = 1.2f;      // identity vs the authored noise frequency
            sky.cloudCoverage = cloudCoverage;
            sky.cloudDensity = cloudDensity;
            sky.cloudScale = cloudScale;
        }
        vc.coverage = cloudCoverage;
        vc.density = 0.02f + 0.065f * cloudDensity;
        // Bigger artistic scale = bigger cells = LOWER noise frequency,
        // anchored to the level's authored value at the 1.2 identity point.
        vc.noiseScale =
            cloudBase_.noiseScale * (1.2f / std::max(cloudScale, 0.3f));
        vc.wind = cloudBase_.wind * cloudWindSpeed;
    }
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
