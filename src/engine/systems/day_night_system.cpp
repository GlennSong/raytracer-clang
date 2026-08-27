#include "day_night_system.h"
#include "../components.h"
#include "../procgen/city/road_net.h"   // RoadEntity: the city's footprint (light pollution)
#include "../vehicle_lamps.h"   // duskRamp: night glow shares the lamp boundary

#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DayNightSystem::onStart(FrameContext& ctx) {
    auto& s = ctx.settings;
    enabled         = s.getBool("daynight.enabled", enabled);
    cycle.timeOfDay = s.getDouble("daynight.timeOfDay", cycle.timeOfDay);
    // The loop length in REAL MINUTES. The retired "daynight.speed" key is
    // deliberately NOT read: a stale 0.02 (or 0.0) persisted in settings.json
    // would silently restore the 50-second day this replaced.
    cycle.dayMinutes  = s.getDouble("daynight.dayMinutes", cycle.dayMinutes);
    cycle.latitudeDeg = s.getDouble("daynight.latitude", cycle.latitudeDeg);
    cycle.dayOfYear   = static_cast<int>(
        s.getDouble("daynight.dayOfYear", cycle.dayOfYear));
    cycle.year        = static_cast<int>(s.getDouble("daynight.year", cycle.year));
    cycle.paused    = s.getBool("daynight.paused", cycle.paused);
    // The moon follows the calendar (the month); a lock (age in days) is an
    // artistic hold. The old "daynight.moonPhase" brightness scalar is not
    // read — its persisted 1.0 would have pinned every night to a full moon.
    cycle.moonLock = s.getDouble("daynight.moonLock", cycle.moonLock);
    skyHud_ = s.getBool("daynight.hud", skyHud_);
    stars_ = s.getBool("daynight.stars", stars_);
    milkyWay_ = static_cast<float>(s.getDouble("daynight.milkyWay", milkyWay_));
    lightPollution_ = static_cast<float>(s.getDouble("daynight.lightPollution", lightPollution_));

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
            seedFromConfig(c);
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
             << " dayMinutes=" << cycle.dayMinutes
             << " latitude=" << cycle.latitudeDeg
             << " dayOfYear=" << cycle.dayOfYear
             << " sunrise=" << cycle.sunriseHour()
             << " sunset=" << cycle.sunsetHour()
             << " moonAge=" << cycle.moonAgeDays()
             << " (" << DayNightCycle::phaseName(cycle.moonAgeDays()) << ")"
             << " hdrActive=" << hdrEnvironmentActive(ctx);

    if (enabled && levelEnabled && !hdrEnvironmentActive(ctx)) applyLighting(ctx);
    applyClouds(ctx);
    // RT_SKY_SVG=<path>: the day's sky chart at boot (the load-time
    // instrument idiom, beside RT_FURNITURE_SVG).
    if (const char* svg = std::getenv("RT_SKY_SVG")) writeSkyChart(svg);
}

void DayNightSystem::writeSkyChart(const std::string& path) {
    const SkyChart chart = buildSkyChart(cycle);
    if (writeSkyChartSvg(chart, path))
        LOG_INFO << "[sky] chart written: " << path << " (day " << chart.dayOfYear
                 << ", sun rise " << chart.sunriseHour << " set " << chart.sunsetHour
                 << ", moon " << chart.moonPhase << " rise " << chart.moonriseHour
                 << " set " << chart.moonsetHour << ")";
    else
        LOG_ERROR << "[sky] cannot write chart: " << path;
}

void DayNightSystem::onStop(FrameContext& ctx) {
    auto& s = ctx.settings;
    s.setBool("daynight.enabled", enabled);
    s.setDouble("daynight.timeOfDay", cycle.timeOfDay);
    s.setDouble("daynight.dayMinutes", cycle.dayMinutes);
    s.setDouble("daynight.latitude", cycle.latitudeDeg);
    s.setDouble("daynight.dayOfYear", cycle.dayOfYear);
    s.setDouble("daynight.year", cycle.year);
    s.setBool("daynight.paused", cycle.paused);
    s.setDouble("daynight.moonLock", cycle.moonLock);
    s.setBool("daynight.hud", skyHud_);
    s.setBool("daynight.stars", stars_);
    s.setDouble("daynight.milkyWay", milkyWay_);
    s.setDouble("daynight.lightPollution", lightPollution_);

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

// Level seeds (DayNightConfig): a level that authors its own day. Applied
// once; the settings-persisted values stand where the level says nothing.
void DayNightSystem::seedFromConfig(const DayNightConfig& c) {
    if (c.timeOfDay >= 0.0f) cycle.timeOfDay = c.timeOfDay;
    if (c.dayMinutes >= 0.0f) cycle.dayMinutes = c.dayMinutes;
    else if (c.speed > 0.0f) cycle.dayMinutes = 1.0 / (c.speed * 60.0);   // legacy days/sec
    if (c.latitude >= -90.0f) cycle.latitudeDeg = c.latitude;
    if (c.dayOfYear >= 1) cycle.dayOfYear = c.dayOfYear;
    if (c.newMoonDay >= 0.0f) cycle.newMoonDay = c.newMoonDay;
    if (c.lightPollution >= 0.0f) lightPollution_ = c.lightPollution;
    if (c.pollutionFalloff >= 0.0f) pollutionFalloff_ = c.pollutionFalloff;
}

// The city's footprint: the bounding circle of every road graph's nodes.
// Found once (roads are level data); a level with no roads has no city and
// no pollution anywhere.
void DayNightSystem::findCityBounds(FrameContext& ctx) {
    if (cityBoundsKnown_) return;
    double minX = 1e30, minZ = 1e30, maxX = -1e30, maxZ = -1e30;
    bool any = false;
    ctx.world.each<RoadEntity>([&](Entity, RoadEntity& r) {
        for (const RoadNode& n : r.graph.nodes) {
            minX = std::min(minX, static_cast<double>(n.pos.x));
            maxX = std::max(maxX, static_cast<double>(n.pos.x));
            minZ = std::min(minZ, static_cast<double>(n.pos.y));
            maxZ = std::max(maxZ, static_cast<double>(n.pos.y));
            any = true;
        }
    });
    if (!any) return;
    cityCentreX_ = (minX + maxX) * 0.5;
    cityCentreZ_ = (minZ + maxZ) * 0.5;
    // The bounding box's half-diagonal overstates a round city; 0.8 of it is
    // where the last streets actually are.
    cityRadius_ = 0.8 * std::hypot((maxX - minX) * 0.5, (maxZ - minZ) * 0.5);
    cityBoundsKnown_ = true;
    LOG_INFO << "[sky] city footprint for light pollution: centre (" << cityCentreX_ << ", "
             << cityCentreZ_ << ") radius " << cityRadius_ << " m, downtown glow "
             << lightPollution_ << ", dark sky " << pollutionFalloff_ << " m out";
}

void DayNightSystem::publishStatus(FrameContext& ctx, bool active) {
    char buf[384];
    const DayNightState st = cycle.evaluate();
    std::snprintf(buf, sizeof(buf),
                  "hour=%.4f tod=%.6f dayMinutes=%.2f sunrise=%.3f sunset=%.3f "
                  "daylight=%.3f latitude=%.1f dayOfYear=%d year=%d sunY=%.3f "
                  "moonAge=%.2f moonIllum=%.3f moonY=%.3f moonX=%.3f moonZ=%.3f "
                  "moonDisc=%.2f stars=%.3f pollution=%.2f phase=%s paused=%d active=%d",
                  cycle.timeOfDay * 24.0, cycle.timeOfDay, cycle.dayMinutes,
                  cycle.sunriseHour(), cycle.sunsetHour(), cycle.daylightFraction(),
                  cycle.latitudeDeg, cycle.dayOfYear, cycle.year,
                  ctx.view.lighting.solarElevation, st.moonAgeDays, st.moonIllumination,
                  st.moonDirection.y, st.moonDirection.x, st.moonDirection.z,
                  st.moonDiscIntensity, stars_ ? st.starVisibility : 0.0f,
                  ctx.view.lighting.sky.lightPollution,
                  DayNightCycle::phaseName(st.moonAgeDays),
                  cycle.paused ? 1 : 0, active ? 1 : 0);
    ctx.settings.setString("daynight.status", buf);
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
            weather.advanceAuto(dt * cycle.speed() * 24.0,
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
    cycle.dayMinutes = ws.getDouble("daynight.dayMinutes", cycle.dayMinutes);
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
        // `daynight minutes <n>`: the loop length, live (0 freezes the sun).
        const double setMinutes = cs.getDouble("daynight.setMinutes", -1.0);
        if (setMinutes >= 0.0) {
            cycle.dayMinutes = setMinutes;
            cs.setDouble("daynight.setMinutes", -1.0);
        }
        // `daynight day <1-365>`: the calendar day (season + moon follow).
        const double setDay = cs.getDouble("daynight.setDay", -1.0);
        if (setDay >= 1.0) {
            cycle.dayOfYear = static_cast<int>(setDay);
            cs.setDouble("daynight.setDay", -1.0);
            enabled = true;
        }
        // `daynight hud on|off`: the sky HUD.
        const double setHud = cs.getDouble("daynight.setHud", -1.0);
        if (setHud >= 0.0) {
            skyHud_ = setHud > 0.5;
            cs.setDouble("daynight.setHud", -1.0);
        }
        // `daynight chart <path>`: write the day's sky chart now.
        const std::string chartPath = cs.getString("daynight.chart", "");
        if (!chartPath.empty()) {
            cs.setString("daynight.chart", "");
            writeSkyChart(chartPath);
        }
        // `daynight moon <age-days>|auto`: hold the moon's age, or free it.
        // -2 = auto (the calendar), since -1 is the consumed sentinel.
        const double setMoon = cs.getDouble("daynight.setMoon", -1.0);
        if (setMoon >= 0.0 || setMoon <= -2.0) {
            cycle.moonLock = setMoon >= 0.0 ? setMoon : -1.0;
            cs.setDouble("daynight.setMoon", -1.0);
            enabled = true;
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
            seedFromConfig(c);
            configSeeded_ = true;
        }
    });
    // edits live even while the simulation is paused.
    const bool active = enabled && levelEnabled && !hdrEnvironmentActive(ctx);
    if (active) applyLighting(ctx);
    else ctx.view.lighting.clockHours = -1.0f;   // no clock staged: the sim keeps its own
    publishStatus(ctx, active);
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
    // AUTHORED values are the truth; the cycle is a day-SHAPE multiplier
    // (captured lazily, normalized against the curve's noon anchors). The
    // raw curve used to replace them — metro's sun 8.0 / ambient 0.6 ran at
    // 5.0 / 0.34, which is most of "the daytime lighting seems really harsh".
    if (baseSunIntensity_ < 0.0f) baseSunIntensity_ = lit.sun.intensity;
    if (baseAmbient_ < 0.0f) baseAmbient_ = lit.ambientMultiplier;
    if (!shadowCaptured_) { baseShadow_ = lit.shadowArtistic; shadowCaptured_ = true; }
    // Weather dims the light (overcast halves it, storm guts it) — and raises
    // the ambient fill + softens shadows below: an overcast sky is a giant
    // diffuser, not a dimmer switch.
    lit.sun.intensity = baseSunIntensity_ *
                        (st.lightIntensity / kCycleNoonSunIntensity) *
                        (weatherActive ? weather.sunScale : 1.0f);
    // Dusk predicates (vehicle lamps, street lights) key on the SUN's truth,
    // not the active light — the moon rising must not read as daylight.
    lit.solarElevation = st.solarElevation;
    // THE WORLD CLOCK (one clock, not two): the citysim bridge opens its day
    // at this hour and runs its schedules at this rate, so rush hour happens
    // under a morning sky and a 30-minute day is 30 minutes for the commuters
    // too — in LOCKSTEP: a `daynight <hour>` jump re-opens the city at that
    // hour, and a held sun (rate 0 here) holds the city's clock while the
    // traffic keeps moving (CityRenderSystem::setWorldClock).
    lit.clockHours = static_cast<float>(cycle.timeOfDay * 24.0);
    lit.clockHoursPerSecond =
        static_cast<float>(cycle.paused ? 0.0 : cycle.speed() * 24.0);

    lit.sky.sunDirection     = st.lightDirection;
    lit.sky.sunColor         = st.lightColor;
    lit.sky.sunDiscIntensity = st.skyDiscIntensity;
    // The moon as a drawn body (the month): lit by the TRUE sun.
    lit.sky.moonDirection     = st.moonDirection;
    lit.sky.sunTrueDirection  = st.sunDirection;
    lit.sky.moonDiscIntensity = st.moonDiscIntensity;
    lit.sky.moonIllumination  = st.moonIllumination;
    // The stars: the celestial frame + tonight's visibility.
    lit.sky.celestialX = st.celestialX;
    lit.sky.celestialY = st.celestialY;
    lit.sky.celestialZ = st.celestialZ;
    lit.sky.starVisibility = stars_ ? st.starVisibility : 0.0f;
    lit.sky.milkyWay = milkyWay_;
    // Light pollution at the camera: downtown glow inside the city's
    // footprint, a dark sky pollutionFalloff_ metres past its edge.
    findCityBounds(ctx);
    if (cityBoundsKnown_) {
        const Vec3 cam = ctx.view.camera.position;
        const double dx = cityCentreX_ - cam.x, dz = cityCentreZ_ - cam.z;
        const double dist = std::hypot(dx, dz);
        lit.sky.lightPollution = static_cast<float>(DayNightCycle::lightPollutionFor(
            dist - cityRadius_, lightPollution_, pollutionFalloff_));
        lit.sky.cityDirection = dist > 1.0 ? Vec3(dx / dist, 0.0, dz / dist) : Vec3(0.0, 0.0, 1.0);
    } else {
        lit.sky.lightPollution = 0.0f;
    }
    lit.sky.zenithColor      = st.zenithColor;
    lit.sky.horizonColor     = st.horizonColor;
    lit.sky.groundColor      = st.groundColor;

    lit.ambientMultiplier = baseAmbient_ * (st.ambient / kCycleNoonAmbient) *
                            (weatherActive ? weather.ambientScale : 1.0f);
    // Shadow response follows the weather: soften toward shadowless as the
    // deck closes (direct light under full overcast is barely directional).
    {
        const float soften = weatherActive ? weather.shadowSoften : 0.0f;
        lit.shadowArtistic = baseShadow_;
        lit.shadowArtistic.strength = baseShadow_.strength * (1.0f - soften);
        lit.shadowArtistic.ambientStrength =
            baseShadow_.ambientStrength * (1.0f - soften);
    }

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

#ifdef RT_ENABLE_IMGUI
// The compass strip + polar chart. Bearings are compass bearings (north =
// -z); the strip is centred on the camera's heading so "the sun is 40 deg to
// your right" reads directly. Chart samples are cached per day (the cycle's
// hour changes every frame; the day's arcs do not).
void DayNightSystem::drawSkyHud(FrameContext& ctx) {
    const long key = cycle.dayOfYear + 1000L * cycle.year +
                     static_cast<long>(cycle.moonLock * 10.0) * 1000000L +
                     static_cast<long>(cycle.latitudeDeg * 10.0) * 100000000L;
    if (key != hudChartKey_) {
        hudChart_ = buildSkyChart(cycle, 10);
        hudChartKey_ = key;
    }
    const DayNightState st = cycle.evaluate();
    const double sunBearing = skyBearingDeg(st.sunDirection);
    const double moonBearing = skyBearingDeg(st.moonDirection);
    const bool sunUp = st.sunDirection.y > 0.0, moonUp = st.moonDirection.y > 0.0;
    // Camera heading from the render view's look-at.
    const Vec3 fwd = ctx.view.camera.target - ctx.view.camera.position;
    const double camBearing = skyBearingDeg(normalize(Vec3(fwd.x, 0.0, fwd.z)));

    const ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(ImVec2(vp->Pos.x + vp->Size.x * 0.5f, vp->Pos.y + 8.0f),
                            ImGuiCond_Always, ImVec2(0.5f, 0.0f));
    ImGui::SetNextWindowBgAlpha(0.45f);
    ImGui::Begin("Sky", nullptr,
                 ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings |
                     ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoNav);
    ImDrawList* dl = ImGui::GetWindowDrawList();
    const ImU32 cText = IM_COL32(200, 211, 234, 255), cDim = IM_COL32(93, 111, 149, 255);
    const ImU32 cSun = IM_COL32(255, 209, 102, 255), cMoon = IM_COL32(232, 236, 255, 255);
    const ImU32 cSunArc = IM_COL32(255, 179, 71, 255), cMoonArc = IM_COL32(170, 182, 224, 255);

    // --- compass strip: 360 deg across, camera heading at the centre ----------
    {
        const float w = 440.0f, h = 30.0f;
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::Dummy(ImVec2(w, h));
        const float cxs = p0.x + w * 0.5f, base = p0.y + 20.0f;
        dl->AddLine(ImVec2(p0.x, base), ImVec2(p0.x + w, base), cDim, 1.0f);
        auto xOf = [&](double bearing) {
            double rel = std::fmod(bearing - camBearing + 540.0, 360.0) - 180.0;   // -180..180
            return cxs + static_cast<float>(rel / 180.0) * (w * 0.5f);
        };
        for (int b = 0; b < 360; b += 30) {
            const float x = xOf(b);
            const bool cardinal = b % 90 == 0;
            dl->AddLine(ImVec2(x, base - (cardinal ? 8.0f : 4.0f)), ImVec2(x, base), cDim, 1.0f);
            if (cardinal) {
                const char* n = b == 0 ? "N" : b == 90 ? "E" : b == 180 ? "S" : "W";
                dl->AddText(ImVec2(x - 4.0f, base - 22.0f), cText, n);
            }
        }
        // The camera's own heading marker.
        dl->AddTriangleFilled(ImVec2(cxs, base + 2.0f), ImVec2(cxs - 5.0f, base + 9.0f),
                              ImVec2(cxs + 5.0f, base + 9.0f), cText);
        // Sun and moon at their bearings; hollow when below the horizon.
        const float sx = xOf(sunBearing), mx = xOf(moonBearing);
        if (sunUp) dl->AddCircleFilled(ImVec2(sx, base - 3.0f), 6.0f, cSun);
        else dl->AddCircle(ImVec2(sx, base - 3.0f), 6.0f, cSun, 0, 1.5f);
        if (moonUp) dl->AddCircleFilled(ImVec2(mx, base - 3.0f), 4.5f, cMoon);
        else dl->AddCircle(ImVec2(mx, base - 3.0f), 4.5f, cMoon, 0, 1.5f);
    }
    // --- the polar chart: the day's arcs, the bodies now --------------------------
    {
        const float R = 72.0f;
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        ImGui::Dummy(ImVec2(R * 2.0f + 40.0f, R * 2.0f + 30.0f));
        const ImVec2 c(p0.x + R + 20.0f, p0.y + R + 14.0f);
        auto at = [&](const Vec3& d) {
            const ChartPoint q = skyChartPoint(d);
            return ImVec2(c.x + static_cast<float>(q.x) * R, c.y + static_cast<float>(q.y) * R);
        };
        dl->AddCircleFilled(c, R, IM_COL32(22, 32, 58, 200));
        dl->AddCircle(c, R, IM_COL32(138, 160, 200, 255), 0, 1.5f);
        dl->AddCircle(c, R * (2.0f / 3.0f), cDim, 0, 1.0f);
        dl->AddCircle(c, R / 3.0f, cDim, 0, 1.0f);
        dl->AddText(ImVec2(c.x - 4.0f, c.y - R - 14.0f), cText, "N");
        dl->AddText(ImVec2(c.x + R + 4.0f, c.y - 7.0f), cText, "E");
        dl->AddText(ImVec2(c.x - 4.0f, c.y + R + 1.0f), cText, "S");
        dl->AddText(ImVec2(c.x - R - 12.0f, c.y - 7.0f), cText, "W");
        auto arc = [&](const std::vector<SkyChartSample>& s, ImU32 col, float th) {
            ImVec2 prev;
            bool have = false;
            for (const SkyChartSample& q : s) {
                if (!q.up) { have = false; continue; }
                const ImVec2 v = at(q.dir);
                if (have) dl->AddLine(prev, v, col, th);
                prev = v;
                have = true;
            }
        };
        arc(hudChart_.moon, cMoonArc, 1.5f);
        arc(hudChart_.sun, cSunArc, 2.5f);
        auto body = [&](const Vec3& d, bool up, float r, ImU32 col) {
            Vec3 dd = d;
            if (!up) {
                const double b = skyBearingDeg(d) * 3.14159265358979323846 / 180.0;
                dd = Vec3(std::sin(b), 0.0, -std::cos(b));
            }
            if (up) dl->AddCircleFilled(at(dd), r, col);
            else dl->AddCircle(at(dd), r, col, 0, 1.5f);
        };
        body(st.moonDirection, moonUp, 4.0f, cMoon);
        body(st.sunDirection, sunUp, 5.5f, cSun);
    }
    ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.4f, 1.0f), "sun  %s alt %+.0f  az %03.0f",
                       sunUp ? "up  " : "down", skyAltitudeDeg(st.sunDirection), sunBearing);
    ImGui::TextColored(ImVec4(0.91f, 0.93f, 1.0f, 1.0f), "moon %s alt %+.0f  az %03.0f  %s %.0f%%",
                       moonUp ? "up  " : "down", skyAltitudeDeg(st.moonDirection), moonBearing,
                       DayNightCycle::phaseName(st.moonAgeDays), st.moonIllumination * 100.0f);
    ImGui::End();
}
#endif

void DayNightSystem::render(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    // Debug-mode only (backtick), and into the shared "Debug" window — the
    // headers used to land in ImGui's implicit fallback window, which is why
    // they floated over plain play.
    if (!ctx.debugOverlayActive) return;
    if (skyHud_) drawSkyHud(ctx);
    ImGui::Begin("Debug");
    if (ImGui::CollapsingHeader("Day / Night")) {
        ImGui::Checkbox("Sky HUD (compass + chart)", &skyHud_);
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
        float minutes = static_cast<float>(cycle.dayMinutes);
        if (ImGui::SliderFloat("Day length (real min)", &minutes, 0.0f, 120.0f,
                               "%.1f", ImGuiSliderFlags_Logarithmic)) {
            cycle.dayMinutes = minutes;
        }
        float lat = static_cast<float>(cycle.latitudeDeg);
        if (ImGui::SliderFloat("Latitude (deg N)", &lat, -66.0f, 66.0f, "%.1f")) {
            cycle.latitudeDeg = lat;
            if (enabled) applyLighting(ctx);
        }
        int doy = cycle.dayOfYear;
        if (ImGui::SliderInt("Day of year", &doy, 1, 365)) {
            cycle.dayOfYear = doy;
            if (enabled) applyLighting(ctx);
        }
        // The stars.
        if (ImGui::Checkbox("Stars##stars", &stars_)) { if (enabled) applyLighting(ctx); }
        ImGui::SameLine();
        if (ImGui::SliderFloat("Milky Way##stars", &milkyWay_, 0.0f, 2.0f, "%.2f")) {
            if (enabled) applyLighting(ctx);
        }
        if (ImGui::SliderFloat("Light pollution (downtown)##stars", &lightPollution_, 0.0f, 1.0f, "%.2f")) {
            if (enabled) applyLighting(ctx);
        }
        ImGui::Text("Pollution here: %.2f (dark sky %.0f m past the city edge)",
                    ctx.view.lighting.sky.lightPollution, pollutionFalloff_);
        // The month: the moon's age from the calendar, or a held age.
        {
            const double age = cycle.moonAgeDays();
            ImGui::Text("Moon: %s, %.0f%% lit, age %.1f d (year %d)",
                        DayNightCycle::phaseName(age), cycle.moonIllumination() * 100.0,
                        age, cycle.year);
            bool locked = cycle.moonLock >= 0.0;
            if (ImGui::Checkbox("Hold moon##moonlock", &locked)) {
                cycle.moonLock = locked ? age : -1.0;
                if (enabled) applyLighting(ctx);
            }
            if (locked) {
                ImGui::SameLine();
                float held = static_cast<float>(cycle.moonLock);
                if (ImGui::SliderFloat("Age (days)##moonlock", &held, 0.0f,
                                       static_cast<float>(kSynodicMonthDays), "%.1f")) {
                    cycle.moonLock = held;
                    if (enabled) applyLighting(ctx);
                }
            }
        }
        // Readout: 24h clock derived from time-of-day (0.0 == midnight), and
        // today's horizon crossings so the day/night asymmetry is legible.
        auto hm = [](double hours, int& h, int& m) {
            const int total = static_cast<int>(hours * 60.0) % (24 * 60);
            h = total / 60;
            m = total % 60;
        };
        int ch, cm, rh, rm, sh, sm;
        hm(cycle.timeOfDay * 24.0, ch, cm);
        hm(cycle.sunriseHour(), rh, rm);
        hm(cycle.sunsetHour(), sh, sm);
        ImGui::Text("Clock: %02d:%02d   sunrise %02d:%02d  sunset %02d:%02d  "
                    "(%.0f%% daylight)",
                    ch, cm, rh, rm, sh, sm, cycle.daylightFraction() * 100.0);
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
