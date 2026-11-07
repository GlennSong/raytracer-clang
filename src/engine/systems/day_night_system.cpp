#include "day_night_system.h"
#include "../components.h"
#include "../procgen/city/roads/road_entity.h"   // RoadEntity: the city's footprint (light pollution)
#include "../procgen/noise.h"
#include "../procgen/terrain.h"          // terrainHeight: the horizon raster
#include "../vehicle_lamps.h"   // duskRamp: night glow shares the lamp boundary
#include "../lights/light_kinds.h"   // lightBook: the glows' emission scale (ADR-0369)

#include "../../log.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

void DayNightSystem::onStart(FrameContext& ctx) {
    onDay_ = ctx.events.listen<DayNightRequest>([this](const DayNightRequest& r) { dayRequests_.push_back(r); });
    onWeather_ = ctx.events.listen<WeatherRequest>([this](const WeatherRequest& r) {
        weatherRequest_ = r;
        weatherPending_ = true;
    });
    onClouds_ = ctx.events.listen<CloudsRequest>([this](const CloudsRequest& r) { cloudRequests_.push_back(r); });
    if (ctx.commands != nullptr) {
        // The clock as numbers: the sky's hour, the loop length, today's sunrise/sunset, and the city sim's own hour
        // beside it -- with a staged cycle the two must agree (one clock). Two reads a real minute apart MEASURE the
        // loop's rate.
        EventBus* bus = &ctx.events;
        answerClock_ = ctx.commands->bind("daynight?", "DayNightSystem", [this, bus](const ControlCommand&) {
            double simHours = -1.0;
            bus->publish(SimHourQuery{&simHours});
            char sim[48];
            std::snprintf(sim, sizeof(sim), " sim=%.4f", simHours);
            return "ok " + status_ + sim;
        });
        answerWeather_ = ctx.commands->bind("weather?", "DayNightSystem", [this](const ControlCommand&) {
            return "ok " + (weatherActive ? std::string(weatherKindName(weather.state)) + (weather.autoMode ? " (auto)" : "")
                                          : std::string("off"));
        });
    }
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
    terrainShadows_ = s.getBool("daynight.terrainShadows", terrainShadows_);

    cloudsEnabled  = s.getBool("clouds.enabled", cloudsEnabled);
    cloudCoverage  = static_cast<float>(s.getDouble("clouds.coverage", cloudCoverage));
    cloudDensity   = static_cast<float>(s.getDouble("clouds.density", cloudDensity));
    cloudScale     = static_cast<float>(s.getDouble("clouds.scale", cloudScale));
    cloudWindSpeed = static_cast<float>(s.getDouble("clouds.windSpeed", cloudWindSpeed));

    // THE WEATHER AT START (ADR-0340): a state chosen by hand (`weather storm`, the panel) survives the session -- an
    // explicit override, snapped; AUTO weather does not: it is the calendar's, drawn in update() from the seed, the
    // in-world date and the level's climate, so a given day always has its own weather (it used to resume whatever the
    // last session quit with, and the day's roll almost never ran).
    weatherSeed_ = static_cast<uint32_t>(s.getDouble("weather.seed", 7.0));
    const std::string ws = s.getString("weather.state", "");
    if (!ws.empty() && ws != "off" && !s.getBool("weather.auto", false)) {
        WeatherKind kind;
        if (weatherKindFromName(ws, kind)) {
            weatherActive = true;
            weatherFromSettings_ = true;
            weather.state = kind;
            weather.autoMode = false;
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
    if (!startApplied_) applyLaunchStart(ctx);

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

    buildHeightRaster(ctx);
    if (enabled && levelEnabled && !hdrEnvironmentActive(ctx)) applyLighting(ctx);
    applyClouds(ctx);
    // RT_SKY_SVG=<path>: the day's sky chart at boot (the load-time
    // instrument idiom, beside RT_FURNITURE_SVG).
    if (const char* svg = std::getenv("RT_SKY_SVG")) writeSkyChart(svg);
}

// The terrain's height field on a 256 x 256 raster over the world square —
// the analytic field the CDLOD tiles are built from, so the horizon is the
// horizon of what is drawn. ~65k samples once at start.
void DayNightSystem::buildHeightRaster(FrameContext& ctx) {
    raster_ = HeightRaster{};
    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) {
        if (!cfg) cfg = &c;
    });
    if (!cfg) return;
    const TerrainParams params = cfg->params;
    const Noise noise(cfg->seed);
    const double half = cfg->worldHalf;
    raster_ = rasterizeHeights(
        [&](double x, double z) { return terrainHeight(params, noise, x, z); },
        -half, -half, 2.0 * half, 256);
    double lo = 1e30, hi = -1e30;
    for (float h : raster_.height) { lo = std::min(lo, static_cast<double>(h)); hi = std::max(hi, static_cast<double>(h)); }
    LOG_INFO << "[sky] terrain horizon raster: " << raster_.size << "^2 over " << 2.0 * half
             << " m, heights " << lo << " .. " << hi << " m";
}

// Re-march the horizon toward the active light when it has moved > 2 deg,
// upload it, and read the camera's own texel for the disc.
void DayNightSystem::refreshHorizon(FrameContext& ctx, const DayNightState& st) {
    auto& lit = ctx.view.lighting;
    if (!terrainShadows_ || raster_.empty()) {
        if (horizonTexValid_) {
            ctx.renderer.setTerrainHorizon(TextureHandle{}, 0, 0, 0, 0, 0);
            ctx.renderer.removeTexture(horizonTex_);
            horizonTexValid_ = false;
            horizonAzimuth_ = 1e9;
        }
        ridgeAtCamera_ = -1.0f;
        lightBehindRidge_ = 0.0f;
        return;
    }
    const Vec3 L = st.lightDirection;
    const double az = skyBearingDeg(L);
    // Stale when never marched, or when the light has swung > 2 deg (the
    // first cut wrapped the delta through 360 with the "never" sentinel
    // still in it and came out negative: the map was never built at all —
    // `daynight?` reported ridge=-0.100, the encoding floor, everywhere).
    bool stale = horizonAzimuth_ > 720.0;
    if (!stale) {
        double dAz = std::fabs(az - horizonAzimuth_);
        if (dAz > 180.0) dAz = 360.0 - dAz;
        stale = dAz > 2.0;
    }
    if (stale) {
        std::function<void(int, const std::function<void(int)>&)> par =
            [&](int rows, const std::function<void(int)>& body) {
                ctx.jobs.parallelFor(0, static_cast<std::size_t>(rows),
                                     [&](std::size_t z) { body(static_cast<int>(z)); });
            };
        const auto tMarch0 = std::chrono::steady_clock::now();
        horizon_ = computeHorizonMap(raster_, az, 4000.0, 2.0, &par);
        const double marchMs = std::chrono::duration<double, std::milli>(
                                   std::chrono::steady_clock::now() - tMarch0).count();
        LOG_INFO << "[sky] terrain horizon re-marched at az " << static_cast<int>(az)
                 << " deg in " << static_cast<int>(marchMs) << " ms (on the frame)";
        TextureHandle fresh = ctx.renderer.uploadTexture(
            horizon_.size, horizon_.size, 1, horizon_.sinElevation.data());
        ctx.renderer.setTerrainHorizon(fresh, static_cast<float>(horizon_.originX),
                                       static_cast<float>(horizon_.originZ),
                                       static_cast<float>(horizon_.extent),
                                       static_cast<float>(HorizonMap::kEncodeLo),
                                       static_cast<float>(HorizonMap::kEncodeHi));
        if (horizonTexValid_) ctx.renderer.removeTexture(horizonTex_);
        horizonTex_ = fresh;
        horizonTexValid_ = true;
        horizonAzimuth_ = az;
    }
    // The camera's texel: is the light behind the ridge from HERE? Then the
    // disc (and the bloom it feeds) is gone, whichever body slot 0 carries.
    const Vec3 cam = ctx.view.camera.position;
    ridgeAtCamera_ = static_cast<float>(horizon_.sinElevationAt(cam.x, cam.z));
    const double behind = 1.0 - std::clamp((L.y - (ridgeAtCamera_ - 0.01)) / 0.04, 0.0, 1.0);
    lightBehindRidge_ = static_cast<float>(behind);
    // The disc and its tight aureole (what the bloom feeds on) go with the
    // ridge; the WIDE twilight glow only softens — forward scatter along the
    // line of sight is real sky, not the sun, and stays after a mountain
    // sunset (so the sky still says where the sun went, without a halo).
    const bool sunActive = dot(st.lightDirection, st.sunDirection) > 0.999;
    if (sunActive) {
        lit.sky.sunDiscIntensity *= static_cast<float>(1.0 - behind);
        lit.sky.sunGlowIntensity *= static_cast<float>(1.0 - 0.6 * behind);
    } else {
        lit.sky.moonDiscIntensity *= static_cast<float>(1.0 - behind);
    }
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
    s.setBool("daynight.terrainShadows", terrainShadows_);

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

// THE LAUNCH'S START TIME (ADR-0338): RT_TIME for this run, else the saved preference daynight.start -- "saved" (the
// default: the saved or level time stands), "now" (the wall clock and today's date, at real speed), an hour, or a moment
// (sunrise, sunset, dusk, ...). After the level's seeds, so a launch's choice wins over both. Once.
void DayNightSystem::applyLaunchStart(FrameContext& ctx) {
    startApplied_ = true;
    const char* env = std::getenv("RT_TIME");
    const std::string spec = env && *env ? std::string(env) : ctx.settings.getString("daynight.start", "saved");
    std::string what;
    if (applyStartTime(cycle, spec, &what)) {
        LOG_INFO << "[daynight] start time " << what;
        // the date may have moved (now): auto weather opens AS the new day's, not easing to it from the saved one's
        if (weatherActive && weather.autoMode && weatherDay_ >= 0 && cycle.dayOfYear != weatherDay_) {
            weather.state = dayWeather();
            weatherDay_ = cycle.dayOfYear;
            weather.snap();
            LOG_INFO << "[weather] today (day " << cycle.dayOfYear << "): " << weatherKindName(weather.state) << " (the calendar)";
        }
    } else if (!what.empty()) {
        LOG_WARN << "[daynight] " << what;
    }
}

// Level seeds (DayNightConfig): a level that authors its own day. Applied
// once; the settings-persisted values stand where the level says nothing.
void DayNightSystem::seedFromConfig(const DayNightConfig& c) {
    if (c.timeOfDay >= 0.0f) cycle.timeOfDay = c.timeOfDay;
    // REAL CLOCK: open at the wall clock's time of day. Applied ONCE here, not
    // polled -- CityRenderSystem::setWorldClock re-seeds the whole population
    // whenever the sky hour jumps past its tolerance, so a periodic resync
    // would rebuild the city on a timer. At dayMinutes 1440 the drift across a
    // session is seconds.
    if (c.clockSource == 1) {
        cycle.timeOfDay = wallClockTimeOfDay(c.utcOffsetHours);
        LOG_INFO << "[daynight] real clock: opening at "
                 << (cycle.timeOfDay * 24.0) << " h (utcOffsetHours "
                 << c.utcOffsetHours << ")";
    }
    if (c.dayMinutes >= 0.0f) cycle.dayMinutes = c.dayMinutes;
    else if (c.speed > 0.0f) cycle.dayMinutes = 1.0 / (c.speed * 60.0);   // legacy days/sec
    if (c.latitude >= -90.0f) cycle.latitudeDeg = c.latitude;
    if (c.dayOfYear >= 1) cycle.dayOfYear = c.dayOfYear;
    if (c.newMoonDay >= 0.0f) cycle.newMoonDay = c.newMoonDay;
    if (c.lightPollution >= 0.0f) lightPollution_ = c.lightPollution;
    if (c.pollutionFalloff >= 0.0f) pollutionFalloff_ = c.pollutionFalloff;
    // The weather policy: a persisted state (yesterday's storm) still wins;
    // otherwise the level's word, and a level that says nothing gets AUTO
    // if it has a volumetric deck (applied in update, where the deck is known).
    if (c.hasClimate) climate_ = c.climate;   // ADR-0340
    if (!weatherFromSettings_ && c.weather >= 0) {
        weatherDefaulted_ = true;
        if (c.weather == 0) {
            weatherActive = false;
        } else if (c.weather == 1) {
            weatherActive = true;
            weather.autoMode = true;
            weather.state = dayWeather();   // (the settings' seed: it was a fixed 7 here, the default path's own)
            weatherDay_ = cycle.dayOfYear;
            weather.snap();
        } else {
            weatherActive = true;
            weather.autoMode = false;
            weather.state = static_cast<WeatherKind>(c.weather - 2);
            weather.snap();
        }
        LOG_INFO << "[weather] level policy: " << (weatherActive ? weatherKindName(weather.state) : "off")
                 << (weather.autoMode ? " (auto)" : "");
    }
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

void DayNightSystem::applyRequests(FrameContext&) {
    for (const DayNightRequest& r : dayRequests_) {
        if (r.hour >= 0.0) {
            // HOURS on the wire, FRACTION in the cycle (0.5 = noon). The first
            // cut dropped hours straight in; the sun trig wraps modulo one DAY
            // FRACTION, so `daynight 12` landed on 12 mod 1 = midnight-and-
            // three-quarters — sunset. Every "golden hour" was found by eye,
            // one wrong number away.
            cycle.timeOfDay = std::fmod(r.hour, 24.0) / 24.0;
            // Asking for an hour IS asking for the cycle's lighting: a level
            // that booted with the cycle off (static authored sun) would
            // otherwise take the time and change nothing — the papercut
            // that ate the first moonlit-midnight attempt.
            enabled = true;
        }
        if (r.hold >= 0) cycle.paused = r.hold > 0;   // "artistic hold", not the sim clock
        if (r.minutes >= 0.0) cycle.dayMinutes = r.minutes;
        if (r.day >= 1) {
            cycle.dayOfYear = r.day;
            enabled = true;
        }
        if (r.terrain >= 0) terrainShadows_ = r.terrain > 0;
        if (r.hud >= 0) skyHud_ = r.hud > 0;
        if (!r.chart.empty()) writeSkyChart(r.chart);
        if (r.moon >= 0.0 || r.moonAuto) {
            cycle.moonLock = r.moonAuto ? -1.0 : r.moon;
            enabled = true;
        }
    }
    dayRequests_.clear();
    // `clouds ...`: the cloud deck live (coverage/density/scale/wind/enabled
    // are otherwise state-entry-only, which made "why is the sky so heavy"
    // answerable only by a reload). onStop persists them with the rest.
    for (const CloudsRequest& r : cloudRequests_) {
        if (r.enabled >= 0) cloudsEnabled = r.enabled > 0;
        if (r.coverage >= 0.0) cloudCoverage = static_cast<float>(r.coverage);
        if (r.density >= 0.0) cloudDensity = static_cast<float>(r.density);
        if (r.scale >= 0.0) cloudScale = static_cast<float>(r.scale);
        if (r.wind >= 0.0) cloudWindSpeed = static_cast<float>(r.wind);
    }
    cloudRequests_.clear();
    if (!weatherPending_) return;
    weatherPending_ = false;
    const WeatherRequest& w = weatherRequest_;
    WeatherKind kind;
    if (w.state == "off") {
        weatherActive = false;
    } else if (w.state == "auto") {
        weatherActive = true;
        weather.autoMode = true;
        weather.state = dayWeather();   // back to the calendar: today's weather (ADR-0340)
        weatherDay_ = cycle.dayOfYear;
        if (w.now) weather.snap();
    } else if (weatherKindFromName(w.state, kind)) {
        // `weather storm` eases the front in over ~40 s; `weather storm now`
        // snaps it (screenshots, the editor's compare).
        weatherActive = true;
        weather.autoMode = false;
        weather.state = kind;
        if (w.now) weather.snap();
    }
}

void DayNightSystem::publishStatus(FrameContext& ctx, bool active) {
    char buf[384];
    const DayNightState st = cycle.evaluate();
    std::snprintf(buf, sizeof(buf),
                  "hour=%.4f tod=%.6f dayMinutes=%.2f sunrise=%.3f sunset=%.3f "
                  "daylight=%.3f latitude=%.1f dayOfYear=%d year=%d sunY=%.3f "
                  "moonAge=%.2f moonIllum=%.3f moonY=%.3f moonX=%.3f moonZ=%.3f "
                  "moonDisc=%.2f stars=%.3f pollution=%.2f ridge=%.3f behind=%.2f phase=%s paused=%d active=%d",
                  cycle.timeOfDay * 24.0, cycle.timeOfDay, cycle.dayMinutes,
                  cycle.sunriseHour(), cycle.sunsetHour(), cycle.daylightFraction(),
                  cycle.latitudeDeg, cycle.dayOfYear, cycle.year,
                  ctx.view.lighting.solarElevation, st.moonAgeDays, st.moonIllumination,
                  st.moonDirection.y, st.moonDirection.x, st.moonDirection.z,
                  st.moonDiscIntensity, stars_ ? st.starVisibility : 0.0f,
                  ctx.view.lighting.sky.lightPollution, ridgeAtCamera_, lightBehindRidge_,
                  DayNightCycle::phaseName(st.moonAgeDays),
                  cycle.paused ? 1 : 0, active ? 1 : 0);
    status_ = buf;
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
    // the cloud knobs — the ImGui sliders and `clouds` rule again after
    // `weather off`.
    // No policy anywhere (no persisted state, the level silent): a volumetric
    // deck gets today's weather and the walk, so days differ by default.
    if (!weatherDefaulted_ && !weatherFromSettings_ && ctx.view.lighting.volumetricClouds.enabled) {
        weatherDefaulted_ = true;
        weatherActive = true;
        weather.autoMode = true;
        weather.state = dayWeather();
        weatherDay_ = cycle.dayOfYear;
        weather.snap();
        LOG_INFO << "[weather] today (day " << cycle.dayOfYear << "): " << weatherKindName(weather.state)
                 << " (auto)";
    }
    if (weatherActive) {
        const double dt = ctx.clock.fixedStep();
        // A NEW DAY under auto weather: the calendar's state for it, eased in (a front arriving overnight), and the walk
        // starts afresh, seeded by the day (ADR-0340)
        if (weather.autoMode && weatherDay_ >= 0 && cycle.dayOfYear != weatherDay_) {
            weather.state = dayWeather();
            weather.autoSteps = 0;
            weather.sinceStepHours = 0.0;
            LOG_INFO << "[weather] day " << cycle.dayOfYear << ": " << weatherKindName(weather.state) << " (the calendar)";
        }
        if (weather.autoMode) weatherDay_ = cycle.dayOfYear;
        if (!cycle.paused)
            weather.advanceAuto(dt * cycle.speed() * 24.0, weatherDaySeed(weatherSeed_, cycle.dayOfYear, cycle.year));
        weather.ease(dt);
        cloudCoverage = weather.coverage;
        cloudDensity  = weather.density;
        cloudScale    = weather.scale;
        cloudTypeLo   = weather.typeLo;
        cloudTypeHi   = weather.typeHi;
    } else {
        cloudTypeLo = cloudTypeHi = -1.0f;   // (the level's own range)
    }
    // THE WIND (ADR-0337): the weather's speed and gustiness (a moderate breeze with the weather off); its direction
    // veers slowly through the day about the seed's prevailing one (+-30 degrees over ~26 in-world hours)
    {
        // (the climate's prevailing wind, ADR-0340: it comes FROM that compass bearing, so it goes toward the opposite;
        // north is -z, east +x)
        const double hours = cycle.absoluteDay() * 24.0;
        const double toward = (climate_.prevailingFromDeg + 180.0) * 0.017453292519943295 +
                              0.52 * std::sin(hours * 6.283185307 / 26.0) + 0.17 * std::sin(hours * 6.283185307 / 7.3);
        wind.dir = Vec3(std::sin(toward), 0.0, -std::cos(toward));
        wind.speed = (weatherActive ? weather.windSpeed : 6.0) * climate_.windScale;
        wind.gust = weatherActive ? weather.windGust : 0.35;
        wind.time += ctx.clock.fixedStep();
        if (!ctx.world.alive(windEntity_)) {
            windEntity_ = ctx.world.create();
            ctx.world.add<Wind>(windEntity_, wind);
        } else if (Wind* w = ctx.world.get<Wind>(windEntity_)) {
            *w = wind;
        }
        ctx.renderer.windDir[0] = static_cast<float>(wind.dir.x);
        ctx.renderer.windDir[1] = static_cast<float>(wind.dir.z);
        ctx.renderer.windSpeed = static_cast<float>(wind.speed);
        ctx.renderer.windGust = static_cast<float>(wind.gust);
    }
}

void DayNightSystem::update(FrameContext& ctx) {
    lastDt_ = static_cast<float>(ctx.frameDelta);
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
    // The cycle's requests (DayNightRequest / WeatherRequest, ADR-0253),
    // applied here so the ImGui panel and the cycle's own animation stay the
    // owners between pokes. They work while the sim clock is paused (update
    // still runs): the "freeze golden hour, keep framing" workflow.
    applyRequests(ctx);

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
    if (!startApplied_) applyLaunchStart(ctx);
    // edits live even while the simulation is paused.
    const bool active = enabled && levelEnabled && !hdrEnvironmentActive(ctx);
    if (active) applyLighting(ctx);
    else { ctx.view.lighting.clockHours = -1.0f; nightAdapt_ = 1.0f; }   // no clock staged: the sim keeps its own
    publishStatus(ctx, active);
    applyClouds(ctx);
    applyNightGlow(ctx);   // even with the cycle off: static dusk levels glow
    applyBeaconBlink(ctx);
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
    lit.sky.sunGlowIntensity = st.skyDiscIntensity;   // the wide aureole; a ridge softens it
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
    refreshHorizon(ctx, st);
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
    nightAdapt_ = baseExposure_ > 0.0f ? lit.exposure / baseExposure_ : 1.0f;

    static const bool physical = [] { const char* e = std::getenv("RT_SKY"); return e && std::string(e) == "physical"; }();
    if (physical) applyPhysicalSky(ctx, st);
    else lit.physicalSky.enabled = false;
}

// THE PHYSICAL SKY (ADR-0367): the sun and the moon through the atmosphere model -- their colour and strength at the
// camera, the sky's colours for everything that reads the day model's (the clouds, underwater, the glass's room), its
// light as the ambient -- and the renderer's sky view fed. Exposure, until the eye adapts (ADR-0368): an incident light
// meter on what falls on the ground, set so a clear noon gives the level's authored exposure.
void DayNightSystem::applyPhysicalSky(FrameContext& ctx, const DayNightState& st) {
    auto& lit = ctx.view.lighting;
    if (!skyTablesBuilt_) {
        buildAtmosphereTables(atmosphereFrom(lit.skyScattering), skyTables_);
        ctx.renderer.setSkyTables(skyTables_.transmittance.data(), AtmosphereTables::kTransW, AtmosphereTables::kTransH,
                                  skyTables_.multiScatter.data(), AtmosphereTables::kMsW, AtmosphereTables::kMsH);
        skyTablesBuilt_ = true;
    }
    const double height = std::max(0.0, ctx.view.camera.position.y);
    const double sunTop = kSunTopLux / kLuxPerUnit;
    const float moonTop = static_cast<float>(kFullMoonTopLux * st.moonIllumination / kLuxPerUnit);
    if ((st.sunDirection - readSunDir_).length() > 0.002 || std::fabs(height - readHeight_) > 50.0) {
        sunReading_ = readSky(skyTables_, height, st.sunDirection, sunTop);
        atmosphereIrradianceSH(skyTables_, height, st.sunDirection, sunTop, sunSH_);
        readSunDir_ = st.sunDirection;
        readHeight_ = height;
        readMoonDir_ = Vec3(0, -2, 0);   // (and the moon again, at this height)
    }
    if ((st.moonDirection - readMoonDir_).length() > 0.002 || std::fabs(moonTop - readMoonTop_) > 1e-9f) {
        moonReading_ = readSky(skyTables_, height, st.moonDirection, moonTop);
        atmosphereIrradianceSH(skyTables_, height, st.moonDirection, moonTop, moonSH_, 64);
        readMoonDir_ = st.moonDirection;
        readMoonTop_ = moonTop;
    }
    // the active light: whichever lights the ground more (the moon only once the sun is down)
    const double sunGround = sunReading_.lightIntensity * std::max(0.0, st.sunDirection.y);
    const double moonGround = moonReading_.lightIntensity * std::max(0.0, st.moonDirection.y);
    const bool sunActive = sunGround >= moonGround;
    const SkyReading& act = sunActive ? sunReading_ : moonReading_;
    const float weatherSun = weatherActive ? weather.sunScale : 1.0f;
    lit.sun.direction = sunActive ? st.sunDirection : st.moonDirection;
    lit.sun.color = act.lightColor;
    lit.sun.intensity = static_cast<float>(act.lightIntensity) * weatherSun;
    lit.sky.sunDirection = lit.sun.direction;
    lit.sky.sunColor = act.lightColor;
    lit.sky.zenithColor = sunReading_.zenith + moonReading_.zenith;
    lit.sky.horizonColor = sunReading_.horizon + moonReading_.horizon;
    lit.sky.groundColor = sunReading_.ground + moonReading_.ground;
    lit.ambientMultiplier = weatherActive ? weather.ambientScale : 1.0f;   // the sky's own light: no day-shape scale

    PhysicalSkyFrame& f = lit.physicalSky;
    const Atmosphere& a = skyTables_.atm;
    f.enabled = true;
    f.lightDir = lit.sun.direction;
    f.lightTop = static_cast<float>(sunActive ? sunTop : moonTop) * weatherSun;
    f.otherDir = sunActive ? st.moonDirection : st.sunDirection;
    f.otherTop = static_cast<float>(sunActive ? moonTop : sunTop);
    f.lightIsSun = sunActive;
    f.cameraHeight = static_cast<float>(height);
    f.groundRadius = static_cast<float>(a.groundRadius);
    f.topRadius = static_cast<float>(a.topRadius);
    f.rayleigh = a.rayleighScatter;
    f.rayleighHeight = static_cast<float>(a.rayleighHeight);
    f.mieScatter = static_cast<float>(a.mieScatter);
    f.mieExtinction = static_cast<float>(a.mieExtinction);
    f.mieHeight = static_cast<float>(a.mieHeight);
    f.mieG = static_cast<float>(a.mieG);
    f.ozone = a.ozoneAbsorb;
    f.groundAlbedo = a.groundAlbedo;
    f.sunAngularRadius = lit.skyScattering.sunAngularRadius;
    f.sunDisc = lit.sky.sunDiscIntensity > 0.0f ? 1.0f : 0.0f;
    f.celestialScale = 1.0e-5f;   // (the stars and the moon's disc were tuned at the old night exposure; ADR-0367)
    f.emissionScale = lightBook().emissionScale;   // (ADR-0369)
    for (int k = 0; k < 9; ++k) f.irradianceSH[k] = (sunSH_[k] + moonSH_[k]) * static_cast<double>(lit.ambientMultiplier);
    {
        static int dbg = 0;
        if (std::getenv("RT_SKY_DEBUG") && ++dbg % 120 == 1)
            LOG_INFO << "[sky] sh0 " << f.irradianceSH[0].x << " " << f.irradianceSH[0].y << " " << f.irradianceSH[0].z << " sh1 "
                     << f.irradianceSH[1].y << " amb " << lit.ambientMultiplier << " exposure " << lit.exposure << " light "
                     << lit.sun.intensity;
    }
    glowPerNit_ = 1.0 / (kLuxPerUnit * std::max(1e-6f, f.emissionScale));   // (the shader multiplies by the scale)
    // THE MOON'S DISC at its real luminance through the air, and the CITY'S GLOW on the sky (sky.frag's shapes peak at
    // 1 x the disc intensity and 0.06: ADR-0369)
    {
        const LightBook& lb = lightBook();
        const Vec3 tm = sampleTransmittance(skyTables_, a.groundRadius + std::max(height, 0.5), st.moonDirection.y);
        const double moonUnits = lb.moonNits / kLuxPerUnit * std::max(tm.x, std::max(tm.y, tm.z));
        if (lit.sky.moonDiscIntensity > 0.0f)
            lit.sky.moonDiscIntensity = static_cast<float>(moonUnits / std::max(1e-12f, f.celestialScale));
        f.cityGlowScale = static_cast<float>(lb.cityGlowNits / kLuxPerUnit / 0.06);
    }
    {   // the deck's own sun: from the middle of the cloud layer, whichever of the sun and the moon lights it more
        const VolumetricCloudParams& vc = lit.volumetricClouds;
        const double r = a.groundRadius + std::max(height, 0.5 * (vc.bottom + vc.top));
        const Vec3 tSun = sampleTransmittance(skyTables_, r, st.sunDirection.y) * sunTop;
        const Vec3 tMoon = sampleTransmittance(skyTables_, r, st.moonDirection.y) * static_cast<double>(moonTop);
        const bool sunLit = tSun.y >= tMoon.y;
        f.cloudLightDir = sunLit ? st.sunDirection : st.moonDirection;
        f.cloudLightColor = (sunLit ? tSun : tMoon) * static_cast<double>(weatherSun);
    }

    // THE METER: incident light on the ground (sun or moon, and the sky), EV100 = log2(E * 100 / C), exposure =
    // kLuxPerUnit / (1.2 * 2^EV); C = 400 puts a clear noon (~99,000 lux) at exposure 0.7. The level's authored
    // exposure is the bias against that.
    const double groundLux = (sunGround + moonGround + sunReading_.irradianceUp.y + moonReading_.irradianceUp.y) * kLuxPerUnit;
    const double ev = std::log2(std::max(groundLux, 1e-4) * 100.0 / 400.0);
    const double meter = kLuxPerUnit / (1.2 * std::exp2(ev));
    const double bias = baseExposure_ / 0.7;
    lit.exposure = static_cast<float>(meter * bias);

    // THE EYE (ADR-0368): the renderer's measure of the frame (the middle of its luminance histogram), and the exposure
    // that puts it at a key -- mid grey by day, lower in the dark, so a night street stays a night street -- eased
    // toward over a second or so (faster into the light than out of it), within EV100 -3 .. 17.
    // RT_EXPOSURE=meter: the meter alone (the A/B).
    static const bool meterOnly = [] { const char* e = std::getenv("RT_EXPOSURE"); return e && std::string(e) == "meter"; }();
    float log2L = 0.0f;
    if (!meterOnly && ctx.renderer.measuredSceneLuminance(log2L)) {
        const double L = std::exp2(static_cast<double>(log2L));   // units
        const double nits = L * kLuxPerUnit;
        const double s = std::clamp((std::log10(std::max(nits, 1e-6)) + 2.0) / 4.0, 0.0, 1.0);   // 0.01 .. 100 cd/m^2
        const double key = 0.18 * (0.25 + 0.75 * s * s * (3.0 - 2.0 * s));
        double target = key / std::max(L, 1e-12) * bias;
        const double lo = kLuxPerUnit / (1.2 * std::exp2(17.0)), hi = kLuxPerUnit / (1.2 * std::exp2(-3.0));
        target = std::clamp(target, lo, hi);
        if (autoExposure_ <= 0.0f) {
            autoExposure_ = static_cast<float>(target);
        } else {
            const double tau = target > autoExposure_ ? 1.2 : 0.5;   // into the dark slower, into the light faster
            const double k = 1.0 - std::exp(-std::max(0.0f, std::min(lastDt_, 0.25f)) / tau);
            autoExposure_ = static_cast<float>(std::exp(std::log(autoExposure_) + (std::log(target) - std::log(autoExposure_)) * k));
        }
        lit.exposure = autoExposure_;
    }
    // (the glows are in units now, scaled in the shader: no night-exposure compensation for the beacons and sprites)
    nightAdapt_ = 1.0f;
}

// The glow at one ramp, per ENTITY (see NightGlow::appliedRamp): the ramp holds still for long stretches (a staged
// hour, the cycle off) while building chunks stream in around the player; each is written once at the ramp it has
// not seen yet.
namespace {
// a glow's full emission: its real luminance in its colour under the physical sky, else its old value
Vec3 glowEmission(const NightGlow& glow, double perNit) {
    if (perNit <= 0.0 || glow.nits <= 0.0f) return glow.fullEmission;
    const Vec3& c = glow.fullEmission;
    const double peak = std::max(c.x, std::max(c.y, c.z));
    return peak > 0.0 ? c * (glow.nits * perNit / peak) : Vec3(0, 0, 0);
}
}  // namespace

void applyNightGlowRamp(World& world, Real ramp, double perNit) {
    const float rf = static_cast<float>(ramp);
    world.each<NightGlow, Renderable>(
        [&](Entity, NightGlow& glow, Renderable& r) {
            if (glow.appliedRamp == rf) return;
            glow.appliedRamp = rf;
            r.material.emission = glowEmission(glow, perNit) * ramp;
            if (glow.nightAlbedo < 1.0f)
                r.material.albedo = glow.dayAlbedo * (1.0 - (1.0 - glow.nightAlbedo) * ramp);
        });
    world.each<NightGlow, InstanceGroup>(
        [&](Entity, NightGlow& glow, InstanceGroup& g) {
            if (glow.appliedRamp == rf) return;
            glow.appliedRamp = rf;
            g.material.emission = glowEmission(glow, perNit) * ramp;
        });
}

// NIGHT GLOW (WS3): everything the loader tagged — street-lamp glow shells,
// lit building windows — fades in on the shared dusk ramp, so every light in
// the city agrees with the headlights on when evening starts. Runs even when
// the CYCLE is off: a static level authored at dusk still glows, because
// solarElevation carries the authored sun's truth. Emission-only writes — no
// structural mutation inside each() (ADR-0006).
void DayNightSystem::applyNightGlow(FrameContext& ctx) {
    const Real ramp = duskRamp(ctx.view.lighting.solarElevation);
    lastGlowRamp_ = ramp;
    applyNightGlowRamp(ctx.world, ramp, glowPerNit_);
}

// BEACON FLASH: the aviation lamps on tall roofs. Their chunks carry NightGlow
// (the dusk ramp, written above whenever it changes) and BeaconBlink; this
// pass re-writes their emission every frame as ramp × gate, so a beacon is
// dark by day, a red pulse at night. Few entities (one per beacon chunk), so
// the per-frame write is nothing.
void DayNightSystem::applyBeaconBlink(FrameContext& ctx) {
    blinkSeconds_ += std::max(0.0, std::min(ctx.frameDelta, 0.25));
    const Real ramp = lastGlowRamp_ < 0 ? Real(0) : lastGlowRamp_;
    // Divided by the night exposure adaptation (up to 6x at midnight): a red
    // lamp brighter than ~1 on screen tonemaps to orange, then white — the
    // beacon must read as the same saturated red at dusk and at midnight.
    const Real adapt = std::max(1.0f, nightAdapt_);
    ctx.view.lighting.beaconSeconds = static_cast<float>(blinkSeconds_);
    ctx.view.lighting.nightAdapt = static_cast<float>(adapt);
    const Vec3 camPos = ctx.view.camera.position;
    // The near tier's reach: the level's light LOD block (lightSphereOut), else 35 m.
    Real sphereOut = 35.0;
    ctx.world.each<CitySimConfig>([&](Entity, CitySimConfig& c) { sphereOut = c.lightSphereOut; });
    const Real sphereFade0 = sphereOut * 0.6;
    ctx.world.each<NightGlow, BeaconBlink, Renderable>(
        [&](Entity, NightGlow& glow, BeaconBlink& b, Renderable& r) {
            const Real gate = ramp <= 0 ? Real(0) : beaconBlinkGate(blinkSeconds_, b.period, b.phase, b.duty);
            r.material.emission = glowEmission(glow, glowPerNit_) * (ramp * gate / adapt);
            if (b.baseOpacity < 1.0f) {
                // The near tier: the translucent bulb and haze belong to the
                // roof you are standing on; past ~20 m the sprite carries
                // the glow, so the spheres fade out by 35 m (per chunk).
                const BoundingSphere bs = ctx.renderer.getMeshBounds(r.mesh);
                const Real d = (bs.center - camPos).length() - bs.radius;
                Real far = (d - sphereFade0) / std::max(Real(0.5), sphereOut - sphereFade0);
                far = std::min(Real(1), std::max(Real(0), far));
                r.material.opacity = b.baseOpacity * static_cast<float>(ramp * gate * (1.0 - far));
            }
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
    // knobs, `clouds`, and weather move it live from there).
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
        vc.typeLo = cloudTypeLo >= 0.0f ? cloudTypeLo : cloudBase_.typeLo;
        vc.typeHi = cloudTypeHi >= 0.0f ? cloudTypeHi : cloudBase_.typeHi;
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
        ImGui::Checkbox("Pause##daynight", &cycle.paused);   // (the clock's own Pause button shares the Debug window)
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
        if (ImGui::Checkbox("Terrain shadows (horizon map)##horizon", &terrainShadows_)) {
            if (enabled) applyLighting(ctx);
        }
        if (terrainShadows_ && !raster_.empty())
            ImGui::Text("Ridge toward the light here: sin %.3f (%.1f deg); light behind it %.0f%%",
                        ridgeAtCamera_, std::asin(std::clamp(ridgeAtCamera_, -1.0f, 1.0f)) * 57.2958f,
                        lightBehindRidge_ * 100.0f);
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
