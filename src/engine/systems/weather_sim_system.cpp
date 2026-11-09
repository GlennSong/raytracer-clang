#include "weather_sim_system.h"

#include "../components.h"     // TerrainLodConfig
#include "../weather_cycle.h"  // WeatherNow
#include "../wind.h"
#include "../../log.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace engine {

namespace {
// the simulation's inputs now: the Wind, the weather's air, the sun's height by the clock (a sine of the hours from
// six: up by day, below by night -- the latitude's finer arc is the sky's, not the weather's)
WeatherSimInputs inputsNow(FrameContext& ctx) {
    WeatherSimInputs in;
    ctx.world.each<Wind>([&](Entity, Wind& w) {
        in.windX = static_cast<float>(w.dir.x);
        in.windZ = static_cast<float>(w.dir.z);
        in.windSpeed = static_cast<float>(w.speed);
    });
    ctx.world.each<WeatherNow>([&](Entity, WeatherNow& now) {
        in.weather.humidity = now.live.humidity;
        in.weather.inversion = now.live.inversion;
        in.weather.convection = now.live.convection;
        in.weather.lift = now.live.lift;
    });
    const double hours = ctx.view.lighting.clockHours >= 0.0f ? ctx.view.lighting.clockHours : 14.0;
    in.sunUp = static_cast<float>(std::sin((hours - 6.0) * 3.14159265358979323846 / 12.0));
    return in;
}
}  // namespace

void WeatherSimSystem::begin(FrameContext& ctx) {
    HeightRaster terrain;
    double sea = -1e30;
    const TerrainLodConfig* cfg = nullptr;
    ctx.world.each<TerrainLodConfig>([&](Entity, TerrainLodConfig& c) { if (!cfg) cfg = &c; });
    if (cfg) {
        terrain = rasterizeTerrain(cfg->params, cfg->seed, std::max(static_cast<double>(cfg->worldHalf), 0.5 * kMinExtent), kGrid);
        sea = cfg->params.seaLevel;
    } else {
        // (no terrain: flat land, no sea, round the origin)
        terrain = rasterizeHeights([](double, double) { return 0.0; }, -0.5 * kMinExtent, -0.5 * kMinExtent, kMinExtent, 4);
    }
    sim_.init(terrain, sea, kGrid);
    LOG_INFO << "[weather] simulation over " << terrain.extent / 1000.0 << " km, " << kGrid << "^2 cells of "
             << sim_.cellSize() << " m" << (sea > -1e29 ? ", with its sea" : ", no sea");
    // settled before the first handover
    const WeatherSimInputs in = inputsNow(ctx);
    pending_ = std::async(std::launch::async, [sim = std::move(sim_), in]() mutable {
        const auto t0 = std::chrono::steady_clock::now();
        sim.settle(in);
        LOG_INFO << "[weather] settled in "
                 << std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() << " s";
        return std::move(sim);
    });
    stepping_ = true;
    begun_ = true;
}

void WeatherSimSystem::publish(FrameContext& ctx) {
    WeatherField f;
    f.sim = std::make_shared<const WeatherSim>(sim_);
    f.version = ++version_;
    if (!ctx.world.alive(fieldEntity_)) {
        fieldEntity_ = ctx.world.create();
        ctx.world.add<WeatherField>(fieldEntity_, f);
    } else if (WeatherField* w = ctx.world.get<WeatherField>(fieldEntity_)) {
        *w = f;
    }
}

void WeatherSimSystem::update(FrameContext& ctx) {
    if (!ctx.view.lighting.volumetricClouds.enabled) return;
    if (!begun_) begin(ctx);
    // the weather's time: the day's clock where it runs, real time where it is held
    const double perSecond = ctx.view.lighting.clockHoursPerSecond > 0.0f ? ctx.view.lighting.clockHoursPerSecond * 3600.0 : 1.0;
    gathered_ += std::clamp(ctx.frameDelta, 0.0, 0.25) * perSecond;
    if (stepping_ && pending_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        sim_ = pending_.get();
        stepping_ = false;
        publish(ctx);
    }
    if (!stepping_ && sim_.ready() && gathered_ >= kStepEvery) {
        WeatherSimInputs in = inputsNow(ctx);
        in.dt = std::min(gathered_, 3600.0);   // (an hour at most at once: a long stall does not leap the sky)
        gathered_ = 0.0;
        pending_ = std::async(std::launch::async, [sim = std::move(sim_), in]() mutable {
            sim.step(in);
            return std::move(sim);
        });
        stepping_ = true;
    }
}

void WeatherSimSystem::onStop(FrameContext&) {
    if (stepping_) pending_.wait();
    stepping_ = false;
}

}  // namespace engine
