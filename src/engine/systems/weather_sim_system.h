#ifndef RAYTRACER_ENGINE_WEATHER_SIM_SYSTEM_H
#define RAYTRACER_ENGINE_WEATHER_SIM_SYSTEM_H

#include "../system.h"
#include "../sky/weather_sim.h"
#include <future>

namespace engine {

// THE WEATHER SIMULATION, STEPPED (ADR-0390; sky/weather_sim.h). Once the level's terrain is known, a grid over it and
// its sea (at least 64 km); then, on a worker of its own, the simulation carried forward in the WEATHER's time (the
// day's clock: a fast day is a fast sky), by the world's Wind and WeatherNow (DayNightSystem) and the sun's height by
// the hour. Each finished step is handed over whole as the WeatherField (the weather map reads it). It begins settled:
// four hours run before the first handover, so a level opens on a sky that has formed.
class WeatherSimSystem : public System {
public:
    void update(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    static constexpr int kGrid = 256;
    static constexpr double kMinExtent = 64000.0;   // m: the grid at least this wide (the sea round a small level)
    static constexpr double kStepEvery = 30.0;      // weather seconds gathered before a step is run

private:
    void begin(FrameContext& ctx);
    void publish(FrameContext& ctx);

    // a step's result: the simulation to carry on, and its snapshot to hand over -- copied on the worker (a few MB:
    // not the main thread's to copy)
    struct Stepped {
        WeatherSim sim;
        std::shared_ptr<const WeatherSim> snapshot;
    };

    WeatherSim sim_;   // (moved to the worker while it steps)
    std::shared_ptr<const WeatherSim> snapshot_;   // the last step's, to publish
    std::future<Stepped> pending_;
    bool stepping_ = false;
    bool begun_ = false;
    uint32_t terrainKey_ = 0;   // the terrain the grid was begun on (0: none, flat land)
    double gathered_ = 0.0;   // weather seconds not yet stepped
    Entity fieldEntity_{};
    uint64_t version_ = 0;
};

}  // namespace engine

#endif
