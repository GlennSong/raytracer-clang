#ifndef RAYTRACER_ENGINE_SKY_WEATHER_SIM_H
#define RAYTRACER_ENGINE_SKY_WEATHER_SIM_H

// THE WEATHER, SIMULATED (ADR-0390; the weather plan's part C; Glenn, 2026-11-08: "there are massive cloud structures
// that noise can't really create so we should simulate something more realistic"). Real island skies -- measured on
// NASA's afternoon views of Puerto Rico, Corsica, Hawaii and the Canaries (CREDITS.md) -- organise by the LAND: cumulus
// over sunlit land and high ground, the sea and the coasts clear, summits above the trade inversion clear, plumes and
// wakes downwind. So a coarse grid over the level and its sea, its fields 2D (the boundary layer as one slab):
// - the WIND: the prevailing one, steered round ground standing above the inversion and drawn in over hot land (a sea
//   breeze: convergence the heating asks for, made by a pressure projection -- Stam's stable fluids), carried by
//   itself so eddies and wakes form;
// - the LIFT: that convergence, the wind climbing a slope (sinking down the lee), sun-heated land's convection, and the
//   weather's large-scale lift (a front);
// - the MOISTURE the sea gives the air (relative humidity), carried by the wind, drier over land;
// - the CLOUD: condensing where lifted moist air saturates, carried downwind (plumes), evaporating where it sinks or
//   dries; none where the ground stands above the inversion.
// Pure: no engine state, stepped by WeatherSimSystem on a worker; read by the weather map (cloud_map.h).

#include "../procgen/terrain_horizon.h"   // HeightRaster
#include <cstdint>
#include <memory>
#include <vector>

namespace engine {

// The weather state's part (clouds.lua weathers.*: eased with the rest)
struct WeatherSimWeather {
    float humidity = 0.80f;      // the relative humidity the sea gives the air
    float inversion = 2000.0f;   // m above the sea: the cap the cumulus cannot rise through
    float convection = 1.0f;     // how strongly sunlit land heats and lifts the air (0 none)
    float lift = 0.0f;           // large-scale lift everywhere (a front, an overcast's deck)
};

struct WeatherSimInputs {
    float windX = 1.0f, windZ = 0.0f;   // where the prevailing wind goes, unit
    float windSpeed = 6.0f;             // m/s
    float sunUp = 1.0f;                 // the sun's elevation, its sine (< 0: night)
    double dt = 0.0;                    // the weather's seconds since the last step
    WeatherSimWeather weather;
};

class WeatherSim {
public:
    // The grid over the raster's square, `n` cells a side, the sea below `seaLevel` (no sea: -1e30)
    void init(const HeightRaster& terrain, double seaLevel, int n = 256);
    // Advance `in.dt` seconds (in sub-steps the wind can cross a cell in)
    void step(const WeatherSimInputs& in);
    // Begin from the weather's steady state for these inputs: run it long enough (hours) that the fields have settled
    void settle(const WeatherSimInputs& in, double seconds = 4.0 * 3600.0);

    bool ready() const { return n_ > 0; }
    int size() const { return n_; }
    double originX() const { return originX_; }
    double originZ() const { return originZ_; }
    double cellSize() const { return cell_; }

    // Bilinear reads at a world XZ (clamped to the grid's edge)
    float coverAt(double x, double z) const { return sample(c_, x, z); }        // cloud, 0..1
    float convectionAt(double x, double z) const { return sample(conv_, x, z); }   // how convective, 0..1
    float liftAt(double x, double z) const { return sample(lift_, x, z); }
    void windAt(double x, double z, float& u, float& v) const { u = sample(u_, x, z); v = sample(v_, x, z); }
    bool landAt(double x, double z) const { return sample(land_, x, z) > 0.5f; }

    // The fields, row by row (z rows, x columns): for pictures and tests
    const std::vector<float>& cover() const { return c_; }
    const std::vector<float>& height() const { return h_; }
    const std::vector<float>& land() const { return land_; }
    const std::vector<float>& humidity() const { return q_; }
    const std::vector<float>& lift() const { return lift_; }
    const std::vector<float>& windU() const { return u_; }
    const std::vector<float>& windV() const { return v_; }

private:
    float sample(const std::vector<float>& f, double x, double z) const;
    void subStep(const WeatherSimInputs& in, double dt);
    void advect(std::vector<float>& f, const std::vector<float>& u, const std::vector<float>& v, double dt,
                std::vector<float>& tmp) const;
    std::size_t at(int i, int j) const { return static_cast<std::size_t>(j) * n_ + i; }

    int n_ = 0;
    double originX_ = 0.0, originZ_ = 0.0, cell_ = 1.0;
    std::vector<float> h_;      // ground above the sea, m (0 over the sea)
    std::vector<float> land_;   // 1 land, 0 sea
    std::vector<float> u_, v_;  // the wind, m/s
    std::vector<float> q_;      // relative humidity
    std::vector<float> c_;      // cloud, 0..1
    std::vector<float> lift_;   // the lift, unitless (~0..1 a cumulus's)
    std::vector<float> conv_;   // the convective share of the lift, 0..1
    std::vector<float> p_, div_, tmp_, tmp2_, uOld_;
};

// THE WEATHER FIELD the simulation last finished (WeatherSimSystem publishes it on an entity of its own): read-only,
// shared, swapped whole -- the weather map bakes from it on a worker while the next step runs
struct WeatherField {
    std::shared_ptr<const WeatherSim> sim;
    uint64_t version = 0;   // counts the handovers
};

}  // namespace engine

#endif
