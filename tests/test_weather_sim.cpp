// THE WEATHER, SIMULATED (ADR-0390; engine/sky/weather_sim.h) on a made island in a made sea -- held to what NASA's
// afternoon views of real islands show (CREDITS.md): fair-weather cumulus over the sunlit land, the sea round it clear,
// a summit above the inversion clear, cloud carried downwind; by night the land clears; an overcast closes over the
// sea too.
#include "test_framework.h"

#include "../src/engine/sky/weather_sim.h"
#include "../src/engine/sky/cloud_map.h"
#include "../src/engine/scripting/clouds_lua.h"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {
// 64 km of sea (-50 m) round a low island (a 12 km cone to 1400 m, at the origin) and a small high one (a 3.5 km cone
// to 3000 m, 18 km north: above a 2000 m inversion)
HeightRaster madeWorld() {
    return rasterizeHeights(
        [](double x, double z) {
            const double low = 1400.0 * std::max(0.0, 1.0 - std::hypot(x, z) / 12000.0) - 50.0 * (std::hypot(x, z) > 12000.0);
            const double high = 3050.0 * std::max(0.0, 1.0 - std::hypot(x, z + 18000.0) / 3500.0);
            return std::max({low, high - 50.0, -50.0});
        },
        -32000.0, -32000.0, 64000.0, 256);
}
WeatherSimInputs fairAfternoon() {
    WeatherSimInputs in;
    in.windX = 1.0f;   // the trade wind blows east (+x)
    in.windZ = 0.0f;
    in.windSpeed = 6.0f;
    in.sunUp = 0.9f;
    return in;
}
// the mean cover over the cells a predicate picks
template <class F>
float meanCover(const WeatherSim& s, F&& pick) {
    double sum = 0.0;
    int n = 0;
    for (int j = 0; j < s.size(); ++j)
        for (int i = 0; i < s.size(); ++i) {
            const double x = s.originX() + (i + 0.5) * s.cellSize(), z = s.originZ() + (j + 0.5) * s.cellSize();
            if (!pick(x, z)) continue;
            sum += s.cover()[static_cast<std::size_t>(j) * s.size() + i];
            ++n;
        }
    return n > 0 ? static_cast<float>(sum / n) : 0.0f;
}
}  // namespace

TEST_CASE(weather_sim_fair_afternoon_clouds_the_land_not_the_sea) {
    WeatherSim s;
    s.init(madeWorld(), 0.0, 256);
    s.settle(fairAfternoon());
    const float land = meanCover(s, [](double x, double z) { return std::hypot(x, z) < 10000.0; });
    const float seaUpwind = meanCover(s, [](double x, double z) { return x < -20000.0 && std::fabs(z) < 8000.0; });
    const float seaDownwind = meanCover(s, [](double x, double z) { return x > 14000.0 && x < 22000.0 && std::fabs(z) < 6000.0; });
    const float summit = s.coverAt(0.0, -18000.0);
    for (float c : s.cover()) CHECK(c >= 0.0f && c <= 1.0f && std::isfinite(c));
    CHECK(land > 0.25f && land < 0.55f);   // about a third of Puerto Rico's land under cumulus on a fair afternoon
    CHECK(seaUpwind < 0.05f);         // ...its sea upwind nearly clear
    CHECK(seaDownwind > seaUpwind);   // ...cloud carried off downwind
    CHECK(summit < 0.1f);             // a summit above the inversion stands clear (Mauna Kea)
}

TEST_CASE(weather_sim_the_land_clears_by_night) {
    WeatherSim day, night;
    day.init(madeWorld(), 0.0, 128);
    night.init(madeWorld(), 0.0, 128);
    WeatherSimInputs in = fairAfternoon();
    day.settle(in);
    in.sunUp = -0.4f;
    night.settle(in);
    auto onLand = [](double x, double z) { return std::hypot(x, z) < 10000.0; };
    CHECK(meanCover(night, onLand) < 0.5f * meanCover(day, onLand));
}

TEST_CASE(weather_sim_an_overcast_closes_over_the_sea) {
    WeatherSim s;
    s.init(madeWorld(), 0.0, 128);
    WeatherSimInputs in = fairAfternoon();
    in.weather.humidity = 0.92f;
    in.weather.lift = 0.35f;
    in.weather.convection = 0.3f;
    s.settle(in);
    const float sea = meanCover(s, [](double x, double) { return x < -20000.0; });
    CHECK(sea > 0.6f);
}

TEST_CASE(weather_sim_places_the_maps_clouds_over_the_land) {
    // the weather map baked from the simulated field (ADR-0390): its cells over the island, the sea upwind clear
    auto sim = std::make_shared<WeatherSim>();
    sim->init(madeWorld(), 0.0, 128);
    sim->settle(fairAfternoon());
    CloudBook book = cloudBook();
    CHECK(book.loaded);
    book.mapExtent = 64000.0f;
    book.mapSize = 512;
    CloudMapInputs in;
    in.typeLo = 0.47f;
    in.typeHi = 0.75f;
    in.field = sim;
    in.fieldVersion = 1;
    CloudMapImage m;
    bakeCloudMap(book, in, m);
    auto coverIn = [&](double x0, double x1, double z0, double z1) {
        double sum = 0.0;
        int n = 0;
        for (int j = 0; j < m.size; ++j)
            for (int i = 0; i < m.size; ++i) {
                const double x = m.originX + (i + 0.5) * m.extent / m.size, z = m.originZ + (j + 0.5) * m.extent / m.size;
                if (x < x0 || x > x1 || z < z0 || z > z1) continue;
                sum += m.rgba[(static_cast<std::size_t>(j) * m.size + i) * 4] >= 128 ? 1.0 : 0.0;
                ++n;
            }
        return n > 0 ? sum / n : 0.0;
    };
    const double island = coverIn(-8000.0, 8000.0, -8000.0, 8000.0);
    const double seaUpwind = coverIn(-30000.0, -18000.0, -8000.0, 8000.0);
    CHECK(island > 0.15);
    CHECK(seaUpwind < 0.03);
}
