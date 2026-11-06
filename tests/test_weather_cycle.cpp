// Weather states over the volumetric cloud deck (engine/weather_cycle.h):
// pure targets + easing + the seeded auto walk. Headless in both suites —
// the DayNightSystem only copies these numbers onto its cloud knobs.
#include "test_framework.h"

#include "../src/engine/weather_cycle.h"
#include "../src/engine/wind.h"

#include <algorithm>
#include <cmath>
#include <set>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(weather_targets_order_mild_to_severe) {
    // The chain is ordered: coverage and density never decrease toward Storm,
    // the sun never brightens toward Storm. This is what makes neighbor-only
    // auto steps read as weather rather than a slot machine.
    const WeatherKind chain[4] = {WeatherKind::Clear, WeatherKind::Fair,
                                  WeatherKind::Overcast, WeatherKind::Storm};
    for (int i = 1; i < 4; ++i) {
        const WeatherTargets a = weatherTargets(chain[i - 1]);
        const WeatherTargets b = weatherTargets(chain[i]);
        CHECK(b.coverage > a.coverage);
        CHECK(b.density >= a.density);
        CHECK(b.sunScale <= a.sunScale);
    }
    // Storm is the old default deck, correctly labeled: full density.
    CHECK(std::fabs(weatherTargets(WeatherKind::Storm).density - 1.0f) < 1e-6);
}

TEST_CASE(weather_ease_converges_without_overshoot) {
    WeatherCycle w;
    w.state = WeatherKind::Storm;
    // From the Fair boot values, ease toward Storm in fixed steps.
    const WeatherTargets t = weatherTargets(WeatherKind::Storm);
    float prev = w.coverage;
    for (int i = 0; i < 20000; ++i) {
        w.ease(1.0 / 60.0);
        CHECK(w.coverage >= prev - 1e-6);   // monotone toward the target
        CHECK(w.coverage <= t.coverage + 1e-4);   // never overshoots
        prev = w.coverage;
    }
    CHECK(std::fabs(w.coverage - t.coverage) < 1e-3);
    CHECK(std::fabs(w.density - t.density) < 1e-3);
    CHECK(std::fabs(w.sunScale - t.sunScale) < 1e-3);
    // A front is not instant: after ONE second the gap has barely moved.
    WeatherCycle fresh;
    fresh.state = WeatherKind::Storm;
    fresh.ease(1.0);
    CHECK(fresh.coverage < 0.35f);
}

TEST_CASE(weather_snap_lands_exactly_on_targets) {
    WeatherCycle w;
    w.state = WeatherKind::Overcast;
    w.snap();
    const WeatherTargets t = weatherTargets(WeatherKind::Overcast);
    CHECK(w.coverage == t.coverage);
    CHECK(w.density == t.density);
    CHECK(w.scale == t.scale);
    CHECK(w.sunScale == t.sunScale);
}

TEST_CASE(weather_auto_walk_steps_neighbors_only_and_is_deterministic) {
    WeatherCycle a, b;
    a.autoMode = b.autoMode = true;
    a.state = b.state = WeatherKind::Fair;
    std::set<int> visited;
    int prev = static_cast<int>(a.state);
    for (int i = 0; i < 200; ++i) {
        a.advanceAuto(WeatherCycle::kAutoHours, 42);
        b.advanceAuto(WeatherCycle::kAutoHours, 42);
        const int s = static_cast<int>(a.state);
        CHECK(std::abs(s - prev) <= 1);          // neighbor steps only
        CHECK(s >= 0 && s <= 3);                 // clamped to the chain
        CHECK(s == static_cast<int>(b.state));   // same seed, same weather
        visited.insert(s);
        prev = s;
    }
    // A 200-roll walk at 25%/25% move odds visits more than one state.
    CHECK(visited.size() >= 2u);
}

TEST_CASE(weather_auto_accumulates_partial_hours) {
    WeatherCycle w;
    w.autoMode = true;
    // 3 hours, then 1 more: exactly one roll at the 4-hour boundary.
    w.advanceAuto(3.0, 7);
    const uint32_t before = w.autoSteps;
    CHECK(before == 0u);
    w.advanceAuto(1.0, 7);
    CHECK(w.autoSteps == 1u);
    // Inactive mode never rolls.
    WeatherCycle off;
    off.advanceAuto(100.0, 7);
    CHECK(off.autoSteps == 0u);
}

TEST_CASE(weather_names_round_trip_and_reject_junk) {
    for (WeatherKind k : {WeatherKind::Clear, WeatherKind::Fair,
                          WeatherKind::Overcast, WeatherKind::Storm}) {
        WeatherKind out;
        CHECK(weatherKindFromName(weatherKindName(k), out));
        CHECK(out == k);
    }
    WeatherKind out;
    CHECK(!weatherKindFromName("drizzle", out));
    CHECK(!weatherKindFromName("", out));
}

TEST_CASE(weather_for_day_spans_the_kinds_and_is_deterministic) {
    // A year of days: clear about three in ten, fair the plurality, storms
    // rare but present; the same seed and day always give the same sky.
    int count[4] = {0, 0, 0, 0};
    for (int d = 1; d <= 365; ++d) ++count[static_cast<int>(weatherForDay(7u, d))];
    CHECK(count[0] > 70 && count[0] < 150);    // clear
    CHECK(count[1] > 110 && count[1] < 200);   // fair
    CHECK(count[2] > 40 && count[2] < 110);    // overcast
    CHECK(count[3] > 8 && count[3] < 60);      // storm
    CHECK(weatherForDay(7u, 100) == weatherForDay(7u, 100));
    bool differs = false;
    for (int d = 1; d <= 30 && !differs; ++d) differs = weatherForDay(7u, d) != weatherForDay(11u, d);
    CHECK(differs);
}

// NIGHT GLOW reaches a chunk that streams in AFTER dusk (Glenn, 2026-09-30: towers he had walked past stayed dark
// at night). The glow used to be written only when the dusk ramp changed; a chunk spawned while the ramp held still
// kept its zero emission. applyNightGlowRamp lights each entity once per ramp value.
#include "../src/engine/systems/day_night_system.h"
#include "../src/engine/world.h"
#include "../src/engine/components.h"
TEST_CASE(night_glow_lights_a_chunk_that_streams_in_after_dusk) {
    World world;
    auto spawn = [&] {
        Entity e = world.create();
        world.add<Renderable>(e, Renderable{});
        world.add<NightGlow>(e, NightGlow{Vec3(1, 1, 1)});
        return e;
    };
    const Entity early = spawn();
    applyNightGlowRamp(world, 1.0);
    CHECK_APPROX(world.get<Renderable>(early)->material.emission.x, 1.0, 1e-6);
    const Entity late = spawn();               // streams in at night, the ramp unchanged
    applyNightGlowRamp(world, 1.0);
    CHECK_APPROX(world.get<Renderable>(late)->material.emission.x, 1.0, 1e-6);
    applyNightGlowRamp(world, 0.0);            // dawn: both go dark
    CHECK_APPROX(world.get<Renderable>(early)->material.emission.x, 0.0, 1e-6);
    CHECK_APPROX(world.get<Renderable>(late)->material.emission.x, 0.0, 1e-6);
}

// THE WIND (engine/wind.h, ADR-0337): the weather's wind rises toward the storm, and windAt keeps to its contract
TEST_CASE(wind_rises_with_the_weather_and_keeps_its_bounds) {
    const WeatherKind chain[4] = {WeatherKind::Clear, WeatherKind::Fair, WeatherKind::Overcast, WeatherKind::Storm};
    for (int i = 1; i < 4; ++i) {
        CHECK(weatherTargets(chain[i]).windSpeed > weatherTargets(chain[i - 1]).windSpeed);
        CHECK(weatherTargets(chain[i]).windGust >= weatherTargets(chain[i - 1]).windGust);
    }
    Wind w;
    w.dir = Vec3(0.6, 0.0, 0.8);
    w.speed = 10.0;
    w.gust = 0.5;
    Real lo = 1e9, hi = -1e9;
    for (int k = 0; k < 400; ++k) {
        const Vec3 p(k * 37.0, 10.0, k * -23.0);
        const Vec3 v = windAt(w, p);
        CHECK_APPROX(v.y, 0.0, 1e-12);                                  // horizontal
        CHECK(dot(v, w.dir) > 0.0);                                     // downwind, always
        CHECK_APPROX(cross(v, w.dir).length(), 0.0, 1e-9);              // along the wind
        lo = std::min(lo, v.length()); hi = std::max(hi, v.length());
    }
    CHECK(lo >= 10.0 * (1.0 - 0.5 * 0.5) - 1e-9);                       // a lull is at most half the gust lighter
    CHECK(hi <= 10.0 * (1.0 + 0.5) + 1e-9);                             // a gust at most `gust` stronger
    CHECK(hi - lo > 2.0);                                               // and it does gust
    // stronger aloft, within its clamp
    CHECK(windAt(w, Vec3(0, 200, 0)).length() / windGustFactor(w, Vec3(0, 200, 0)) > 12.0);
    CHECK(windAt(w, Vec3(0, 2, 0)).length() / windGustFactor(w, Vec3(0, 2, 0)) >= 6.0 - 1e-9);
    // a gust travels downwind at the wind's speed: what is here at t was 10 * dt upwind at t - dt
    Wind w2 = w;
    w2.time = 3.0;
    const Vec3 p(123.0, 10.0, -77.0);
    CHECK_APPROX(windGustFactor(w2, p), windGustFactor(w, p - w.dir * 30.0), 1e-9);
    // a calm is a calm
    Wind calm = w;
    calm.speed = 0.0;
    CHECK_APPROX(windAt(calm, p).length(), 0.0, 1e-12);
}

// THE CALENDAR (ADR-0340): a day's weather is the same every time, the climate's season moves the odds, the south
// has its winter in July, and the default climate keeps the old year-round odds
TEST_CASE(weather_calendar_climate_and_season) {
    WeatherClimate atlantic;
    const float summer[4] = {10, 35, 40, 15}, winter[4] = {5, 20, 45, 30};
    for (int i = 0; i < 4; ++i) { atlantic.summer[i] = summer[i]; atlantic.winter[i] = winter[i]; }
    CHECK(weatherForDay(7u, 200, 2026, atlantic, 60.0) == weatherForDay(7u, 200, 2026, atlantic, 60.0));
    // storms in January against July, over many years
    int janStorm = 0, julStorm = 0, julStormSouth = 0;
    for (int y = 2000; y < 2200; ++y)
        for (int d = 0; d < 10; ++d) {
            janStorm += weatherForDay(7u, 10 + d, y, atlantic, 60.0) == WeatherKind::Storm;
            julStorm += weatherForDay(7u, 192 + d, y, atlantic, 60.0) == WeatherKind::Storm;
            julStormSouth += weatherForDay(7u, 192 + d, y, atlantic, -45.0) == WeatherKind::Storm;
        }
    CHECK(janStorm > julStorm * 1.5);              // winter's 30 against summer's 15
    CHECK(julStormSouth > julStorm * 1.5);         // July is the south's winter
    // the default climate: the old odds all year (30 / 42 / 20 / 8 %), within sampling
    WeatherClimate def;
    int n[4] = {0, 0, 0, 0};
    for (int y = 2000; y < 2010; ++y)
        for (int d = 1; d <= 365; ++d) ++n[static_cast<int>(weatherForDay(7u, d, y, def, 40.0))];
    const double total = 3650.0;
    CHECK_APPROX(n[0] / total, 0.30, 0.03);
    CHECK_APPROX(n[1] / total, 0.42, 0.03);
    CHECK_APPROX(n[2] / total, 0.20, 0.03);
    CHECK_APPROX(n[3] / total, 0.08, 0.02);
    // the walk's seed follows the day: different days walk differently, the same day the same way
    CHECK(weatherDaySeed(7u, 100, 2026) == weatherDaySeed(7u, 100, 2026));
    CHECK(weatherDaySeed(7u, 100, 2026) != weatherDaySeed(7u, 101, 2026));
}
