// Weather states over the volumetric cloud deck (engine/weather_cycle.h):
// pure targets + easing + the seeded auto walk. Headless in both suites —
// the DayNightSystem only copies these numbers onto its cloud knobs.
#include "test_framework.h"

#include "../src/engine/weather_cycle.h"

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
