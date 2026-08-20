#ifndef RAYTRACER_ENGINE_WEATHER_CYCLE_H
#define RAYTRACER_ENGINE_WEATHER_CYCLE_H

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

namespace engine {

// Sky WEATHER over the volumetric cloud deck (Glenn: "most of the time the
// clouds shouldn't be that dramatic unless there's a storm"). Four states,
// each a target point in the deck's knob space plus a sun dimmer. The states
// ride the SAME volumetrics: coverage is a threshold sliding through a
// world-anchored noise field, so EASING between targets grows or erodes the
// existing clouds in place — a front arriving, not a sky crossfade. The old
// default deck (coverage .5, density 1.0) is the Storm entry, correctly
// labeled at last; Fair is the everyday sky.
//
// Pure and window-free (the DayNightCycle idiom): DayNightSystem owns one,
// eases it per fixed step, and copies the live values onto its cloud knobs
// and sun intensity. Unit-tested headless in tests/test_weather_cycle.cpp.

// Ordered mild -> severe; the auto walk only ever steps to a neighbor, so a
// clear noon never jump-cuts to a thunderhead.
enum class WeatherKind : int { Clear = 0, Fair = 1, Overcast = 2, Storm = 3 };

struct WeatherTargets {
    float coverage;   // fraction of the noise field above the cloud threshold
    float density;    // optical thickness (Beer term) — dark bases at 1.0
    float scale;      // noise domain scale — bigger cells for heavy decks
    float sunScale;   // multiplier on the sun's intensity (kills shadows too)
    // An overcast sky is a giant diffuser: LESS direct light, MORE ambient,
    // and shadows nearly gone. Dimming only the sun (the first cut) kept
    // clear-sky contrast at lower brightness — "the lighting under the
    // clouds is still very harsh". ambientScale raises the fill; shadowSoften
    // fades the artistic shadow response toward shadowless.
    float ambientScale;   // multiplier on the level's ambient fill
    float shadowSoften;   // 0 = authored shadows, 1 = fully diffuse (no shadow)
};

inline WeatherTargets weatherTargets(WeatherKind k) {
    switch (k) {
        case WeatherKind::Clear:    return {0.08f, 0.30f, 0.90f, 1.00f, 1.00f, 0.00f};
        case WeatherKind::Overcast: return {0.80f, 0.60f, 1.90f, 0.55f, 1.35f, 0.60f};
        case WeatherKind::Storm:    return {0.92f, 1.00f, 1.60f, 0.28f, 1.45f, 0.85f};
        case WeatherKind::Fair:     break;
    }
    return {0.32f, 0.45f, 1.10f, 1.00f, 1.05f, 0.08f};   // Fair — the everyday sky
}

inline const char* weatherKindName(WeatherKind k) {
    switch (k) {
        case WeatherKind::Clear:    return "clear";
        case WeatherKind::Overcast: return "overcast";
        case WeatherKind::Storm:    return "storm";
        case WeatherKind::Fair:     break;
    }
    return "fair";
}

inline bool weatherKindFromName(const std::string& name, WeatherKind& out) {
    if (name == "clear")    { out = WeatherKind::Clear;    return true; }
    if (name == "fair")     { out = WeatherKind::Fair;     return true; }
    if (name == "overcast") { out = WeatherKind::Overcast; return true; }
    if (name == "storm")    { out = WeatherKind::Storm;    return true; }
    return false;
}

// splitmix32 — the deterministic roll for the auto walk. Seeded per step so a
// fixed (seed, step) pair always rolls the same weather: replays replay.
inline uint32_t weatherHash(uint32_t seed, uint32_t step) {
    uint32_t x = seed * 0x9E3779B9u + step * 0x85EBCA6Bu;
    x ^= x >> 16; x *= 0x21F0AAADu;
    x ^= x >> 15; x *= 0x735A2D97u;
    x ^= x >> 15;
    return x;
}

struct WeatherCycle {
    WeatherKind state = WeatherKind::Fair;

    // Live (eased) values — what the renderer shows THIS frame.
    float coverage = 0.32f;
    float density  = 0.45f;
    float scale    = 1.10f;
    float sunScale = 1.00f;
    float ambientScale = 1.05f;
    float shadowSoften = 0.08f;

    bool autoMode = false;
    double sinceStepHours = 0.0;   // in-world hours since the last auto roll
    uint32_t autoSteps = 0;        // roll counter — the hash's step input

    static constexpr double kAutoHours = 4.0;    // in-world hours between rolls
    // Exponential approach: ~63% of the gap closes per tau of sim time, so a
    // front takes a couple of minutes to fully arrive instead of popping.
    static constexpr double kTauSeconds = 40.0;

    void ease(double dtSeconds) {
        const WeatherTargets t = weatherTargets(state);
        const float a =
            static_cast<float>(1.0 - std::exp(-dtSeconds / kTauSeconds));
        coverage += (t.coverage - coverage) * a;
        density  += (t.density  - density)  * a;
        scale    += (t.scale    - scale)    * a;
        sunScale += (t.sunScale - sunScale) * a;
        ambientScale += (t.ambientScale - ambientScale) * a;
        shadowSoften += (t.shadowSoften - shadowSoften) * a;
    }

    // Jump straight to the state's targets (state restore at level start —
    // yesterday's storm shouldn't fade in from today's clear boot).
    void snap() {
        const WeatherTargets t = weatherTargets(state);
        coverage = t.coverage;
        density  = t.density;
        scale    = t.scale;
        sunScale = t.sunScale;
        ambientScale = t.ambientScale;
        shadowSoften = t.shadowSoften;
    }

    // Advance the auto clock by in-world hours. At each kAutoHours boundary:
    // half stay, a quarter step milder, a quarter step harsher — clamped at
    // the chain's ends, one neighbor at most.
    void advanceAuto(double hours, uint32_t seed) {
        if (!autoMode) return;
        sinceStepHours += hours;
        while (sinceStepHours >= kAutoHours) {
            sinceStepHours -= kAutoHours;
            const uint32_t roll = weatherHash(seed, ++autoSteps) & 3u;
            int s = static_cast<int>(state);
            if (roll == 2u) s = std::max(0, s - 1);
            else if (roll == 3u) s = std::min(3, s + 1);
            state = static_cast<WeatherKind>(s);
        }
    }
};

}  // namespace engine

#endif
