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
    // The CLOUD TYPES the weather's map spreads over (ADR-0336; 0 stratus, 1/3 stratocumulus, 2/3 cumulus, 1
    // cumulonimbus): easing between states morphs the types too -- a front's puffs flatten into a deck.
    float typeLo, typeHi;
    // THE WIND the state blows (ADR-0337, wind.h): m/s at 10 m, and how squally
    float windSpeed, windGust;
};

inline WeatherTargets weatherTargets(WeatherKind k) {
    switch (k) {
        // 0.03, not 0.08: the coverage threshold rides the top of a noise
        // field, and 8 % still filled the sky with popcorn — a clear day is a
        // few wisps at most (Glenn, 2026-09-14: "can we see different weather?").
        case WeatherKind::Clear:    return {0.03f, 0.25f, 0.90f, 1.00f, 1.00f, 0.00f, 0.62f, 0.70f, 3.0f, 0.20f};   // fair-weather puffs, a light breeze
        case WeatherKind::Overcast: return {0.80f, 0.60f, 1.90f, 0.55f, 1.35f, 0.60f, 0.00f, 0.40f, 9.0f, 0.40f};   // stratus deck, a fresh wind
        case WeatherKind::Storm:    return {0.92f, 1.00f, 1.60f, 0.28f, 1.45f, 0.85f, 0.60f, 1.00f, 16.0f, 0.70f};   // towers, a gale gusting
        case WeatherKind::Fair:     break;
    }
    return {0.32f, 0.45f, 1.10f, 1.00f, 1.05f, 0.08f, 0.40f, 0.75f, 6.0f, 0.35f};   // Fair — the everyday sky, a moderate breeze
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
// TODAY'S WEATHER (Glenn: "the city doesn't seem to have clear days, cloudy
// days, days in between"): the state a day starts in, from the level's seed
// and the day of year — clear three days in ten, fair four, overcast two,
// a storm one in twelve. The auto walk then steps from there every few
// hours. Deterministic, so a level's calendar is the same every launch.
inline WeatherKind weatherForDay(uint32_t seed, int dayOfYear);
inline uint32_t weatherHash(uint32_t seed, uint32_t step) {
    uint32_t x = seed * 0x9E3779B9u + step * 0x85EBCA6Bu;
    x ^= x >> 16; x *= 0x21F0AAADu;
    x ^= x >> 15; x *= 0x735A2D97u;
    x ^= x >> 15;
    return x;
}

// A PLACE'S CLIMATE (ADR-0340): the odds of each state (clear, fair, overcast, storm; any scale) in midsummer and in
// midwinter, blended by the day of the year (cosine, coldest about 15 January in the north, flipped south of the
// equator); where the prevailing wind comes FROM (compass degrees, north 0, east 90); and a scale on the weather's wind
// (a windy coast, a sheltered valley). The default is the old fixed odds all year (30 / 42 / 20 / 8), a south-westerly.
// A level's "dayNight": {"climate": {...}} sets it.
struct WeatherClimate {
    float summer[4] = {36.0f, 50.0f, 24.0f, 10.0f};
    float winter[4] = {36.0f, 50.0f, 24.0f, 10.0f};
    float prevailingFromDeg = 225.0f;
    float windScale = 1.0f;
};

// The state a DAY opens with: the seed, the year and the day picked through the climate's odds for that day
inline WeatherKind weatherForDay(uint32_t seed, int dayOfYear, int year, const WeatherClimate& c, double latitudeDeg) {
    const double w = 0.5 + 0.5 * std::cos((dayOfYear - 15) * 6.283185307179586 / 365.25);   // 1 midwinter (north)
    const double wint = latitudeDeg < 0.0 ? 1.0 - w : w;
    double odds[4], sum = 0.0;
    for (int i = 0; i < 4; ++i) {
        odds[i] = std::max(0.0, c.summer[i] + (c.winter[i] - c.summer[i]) * wint);
        sum += odds[i];
    }
    if (sum <= 0.0) return WeatherKind::Fair;
    const uint32_t step = static_cast<uint32_t>(year) * 400u + static_cast<uint32_t>(dayOfYear) + 1000u;
    double r = (weatherHash(seed ^ 0xA5A5u, step) % 100000u) / 100000.0 * sum;
    for (int i = 0; i < 4; ++i) {
        if (r < odds[i]) return static_cast<WeatherKind>(i);
        r -= odds[i];
    }
    return WeatherKind::Storm;
}

// The walk within a day is seeded by the day too, so a given day's weather is the same every time it is lived
inline uint32_t weatherDaySeed(uint32_t seed, int dayOfYear, int year) {
    return weatherHash(seed ^ 0x51ED27u, static_cast<uint32_t>(year) * 400u + static_cast<uint32_t>(dayOfYear));
}

inline WeatherKind weatherForDay(uint32_t seed, int dayOfYear) {
    const uint32_t r = weatherHash(seed ^ 0xA5A5u, static_cast<uint32_t>(dayOfYear) + 1000u) % 120u;
    if (r < 36u) return WeatherKind::Clear;      // 30 %
    if (r < 86u) return WeatherKind::Fair;       // 42 %
    if (r < 110u) return WeatherKind::Overcast;  // 20 %
    return WeatherKind::Storm;                   // 8 %
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
    float typeLo = 0.40f, typeHi = 0.75f;
    float windSpeed = 6.0f, windGust = 0.35f;

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
        typeLo += (t.typeLo - typeLo) * a;
        typeHi += (t.typeHi - typeHi) * a;
        windSpeed += (t.windSpeed - windSpeed) * a;
        windGust += (t.windGust - windGust) * a;
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
        typeLo = t.typeLo;
        typeHi = t.typeHi;
        windSpeed = t.windSpeed;
        windGust = t.windGust;
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
