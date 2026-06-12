#include "test_framework.h"

#include "../src/engine/day_night_cycle.h"

using namespace engine;

namespace {
constexpr Real EPS = 1e-9;

// Tilt isolated to the +z axis keeps the sun in the x-y plane for the cardinal
// checks below (elevation == sin of the arc angle), so they read cleanly.
constexpr double NO_TILT = 0.0;
}

TEST_CASE(day_night_noon_sun_is_overhead) {
    DayNightState s = DayNightCycle::evaluateAt(0.5, NO_TILT);
    // At noon the sun points straight up.
    CHECK_APPROX(s.sunDirection.y, 1.0, 1e-6);
    CHECK(s.sunIntensity > 1.0f);
    CHECK(s.skyDiscIntensity > 0.99f);
}

TEST_CASE(day_night_midnight_sun_below_horizon) {
    DayNightState s = DayNightCycle::evaluateAt(0.0, NO_TILT);
    CHECK_APPROX(s.sunDirection.y, -1.0, 1e-6);
    // Night: directional sun and the visible disc are extinguished.
    CHECK_APPROX(s.sunIntensity, 0.0, 1e-6);
    CHECK_APPROX(s.skyDiscIntensity, 0.0, 1e-6);
}

TEST_CASE(day_night_sunrise_sunset_on_horizon) {
    DayNightState rise = DayNightCycle::evaluateAt(0.25, NO_TILT);
    DayNightState set  = DayNightCycle::evaluateAt(0.75, NO_TILT);
    // Sun grazes the horizon at both, on opposite sides (east vs west).
    CHECK_APPROX(rise.sunDirection.y, 0.0, 1e-6);
    CHECK_APPROX(set.sunDirection.y, 0.0, 1e-6);
    CHECK(rise.sunDirection.x > 0.5);   // east
    CHECK(set.sunDirection.x < -0.5);   // west
}

TEST_CASE(day_night_sun_direction_is_normalized) {
    for (int i = 0; i < 8; i++) {
        double t = i / 8.0;
        Vec3 d = DayNightCycle::evaluateAt(t, 0.35).sunDirection;
        CHECK_APPROX(d.length(), 1.0, 1e-9);
    }
}

TEST_CASE(day_night_noon_brighter_than_dusk) {
    DayNightState noon = DayNightCycle::evaluateAt(0.5, NO_TILT);
    DayNightState dusk = DayNightCycle::evaluateAt(0.75, NO_TILT);
    CHECK(noon.sunIntensity > dusk.sunIntensity);
    CHECK(noon.ambient > dusk.ambient);
}

TEST_CASE(day_night_advance_wraps_into_unit_interval) {
    DayNightCycle c;
    c.timeOfDay = 0.9;
    c.speed = 0.5;       // 0.5 days/sec
    c.paused = false;
    c.advance(1.0);      // +0.5 -> 1.4 -> wraps to 0.4
    CHECK_APPROX(c.timeOfDay, 0.4, 1e-9);
    CHECK(c.timeOfDay >= 0.0 && c.timeOfDay < 1.0);
}

TEST_CASE(day_night_advance_paused_is_noop) {
    DayNightCycle c;
    c.timeOfDay = 0.3;
    c.paused = true;
    c.advance(10.0);
    CHECK_APPROX(c.timeOfDay, 0.3, EPS);
}

TEST_CASE(day_night_full_cycle_returns_to_start) {
    // Stepping exactly one full day lands back on the starting time.
    DayNightCycle c;
    c.timeOfDay = 0.2;
    c.speed = 1.0;
    c.advance(1.0);
    CHECK_APPROX(c.timeOfDay, 0.2, 1e-9);
}

TEST_CASE(day_night_dusk_horizon_is_warm) {
    // At sunset the horizon should skew warm (red > blue).
    DayNightState set = DayNightCycle::evaluateAt(0.75, NO_TILT);
    CHECK(set.horizonColor.x > set.horizonColor.z);
}
