#include "test_framework.h"

#include "../src/engine/day_night_cycle.h"

#include <algorithm>
#include <cmath>

using namespace engine;

namespace {
constexpr Real EPS = 1e-9;

// The equator at an equinox: the sun rises due east at 0.25, stands straight
// overhead at 0.5, sets due west at 0.75 — the symmetric arc the cardinal
// checks below read cleanly against (elevation == cos of the hour angle).
constexpr double EQUATOR = 0.0;
constexpr double EQUINOX = 0.0;

// Sample one loop minute by minute and count the lit ones — the measured
// daylight, to hold against the analytic sunrise/sunset.
struct LitCensus {
    int litMinutes = 0;
    double firstLit = -1, lastLit = -1;   // hours
};
LitCensus census(double latitudeDeg, double declinationDeg) {
    LitCensus c;
    for (int m = 0; m < 24 * 60; ++m) {
        const double t = (m + 0.5) / (24.0 * 60.0);
        if (DayNightCycle::evaluateAt(t, latitudeDeg, declinationDeg).solarElevation > 0.0f) {
            ++c.litMinutes;
            if (c.firstLit < 0) c.firstLit = t * 24.0;
            c.lastLit = t * 24.0;
        }
    }
    return c;
}
}

TEST_CASE(day_night_noon_sun_is_overhead) {
    DayNightState s = DayNightCycle::evaluateAt(0.5, EQUATOR, EQUINOX);
    // At noon the sun points straight up.
    CHECK_APPROX(s.sunDirection.y, 1.0, 1e-6);
    CHECK(s.sunIntensity > 1.0f);
    CHECK(s.skyDiscIntensity > 0.99f);
}

TEST_CASE(day_night_midnight_sun_below_horizon) {
    DayNightState s = DayNightCycle::evaluateAt(0.0, EQUATOR, EQUINOX);
    CHECK_APPROX(s.sunDirection.y, -1.0, 1e-6);
    // Night: directional sun and the visible disc are extinguished.
    CHECK_APPROX(s.sunIntensity, 0.0, 1e-6);
    CHECK_APPROX(s.skyDiscIntensity, 0.0, 1e-6);
}

// WS2 ("night is way too dark"): midnight is MOONLIT, not unlit. The active
// light hands off to the moon — overhead at midnight, cool, ~200x dimmer than
// the noon sun — while the solar truth stays below the horizon for the lamp
// predicates.
TEST_CASE(day_night_midnight_is_moonlit) {
    DayNightState s = DayNightCycle::evaluateAt(0.0, EQUATOR, EQUINOX);
    CHECK(s.solarElevation < -0.9f);            // the sun's truth: deep night
    CHECK_APPROX(s.lightDirection.y, 1.0, 1e-6);  // full moon overhead
    CHECK(s.lightIntensity > 0.05f);            // lit — readable with adaptation
    CHECK(s.lightIntensity < 0.3f);             // but unmistakably night
    CHECK(s.lightColor.z > s.lightColor.x);     // cool blue moonlight
    // New moon: the intensity scales away, nights can still be truly dark.
    DayNightState newMoon = DayNightCycle::evaluateAt(0.0, EQUATOR, EQUINOX, 0.0);
    CHECK_APPROX(newMoon.lightIntensity, 0.0, 1e-6);
}

TEST_CASE(day_night_noon_active_light_is_the_sun) {
    DayNightState s = DayNightCycle::evaluateAt(0.5, EQUATOR, EQUINOX);
    CHECK_APPROX(s.lightDirection.y, 1.0, 1e-6);
    CHECK_APPROX(s.lightIntensity, s.sunIntensity, 1e-6);
    CHECK(s.solarElevation > 0.9f);
}

TEST_CASE(day_night_handoff_is_seamless_because_both_lights_are_dim) {
    // Scan dusk: wherever the active light switches sun->moon, both sides of
    // the switch are near-black, so the sky bake never visibly pops.
    float maxSwitchIntensity = 0.0f;
    Vec3 prevDir = DayNightCycle::evaluateAt(0.70, EQUATOR, EQUINOX).lightDirection;
    for (int i = 1; i <= 40; ++i) {
        double t = 0.70 + 0.10 * (i / 40.0);   // sunset window
        DayNightState s = DayNightCycle::evaluateAt(t, EQUATOR, EQUINOX);
        const bool switched = (prevDir - s.lightDirection).length() > 1.0;
        if (switched)
            maxSwitchIntensity = std::max(maxSwitchIntensity, s.lightIntensity);
        prevDir = s.lightDirection;
    }
    CHECK(maxSwitchIntensity < 0.06f);
}

TEST_CASE(day_night_sunrise_sunset_on_horizon) {
    DayNightState rise = DayNightCycle::evaluateAt(0.25, EQUATOR, EQUINOX);
    DayNightState set  = DayNightCycle::evaluateAt(0.75, EQUATOR, EQUINOX);
    // Sun grazes the horizon at both, on opposite sides (east vs west).
    CHECK_APPROX(rise.sunDirection.y, 0.0, 1e-6);
    CHECK_APPROX(set.sunDirection.y, 0.0, 1e-6);
    CHECK(rise.sunDirection.x > 0.5);   // east
    CHECK(set.sunDirection.x < -0.5);   // west
}

TEST_CASE(day_night_sun_direction_is_normalized) {
    for (int i = 0; i < 8; i++) {
        double t = i / 8.0;
        Vec3 d = DayNightCycle::evaluateAt(t, 40.0, 23.44).sunDirection;
        CHECK_APPROX(d.length(), 1.0, 1e-9);
    }
}

TEST_CASE(day_night_noon_brighter_than_dusk) {
    DayNightState noon = DayNightCycle::evaluateAt(0.5, EQUATOR, EQUINOX);
    DayNightState dusk = DayNightCycle::evaluateAt(0.75, EQUATOR, EQUINOX);
    CHECK(noon.sunIntensity > dusk.sunIntensity);
    CHECK(noon.ambient > dusk.ambient);
}

TEST_CASE(day_night_advance_wraps_into_unit_interval) {
    DayNightCycle c;
    c.timeOfDay = 0.9;
    c.dayMinutes = 2.0 / 60.0;   // a 2-second day: 0.5 days per second
    c.paused = false;
    CHECK_APPROX(c.speed(), 0.5, 1e-12);
    c.advance(1.0);              // +0.5 -> 1.4 -> wraps to 0.4
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
    c.dayMinutes = 1.0 / 60.0;   // a one-second day
    c.advance(1.0);
    CHECK_APPROX(c.timeOfDay, 0.2, 1e-9);
}

TEST_CASE(day_night_dusk_horizon_is_warm) {
    // At sunset the horizon should skew warm (red > blue).
    DayNightState set = DayNightCycle::evaluateAt(0.75, EQUATOR, EQUINOX);
    CHECK(set.horizonColor.x > set.horizonColor.z);
}

// THE THIRTY-MINUTE DAY (device: "the day/night cycle zips by really fast
// ... I'd like to be able to see a day-night loop happen over 30 min").
// The loop is authored in real minutes, and a fresh cycle stepped at the
// engine's 60 Hz fixed step for exactly 30 real minutes comes back to where
// it started — half way through it is half a day on.
TEST_CASE(day_night_default_loop_is_thirty_real_minutes) {
    DayNightCycle c;
    CHECK_APPROX(c.dayMinutes, 30.0, EPS);
    CHECK_APPROX(c.speed(), 1.0 / 1800.0, 1e-15);
    c.timeOfDay = 0.2;
    const double step = 1.0 / 60.0;
    for (int i = 0; i < 15 * 60 * 60; ++i) c.advance(step);   // 15 real minutes
    CHECK_APPROX(c.timeOfDay, 0.7, 1e-6);
    for (int i = 0; i < 15 * 60 * 60; ++i) c.advance(step);   // 30 in all
    CHECK_APPROX(c.timeOfDay, 0.2, 1e-6);
    // Ten-minute day, ten-minute night is the other spelling of the ask:
    // twenty minutes for the loop.
    c.dayMinutes = 20.0;
    CHECK_APPROX(c.speed(), 1.0 / 1200.0, 1e-15);
    c.dayMinutes = 0.0;          // a level-authored still: frozen, not NaN
    CHECK_APPROX(c.speed(), 0.0, EPS);
    c.timeOfDay = 0.3;
    c.advance(100.0);
    CHECK_APPROX(c.timeOfDay, 0.3, EPS);
}

// A REAL-WORLD CADENCE (device: "base it more on a real world cadence — I
// guess there's more daylight than night"). The default sun is drawn for
// 40° N on June 21: measured minute by minute, the sun is up for 14.8 of
// the 24 hours, from ~04:35 to ~19:25 — and the analytic sunrise/sunset the
// panel and `daynight?` report agree with the census to the minute.
TEST_CASE(day_night_default_summer_day_is_longer_than_its_night) {
    DayNightCycle c;
    CHECK_APPROX(c.latitudeDeg, 40.0, EPS);
    CHECK(c.dayOfYear == 172);
    CHECK(c.declinationDeg() > 23.0);
    const LitCensus lit = census(c.latitudeDeg, c.declinationDeg());
    const double litHours = lit.litMinutes / 60.0;
    CHECK(litHours > 14.5);
    CHECK(litHours < 15.1);
    CHECK(lit.firstLit > 4.4);            // ~04:35
    CHECK(lit.firstLit < 4.8);
    CHECK(lit.lastLit > 19.2);            // ~19:25
    CHECK(lit.lastLit < 19.6);
    // The analytic crossings match the measured ones (within a sample).
    CHECK_APPROX(c.sunriseHour(), lit.firstLit, 1.0 / 60.0 + 1e-9);
    CHECK_APPROX(c.sunsetHour(), lit.lastLit, 1.0 / 60.0 + 1e-9);
    CHECK_APPROX(c.daylightFraction() * 24.0, litHours, 2.0 / 60.0);
    // Of a 30-minute loop that is 18.5 minutes of day and 11.5 of night.
    CHECK(c.daylightFraction() * c.dayMinutes > 18.0);
    CHECK(c.daylightFraction() * c.dayMinutes < 19.0);
    // Noon leans south (+z, where the old fixed tilt leaned), never overhead
    // at this latitude — shadows keep falling the way every level was lit.
    DayNightState noon = c.evaluate();
    (void)noon;
    DayNightState n = DayNightCycle::evaluateAt(0.5, c.latitudeDeg, c.declinationDeg());
    CHECK(n.sunDirection.z > 0.2);
    CHECK(n.sunDirection.y > 0.9);
}

TEST_CASE(day_night_seasons_swing_the_day_length) {
    // Winter solstice at the same latitude: a 9.2-hour day, a long night.
    const double winter = DayNightCycle::declinationForDay(355);
    CHECK(winter < -23.0);
    const LitCensus dec = census(40.0, winter);
    CHECK(dec.litMinutes / 60.0 > 9.0);
    CHECK(dec.litMinutes / 60.0 < 9.5);
    // Equinoxes: twelve and twelve, anywhere on Earth.
    CHECK(std::fabs(DayNightCycle::declinationForDay(80)) < 1.0);
    CHECK_APPROX(DayNightCycle::daylightFractionFor(40.0, 0.0), 0.5, 1e-12);
    CHECK_APPROX(DayNightCycle::daylightFractionFor(-33.0, 0.0), 0.5, 1e-12);
    CHECK_APPROX(census(40.0, 0.0).litMinutes / 60.0, 12.0, 2.0 / 60.0);
    // Southern hemisphere in June: their winter.
    CHECK(DayNightCycle::daylightFractionFor(-40.0, 23.44) < 0.4);
    // Polar: the sun never sets past the circle in June, never rises in Dec.
    CHECK_APPROX(DayNightCycle::daylightFractionFor(80.0, 23.44), 1.0, 1e-12);
    CHECK_APPROX(DayNightCycle::daylightFractionFor(80.0, -23.44), 0.0, 1e-12);
    CHECK_APPROX(DayNightCycle::sunriseHourFor(80.0, 23.44), 0.0, 1e-9);
    CHECK_APPROX(DayNightCycle::sunsetHourFor(80.0, 23.44), 24.0, 1e-9);
}

TEST_CASE(day_night_equator_equinox_is_the_symmetric_arc) {
    // The legacy arc, exactly: sunrise 06:00 due east, sunset 18:00 due west.
    CHECK_APPROX(DayNightCycle::sunriseHourFor(EQUATOR, EQUINOX), 6.0, 1e-9);
    CHECK_APPROX(DayNightCycle::sunsetHourFor(EQUATOR, EQUINOX), 18.0, 1e-9);
    DayNightState rise = DayNightCycle::evaluateAt(0.25, EQUATOR, EQUINOX);
    CHECK_APPROX(rise.sunDirection.x, 1.0, 1e-6);
    CHECK_APPROX(rise.sunDirection.z, 0.0, 1e-6);
}

TEST_CASE(day_night_moon_is_up_exactly_when_the_sun_is_down) {
    // The antipode: through a long summer night the moon is above the
    // horizon precisely while the sun is below it, so the night light never
    // switches off early or lingers into the morning.
    for (int m = 0; m < 24 * 60; m += 7) {
        const double t = (m + 0.5) / (24.0 * 60.0);
        DayNightState s = DayNightCycle::evaluateAt(t, 40.0, 23.44);
        const bool sunUp = s.solarElevation > 0.0f;
        const bool moonIsActive = s.lightDirection.y > 0.0f && s.solarElevation < -0.15f;
        if (s.solarElevation < -0.15f) CHECK(moonIsActive);
        if (sunUp) CHECK_APPROX(s.lightDirection.y, s.sunDirection.y, 1e-6);
    }
}
