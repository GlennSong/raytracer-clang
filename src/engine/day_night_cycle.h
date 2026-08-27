#ifndef RAYTRACER_ENGINE_DAY_NIGHT_CYCLE_H
#define RAYTRACER_ENGINE_DAY_NIGHT_CYCLE_H

#include "../rt_math.h"

namespace engine {

// Lighting + sky colors produced by the day/night model at a moment in time.
// Consumed by the renderer's procedural sky (ADR-0016) and the scene's
// directional sun light, so the sky and the shading that lights the world stay
// locked together as the cycle advances.
// The curve's NOON anchors — what sunIntensity/ambient evaluate to at full
// day. DayNightSystem NORMALIZES against these so the cycle acts as a
// day-shape MULTIPLIER on the level's AUTHORED sun/ambient instead of
// replacing them (metro authors sun 8.0 + ambient 0.6; the raw curve values
// 5.0 + 0.34 silently overrode both — "certain values are being ignored").
constexpr float kCycleNoonSunIntensity = 5.0f;
constexpr float kCycleNoonAmbient = 0.34f;

// THE MONTH (device: "give the moon phases — I guess we introduce concepts of
// a month"): one synodic month, new moon to new moon.
constexpr double kSynodicMonthDays = 29.530589;

struct DayNightState {
    Vec3  sunDirection;     // normalized, toward the sun (world space)
    Vec3  sunColor;         // directional-light + sky-disc tint (linear)
    float sunIntensity;     // directional-light intensity (≈0 at night)
    float skyDiscIntensity; // procedural sun disc brightness (fades through dusk)
    Vec3  zenithColor;      // sky dome at the top
    Vec3  horizonColor;     // sky at the horizon
    Vec3  groundColor;      // below-horizon tint
    float ambient;          // ambient multiplier (cool + low at night)

    // The ACTIVE light — what the renderer should drive the sky and shading
    // with. Sun by day; the MOON by night (WS2, "night is way too dark"): a
    // moonlit sky is a blue-shifted day sky a few hundred times dimmer, so
    // driving the same scattering pipeline with a dim cool moon buys sky,
    // IBL ambient, and aerial haze at night for free. The handoff is a hard
    // switch at the intensity crossover — both lights are near-black there,
    // so nothing visibly pops (the sky bake re-keys on direction+intensity).
    Vec3  lightDirection;   // normalized, toward the active light
    Vec3  lightColor;
    float lightIntensity;
    // The sun's TRUTH regardless of which light is active: dusk predicates
    // (vehicle lamps, street lights) must key on solar elevation, or the
    // moon rising would read as daylight and switch every lamp off.
    float solarElevation;

    // THE MOON, as a body: where it is, how much of its face the sun lights,
    // and how bright its disc should be drawn (0 below the horizon). The sky
    // shader lights the disc as a sphere from `sunDirection`, so the crescent
    // faces the sun by construction.
    Vec3  moonDirection;    // normalized, toward the moon
    float moonAgeDays;      // 0 = new, kSynodicMonthDays/2 = full
    float moonIllumination; // lit fraction of the disc, 0..1
    float moonDiscIntensity;

    // THE STARS: the celestial (equatorial) frame in local space, rotated by
    // local sidereal time — X toward RA 0 / Dec 0, Y toward RA 90, Z the
    // pole (due north, altitude = latitude). The sky shader dots a view
    // direction with these to read its (RA, Dec) and hashes a star field
    // there, so the stars wheel about the pole with the hour and slide with
    // the season. starVisibility: 0 by day, 1 in deep night, washed by a
    // bright moon.
    Vec3  celestialX, celestialY, celestialZ;
    float starVisibility;
};

// Maps a normalized time-of-day to a sun arc and a graded sky/light palette.
// Time runs in [0, 1): 0.0 = midnight, 0.5 = noon; sunrise and sunset fall
// where the arc crosses the horizon, which depends on the SEASON and the
// LATITUDE below (0.25 / 0.75 only at the equator, or anywhere at an
// equinox). Pure and window-free — the visual curve is unit-tested headless;
// the renderer-side shader only interpolates the colors this produces.
//
// THE SUN IS REAL GEOMETRY (device: "base it more on a real world cadence
// ... there's more daylight than night"). The daily path is the standard
// solar model — hour angle around the celestial pole, pole tilted by the
// latitude, sun offset from the equator by the declination — so a summer day
// at 40° N is 14.8 h of light and 9.2 h of dark, exactly as outside. The
// old arc was a circle tilted by a fixed `axisTilt`: always 12 h / 12 h.
// Engine frame: +x east, +y up, and the noon sun leans toward +z (the old
// tilt leaned +z too, so every level's shadows keep falling the way they
// did). +z is therefore SOUTH here.
//
// THE CALENDAR: the day of year turns when the clock wraps midnight (a year
// counter keeps the day count continuous across New Year), and the MOON
// rides it — its age since the last new moon sets both its lit fraction and
// WHERE it is: elongation from the sun along the ecliptic, so a new moon
// travels with the sun (invisible, up by day), a first quarter sets at
// midnight, a full moon rises at sunset and is up all night.
class DayNightCycle {
public:
    double timeOfDay = 0.35;   // start mid-morning
    // THE LOOP IS AUTHORED IN REAL MINUTES (device: "the day/night cycle zips
    // by really fast ... a day-night loop over 30 min"). The previous knob
    // was a bare 0.02 days-per-second — a 50-second day — and every surface
    // (settings, level JSON, web slider) restated it in that unit. <= 0
    // freezes the sun where it is (a level-authored still).
    double dayMinutes = 30.0;
    double latitudeDeg = 40.0;   // observer latitude (°N; negative = south)
    int    dayOfYear = 172;      // 1..365; 172 = June 21 (longest day north)
    int    year = 0;             // turns with the calendar; keeps the moon continuous
    // The day of year (in year 0) of a new moon the month is counted from.
    // 18 = January 18, 2026's new moon.
    double newMoonDay = 18.0;
    // Artistic hold on the moon's AGE in days (>= 0); < 0 = follow the
    // calendar. Replaces the old 0..1 `moonPhase` brightness scalar.
    double moonLock = -1.0;
    bool   paused    = false;

    // Day fraction per real second — derived, never stored (two knobs for
    // one rate is how the 50-second day survived: nobody could say which
    // number was the truth).
    double speed() const { return dayMinutes > 0.0 ? 1.0 / (dayMinutes * 60.0) : 0.0; }
    double declinationDeg() const { return declinationForDay(dayOfYear); }
    // Ecliptic longitude of the sun for the day (0 at the March equinox).
    double solarLongitudeDeg() const { return solarLongitudeForDay(dayOfYear); }

    // Days since the calendar's origin (year 0, January 1, midnight),
    // fractional: continuous across midnight and New Year.
    double absoluteDay() const { return year * 365.0 + (dayOfYear - 1) + timeOfDay; }
    // Days since the last new moon, [0, kSynodicMonthDays).
    double moonAgeDays() const;
    double moonIllumination() const { return illuminationForAge(moonAgeDays()); }

    // Advance time by dt seconds (no-op when paused), wrapping into [0, 1)
    // and turning the calendar for every midnight crossed.
    void advance(double dt);

    DayNightState evaluate() const {
        return evaluateAt(timeOfDay, latitudeDeg, declinationDeg(), moonAgeDays(),
                          solarLongitudeDeg());
    }
    // Where the arc crosses the horizon TODAY, as hours [0, 24). A polar day
    // reports 0 / 24; a polar night 12 / 12.
    double sunriseHour() const { return sunriseHourFor(latitudeDeg, declinationDeg()); }
    double sunsetHour() const { return sunsetHourFor(latitudeDeg, declinationDeg()); }
    // Fraction of the loop the sun is above the horizon (0.5 = equinox).
    double daylightFraction() const { return daylightFractionFor(latitudeDeg, declinationDeg()); }

    // Stateless evaluation, exposed for testing and reuse. moonAgeDays
    // defaults to a full moon; solarLongitudeDeg (for the moon's declination)
    // is derived from the declination when left at the sentinel.
    static DayNightState evaluateAt(double timeOfDay, double latitudeDeg,
                                    double declinationDeg = 0.0,
                                    double moonAgeDays = kSynodicMonthDays * 0.5,
                                    double solarLongitudeDeg = 1e9);
    // Solar declination for a day of the year (Cooper's approximation:
    // ±23.44° at the solstices, 0 at the equinoxes, within ~1° all year).
    static double declinationForDay(int dayOfYear);
    static double solarLongitudeForDay(int dayOfYear);
    // Half the day arc in radians: acos(-tan(lat) tan(dec)), clamped to
    // [0, π] for polar night/day.
    static double halfDayArc(double latitudeDeg, double declinationDeg);
    static double daylightFractionFor(double latitudeDeg, double declinationDeg);
    static double sunriseHourFor(double latitudeDeg, double declinationDeg);
    static double sunsetHourFor(double latitudeDeg, double declinationDeg);
    // Lit fraction of the disc for an age: (1 - cos(elongation)) / 2.
    static double illuminationForAge(double ageDays);
    // "New", "Waxing crescent", "First quarter", ... "Waning crescent".
    static const char* phaseName(double ageDays);
};

}  // namespace engine

#endif
