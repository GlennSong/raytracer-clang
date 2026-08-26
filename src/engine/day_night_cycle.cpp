#include "day_night_cycle.h"

#include <algorithm>
#include <cmath>

namespace engine {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDeg = kPi / 180.0;

// Hermite smoothstep, clamped — the standard 0→1 ease used throughout.
double smoothstep(double edge0, double edge1, double x) {
    double t = (x - edge0) / (edge1 - edge0);
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return t * t * (3.0 - 2.0 * t);
}

double mix(double a, double b, double t) { return a + (b - a) * t; }

}  // namespace

void DayNightCycle::advance(double dt) {
    if (paused) return;
    timeOfDay += speed() * dt;
    timeOfDay -= std::floor(timeOfDay);  // wrap into [0, 1)
}

double DayNightCycle::declinationForDay(int dayOfYear) {
    // Cooper (1969): δ = 23.44° · sin(2π (284 + N) / 365).
    const int n = std::clamp(dayOfYear, 1, 365);
    return 23.44 * std::sin(kTwoPi * (284.0 + n) / 365.0);
}

double DayNightCycle::halfDayArc(double latitudeDeg, double declinationDeg) {
    // cos H0 = -tan φ tan δ; beyond ±1 the sun never sets / never rises.
    const double c = -std::tan(latitudeDeg * kDeg) * std::tan(declinationDeg * kDeg);
    return std::acos(std::clamp(c, -1.0, 1.0));
}

double DayNightCycle::daylightFractionFor(double latitudeDeg, double declinationDeg) {
    return halfDayArc(latitudeDeg, declinationDeg) / kPi;
}

double DayNightCycle::sunriseHourFor(double latitudeDeg, double declinationDeg) {
    return 12.0 - 12.0 * daylightFractionFor(latitudeDeg, declinationDeg);
}

double DayNightCycle::sunsetHourFor(double latitudeDeg, double declinationDeg) {
    return 12.0 + 12.0 * daylightFractionFor(latitudeDeg, declinationDeg);
}

DayNightState DayNightCycle::evaluateAt(double timeOfDay, double latitudeDeg,
                                        double declinationDeg, double moonPhase) {
    // Sun position from the solar model. Hour angle H: 0 at local noon,
    // negative through the morning (sun in the east), +π at midnight.
    //   up    = sin φ sin δ + cos φ cos δ cos H
    //   east  = -cos δ sin H
    //   north = cos φ sin δ - sin φ cos δ cos H
    // Engine z points SOUTH (see the header), so z = -north: at noon in the
    // northern hemisphere the sun leans toward +z, as the old tilt did.
    const double H = (timeOfDay - 0.5) * kTwoPi;
    const double sp = std::sin(latitudeDeg * kDeg), cp = std::cos(latitudeDeg * kDeg);
    const double sd = std::sin(declinationDeg * kDeg), cd = std::cos(declinationDeg * kDeg);
    const double up = sp * sd + cp * cd * std::cos(H);
    const double east = -cd * std::sin(H);
    const double north = cp * sd - sp * cd * std::cos(H);
    Vec3 sunDir = normalize(Vec3(east, up, -north));
    double elev = sunDir.y;  // -1 (deepest night) .. 1 (highest noon)

    // Blend weights. day: night→day. dusk: warm band hugging the horizon, gated
    // off once the sun drops well below it so deep night stays cold.
    double day  = smoothstep(-0.10, 0.20, elev);
    double dusk = (1.0 - smoothstep(0.0, 0.30, std::abs(elev))) *
                  smoothstep(-0.15, 0.02, elev);

    // Palettes (linear RGB).
    const Vec3 dayZenith(0.25, 0.45, 0.85);
    const Vec3 dayHorizon(0.62, 0.76, 0.92);
    const Vec3 dayGround(0.35, 0.30, 0.25);
    const Vec3 nightZenith(0.015, 0.02, 0.06);
    const Vec3 nightHorizon(0.03, 0.04, 0.10);
    const Vec3 nightGround(0.01, 0.01, 0.02);
    const Vec3 duskHorizon(0.95, 0.45, 0.18);
    const Vec3 duskZenith(0.30, 0.22, 0.45);

    Vec3 zenith  = lerp(nightZenith,  dayZenith,  day);
    Vec3 horizon = lerp(nightHorizon, dayHorizon, day);
    Vec3 ground  = lerp(nightGround,  dayGround,  day);
    horizon = lerp(horizon, duskHorizon, dusk);
    zenith  = lerp(zenith,  duskZenith,  dusk * 0.6);

    // Sun color: warm orange grazing the horizon, neutral high in the sky.
    const Vec3 warm(1.0, 0.55, 0.28);
    const Vec3 midday(1.0, 0.97, 0.92);
    Vec3 sunColor = lerp(warm, midday, smoothstep(0.0, 0.45, elev));

    DayNightState s;
    s.sunDirection     = sunDir;
    s.sunColor         = sunColor;
    // Illuminance-scale units (ADR-0017 Phase 1): the Lambert 1/π in the shader
    // means ~π × the old Blinn-Phong value preserves the same noon brightness.
    s.sunIntensity     = static_cast<float>(5.0 * day);
    s.skyDiscIntensity = static_cast<float>(smoothstep(-0.04, 0.10, elev));
    s.zenithColor      = zenith;
    s.horizonColor     = horizon;
    s.groundColor      = ground;
    // Night floor raised from 0.04 (WS2): the multiplier now scales a MOONLIT
    // sky's irradiance instead of a black one, so the old near-zero floor
    // would crush the very light the moon just paid for. Day value unchanged.
    s.ambient          = static_cast<float>(mix(0.28, 0.34, day));

    // --- the moon -----------------------------------------------------------
    // The sun's antipode: highest at midnight, up whenever the sun is down
    // (a long summer night's moon is up for its 9 hours, no more). True
    // moonlight is ~1/400000 of sunlight; games that read as "moonlit" sit
    // near 1/50 with adapted exposure (Glenn picked moonlit-realistic:
    // terrain and roads readable, lamps still matter). Measured on the metro
    // skyline: 1/200 lit the CLOUDS beautifully (in-scatter accumulates along
    // the whole ray) while the ground's single diffuse bounce stayed black —
    // with night exposure adaptation x6, 1/50 puts the ground near 12% of its
    // daytime tonemapped level: dim blue shapes, readable roads.
    Vec3 moonDir = Vec3(-sunDir.x, -sunDir.y, -sunDir.z);
    const Vec3 moonColor(0.62, 0.72, 0.95);
    double moonUp = smoothstep(-0.10, 0.20, moonDir.y);
    // The moon waits for true twilight (sun a few degrees down), not golden
    // hour — this keeps sunset light sun-owned AND makes the slot-0 handoff
    // dim by construction (both lights near-black at the crossover).
    double nightGate = smoothstep(0.02, 0.15, -elev);
    double moonI = (5.0 / 50.0) * moonPhase * nightGate * moonUp;

    s.solarElevation = static_cast<float>(elev);
    if (s.sunIntensity >= moonI) {
        s.lightDirection = sunDir;
        s.lightColor     = sunColor;
        s.lightIntensity = s.sunIntensity;
    } else {
        s.lightDirection = moonDir;
        s.lightColor     = moonColor;
        s.lightIntensity = static_cast<float>(moonI);
    }
    return s;
}

}  // namespace engine
