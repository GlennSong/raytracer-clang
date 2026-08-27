#ifndef RAYTRACER_ENGINE_SKY_CHART_H
#define RAYTRACER_ENGINE_SKY_CHART_H

#include "day_night_cycle.h"

#include <string>
#include <vector>

namespace engine {

// THE SKY CHART (device: "can we do sun and moon positions in the sky?"). One
// day of the cycle's sun and moon as POSITIONS: altitude/azimuth samples
// across the 24 hours, the horizon crossings, and the two bodies right now.
// Pure — built from a DayNightCycle copy, so the same numbers feed the SVG
// instrument (`daynight chart <path>` / RT_SKY_SVG), the in-game HUD, and
// the tests. Azimuth is a compass bearing: 0 north (-z), 90 east (+x).
struct SkyChartSample {
    double hour = 0;         // 0..24
    double azimuthDeg = 0;   // bearing, [0, 360)
    double altitudeDeg = 0;  // -90..90
    Vec3   dir;              // unit, toward the body
    bool   up = false;       // above the horizon
};

struct SkyChart {
    std::vector<SkyChartSample> sun, moon;   // one per `stepMinutes`
    double sunriseHour = 0, sunsetHour = 24; // analytic (DayNightCycle)
    // From the samples: the first horizon crossings of the day, -1 = none
    // (a moon that is up all day, or never, sets neither).
    double moonriseHour = -1, moonsetHour = -1;
    bool   moonAlwaysUp = false, moonNeverUp = false;
    // Now.
    double nowHour = 0;
    SkyChartSample sunNow, moonNow;
    double moonAgeDays = 0, moonIllumination = 0;
    const char* moonPhase = "";
    // The day the chart is for.
    double latitudeDeg = 0;
    int    dayOfYear = 1, year = 0;
    double dayMinutes = 0;
};

// Bearing/altitude of a unit direction in the engine frame (+x east, +y up,
// +z south).
double skyBearingDeg(const Vec3& dir);
double skyAltitudeDeg(const Vec3& dir);

// Chart-plane point for a direction, in [-1, 1]^2: the dome projected onto
// the ground in the STREET MAP's orientation (RT_FURNITURE_SVG: east right,
// south down), radius = 1 - altitude/90 — the zenith at the centre, the
// horizon on the rim. Below-horizon directions land outside the rim.
struct ChartPoint { double x = 0, y = 0; };
ChartPoint skyChartPoint(const Vec3& dir);

SkyChart buildSkyChart(const DayNightCycle& cycle, int stepMinutes = 5);

// The SVG instrument: both arcs with hour ticks, rise/set marks, the bodies
// now, a phase glyph, and a legend. False when the file cannot be written.
bool writeSkyChartSvg(const SkyChart& chart, const std::string& path);

}  // namespace engine

#endif
