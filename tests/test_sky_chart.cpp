#include "test_framework.h"

#include "../src/engine/sky_chart.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

// THE SKY CHART (device: "can we do sun and moon positions in the sky?").
// Positions as numbers: bearing and altitude, the day's arcs, the horizon
// crossings — held against the analytic sun and the moon's phase geometry.

TEST_CASE(sky_chart_point_uses_the_street_maps_orientation) {
    // East is right, north is up the page (south down), zenith the centre.
    ChartPoint e = skyChartPoint(Vec3(1, 0, 0));
    CHECK_APPROX(e.x, 1.0, 1e-9);
    CHECK_APPROX(e.y, 0.0, 1e-9);
    ChartPoint n = skyChartPoint(Vec3(0, 0, -1));
    CHECK_APPROX(n.x, 0.0, 1e-9);
    CHECK_APPROX(n.y, -1.0, 1e-9);
    ChartPoint z = skyChartPoint(Vec3(0, 1, 0));
    CHECK_APPROX(z.x, 0.0, 1e-9);
    CHECK_APPROX(z.y, 0.0, 1e-9);
    // 45 deg up in the south: half way from the centre, straight down the page.
    ChartPoint s45 = skyChartPoint(normalize(Vec3(0, 1, 1)));
    CHECK_APPROX(s45.x, 0.0, 1e-9);
    CHECK_APPROX(s45.y, 0.5, 1e-9);
    CHECK_APPROX(skyBearingDeg(Vec3(0, 0, -1)), 0.0, 1e-9);      // north
    CHECK_APPROX(skyBearingDeg(Vec3(1, 0, 0)), 90.0, 1e-9);      // east
    CHECK_APPROX(skyBearingDeg(Vec3(0, 0, 1)), 180.0, 1e-9);     // south
    CHECK_APPROX(skyBearingDeg(Vec3(-1, 0, 0)), 270.0, 1e-9);    // west
    CHECK_APPROX(skyAltitudeDeg(normalize(Vec3(0, 1, 1))), 45.0, 1e-9);
}

TEST_CASE(sky_chart_sun_arc_matches_the_analytic_day) {
    // 40 N on June 21: culminates due south at 90 - (40 - 23.44) = 73.4 deg,
    // rises at bearing acos(sin(dec)/cos(lat)) = 58.7 deg (north of east).
    DayNightCycle c;   // defaults: 40 N, day 172
    c.timeOfDay = 0.5;
    const SkyChart chart = buildSkyChart(c, 5);
    double peakAlt = -90, peakAz = 0;
    for (const SkyChartSample& s : chart.sun)
        if (s.altitudeDeg > peakAlt) { peakAlt = s.altitudeDeg; peakAz = s.azimuthDeg; }
    CHECK_APPROX(peakAlt, 90.0 - (40.0 - c.declinationDeg()), 0.3);
    CHECK_APPROX(peakAz, 180.0, 2.0);
    // The first lit sample sits on the rim near the analytic sunrise bearing.
    const SkyChartSample* first = nullptr;
    for (const SkyChartSample& s : chart.sun) if (s.up) { first = &s; break; }
    CHECK(first != nullptr);
    if (first) {
        const double dec = c.declinationDeg() * 3.14159265358979323846 / 180.0;
        const double lat = 40.0 * 3.14159265358979323846 / 180.0;
        const double riseBearing = std::acos(std::sin(dec) / std::cos(lat)) * 180.0 / 3.14159265358979323846;
        CHECK_APPROX(first->azimuthDeg, riseBearing, 3.0);
        CHECK_APPROX(first->hour, chart.sunriseHour, 6.0 / 60.0);
        CHECK(first->altitudeDeg < 2.0);
    }
    // "Now" is noon: the sun on the meridian, its bearing south.
    CHECK_APPROX(chart.sunNow.azimuthDeg, 180.0, 1e-6);
    CHECK_APPROX(chart.sunNow.altitudeDeg, peakAlt, 0.3);
    CHECK(chart.sunNow.up);
}

TEST_CASE(sky_chart_moon_rises_and_sets_where_its_phase_says) {
    DayNightCycle c;
    c.latitudeDeg = 0.0;
    c.dayOfYear = 80;            // ~equinox
    c.timeOfDay = 0.0;
    // Full moon: rises with sunset, sets with sunrise.
    c.moonLock = kSynodicMonthDays * 0.5;
    SkyChart full = buildSkyChart(c, 5);
    CHECK_APPROX(full.moonriseHour, full.sunsetHour, 0.25);
    CHECK_APPROX(full.moonsetHour, full.sunriseHour, 0.25);
    CHECK(full.moonNow.up);      // midnight: the full moon is up
    CHECK(!full.sunNow.up);
    // First quarter: rises at noon, sets at midnight.
    c.moonLock = kSynodicMonthDays * 0.25;
    SkyChart q1 = buildSkyChart(c, 5);
    CHECK_APPROX(q1.moonriseHour, 12.0, 0.25);
    CHECK(q1.moonsetHour < 0.25 || q1.moonsetHour > 23.75);
    // New moon: with the sun — up by day only.
    c.moonLock = 0.0;
    SkyChart nm = buildSkyChart(c, 5);
    CHECK_APPROX(nm.moonriseHour, nm.sunriseHour, 0.25);
    CHECK_APPROX(nm.moonsetHour, nm.sunsetHour, 0.25);
    CHECK(std::string(nm.moonPhase) == "New");
}

TEST_CASE(sky_chart_svg_carries_both_arcs_and_the_legend) {
    DayNightCycle c;
    c.timeOfDay = 21.0 / 24.0;
    c.moonLock = kSynodicMonthDays * 0.25;
    const SkyChart chart = buildSkyChart(c, 5);
    const std::string path = "sky_chart_test.svg";
    CHECK(writeSkyChartSvg(chart, path));
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string svg = ss.str();
    std::remove(path.c_str());
    CHECK(svg.find("<svg") != std::string::npos);
    CHECK(svg.find("stroke='#ffb347'") != std::string::npos);   // the sun arc
    CHECK(svg.find("stroke='#aab6e0'") != std::string::npos);   // the moon arc
    CHECK(svg.find("rise ") != std::string::npos);
    CHECK(svg.find("moonrise") != std::string::npos);
    CHECK(svg.find(">N</text>") != std::string::npos);
    CHECK(svg.find("First quarter") != std::string::npos);
    CHECK(svg.find("Same orientation as the street map") != std::string::npos);
    // At 21:00 in June the sun is set (hollow) and the quarter moon is up.
    CHECK(svg.find("sun 21:00 (set)") != std::string::npos);
    CHECK(svg.find("moon 21:00<") != std::string::npos);
}
