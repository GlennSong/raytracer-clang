#include "sky_chart.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>

namespace engine {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg = kPi / 180.0;

SkyChartSample sampleOf(double hour, const Vec3& dir) {
    SkyChartSample s;
    s.hour = hour;
    s.dir = dir;
    s.azimuthDeg = skyBearingDeg(dir);
    s.altitudeDeg = skyAltitudeDeg(dir);
    s.up = dir.y > 0.0;
    return s;
}

void hm(double hours, int& h, int& m) {
    int total = static_cast<int>(std::floor(hours * 60.0 + 0.5));
    total = ((total % (24 * 60)) + 24 * 60) % (24 * 60);
    h = total / 60;
    m = total % 60;
}

std::string clock(double hours) {
    if (hours < 0.0) return "—";
    int h, m;
    hm(hours, h, m);
    char buf[16];
    std::snprintf(buf, sizeof(buf), "%02d:%02d", h, m);
    return buf;
}
}  // namespace

double skyBearingDeg(const Vec3& dir) {
    // North is -z, east +x: bearing clockwise from north.
    double b = std::atan2(dir.x, -dir.z) / kDeg;
    if (b < 0.0) b += 360.0;
    return b;
}

double skyAltitudeDeg(const Vec3& dir) {
    return std::asin(std::clamp(dir.y, -1.0, 1.0)) / kDeg;
}

ChartPoint skyChartPoint(const Vec3& dir) {
    const double alt = skyAltitudeDeg(dir);
    const double r = 1.0 - alt / 90.0;          // zenith 0, horizon 1, nadir 2
    const double b = skyBearingDeg(dir) * kDeg;
    // Map orientation: east (+x) right, south (+z) DOWN the page — north up.
    return ChartPoint{r * std::sin(b), -r * std::cos(b)};
}

SkyChart buildSkyChart(const DayNightCycle& cycle, int stepMinutes) {
    SkyChart c;
    if (stepMinutes < 1) stepMinutes = 1;
    c.latitudeDeg = cycle.latitudeDeg;
    c.dayOfYear = cycle.dayOfYear;
    c.year = cycle.year;
    c.dayMinutes = cycle.dayMinutes;
    c.sunriseHour = cycle.sunriseHour();
    c.sunsetHour = cycle.sunsetHour();
    c.nowHour = cycle.timeOfDay * 24.0;

    DayNightCycle day = cycle;   // a copy walks the day; the cycle is untouched
    bool prevUp = false, first = true;
    int upCount = 0, n = 0;
    for (int m = 0; m <= 24 * 60; m += stepMinutes) {
        const double hour = std::min(m / 60.0, 24.0 - 1e-9);
        day.timeOfDay = hour / 24.0;
        const DayNightState st = day.evaluate();
        c.sun.push_back(sampleOf(hour, st.sunDirection));
        const SkyChartSample ms = sampleOf(hour, st.moonDirection);
        c.moon.push_back(ms);
        ++n;
        if (ms.up) ++upCount;
        if (!first) {
            if (ms.up && !prevUp && c.moonriseHour < 0.0) c.moonriseHour = hour;
            if (!ms.up && prevUp && c.moonsetHour < 0.0) c.moonsetHour = hour;
        }
        prevUp = ms.up;
        first = false;
    }
    c.moonAlwaysUp = upCount == n;
    c.moonNeverUp = upCount == 0;

    const DayNightState now = cycle.evaluate();
    c.sunNow = sampleOf(c.nowHour, now.sunDirection);
    c.moonNow = sampleOf(c.nowHour, now.moonDirection);
    c.moonAgeDays = now.moonAgeDays;
    c.moonIllumination = now.moonIllumination;
    c.moonPhase = DayNightCycle::phaseName(now.moonAgeDays);
    return c;
}

bool writeSkyChartSvg(const SkyChart& chart, const std::string& path) {
    std::ofstream out(path);
    if (!out) return false;
    out << std::fixed << std::setprecision(1);

    const double W = 860, H = 800;
    const double cx = 430, cy = 350, R = 290;   // the dome (rim labels need the margin)
    auto px = [&](const Vec3& d) {
        const ChartPoint p = skyChartPoint(d);
        return ChartPoint{cx + p.x * R, cy + p.y * R};
    };

    out << "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 " << W << " " << H
        << "' width='" << W << "' height='" << H << "' font-family='Helvetica, Arial, sans-serif'>\n";
    out << "<rect width='" << W << "' height='" << H << "' fill='#0e1320'/>\n";
    // The dome: horizon rim, altitude rings every 30°, cardinal cross.
    out << "<circle cx='" << cx << "' cy='" << cy << "' r='" << R << "' fill='#16203a' stroke='#8aa0c8' stroke-width='2'/>\n";
    for (int alt = 30; alt < 90; alt += 30) {
        out << "<circle cx='" << cx << "' cy='" << cy << "' r='" << R * (1.0 - alt / 90.0)
            << "' fill='none' stroke='#2c3a5c' stroke-width='1' stroke-dasharray='6 5'/>\n";
        out << "<text x='" << cx + 4 << "' y='" << cy - R * (1.0 - alt / 90.0) - 3
            << "' fill='#5d6f95' font-size='11'>" << alt << "°</text>\n";
    }
    out << "<line x1='" << cx - R << "' y1='" << cy << "' x2='" << cx + R << "' y2='" << cy
        << "' stroke='#2c3a5c' stroke-width='1'/>\n";
    out << "<line x1='" << cx << "' y1='" << cy - R << "' x2='" << cx << "' y2='" << cy + R
        << "' stroke='#2c3a5c' stroke-width='1'/>\n";
    out << "<g fill='#c8d3ea' font-size='18' font-weight='bold' text-anchor='middle'>\n"
        << "<text x='" << cx << "' y='" << cy - R - 10 << "'>N</text>\n"
        << "<text x='" << cx + R + 16 << "' y='" << cy + 6 << "'>E</text>\n"
        << "<text x='" << cx << "' y='" << cy + R + 22 << "'>S</text>\n"
        << "<text x='" << cx - R - 16 << "' y='" << cy + 6 << "'>W</text>\n</g>\n";

    // An arc: the above-horizon samples as a polyline, split where it dips.
    auto arc = [&](const std::vector<SkyChartSample>& s, const char* colour, double width,
                   const char* dash, int tickEveryHours, const char* tickColour) {
        out << "<g fill='none' stroke='" << colour << "' stroke-width='" << width
            << "' stroke-linecap='round' stroke-linejoin='round'";
        if (dash) out << " stroke-dasharray='" << dash << "'";
        out << ">\n";
        bool open = false;
        for (const SkyChartSample& p : s) {
            if (!p.up) { if (open) { out << "'/>\n"; open = false; } continue; }
            const ChartPoint q = px(p.dir);
            if (!open) { out << "<polyline points='"; open = true; }
            out << q.x << "," << q.y << " ";
        }
        if (open) out << "'/>\n";
        out << "</g>\n";
        // Hour ticks + labels on the up samples that land on whole hours.
        out << "<g fill='" << tickColour << "' font-size='11'>\n";
        for (const SkyChartSample& p : s) {
            const double frac = p.hour - std::floor(p.hour + 1e-9);
            const int hour = static_cast<int>(std::floor(p.hour + 1e-9));
            if (!p.up || frac > 1e-6 || hour % tickEveryHours != 0 || hour >= 24) continue;
            const ChartPoint q = px(p.dir);
            out << "<circle cx='" << q.x << "' cy='" << q.y << "' r='2.5'/>\n";
            out << "<text x='" << q.x + 5 << "' y='" << q.y - 4 << "'>" << std::setw(2)
                << std::setfill('0') << hour << std::setfill(' ') << "</text>\n";
        }
        out << "</g>\n";
    };
    arc(chart.moon, "#aab6e0", 2.0, "7 4", 2, "#aab6e0");
    arc(chart.sun, "#ffb347", 3.5, nullptr, 1, "#ffd9a0");

    // Rise/set marks on the rim, with times.
    auto rimMark = [&](const std::vector<SkyChartSample>& s, double hour, const char* colour,
                       const char* label) {
        if (hour < 0.0 || s.empty()) return;
        // The sample nearest the hour, projected onto the rim.
        const SkyChartSample* best = &s.front();
        for (const SkyChartSample& p : s)
            if (std::fabs(p.hour - hour) < std::fabs(best->hour - hour)) best = &p;
        const double b = best->azimuthDeg * kDeg;
        const double x = cx + R * std::sin(b), y = cy - R * std::cos(b);
        const double ox = cx + (R + 14) * std::sin(b), oy = cy - (R + 14) * std::cos(b);
        out << "<circle cx='" << x << "' cy='" << y << "' r='5' fill='" << colour
            << "' stroke='#0e1320' stroke-width='1.5'/>\n";
        out << "<text x='" << ox << "' y='" << oy + 4 << "' fill='" << colour
            << "' font-size='11' text-anchor='" << (std::sin(b) < 0 ? "end" : "start") << "'>"
            << label << " " << clock(hour) << "</text>\n";
    };
    rimMark(chart.sun, chart.sunriseHour, "#ffb347", "rise");
    rimMark(chart.sun, chart.sunsetHour, "#ffb347", "set");
    rimMark(chart.moon, chart.moonriseHour, "#aab6e0", "moonrise");
    rimMark(chart.moon, chart.moonsetHour, "#aab6e0", "moonset");

    // The bodies now (hollow when below the horizon, drawn at the rim).
    auto body = [&](const SkyChartSample& s, double r, const char* fill, const char* label) {
        Vec3 d = s.dir;
        if (!s.up) {   // clamp to the rim so a set body still shows its bearing
            const double b = s.azimuthDeg * kDeg;
            d = Vec3(std::sin(b), 0.0, -std::cos(b));
        }
        const ChartPoint q = px(d);
        out << "<circle cx='" << q.x << "' cy='" << q.y << "' r='" << r << "' fill='"
            << (s.up ? fill : "none") << "' stroke='" << fill << "' stroke-width='2'/>\n";
        out << "<text x='" << q.x + r + 4 << "' y='" << q.y + 4 << "' fill='" << fill
            << "' font-size='12' font-weight='bold'>" << label << " " << clock(s.hour)
            << (s.up ? "" : " (set)") << "</text>\n";
    };
    body(chart.moonNow, 7, "#e8ecff", "moon");
    body(chart.sunNow, 9, "#ffd166", "sun");

    // Phase glyph: a dark disc with the lit lune (right side lit while waxing,
    // as almanacs draw it; the terminator is a half-ellipse whose width is
    // |2k - 1| of the radius).
    {
        const double gx = 56, gy = 700, gr = 22;
        const double k = std::clamp(chart.moonIllumination, 0.0, 1.0);
        const bool waxing = chart.moonAgeDays < kSynodicMonthDays * 0.5;
        const double rx = gr * std::fabs(2.0 * k - 1.0);
        out << "<circle cx='" << gx << "' cy='" << gy << "' r='" << gr
            << "' fill='#2a3350' stroke='#8aa0c8' stroke-width='1'/>\n";
        out << "<g transform='translate(" << gx << " " << gy << ") scale(" << (waxing ? 1 : -1)
            << " 1)'>\n";
        out << "<path d='M 0 " << -gr << " A " << gr << " " << gr << " 0 0 1 0 " << gr << " A "
            << rx << " " << gr << " 0 0 " << (k > 0.5 ? 1 : 0) << " 0 " << -gr
            << " Z' fill='#e8ecff'/>\n</g>\n";
    }

    // Legend.
    int h, m;
    hm(chart.nowHour, h, m);
    char now[16];
    std::snprintf(now, sizeof(now), "%02d:%02d", h, m);
    out << "<g fill='#c8d3ea' font-size='13'>\n";
    out << "<text x='96' y='684'>Day " << chart.dayOfYear << " of year " << chart.year
        << " · latitude " << chart.latitudeDeg << "° · " << chart.dayMinutes
        << " real min per day · now " << now << "</text>\n";
    out << "<text x='96' y='704' fill='#ffd9a0'>Sun: rise " << clock(chart.sunriseHour)
        << " · set " << clock(chart.sunsetHour) << " · "
        << (chart.sunsetHour - chart.sunriseHour) << " h daylight · now alt "
        << chart.sunNow.altitudeDeg << "° az " << chart.sunNow.azimuthDeg << "°</text>\n";
    out << "<text x='96' y='724' fill='#dfe5ff'>Moon: " << chart.moonPhase << " · "
        << chart.moonIllumination * 100.0 << " % lit · age " << chart.moonAgeDays
        << " d · rise " << (chart.moonAlwaysUp ? "up all day" : chart.moonNeverUp ? "never" : clock(chart.moonriseHour))
        << " · set " << (chart.moonAlwaysUp || chart.moonNeverUp ? "—" : clock(chart.moonsetHour))
        << " · now alt " << chart.moonNow.altitudeDeg << "° az " << chart.moonNow.azimuthDeg
        << "°</text>\n";
    out << "<text x='96' y='748' fill='#5d6f95' font-size='11'>Same orientation as the street map "
           "(RT_FURNITURE_SVG): east right, south down. Rim = horizon, centre = zenith, rings every "
           "30° of altitude.</text>\n";
    out << "<text x='96' y='764' fill='#5d6f95' font-size='11'>Sun path solid with hourly ticks; "
           "moon path dashed with 2-hourly ticks; hollow body = below the horizon (at its "
           "bearing).</text>\n";
    out << "</g>\n</svg>\n";
    return static_cast<bool>(out);
}

}  // namespace engine
