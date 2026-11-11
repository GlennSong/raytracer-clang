#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_POLYLINE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_POLYLINE_H

// THE SEGMENT AND POLYLINE BASICS, once (Glenn, 2026-11-03: "we should put them into a shared utility file"): the
// point of a segment nearest a point, the distance to it, where two lines cross, and the nearest point of a polyline.
// Every one of these had been written again in the file that needed it -- a dozen copies across the world planner,
// the street planner, the road builder, the nav graph, the drawn road and the floor planners -- each with the same
// arithmetic and its own name -- and Douglas-Peucker seven times. Add what a caller needs here; never a private copy.
//
// A degenerate segment (squared length <= 1e-12, or the caller's `minL2`) is its first point: its fraction is 0.

#include "polygon.h"   // Vec2, dot, cross

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace engine {

// Where along a->b the point nearest p lies: the fraction, clamped to the segment. `minL2`: the squared length a
// segment must pass not to count as a point (the road twin measures down to a nanometre: its polylines carry them).
inline Real segmentParam(const Vec2& p, const Vec2& a, const Vec2& b, Real minL2 = 1e-12) {
    const Vec2 ab = b - a;
    const Real L2 = ab.lengthSquared();
    return L2 > minL2 ? std::clamp(dot(p - a, ab) / L2, Real(0), Real(1)) : Real(0);
}

// The point of a->b nearest p (and, `t`, its fraction along it).
inline Vec2 closestOnSegment(const Vec2& p, const Vec2& a, const Vec2& b, Real* t = nullptr, Real minL2 = 1e-12) {
    const Real u = segmentParam(p, a, b, minL2);
    if (t) *t = u;
    return a + (b - a) * u;
}

inline Real distToSegment(const Vec2& p, const Vec2& a, const Vec2& b) { return (p - closestOnSegment(p, a, b)).length(); }

// Where the LINES through a->b and c->d cross: `t` along a->b and `u` along c->d (0 and 1 the ends); false when they
// are parallel. Which ends count is the caller's: a road through a vertex counted once, a crossing strictly inside.
inline bool lineCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, Real& t, Real& u) {
    const Vec2 r = b - a, s = d - c;
    const Real den = cross(r, s);
    if (std::fabs(den) < 1e-12) return false;
    const Vec2 ac = c - a;
    t = cross(ac, s) / den;
    u = cross(ac, r) / den;
    return true;
}

// Do the SEGMENTS a->b and c->d cross (their ends included), and where.
inline bool segmentsCross(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d, Vec2& at) {
    Real t = 0, u = 0;
    if (!lineCross(a, b, c, d, t, u) || t < 0 || t > 1 || u < 0 || u > 1) return false;
    at = a + (b - a) * t;
    return true;
}

// THE NEAREST POINT OF A POLYLINE (closed: a ring, its last point joined to its first). The first nearest segment
// wins a tie.
struct PolylineNearest {
    Real dist = 1e30;       // to it (1e30: no segment)
    Vec2 foot;              // where
    Vec2 dir;               // the way the polyline runs there (unit; the last non-degenerate nearest segment's)
    std::size_t seg = 0;    // its segment: P[seg] -> P[seg + 1]
    Real t = 0;             // the fraction along it
    // (an open polyline) nearest its first point from before it, or its last from beyond it: a road that continues
    // where this one stops, not one beside it
    bool pastEnd = false;
};
inline PolylineNearest nearestOnPolyline(const Vec2& p, const std::vector<Vec2>& P, bool closed = false) {
    PolylineNearest best;
    const std::size_t n = P.size();
    const std::size_t segs = closed ? n : (n ? n - 1 : 0);
    for (std::size_t i = 0; i < segs; ++i) {
        const Vec2 a = P[i], ab = P[(i + 1) % n] - a;
        const Real L2 = ab.lengthSquared();
        const Real raw = L2 > 1e-12 ? dot(p - a, ab) / L2 : Real(0);
        const Real t = std::clamp(raw, Real(0), Real(1));
        const Vec2 f = a + ab * t;
        const Real d = (p - f).length();
        if (d < best.dist) {
            best.dist = d;
            best.foot = f;
            best.seg = i;
            best.t = t;
            if (L2 > 1e-12) best.dir = ab * (1 / std::sqrt(L2));
            best.pastEnd = !closed && ((i == 0 && raw < 0) || (i + 2 == n && raw > 1));
        }
    }
    return best;
}

inline Real distToPolyline(const Vec2& p, const std::vector<Vec2>& P, bool closed = false) {
    return nearestOnPolyline(p, P, closed).dist;
}

// THE STATIONS of an open polyline: each point's distance along it from the first (0 first, the length last).
inline std::vector<Real> stations(const std::vector<Vec2>& P) {
    std::vector<Real> s(P.size(), 0.0);
    for (std::size_t i = 1; i < P.size(); ++i) s[i] = s[i - 1] + (P[i] - P[i - 1]).length();
    return s;
}

// READING AN OPEN POLYLINE AT A STATION (a distance along it), given as a pointer and a count so a range of one of the
// lane graph's shared arrays serves as well as a whole vector. `s` holds each point's station, rising (stations()).
// A station before the first point reads the first, one past the last reads the last. Each finds its segment by a
// binary search over the stations. The road builder's pointAt / interp / tangentAtStation (lanes/polyline_ops.h) and
// the lane graph's path lookups (LaneGraph::pointAt and the rest) are all these.

// The shortest segment a station divides: a shorter one (two points at one station) is read as this long.
constexpr Real kShortestStationSpan = 1e-12;
// How far each way of a station a polyline's direction is measured over (tangentAtStation's span): the direction is
// the chord from (station - this) to (station + this), each end clamped to the polyline's ends.
constexpr Real kTangentHalfSpan = 0.5;

// Where along the polyline a station falls: the segment (P[seg] -> P[seg + 1]) and the fraction along it. Before the
// start: segment 0 at fraction 0; past the end: the last segment at fraction 1. Fewer than two points: segment 0, 0.
struct StationSpot {
    std::size_t seg = 0;
    Real t = 0;
};
inline StationSpot spotAtStation(const Real* s, std::size_t n, Real station) {
    StationSpot spot;
    if (n < 2 || station <= s[0]) return spot;
    if (station >= s[n - 1]) {
        spot.seg = n - 2;
        spot.t = 1;
        return spot;
    }
    const std::size_t k = static_cast<std::size_t>(std::upper_bound(s, s + n, station) - s);
    spot.seg = k - 1;
    spot.t = (station - s[k - 1]) / std::max(s[k] - s[k - 1], kShortestStationSpan);
    return spot;
}

// The point at a station. No points: the origin.
inline Vec2 pointAtStation(const Vec2* xy, const Real* s, std::size_t n, Real station) {
    if (n == 0) return Vec2();
    if (station <= s[0]) return xy[0];
    if (station >= s[n - 1]) return xy[n - 1];
    const StationSpot spot = spotAtStation(s, n, station);
    return lerp(xy[spot.seg], xy[spot.seg + 1], spot.t);
}

// A value carried at each point (a height, a width), read at a station by straight-line interpolation (numpy's interp,
// clamped). No points: 0.
inline Real valueAtStation(const Real* s, const Real* v, std::size_t n, Real station) {
    if (n == 0) return 0;
    if (station <= s[0]) return v[0];
    if (station >= s[n - 1]) return v[n - 1];
    const StationSpot spot = spotAtStation(s, n, station);
    return v[spot.seg] + (v[spot.seg + 1] - v[spot.seg]) * spot.t;
}

// The CHORD across a station: from the point `halfSpan` before it to the point `halfSpan` after it, each end clamped to
// the polyline's ends; not normalised. Its direction blends the directions of the segments it spans, so it turns
// smoothly through a vertex instead of jumping there. It is zero when the span covers no length (one point, or points
// all in one place): the caller picks its own direction then.
inline Vec2 chordAtStation(const Vec2* xy, const Real* s, std::size_t n, Real station, Real halfSpan = kTangentHalfSpan) {
    if (n == 0) return Vec2();
    const Vec2 a = pointAtStation(xy, s, n, std::max(station - halfSpan, s[0]));
    const Vec2 b = pointAtStation(xy, s, n, std::min(station + halfSpan, s[n - 1]));
    return b - a;
}

// How far p lies off the LINE through a and b (to a itself when they meet).
inline Real distToLine(const Vec2& p, const Vec2& a, const Vec2& b) {
    const Vec2 ab = b - a;
    const Real L = ab.length();
    return L > 1e-9 ? std::fabs(cross(p - a, ab)) / L : (p - a).length();
}

// DOUGLAS-PEUCKER over the run of points first..last (the caller's indices into its own points): marks in `keep`
// every point it keeps (the ends are the caller's to mark). A run's worst point -- the one `off(i, a, b)`, how far
// point i lies off the chord a..b, puts farthest -- is kept when it is off by more than `tol`, and both halves are run
// again. What "off" is, is the caller's: from the line (distToLine), the segment (distToSegment), with its height, in
// tolerances. `split(a, b)`: split a run anyway, at its worst point, or at its middle when none is off (a long run kept
// sampled). The first worst point wins a tie.
template <class Off, class Split>
void douglasPeucker(std::size_t first, std::size_t last, Real tol, Off off, Split split, std::vector<char>& keep) {
    std::vector<std::pair<std::size_t, std::size_t>> runs{{first, last}};
    while (!runs.empty()) {
        const auto [a, b] = runs.back();
        runs.pop_back();
        if (b <= a + 1) continue;
        Real worst = -1;
        std::size_t at = a;
        for (std::size_t i = a + 1; i < b; ++i) {
            const Real d = off(i, a, b);
            if (d > worst) { worst = d; at = i; }
        }
        const bool over = worst > tol;
        if (!over && !split(a, b)) continue;
        if (!over) at = (a + b) / 2;
        keep[at] = 1;
        runs.push_back({a, at});
        runs.push_back({at, b});
    }
}
template <class Off>
void douglasPeucker(std::size_t first, std::size_t last, Real tol, Off off, std::vector<char>& keep) {
    douglasPeucker(first, last, tol, off, [](std::size_t, std::size_t) { return false; }, keep);
}

// The usual: the indices of an open run of points kept within `tol` of the LINE between kept neighbours, its two ends
// always (fewer than two points: all of them).
inline std::vector<std::size_t> simplifyRun(const std::vector<Vec2>& P, Real tol) {
    std::vector<std::size_t> out;
    if (P.size() < 2) { for (std::size_t i = 0; i < P.size(); ++i) out.push_back(i); return out; }
    std::vector<char> keep(P.size(), 0);
    keep.front() = keep.back() = 1;
    douglasPeucker(0, P.size() - 1, tol, [&](std::size_t i, std::size_t a, std::size_t b) { return distToLine(P[i], P[a], P[b]); }, keep);
    for (std::size_t i = 0; i < P.size(); ++i) if (keep[i]) out.push_back(i);
    return out;
}

}  // namespace engine

#endif
