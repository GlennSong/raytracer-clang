#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_POLYLINE_OPS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_POLYLINE_OPS_H

// Small polyline toolkit shared by lanelab's stages: frames, projection, interpolation,
// and a uniform-grid segment index for the nearest-distance queries the height and conform
// passes make by the hundred thousand.

#include "engine/procgen/city/polygon.h"
#include "engine/procgen/city/polyline.h"   // the segment and polyline basics this toolkit builds on
#include <vector>

namespace engine {
namespace roads::lanes {

// Central-difference tangents (one-sided at the ends) and left normals.
void frames(const std::vector<Vec2>& xy, std::vector<Vec2>& tan, std::vector<Vec2>& nrm);
// (`minLen`: the shortest neighbours' span that is a direction; else +x)
Vec2 tangentAt(const std::vector<Vec2>& xy, size_t i, double minLen = 1e-12);

struct Projection { double station = 0, distance = 0; size_t segment = 0; double t = 0; };
// Nearest point of the polyline to p (brute force).
Projection project(const std::vector<Vec2>& xy, const std::vector<double>& s, const Vec2& p);
Vec2 pointAt(const std::vector<Vec2>& xy, const std::vector<double>& s, double station);
Vec2 tangentAtStation(const std::vector<Vec2>& xy, const std::vector<double>& s, double station);
double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x);   // np.interp, clamped

// THE STRETCH OF A POLYLINE between two stations (clamped to its ends; `to` below `from` reads as `from`): the points at
// both stations, and every vertex between them standing more than `minGap` from either (so a cut a hair from a vertex
// does not leave a sliver segment).
std::vector<Vec2> sliceAtStations(const std::vector<Vec2>& xy, const std::vector<double>& s, double from, double to,
                                  double minGap = 1e-3);
// THE STATION A POINT LIES ACROSS: the station whose normal (perp of tangentAtStation, how the builder offsets a point
// from a spine: a ramp's lane beside its host) passes through p, found from the nearest point's station. Where the
// spine bends at a vertex the nearest point and this differ by up to the offset times half the bend.
double stationAcross(const std::vector<Vec2>& xy, const std::vector<double>& s, const Vec2& p);
// The radius of the circle through three points: infinity when they lie on a line (or two coincide).
double circleRadius(const Vec2& a, const Vec2& b, const Vec2& c);


// Polyline x polyline intersections (interior crossings), brute force with a bounds test.
std::vector<Vec2> crossings(const std::vector<Vec2>& a, const std::vector<Vec2>& b);

// Uniform-grid index over one polyline's segments: nearest distance within a radius.
class SegmentGrid {
public:
    SegmentGrid() = default;
    SegmentGrid(const std::vector<Vec2>& xy, double cell);
    // Distance from p to the polyline if <= radius, else +infinity; optional nearest station.
    // `side` (optional): cross(segment direction, p - a) of the nearest segment (> 0 = p on its left).
    double distanceWithin(const Vec2& p, double radius, double* station = nullptr, double* side = nullptr) const;
private:
    std::vector<Vec2> pts_;
    std::vector<double> s_;
    double cell_ = 8, x0_ = 0, y0_ = 0; int nx_ = 0, ny_ = 0;
    std::vector<std::vector<int>> cells_;
    int cellIndex(int i, int j) const { return j * nx_ + i; }
};

}  // namespace roads::lanes
}  // namespace engine

#endif
