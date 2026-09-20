#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_POLYLINE_OPS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_POLYLINE_OPS_H

// Small polyline toolkit shared by lanelab's stages: frames, projection, interpolation,
// and a uniform-grid segment index for the nearest-distance queries the height and conform
// passes make by the hundred thousand.

#include "engine/procgen/city/polygon.h"
#include <vector>

namespace engine {
namespace roads::lanes {

// Central-difference tangents (one-sided at the ends) and left normals.
void frames(const std::vector<Vec2>& xy, std::vector<Vec2>& tan, std::vector<Vec2>& nrm);
Vec2 tangentAt(const std::vector<Vec2>& xy, size_t i);

struct Projection { double station = 0, distance = 0; size_t segment = 0; double t = 0; };
// Nearest point of the polyline to p (brute force).
Projection project(const std::vector<Vec2>& xy, const std::vector<double>& s, const Vec2& p);
Vec2 pointAt(const std::vector<Vec2>& xy, const std::vector<double>& s, double station);
Vec2 tangentAtStation(const std::vector<Vec2>& xy, const std::vector<double>& s, double station);
double interp(const std::vector<double>& xs, const std::vector<double>& ys, double x);   // np.interp, clamped

// Crossing-number point-in-ring test (ring closed implicitly); boundary points count as inside.
bool pointInRing(const std::vector<Vec2>& ring, const Vec2& p);

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
