#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_GROUND_GRID_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_GROUND_GRID_H

#include <vector>

// A baked patch of ground: ABSOLUTE world heights on a regular grid, the form a
// road builder hands back when it makes the ground rather than editing it
// (ADR-0089). `z` is row-major, `ny` rows of `nx`, spaced `res` metres from
// (x0, y0) — y being world Z, the plan convention the lane builder uses
// throughout. Nothing here knows about roads: by the time a grid exists the cut
// and fill are already IN it, so a reader just samples.
//
// This is the lane builder's own `HeightGrid`, lifted one directory up so the
// builder interface can name it without depending on the lanes sources (which a
// build without the Clipper2/CDT submodules does not compile).

namespace engine::roads {

struct GroundGrid {
    double x0 = 0, y0 = 0, res = 1;
    int nx = 0, ny = 0;
    std::vector<double> z;                              // row-major, ny rows of nx
    double& at(int i, int j) { return z[static_cast<std::size_t>(j) * nx + i]; }
    double at(int i, int j) const { return z[static_cast<std::size_t>(j) * nx + i]; }
    double sample(double x, double y) const;            // bilinear, clamped to the grid
    bool inside(double x, double y) const {
        return x >= x0 && x <= x0 + res * (nx - 1) && y >= y0 && y <= y0 + res * (ny - 1);
    }
    double cellArea() const { return res * res; }
    bool empty() const { return z.empty() || nx < 2 || ny < 2; }
};

}  // namespace engine::roads

#endif
