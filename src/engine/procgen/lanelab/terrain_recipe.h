#ifndef RAYTRACER_ENGINE_PROCGEN_LANELAB_TERRAIN_RECIPE_H
#define RAYTRACER_ENGINE_PROCGEN_LANELAB_TERRAIN_RECIPE_H

// lanelab terrain: a TerrainSpec becomes an engine HeightField (value noise + troughs +
// hills + tilt, or flat, or a stored grid), and a regular grid the conform pass edits.

#include "engine/procgen/lanelab/road_graph_spec.h"
#include "engine/procgen/terrain_field.h"   // HeightField
#include <vector>

namespace engine {
namespace lanelab {

struct HeightGrid {
    double x0 = 0, y0 = 0, res = 1; int nx = 0, ny = 0;
    std::vector<double> z;                              // row-major, ny rows of nx
    double& at(int i, int j) { return z[static_cast<size_t>(j) * nx + i]; }
    double at(int i, int j) const { return z[static_cast<size_t>(j) * nx + i]; }
    double sample(double x, double y) const;            // bilinear, clamped to the grid
    bool inside(double x, double y) const { return x >= x0 && x <= x0 + res * (nx - 1) && y >= y0 && y <= y0 + res * (ny - 1); }
    double cellArea() const { return res * res; }
};

// x is world x, y is world z (lanelab's plan convention); the field returns height.
HeightField makeTerrain(const TerrainSpec& spec, const std::array<double, 4>& bounds);
HeightGrid bakeGrid(const HeightField& h, const TerrainSpec& spec, const std::array<double, 4>& bounds);

}  // namespace lanelab
}  // namespace engine

#endif
