#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_TERRAIN_RECIPE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROADS_LANES_TERRAIN_RECIPE_H

// lanelab terrain: a TerrainSpec becomes an engine HeightField (value noise + troughs +
// hills + tilt, or flat, or a stored grid), and a regular grid the conform pass edits.

#include "engine/procgen/city/roads/lanes/road_graph_spec.h"
#include "engine/procgen/city/roads/ground_grid.h"   // GroundGrid (= HeightGrid)
#include "engine/procgen/terrain_field.h"   // HeightField
#include <vector>

namespace engine {
namespace roads::lanes {

// The lane builder's ground grid IS the roads module's (ADR-0089): one type, so a
// builder can hand it back through GroundPlan without the interface depending on
// the lanes sources.
using HeightGrid = engine::roads::GroundGrid;

// x is world x, y is world z (lanelab's plan convention); the field returns height.
HeightField makeTerrain(const TerrainSpec& spec, const std::array<double, 4>& bounds);
HeightGrid bakeGrid(const HeightField& h, const TerrainSpec& spec, const std::array<double, 4>& bounds);

}  // namespace roads::lanes
}  // namespace engine

#endif
