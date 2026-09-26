#ifndef RAYTRACER_ENGINE_PROCGEN_TERRAIN_MAPS_H
#define RAYTRACER_ENGINE_PROCGEN_TERRAIN_MAPS_H

// THE GROUND'S MAPS (ADR-0128): what the weathered ground's shape says about its surface, read off
// the baked grid once -- where water gathers, where rock falls, where soil can lie -- so the ground
// cover (procgen/ground_cover.h), the rocks and the forests are placed by the terrain's processes
// instead of by height bands alone.
//   wet    -- drainage: log of the upslope area (0 on a crest, 1 on a river of ~0.3 km^2 and more)
//   scree  -- talus: moderate slopes lying below cliffs (within ~70 m of ground steeper than 40 deg)
//   soil   -- how much loose soil can stay: gentle and concave ground holds it, steep and convex sheds it
//   convex -- curvature at ~15 m, -1 (gully, hollow) .. +1 (ridge, spur)
// Stored at half the grid's resolution (10 m on the island), 8 bits each; cached with the ground.

#include "erosion.h"   // Heightmap

#include <nlohmann/json.hpp>

#include <cstdint>
#include <memory>
#include <vector>

namespace engine {

struct TerrainMapSample {
    double wet = 0.0, scree = 0.0, soil = 1.0, convex = 0.0;
};

struct TerrainMaps {
    int n = 0;
    float worldSize = 0.0f;
    std::vector<uint8_t> wet, scree, soil, convex;   // n*n, row-major from (-size/2, -size/2)
    TerrainMapSample at(double worldX, double worldZ) const;   // bilinear
};

// Compute the maps from a ground grid (seaLevel: cells below it are sea and read wet 0, soil 1).
TerrainMaps computeTerrainMaps(const Heightmap& hm, double seaLevel);

// The weathered ground's maps for a terrain block with a "weather" block (nullptr otherwise), cached
// on disk next to the ground and in memory per block.
std::shared_ptr<const TerrainMaps> weatheredMapsFor(const nlohmann::json& terrainBlock);

}  // namespace engine

#endif
