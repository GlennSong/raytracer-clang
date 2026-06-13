#ifndef RAYTRACER_ENGINE_SCATTER_H
#define RAYTRACER_ENGINE_SCATTER_H

#include "../rt_math.h"   // Vec3
#include "terrain.h"
#include "noise.h"
#include <cstdint>
#include <vector>

namespace engine {

// Scatter parameters (ROADMAP 4 Phase B.4). The generator attempts `count`
// candidate placements over the region and keeps those that pass the rules —
// "in ways that make sense": not on cliffs, within an altitude band, gated by a
// density mask so instances clump rather than spread uniformly.
struct ScatterParams {
    float regionSize = 200.0f;       // square region centered on the origin (XZ)
    int   count = 1000;              // candidate placements attempted
    float minScale = 0.8f;
    float maxScale = 1.2f;
    float maxSlopeDeg = 30.0f;       // reject ground steeper than this
    float minHeight = -1e9f;         // keep only within this world-Y band
    float maxHeight = 1e9f;
    double densityScale = 0.03;      // frequency of the density-mask noise
    float densityThreshold = 0.0f;   // keep where density noise > threshold
    uint32_t seed = 0;
};

// One scattered instance, on the terrain surface. Feeds an InstanceGroup for
// instanced rendering (the Frame value type of ADR-0021).
struct Placement {
    Vec3 position;   // on the surface (y = terrain height)
    float yaw;       // radians, random about +Y
    float scale;     // uniform
};

// Scatter over a terrain surface with seeded density / slope / altitude rules.
// Deterministic for a seed; the RNG draw order is fixed by `count`, independent
// of which candidates are accepted, so results are stable.
std::vector<Placement> scatterOnTerrain(const ScatterParams& params,
                                        const TerrainParams& terrain,
                                        const Noise& terrainNoise);

}  // namespace engine

#endif
