#ifndef RAYTRACER_ENGINE_SCATTER_H
#define RAYTRACER_ENGINE_SCATTER_H

#include "../../rt_math.h"   // Vec3
#include "terrain.h"
#include "noise.h"
#include <cstdint>
#include <functional>
#include <vector>

namespace engine {

// Scatter parameters (ROADMAP 4 Phase B.4). The generator attempts `count`
// candidate placements over the region and keeps those that pass the rules —
// "in ways that make sense": not on cliffs, within an altitude band, gated by a
// density mask so instances clump rather than spread uniformly.
struct ScatterParams {
    float regionSize = 200.0f;       // square region centered on the origin (XZ)
    int   count = 1000;              // candidate placements attempted
    // Flatten dilation used when sampling the ground — pass the terrain
    // mesher's LEAF dilate (step*1.45) so a placement sees the same surface
    // the rendered/collidable mesh shows. Sampling with 0 while the mesh
    // dilates is why trees floated or sank near every flatten edge.
    double placeDilate = 0.0;
    float minScale = 0.8f;
    float maxScale = 1.2f;
    float maxSlopeDeg = 30.0f;       // reject ground steeper than this
    float minHeight = -1e9f;         // keep only within this world-Y band
    float maxHeight = 1e9f;
    double densityScale = 0.03;      // frequency of the density-mask noise
    float densityThreshold = 0.0f;   // keep where density noise > threshold
    float minSpacing = 0.0f;         // reject candidates within this distance of
                                     // an accepted one (0 = off). Set to the
                                     // instance footprint so big meshes (trees)
                                     // don't jumble. Dart-throwing Poisson disk.
    // Focal point (XZ): make a clearing around a hero and step instance size down
    // with distance from it, so the hero reads as the focus.
    Vec3  focus{0, 0, 0};
    float focusRadius = 0.0f;        // distance over which scale fades to base (0 = off)
    float focusScale = 1.0f;         // scale multiplier at the focus, lerped to 1 at focusRadius
    float focusClear = 0.0f;         // reject placements within this distance of the focus
    // Clustering (Thomas point process): instead of uniform candidates, draw
    // `clusterCount` parent points and scatter candidates in Gaussian clumps of
    // std-dev `clusterRadius` around them — natural groves and clearings rather
    // than even spacing. 0 = uniform (off).
    int   clusterCount = 0;
    float clusterRadius = 6.0f;
    // Exclusion predicate (world XZ -> keep out): return true to reject a
    // candidate. The loader wires this to the terrain's flatten footprints
    // (roads / building pads / graded lots), so vegetation fills a level right
    // up to — but never onto — anything the city graded. Null = off.
    std::function<bool(double, double)> exclude;
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

// Assign each placement a species (deterministic by seed) and bake it to a world
// matrix (position + yaw + uniform scale), returning one matrix list per species
// index. Feeds InstanceGroup creation: thousands of placements collapse to a few
// instanced batches instead of one entity each. No placements are lost.
std::vector<std::vector<Mat4>> bucketPlacementsBySpecies(
    const std::vector<Placement>& placements, std::size_t numSpecies, uint32_t seed);

}  // namespace engine

#endif
