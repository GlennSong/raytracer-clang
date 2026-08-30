#ifndef RAYTRACER_ENGINE_PROCGEN_EARTHWORK_H
#define RAYTRACER_ENGINE_PROCGEN_EARTHWORK_H

#include "terrain.h"   // TerrainFlatten, EarthworkParams

#include <functional>
#include <memory>
#include <vector>

namespace engine {

// THE EARTHWORK DISPLACEMENT FIELD (terrain-earthwork plan, Phase 2).
//
// The road network's vertical alignment is solved jointly (weldChainProfiles);
// the terrain then has to CARRY those roads. Before this, every road was a
// stamp that drove the ground to a plane inside its footprint and feathered
// back to natural ground over a FIXED 8 m — so wherever the road sat 8 m off
// the hillside, an 8 m bank appeared beside it, and a block between two such
// roads was a staircase of flat terraces with vertical risers (Glenn: "I don't
// like the harsh stair steps ... the terrain and the road network need to be
// aware of each other").
//
// This is the awareness: ONE smooth displacement D(x,z) over the city such
// that natural + D equals the carved road plane at every road, decays to
// nothing far from any road, and is as smooth as possible in between — a
// screened Poisson field, D - reach^2 * laplacian(D) = 0, with the road cells
// as Dirichlet data. It is ADDED to the natural ground (not a target plane), so
// noise and erosion detail inside it survive: a raised hill is still a hill, a
// lowered valley keeps its shape, and a harmonic field has no steps by
// construction. `reach` is Glenn's "gradient falloff": how far from a road the
// land is reshaped (~100 m = one block: the ground between two streets becomes
// one slope; hills further out keep their natural height).
//
// Installed as a BASE LAYER (TerrainParams::earthwork, the erodedBase pattern):
// terrainHeight = base + D, then the flatten stamps apply on top as residual
// corrections of centimetres instead of metres. Everything reads
// terrainHeight, so CDLOD, colliders, lots, pads, walls and the census all see
// it. The road SOLVE must never read it (parity: the mesher drapes over the
// natural ground and recomputes the carve's profile), which is why the loader
// installs it into the params the city is built on and not into the shared
// natural sampler.

struct EarthworkStats {
    int    cells = 0;        // finest-grid nodes
    int    fixed = 0;        // Dirichlet nodes (road-covered + sea + boundary)
    double maxAbsD = 0.0;    // metres
    double maxAbsDX = 0.0, maxAbsDZ = 0.0;
    double residual = 0.0;   // max |update| on the last finest sweep
    double cell = 0.0;       // finest cell actually used (m)
    double minX = 0.0, minZ = 0.0, maxX = 0.0, maxZ = 0.0;
};

// Build the field from the ROAD regions (priority kRoadFlattenPriority — the
// carve rectangles and junction discs the loader assembled) measured against
// the NATURAL ground. Returns null when disabled or when there is nothing to
// fit. Deterministic: fixed sweep counts, fixed row-major order, one thread.
std::shared_ptr<const std::function<double(double, double)>> buildEarthworkField(
    const std::vector<TerrainFlatten>& roadRegions,
    const std::function<double(double, double)>& natural,
    const EarthworkParams& params, double seaLevel,
    EarthworkStats* statsOut = nullptr);

}  // namespace engine

#endif
