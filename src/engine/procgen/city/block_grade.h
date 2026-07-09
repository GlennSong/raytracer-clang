#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_BLOCK_GRADE_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_BLOCK_GRADE_H

#include "polygon.h"          // Poly2, Vec2
#include "buildability.h"     // HeightSampler
#include "../terrain.h"       // TerrainFlatten

#include <vector>

namespace engine {

// Grade a city block down to its bounding streets (ADR-0075 Phase 2, the cascade's
// next layer up from the road corridor). A block interior shouldn't step off the
// raw terrain — it should TILT with the roads that frame it, so the lots and
// building pads inside sit on ground that meets the street. For each block we fit
// a plane to the heights sampled around its boundary (which, in the loader, is the
// ROAD-CARVED ground — so the plane is pinned to the street decks) and emit a
// TerrainFlatten grading the interior to that plane. Building pads then flatten
// LOCALLY against this graded block instead of picking an independent level off
// the raw noise — the grade cascade terrain -> road -> block -> pad.
//
// Where the frontage-to-frontage drop across a block exceeds `maxDrop`, one tilted
// plane would need a wall too tall to hold; such a block is TERRACED into stepped
// sub-pads (each a shorter plane), so the grade break is absorbed by several
// modest steps instead of one cliff — the way a hillside subdivision really steps.
struct BlockGradeParams {
    double falloff  = 4.0;   // edge feather (short — the block meets the graded street)
    double inset    = 1.5;   // sample the boundary this far inside (off the curb/sidewalk)
    int    boundarySamples = 24;  // points around the boundary the plane is fit to
    double maxDrop  = 6.0;   // terrace when the plane's range over the block exceeds this (m)
    double minTerraceArea = 400.0;  // don't terrace a block smaller than this (m^2)
};

// One (usually) or several (terraced) graded footprints per block. `ground` is the
// height the boundary is measured against — pass the road-carved sampler so blocks
// pin to their streets.
std::vector<TerrainFlatten> gradeBlocks(const std::vector<Poly2>& blocks,
                                        const HeightSampler& ground,
                                        const BlockGradeParams& p = {});

// Grade ONE block (exposed for tests / a single-block probe): returns its graded
// footprint(s) — one tilted plane, or several terrace steps when it's too steep.
std::vector<TerrainFlatten> gradeBlock(const Poly2& block, const HeightSampler& ground,
                                       const BlockGradeParams& p = {});

}  // namespace engine

#endif
