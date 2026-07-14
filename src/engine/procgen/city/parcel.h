#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PARCEL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PARCEL_H

#include "polygon.h"
#include <cstdint>
#include <vector>

namespace engine {

// Parcel (lot) subdivision of a city block (ADR-0038 §3 / city-plan §3.3). A
// block footprint is split into building lots by recursive oriented-bounding-box
// bisection along the longer axis until lots fall in a target area band. Each lot
// keeps a frontage direction (outward, toward the surrounding street) so the
// building can face the road.

struct Lot {
    Poly2 footprint;
    Vec2  frontage{0, 1};   // unit outward direction toward the nearest street
    Real  area = 0;
    int   district = 0;     // inherited from the block
    bool  court = false;    // a block-interior court (plaza/park), not buildable —
                            // the frontage parceler emits this for the leftover
                            // core of a deep block (city-pipeline v2 step 10)
};

struct ParcelParams {
    Real targetArea = 420;  // m² — aim for lots near this size
    Real minArea = 110;     // don't split below this
    Real minEdge = 7;       // don't produce lots thinner than this (m)
    Real jitter = 0.18;     // split-position randomization (fraction of extent)
    // Frontage-first parceling (city-pipeline v2 step 10): every lot faces a
    // street. Lots are cut as a ring of `lotDepth`-deep strips inward from the
    // block edges, each ~`frontWidth` wide along the street; the leftover core
    // of a deep block becomes a court. A block too shallow to leave a court
    // falls back to back-to-back bisection (still street-fronting).
    Real frontWidth = 16;   // target lot width along the street (m)
    Real lotDepth   = 28;   // lot depth inward from the street (m)
    Real courtMinArea = 400; // interior smaller than this => shallow block
    uint32_t seed = 0;
};

// Subdivide a (CCW, simple) block footprint into lots. Deterministic for the seed.
std::vector<Lot> subdivideBlock(const Poly2& block, const ParcelParams& params,
                                int district = 0);

}  // namespace engine

#endif
