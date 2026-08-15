#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PARCEL_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PARCEL_H

#include "polygon.h"
#include <cstdint>
#include <vector>

namespace engine {

// Parcel (lot) subdivision of a city block (ADR-0038 §3 / city-plan §3.3). A
// block footprint is parcelled FRONTAGE-FIRST: walk the block polygon's actual
// edges and lay a ring of frontWidth x lotDepth lots along each one (mitered
// at corners, clipped against neighbours), with the interior remainder as one
// court — so every lot fronts a street BY CONSTRUCTION on any simple polygon.
// Recursive oriented-bounding-box bisection remains only as the fallback for
// blocks the boundary walk can't parcel. Each lot keeps a frontage direction
// (outward, toward the surrounding street) so the building can face the road.

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
    // Frontage-first parceling (boundary-walk ring): every lot faces a street
    // BY CONSTRUCTION. The parceler walks the block polygon's actual edges and
    // lays ~`frontWidth`-wide, `lotDepth`-deep lots along each one (mitered at
    // corners, depth-clamped where the block is shallow); the interior
    // remainder becomes ONE court lot. Works on any simple polygon; a block
    // whose walk yields nothing falls back to OBB bisection.
    Real frontWidth = 16;   // target lot width along the street (m)
    Real lotDepth   = 28;   // lot depth inward from the street (m)
    // DEAD FIELD — nothing reads it. `parcelFrontage` emits the interior remainder
    // as a court whenever its area >= 40 m² (a hard-coded literal), so block cores
    // never render as bare dirt regardless of what this says.
    //
    // It is not merely unused, it is a FALSE KNOB: the level schema plumbs
    // `citysim.parcel.courtMinArea` through LevelParams -> LotParams::
    // parcelCourtMinArea -> here (city_lots.cpp), so an author can set it, see it
    // arrive, and get no effect. Either wire it into the 40 m² gate or delete the
    // whole chain — don't leave it half-connected.
    Real courtMinArea = 400;
    uint32_t seed = 0;
};

// Subdivide a (CCW, simple) block footprint into lots. Deterministic for the seed.
std::vector<Lot> subdivideBlock(const Poly2& block, const ParcelParams& params,
                                int district = 0);

}  // namespace engine

#endif
