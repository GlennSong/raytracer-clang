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
};

struct ParcelParams {
    Real targetArea = 420;  // m² — aim for lots near this size
    Real minArea = 110;     // don't split below this
    Real minEdge = 7;       // don't produce lots thinner than this (m)
    Real jitter = 0.18;     // split-position randomization (fraction of extent)
    uint32_t seed = 0;
};

// Subdivide a (CCW, simple) block footprint into lots. Deterministic for the seed.
std::vector<Lot> subdivideBlock(const Poly2& block, const ParcelParams& params,
                                int district = 0);

}  // namespace engine

#endif
