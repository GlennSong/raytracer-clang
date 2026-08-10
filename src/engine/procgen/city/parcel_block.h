#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_PARCEL_BLOCK_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_PARCEL_BLOCK_H

#include "lot_program.h"
#include "shape_ops.h"
#include <cstdint>
#include <vector>

namespace engine {

// Cutting a block into lots, PROGRAM FIRST (docs/lot-system-plan.md §17.1).
//
// This is the other half of the buildability inversion. lot_program.h supplies
// the measurement — can this parcel hold that building? — and this supplies the
// cutting that only ever produces parcels which can.
//
// The old parceller cuts a rhythm and lets the builder reject what will not fit,
// which is why a skinny trapezoid still gets a tiny triangular house: nothing
// upstream ever asked whether a building could stand there. Here the question is
// asked BEFORE the cut is kept:
//
//   * a region is split only while the halves would still hold a program;
//   * a region that can hold no program at all is OPEN SPACE BY DESIGN, handed
//     to the site layer as green rather than counted as a failure;
//   * every emitted lot carries its tags, computed once, at cut time.
//
// There is nothing left to reject downstream, so there are no rejection
// counters — their absence is the point, not an oversight.

struct ParcelParams {
    Real roadMargin = 2.0;     // verge between the carriageway and the first lot
    Real minLotArea = 55.0;    // below this nothing is worth cutting
    Real partyMaxFront = 9.0;  // narrower than this and the side walls are PARTY
    int maxDepth = 7;          // recursion guard
};

struct ParcelledLot {
    Shape2 shape;             // edges tagged Street / Side / Rear / Party
    LotTags tags;
    int program = -1;         // index into the ProgramSet, or -1
};

struct ParcelledBlock {
    Shape2 block;                     // the block as handed in
    Shape2 parcellable;               // after the road margin
    std::vector<ParcelledLot> lots;
    std::vector<Shape2> openSpace;    // residue, open BY DESIGN
    Real lotArea() const;
    Real openArea() const;
};

// Cut one block. `enclosed` false marks a rim block, which admits the
// campus-scale programs (§17.6) purely through their larger minimums.
// Deterministic for `seed`.
ParcelledBlock parcelBlock(const Shape2& block, ProgramSet& programs,
                           const ParcelParams& params, bool enclosed,
                           Real coreness, StreetClass klass, std::uint32_t seed);

// Tag a lot's edges against the region it was cut out of. An edge lying on that
// boundary INHERITS the boundary's tag there — a block edge fronting a road
// makes a lot edge that fronts a road, and a block edge fronting a park does
// not become a street just because it is on the outside.
//
// `reference` must be the PARCELLABLE region (the block after its road margin),
// not the raw block: once the margin is taken, no lot edge lies on the raw
// block boundary at all, every lot reads as having zero frontage, and every
// program becomes ineligible.
//
// The far edge is the rear, the rest are sides — promoted to PARTY on a lot too
// narrow to leave a gap between neighbours.
void tagLotEdges(Shape2& lot, const Shape2& reference, Real partyMaxFront);

}  // namespace engine

#endif
