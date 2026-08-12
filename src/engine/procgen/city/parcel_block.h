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

struct BlockParcelParams {
    // Inset from THE SHAPE HANDED IN to the first lot. Measure it against
    // whatever the caller passes: a block face straight off extractBlocks is a
    // road CENTRELINE, so it needs a half-carriageway + sidewalk + verge (~11 m,
    // LotParams::roadMargin); an already-inset buildable interior needs only the
    // verge, which is what this default is. Reading it as "the verge" and
    // handing in a centreline face puts buildings in the road — that shipped.
    Real roadMargin = 2.0;
    Real minLotArea = 55.0;    // below this nothing is worth cutting
    Real partyMaxFront = 9.0;  // narrower than this and the side walls are PARTY
    int maxDepth = 7;          // recursion guard

    // SERVICE LANES. A block deeper than about two lots cannot be served from
    // its perimeter alone: a frontage-first parceller rings the outside and
    // leaves the middle landlocked, which reads as a vast green nobody can
    // reach. Real cities answer this with a mews or a back lane, so a block too
    // deep for its programs gets one cut through it — which also makes the
    // resulting sub-blocks more rectangular.
    bool lanes = true;
    Real laneWidth = 6.0;
    // A perimeter ring of lots consumes roughly this multiple of a program's
    // minimum depth on each side. What is left over in the middle is the
    // STRANDED CORE, and a lane is worth cutting only when that core is big
    // enough to be a problem — asking "is the block deep?" instead cuts lanes
    // into blocks that a perimeter ring already serves perfectly well.
    Real ringDepthFactor = 1.6;
    Real strandedCoreDepth = 26.0;   // metres of core before a lane is justified
    int maxLanes = 2;                // recursion depth: at most 3 corridors

    // PASSAGES. Whatever green survives should still be reachable. A shared
    // green above this size gets a pedestrian passage cut to the nearest street
    // or lane; below it, the green is a private back garden reached through the
    // house, which needs no passage of its own.
    Real sharedGreenMinArea = 260.0;
    Real passageWidth = 2.6;
};

// A lane or passage cut through a block. `paved` distinguishes a service lane
// (hard surface, vehicles) from a footpath into a green (soft, pedestrian).
struct BlockLane {
    Shape2 area;               // the corridor itself
    Vec2 from, to;             // its centreline endpoints
    Real width = 6.0;
    bool paved = true;
    bool passage = false;      // true = pedestrian spur into a green
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
    std::vector<BlockLane> lanes;     // service lanes + pedestrian passages
    Real lotArea() const;
    Real openArea() const;
    Real laneArea() const;
    // Is every SHARED open region reachable? True when each one touches a
    // street, a lane or a passage — the invariant the lanes exist to guarantee.
    // Greens below `minArea` are exempt: a small back garden is reached through
    // the house that owns it and needs no passage of its own.
    bool openSpaceIsReachable(Real minArea = 200.0, Real tol = 1.0) const;
};

// Cut one block. `enclosed` false marks a rim block, which admits the
// campus-scale programs (§17.6) purely through their larger minimums.
// Deterministic for `seed`.
ParcelledBlock parcelBlock(const Shape2& block, ProgramSet& programs,
                           const BlockParcelParams& params, bool enclosed,
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

// TERRACE THE ENVELOPE. A long buildable envelope is not a long building — a
// 60 m street frontage is a ROW of houses everywhere in the world, not one
// 60 m house. Cut it into units of roughly `unitWidth` across its long axis and
// let the architect build each one, so the party walls the lot tagger already
// talks about (`partyMaxFront`, LotTags::neighbours) finally have masses either
// side of them.
//
// `unitWidth` is the program's OWN declared unit size (LotProgram::targetWidth),
// not a constant: a cottage row and an office plate terrace at their own grain
// from the same rule, which is the difference between a heuristic and a tuned
// number. Returns the envelope unchanged when it is already about one unit
// wide, so the common case costs nothing.
std::vector<Shape2> terraceEnvelope(const Shape2& envelope, Real unitWidth);

}  // namespace engine

#endif
