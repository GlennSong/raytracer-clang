#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_LOT_PROGRAM_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_LOT_PROGRAM_H

#include "shape_ops.h"
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// Programs, lot tags, and the buildability inversion (docs/lot-system-plan.md
// §8.1, §17.1-17.3).
//
// THE INVERSION. Today the parceller cuts a rhythm and the builder rejects what
// will not fit — six rejection counters, and a skinny trapezoid still gets a
// tiny triangular house, because NOTHING UPSTREAM EVER ASKED whether a building
// could stand there. The fix is to turn the question around: a program declares
// the minimum rectangle its building needs, the parceller only emits lots that
// can hold one, and land that can carry no program becomes open space BY
// DESIGN. There is then nothing left to reject.

// --- lot tags (§17.3) ------------------------------------------------------

// What kind of position the lot occupies in its block. Computed by the
// parceller from edge tagging, never re-derived downstream.
enum class LotShape : uint8_t {
    MidBlock,   // one street frontage
    Corner,     // two streets meet here: corner shops, chamfers, entrances
    Through,    // frontage on opposite sides
    Island,     // street on every side
};

enum class StreetClass : uint8_t { Lane, Street, Arterial };

const char* lotShapeName(LotShape s);

// FACTS COMPUTED ONCE by the parceller and only READ by the architect. This is
// the piece the plan named but never specified, and the reason the architect's
// decision can be a filter rather than a dice roll.
struct LotTags {
    int frontages = 1;
    LotShape shape = LotShape::MidBlock;
    StreetClass streetClass = StreetClass::Street;

    Real area = 0;
    Real inscribedW = 0, inscribedD = 0;   // the largest rectangle that fits
    Real frontWidth = 0;                   // metres of STREET frontage
    // Metres of LANE frontage. Kept separate from frontWidth on purpose: a lane
    // is access (a lot on one is reachable and buildable) but it is not a show
    // frontage, so it must not turn a mid-block lot into a corner shop site.
    Real laneWidth = 0;
    Real maxStoreys = 0;                   // from the plate, see §17.2

    bool enclosed = true;    // false on a rim block -> campus-scale programs
    Real coreness = 0;       // 0 at the edge, 1 downtown: the skyline model
    Real slope = 0;          // radians of the pad's tilt
    int neighbours = 0;      // adjacency, for party walls and terrace runs
};

// HEIGHT IS CAPPED BY THE PLATE, ONCE (§17.2). Lifts, cores and structure scale
// with the storeys they serve, so a hundred-storey tower needs a big floor
// plate — you cannot put one on 200 m2. Today six tower recipes each carry
// their own `coreness * N` guess; this is the single model above the tables.
Real maxStoreysFor(Real shortSide, Real plateArea);

// The largest axis-aligned rectangle inside `shape` when measured in the frame
// of its own oriented bounding box — the same shrink-to-fit construction that
// already seats a house, promoted to a QUALIFYING MEASUREMENT. Returns the
// rectangle's dimensions; `outRect` receives the rectangle itself.
void inscribedRect(const Shape2& shape, Real& w, Real& d,
                   Shape2* outRect = nullptr);

// Measure a parcel into tags. `streetDirs` are outward directions of the lot's
// street-facing edges (the parceller knows them; nothing downstream should have
// to guess).
LotTags measureLot(const Shape2& lot, const std::vector<Vec2>& streetDirs,
                   StreetClass klass, bool enclosed, Real coreness);

// --- programs (§8.1) -------------------------------------------------------

enum class FrontageKind : uint8_t { Direct, Forecourt, Garden, Plaza, Patio, Yard };
enum class OpenKind : uint8_t { None, Lawn, Garden, Court, Park };
enum class ServiceKind : uint8_t { None, Bins, Loading };

// The brief for a lot. Note what MOVED here: setbacks and coverage are program
// properties, not global constants. Today `lotSetback` is one number per
// district; a villa wants 8 m of front garden, a shopfront wants zero, and a
// tower wants a plaza on the corner and nothing at the rear.
struct LotProgram {
    std::string name;

    // THE MINIMUM BUILDABLE RECTANGLE — a property of the BUILDING TYPE, not of
    // the lot. This single pair of numbers is what makes the inversion work.
    Real minW = 8, minD = 8;
    Real minArea = 90;
    Real minFrontage = 6;

    // THE LOT THIS PROGRAM IS BUILT ON, as opposed to the smallest one it will
    // accept. Minimums decide ELIGIBILITY; targets decide GRAIN, and they are
    // not the same question.
    //
    // Without targets a recursive cutter can only halve until it happens to
    // fall under a minimum, so the lot size is a power-of-two artefact of the
    // block's dimensions rather than anything a program asked for — measured
    // before this existed, a 110 m downtown block produced 2100 m² tower
    // plates and a 150 m one produced 1040 m² plates, so the BIGGER block gave
    // the SMALLER lots and whether a tower had room for wings was an accident
    // of where the cascade landed.
    //
    // 0 means "derive from the minimum", which is right for anything whose
    // minimum already is its normal size (a terrace house).
    Real targetW = 0, targetD = 0;
    Real targetWidth() const { return targetW > 0 ? targetW : minW * 1.35; }
    Real targetDepth() const { return targetD > 0 ? targetD : minD * 1.35; }
    Real targetArea() const { return targetWidth() * targetDepth(); }

    Real coverage = 0.45;                  // footprint / lot area
    Real frontSetback = 4, sideSetback = 2, rearSetback = 5;

    FrontageKind frontage = FrontageKind::Garden;
    OpenKind open = OpenKind::Lawn;
    int parkingStalls = 0;
    ServiceKind service = ServiceKind::None;

    int minStoreys = 1, maxStoreys = 3;

    // Eligibility bias. A program can require a corner, a rim block, or a
    // street class — the tags the parceller already computed.
    bool requiresCorner = false;
    bool requiresRim = false;
    StreetClass minStreetClass = StreetClass::Lane;

    // WHICH BUILDINGS BELONG HERE. The plan's `BuildingRecipeRef building`,
    // widened to a list so one program can vary. Without it "a recipe that
    // fits" degenerates into "a recipe whose storey range overlaps", and a
    // residential walkup gets handed an industrial works shed — which is
    // exactly what the first drawn sheet showed. Empty means "any recipe whose
    // storey range fits", which is only ever a fallback.
    std::vector<std::string> recipes;

    // Relative likelihood among eligible survivors, before tag biasing.
    Real weight = 1.0;

    // A QUOTA runs before the weighted pass: at most this many per district
    // (0 = unlimited). Rarity as a quota, not a probability — the fix for
    // "one in fifty buildings is a landmark" producing either none or twelve.
    int quota = 0;
};

// Does this lot satisfy the program's minimums? The whole eligibility test, in
// one place, used identically by the parceller (to decide what to cut) and by
// the architect (to decide what to build).
bool lotFitsProgram(const LotTags& tags, const LotProgram& p);

// --- the architect's decision (§17.3) --------------------------------------

// A district's program mix plus its quota state. Quotas are consumed as they
// are placed, which is what makes "one cathedral per district" mean exactly
// one rather than a probability that yields zero or three.
struct ProgramSet {
    std::vector<LotProgram> programs;
    std::vector<int> placed;               // parallel to `programs`

    void reset();
    // Eligible programs for a lot, with their tag-biased weights. Weights also
    // account for SCALE FIT: a program whose minimums are tiny relative to the
    // land is heavily downweighted, so the smallest entry in a mix cannot
    // dominate large parcels.
    std::vector<std::pair<int, Real>> eligible(const LotTags& tags) const;
    // What this land is BEST for: highest-weighted eligible program, no RNG,
    // no quota consumed. The parceller asks this to decide how finely to cut.
    int bestFor(const LotTags& tags) const;
    // Two-stage filter: quota-first, then a weighted pick among survivors.
    // Returns -1 when nothing is eligible — which is a legitimate answer
    // meaning "this land is open space", not a failure.
    int pick(const LotTags& tags, std::uint32_t seed);
};

// The stock residential/commercial/civic mixes, as data.
// --- what block a district needs -------------------------------------------

// The buildability inversion, one level UP. A block is not a number of metres
// somebody liked; it is however much land the buildings a district intends to
// put there actually need. Two rows of lots back to back plus the verge each
// side IS the street-to-street depth, and a few lots along the frontage is the
// length — so a downtown of tower plates gets deep blocks and a dense
// neighbourhood of terraces gets shallow ones, from the same rule.
//
// This is the number the road layer wants: without it a city lays one grain
// everywhere and the district character has to be faked with street pattern
// alone, which cannot produce a lot big enough for a tower with wings.
struct BlockGrain {
    Real depth = 110;    // street to street
    Real length = 150;   // along the street
};

// Derived from the mix's DRIVING program: the largest ordinary one. Quota'd
// landmarks are excluded on purpose — a cathedral is a one-off that takes
// whatever block it lands on, and sizing every block in the district to it
// would coarsen the whole grid for one building.
BlockGrain blockGrainFor(const ProgramSet& mix);

ProgramSet residentialPrograms();
ProgramSet commercialPrograms();
ProgramSet downtownPrograms();
ProgramSet rimPrograms();          // campus, office park, big box, works

}  // namespace engine

#endif
