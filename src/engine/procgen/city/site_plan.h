#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_SITE_PLAN_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_SITE_PLAN_H

#include "lot_program.h"
#include "shape_ops.h"
#include <cstdint>
#include <vector>

namespace engine {

// The site layer (docs/lot-system-plan.md §8.2-8.3, layer L3) — designing the
// WHOLE lot, not just the building on it. This is the heart of the brief:
// landscaping, paths, gardens, fences, and gates that exist because the
// geometry demands one.
//
// The structural idea is that allocation is ORDERED AND SUBTRACTIVE. Each step
// takes what it needs out of the remaining free region, so two things can never
// occupy the same ground — the brief's "make sure they all have their place and
// don't collide over one another", solved by construction rather than by
// rejection sampling.

enum class Zone : std::uint8_t {
    Building,      // the envelope the plan grammar may play in
    Frontage,      // between the street and the building: garden, forecourt, plaza
    Circulation,   // paths and driveways
    Open,          // lawn, garden, court, park
    Parking,
    Service,       // bins, loading
};

const char* zoneName(Zone z);

// A zone is a LIST of regions, not one. This is not a convenience: a
// circulation zone legitimately has a front path AND a separate rear driveway,
// and an open zone can be two garden strips either side of a building. Storing
// one region forces the allocator to throw the others away, which silently
// deletes the path a lot needs to function.
struct ZoneArea {
    Zone zone = Zone::Open;
    std::vector<Shape2> parts;

    Real totalArea() const;
    bool contains(const Vec2& p) const;
};

// A boundary line along a lot edge: what runs there, and where the gaps are.
enum class BoundaryKind : std::uint8_t { None, Hedge, Fence, Wall, Railing };

struct BoundaryRun {
    Vec2 a, b;
    BoundaryKind kind = BoundaryKind::Fence;
    Real height = 1.2;
    EdgeTag onEdge = EdgeTag::None;
};

// A GATE, emitted where a Circulation zone crosses a boundary run. It is placed
// because the geometry demands one — a path has to get through the fence — not
// because a die said so. This is the object the brief insists must be real:
// it carries its own hinge axis and swing so the interactivity layer can build
// a physics constraint from it without re-deriving anything.
struct GateSpec {
    Vec2 centre;
    Vec2 axis;              // the hinge post, normalized along the boundary
    Real width = 1.2;
    Real height = 1.3;
    Real swing = 1.6;       // radians of travel
    bool hingeLeft = true;
    BoundaryKind style = BoundaryKind::Fence;
};

// Where the lot meets the street — the start of every path.
struct AccessPoint {
    Vec2 at;
    Vec2 inward;            // into the lot
    int edge = -1;
};

struct SitePlan {
    Shape2 lot;
    std::vector<ZoneArea> zones;      // disjoint by construction
    Shape2 free;                      // the unallocated remainder
    std::vector<BoundaryRun> boundaries;
    std::vector<GateSpec> gates;
    std::vector<AccessPoint> access;
    Shape2 buildEnvelope;             // NOT the building: the box the plan may fill
    Vec2 entrance;                    // where the path meets the building
    bool hasEntrance = false;

    const ZoneArea* zoneOf(Zone z) const;
    Real zoneArea(Zone z) const;
    Real allocatedArea() const;
};

// Lay out a lot for a program. Deterministic for `seed`.
//
// The order is the design: access, setbacks, building envelope, circulation,
// then frontage/open/parking/service filling what is left, then boundaries and
// the gates the paths demand.
SitePlan layoutSite(const Shape2& lot, const LotTags& tags,
                    const LotProgram& program, std::uint32_t seed);

// --- furnishing ------------------------------------------------------------

enum class PropKind : std::uint8_t {
    Tree, Shrub, Hedge, Planter, Bench, Bin, Lamp, Bollard, Car, Sign, Table,
};

const char* propKindName(PropKind k);

struct PropInstance {
    PropKind kind = PropKind::Shrub;
    Vec2 at;
    Real radius = 0.5;      // footprint, used for spacing and exclusion
    Real yaw = 0;
    Real scale = 1.0;
};

// Scatter props into a zone with Poisson-disk dart throwing against the zone's
// FREE region — each placed prop subtracts its own footprint, so props cannot
// overlap each other any more than zones can overlap each other. The same
// invariant, one level down.
std::vector<PropInstance> furnishZone(const ZoneArea& zone,
                                      const LotProgram& program,
                                      std::uint32_t seed);

// Everything a laid-out site produces, ready for the mesher.
struct SiteFurnishing {
    std::vector<PropInstance> props;
    std::vector<BoundaryRun> boundaries;
    std::vector<GateSpec> gates;
};

SiteFurnishing furnishSite(const SitePlan& site, const LotProgram& program,
                           std::uint32_t seed);

}  // namespace engine

#endif
