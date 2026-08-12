#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_ROOF_PLANT_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_ROOF_PLANT_H

#include "polygon.h"   // Poly2, Vec2
#include <cstdint>
#include <functional>
#include <vector>

namespace engine {

// ROOFTOP PLANT — what stands on a roof, and where.
//
// A flat roof is not empty in real life: it carries the stair/lift overrun, the
// water tank, the air handling. Leaving it bare is what makes a building read as
// an extruded box, and it was the single largest thing the Lot System dropped
// when it stopped going through the shipping grammar (audit, docs §18.2).
//
// This is the PLANNER, deliberately separated from the geometry. Deciding what a
// roof carries and arranging it so two units never overlap is one design, and it
// belongs in one place; drawing a bulkhead is a mesher's job, and the two meshers
// build into different structures (BuildingMesh scopes vs the Lot System's part
// accumulator). Sharing the plan and not the emit is what stops this becoming a
// third copy of the arrangement logic — which is the failure the audit found.
//
// The arrangement is the street grid's own trick applied to a roof: recursive
// longest-axis bisection carves the parapet-inset rectangle into cells, and every
// item gets its OWN cell. Rejection sampling put a water tank through an HVAC
// unit often enough to be the reported bug ("partition the rooftop space so the
// water tower, HVAC unit(s) and roof access aren't sitting on top of one
// another").

struct RoofPlantItem {
    enum class Kind : std::uint8_t {
        Access,     // stair / lift overrun: a bulkhead with a capped lid and a door
        WaterTank,  // timber tank on a leg frame
        Hvac,       // louvred air handler with fan discs on top
    };
    Kind kind = Kind::Access;
    // Seat in the caller's own (origin, axisU, axisV) roof frame — the same
    // frame `width`/`depth` are measured in. u/v are the item's near corner.
    Real u = 0, v = 0;
    Real w = 0, d = 0;
};

// Plan the plant for one roof.
//
// `topPlan` (optional, world XZ) is the roof's REAL polygon: an L, a U or a
// courtyard is not its bounding box, and an item seated by box coordinates alone
// can hang over the notch. When given, every item proves all four of its corners
// lie on the polygon before it is placed, and re-jitters within its cell if not.
//
// `frameOrigin` + `axisU`/`axisV` (unit, perpendicular, world XZ) and
// `width`/`depth` describe the roof footprint the cells are cut from.
//
// `floors` and `curtainWall` gate what a roof plausibly carries — a two-storey
// house has no lift overrun, and a sealed glass tower has no timber tank.
//
// Randomness arrives as a `unit01` callback rather than as an Rng, because the
// two callers do not share one: shape_grammar.cpp carries its own copy of the
// generator, and rng.h says in as many words that unifying them would change the
// geometry the shipped city generates and wants its own reviewed commit. A
// callback lets both pass their own stream, and since Rng::range(a,b) is exactly
// `a + (b-a)*unit()`, the call SEQUENCE is preserved — so this refactor moves the
// planner without moving a single building.
std::vector<RoofPlantItem> planRoofPlant(const Poly2* topPlan,
                                         const Vec2& frameOrigin,
                                         const Vec2& axisU, const Vec2& axisV,
                                         Real width, Real depth, int floors,
                                         bool curtainWall,
                                         const std::function<Real()>& unit01);

}  // namespace engine

#endif
