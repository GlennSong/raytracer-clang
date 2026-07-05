#ifndef RAYTRACER_APPS_CITYSIM_CITY_LOTS_H
#define RAYTRACER_APPS_CITYSIM_CITY_LOTS_H

#include "places.h"                                // PlaceType
#include "../../engine/procgen/city/polygon.h"     // Poly2, Vec2
#include "../../rt_math.h"                          // Vec3
#include <cstdint>
#include <vector>

namespace citysim {

using engine::Poly2;
using engine::Real;
using engine::Vec2;
using engine::Vec3;

// Grow BUILDINGS on the blocks of a road network (Living City, ADR-0066). This is
// the "real roads → city blocks → lots → buildings" pass the user described: take
// the block faces the road graph encloses (extractBlocks), parcel each block's
// interior into lots (subdivideBlock), and place one typed box building, set back
// from its lot lines, in each viable lot. A building carries a PlaceType TAG so
// agents can start/end their schedules there (home / shop / office / civic / park).
//
// Pure geometry — no ECS, no rendering — so it is unit-tested headless and stays
// deterministic (ADR-0002). The caller (level_loader) spawns each LotBuilding as a
// box entity + collider and registers it as a Place.

struct LotBuilding {
    Vec2 site;              // footprint centroid (world XZ)
    Real width = 0;         // extent along the lot's long axis (m)
    Real depth = 0;         // extent along the short axis (m)
    Real height = 0;        // building height (m); a park is a low green pad
    Real yaw = 0;           // rotation about +Y so the box aligns to its lot (rad)
    PlaceType type = PlaceType::Home;
    Vec3 color{0.72, 0.70, 0.64};
};

struct LotParams {
    Real roadMargin = 11.0;   // inset from the block edge to the buildable interior
                              // (road half-width + sidewalk) — wider = more sidewalk
    Real lotSetback = 1.4;    // building inset from its own lot lines
    Real minShort = 9.0;      // reject a building whose short side is under this (m)
    Real maxAspect = 3.5;     // ...or whose long/short exceeds this (no knife blades)
    Real minLotArea = 90.0;   // skip tiny leftover lots
    Real buildChance = 0.92;  // per-lot occupancy (rest become plazas/gaps)
    Vec2 center{0, 0};        // downtown centre for the radial zoning
    Real innerRadius = 55.0;  // < this: downtown (offices/shops)
    Real midRadius = 135.0;   // < this: mixed; beyond: residential
    uint32_t seed = 1;
};

// One box building per viable lot across every block. Deterministic in seed.
std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& params);

}  // namespace citysim

#endif
