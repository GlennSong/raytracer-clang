#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_CITY_LOTS_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_CITY_LOTS_H

#include "polygon.h"        // Poly2, Vec2
#include "../../../rt_math.h"   // Vec3
#include "../../../renderer/renderer.h"   // RenderMesh (the grown building geometry)
#include <cstdint>
#include <string>
#include <vector>

namespace engine {

// Grow BUILDINGS on the blocks a road network encloses (Living City, ADR-0066).
// This is the "real roads → city blocks → lots → buildings" pass: take the block
// faces (extractBlocks), parcel each block's interior into lots (subdivideBlock),
// and place one typed box building — set back from its lot lines — in each viable
// lot. Each building carries a place-type TAG (a string, so this stays engine-core
// with no citysim dependency) that the caller turns into a Place agents route to.
//
// Pure geometry — no ECS, no rendering — so it is unit-tested headless and stays
// deterministic (ADR-0002). The host (level_loader) spawns each LotBuilding as a
// box entity + collider and registers it as a place for the agents' schedules.

struct LotBuilding {
    Vec2 site;              // footprint centroid (world XZ)
    Real width = 0;         // OBB extent along the lot's long axis (m) — collider
    Real depth = 0;         // OBB extent along the short axis (m) — collider
    Real height = 0;        // building height (m); a park is a low green pad
    Real yaw = 0;           // OBB rotation about +Y (rad) — collider orientation
    std::string type;       // "home" | "shop" | "office" | "civic" | "park"
    Vec3 color{0.72, 0.70, 0.64};
    // The grown building geometry, already world-space and vertex-coloured (a real
    // shape-grammar mass — floors, windows, roof — that FITS this lot), ready to
    // upload. Empty only for a park (a low green pad the caller draws as a box).
    RenderMesh mesh;
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

// The intermediate planning geometry, exposed for debug visualization: the block
// interiors the parcel pass actually subdivided (post road-margin inset) and every
// lot it produced (built or not) — so "blocks → lots → buildings" can be SEEN.
struct LotPlanDebug {
    std::vector<Poly2> blocks;   // buildable block interiors (inset from the roads)
    std::vector<Poly2> lots;     // every parcelled lot
};

// One building per viable lot across every block. Deterministic in seed.
// `debug`, when non-null, receives the intermediate blocks + lots.
std::vector<LotBuilding> growLotBuildings(const std::vector<Poly2>& blocks,
                                          const LotParams& params,
                                          LotPlanDebug* debug = nullptr);

// EDGE blocks (ADR-0066, device feedback): only fully ENCLOSED faces become city
// blocks, which leaves the town rim bare. Synthesize rectangular blocks on the
// OPEN side of boundary roads: walk each road chain between junctions, subdivide
// it into [minLen, maxLen] pieces, and where a side faces open ground (inside no
// closed block, clear of other roads) emit a `depth`-wide rectangle set back by
// `margin`. The rectangles feed growLotBuildings like any other block. Pure
// geometry, deterministic.
struct EdgeBlockParams {
    Real margin = 9.0;     // setback from the road centreline to the block edge
    Real depth = 34.0;     // block width, outward from the road (m)
    Real minLen = 26.0;    // shortest block along the road (m)
    Real maxLen = 58.0;    // longest block along the road (m)
};
struct RoadGraph;   // road_network.h (forward-declared to keep this header light)
std::vector<Poly2> edgeBlocks(const RoadGraph& roads,
                              const std::vector<Poly2>& closedBlocks,
                              const EdgeBlockParams& params);

}  // namespace engine

#endif
