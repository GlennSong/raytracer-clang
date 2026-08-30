#ifndef RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_COLLIDER_H
#define RAYTRACER_ENGINE_PROCGEN_CITY_BUILDING_COLLIDER_H

#include "city_lots.h"   // DoorSpec, Poly2, Vec2/Vec3/Real
#include <cstdint>
#include <vector>

namespace engine {

// The building COLLIDER (ADR-0080): every non-park lot building collides as
// its grown plan polygon extruded to a prism — side walls split around each
// door aperture, a roof cap, and a FLOOR CAP at `floorY` (the drawn ground
// slab height, baseY + 0.05) so whoever is inside stands on the floor they
// SEE, not on the terrain pad half a metre below it. When the plinth is tall
// (> 0.3 m) each door also gets a half-height threshold step outside — the
// 0.5 m curb from pad to floor is legal for the character's 0.55 stepHeight
// but tight on a slope; the tread makes it two easy steps.
//
// Extracted from level_loader's inline prism emission so a unit test can walk
// a character through the notch (tests/test_building_collider.cpp). Pure
// geometry on plain vertex/index arrays. Triangles are emitted SINGLE-sided;
// grown plans arrive with mixed winding, so call mirrorTriangles() ONCE per
// finished collider to double every triangle (the loader's long-standing
// "shoot through them at certain angles" fix).
//
// Door spans are found by projecting each DoorSpec::foot onto the plan's
// edges (nearest within 0.3 m) — NEVER by edge index: growPlanBuilding
// ensureCCWs its own copy of the plan, so indices do not map back.
void appendBuildingPrism(std::vector<Vec3>& vertices,
                         std::vector<uint32_t>& indices, const Poly2& plan,
                         Real base, Real top, Real floorY,
                         const std::vector<DoorSpec>& doors, Real plinth);

// Append the reversed copy of every triangle currently in `indices`.
void mirrorTriangles(std::vector<uint32_t>& indices);

}  // namespace engine

#endif
