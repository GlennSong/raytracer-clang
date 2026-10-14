#ifndef ENGINE_PROCGEN_CITY_LADDERS_H
#define ENGINE_PROCGEN_CITY_LADDERS_H

// ROOF-ACCESS LADDERS (Glenn, 2026-10-13: "auxiliary access ways to roofs"): which buildings carry one, where, and its
// mesh. Pure in the building's record (plan, params, base, height), so it needs nothing baked: the LadderSystem grows
// the list when a level loads and draws the ones near the player.

#include "../../ladder.h"
#include "polygon.h"
#include "shape_grammar.h"

namespace engine {

// A building's roof ladder, if it carries one: low (one to four storeys over the ground floor) flat-roofed
// buildings -- not a glass tower, a big box, a house with a pitched roof, a campus hall, nor one with a fire escape
// (one way up the back, not two) -- two in three of them,
// on the rear face a little in from its end, a caged ladder from the ground to over the parapet. `roofY` is the
// roof's surface (world y). False: none.
bool roofLadderFor(const Poly2& plan, const BuildingParams& params, Real baseY, Real roofY, LadderSpec& out);

// Its mesh, world space: two rails, a rung every 0.3 m, brackets to the wall, the rails bent over the parapet onto
// the roof, and (cage) hoops round the climb above head height.
RenderMesh ladderMesh(const LadderSpec& l, const Vec3& colour = Vec3(0.55, 0.56, 0.58));

}  // namespace engine

#endif
