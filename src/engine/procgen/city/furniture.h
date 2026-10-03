#ifndef ENGINE_PROCGEN_CITY_FURNITURE_H
#define ENGINE_PROCGEN_CITY_FURNITURE_H

// FURNISHING ROOMS (buildings M4, M4b; Glenn, 2026-09-30: "legit build furniture"). Every room the room planner
// names gets the kit's pieces (procgen/furniture_kit.h) its use implies, set against its walls in the room's own
// frame:
//
//   Office   a desk with its office chair and monitor, a filing cabinet; a second desk in a wide room
//   Flat     a bed between nightstands, a sofa with its coffee table (the ring's apartments are studios)
//   Living   a sofa and coffee table facing the TV unit, a lounge chair where it fits
//   Kitchen  a counter run of 0.6 m modules -- the fridge tower, the sink, the hob, base units -- with wall
//            cupboards over it; a dining table with two chairs
//   Bed      a bed between nightstands, a wardrobe
//   Bath     a tub, a toilet, a vanity
//   Hall     nothing
//
// Rules: a piece never stands in a doorway's swing (1.2 m square at every door on the room's walls), never outside
// the room, never on another piece; a piece that fits nowhere is left out and the room stays usable. Pieces are
// PLACED, not built here: each is a PlacedPiece the interior system draws instanced, so a room costs transforms,
// not triangles. The solid pieces add their footprint box to the collider.

#include "room_plan.h"
#include <cstdint>

namespace engine {

// Furnish every room of one storey whose floor is at y0. `seed` picks the building's wood, fabric and kitchen style.
// `ceilingY` > 0: the rooms' ceiling height (world) -- each room gets its light fixtures there.
void emitFurniture(std::vector<PlacedPiece>& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, uint32_t seed,
                   Real ceilingY = 0);

}  // namespace engine

#endif
