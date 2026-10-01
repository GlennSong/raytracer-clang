#ifndef ENGINE_PROCGEN_CITY_FURNITURE_H
#define ENGINE_PROCGEN_CITY_FURNITURE_H

// FURNITURE (buildings M4; Glenn, 2026-09-30: "yes to all of that" -- the rooms were empty boxes). Every room the
// room planner names gets the pieces its use implies, set against its walls in the room's own frame:
//
//   Office   a desk facing the window with its chair and monitor, a filing cabinet; a second desk in a wide room
//   Flat     a bed against a partition, a sofa, a small table (the ring's apartments are studios)
//   Living   a sofa and coffee table facing a TV on its stand
//   Kitchen  a counter run along the longest blank wall, a table with two chairs
//   Bed      a bed with a nightstand either side, a wardrobe
//   Bath     a tub, a toilet, a vanity
//   Hall     nothing
//
// Rules: a piece never stands in a doorway's swing (1.2 m square at every door on the room's walls), never outside
// the room, never on another piece; a piece that fits nowhere is left out, the room stays usable. The big pieces
// (desk, bed, sofa, counter, wardrobe, tub, table) add their boxes to the collider. Every piece is a handful of
// boxes, five faces each (no underside): ~50 triangles a room, so the streamed window stays inside its budget
// (core_window_grow_cost_is_bounded). Wood (`wood`) and fabric, laminate, ceramic and metal (`soft`) both ride
// PartId::Furniture, coloured per vertex -- one part for every piece, so a furnished floor is one more draw call,
// not one per piece. (The Wood part's siding planks read as decking on a desk top.)

#include "room_plan.h"
#include <cstdint>

namespace engine {

struct FurnitureMeshes {
    RenderMesh wood;   // the timber pieces (PartId::Furniture too)
    RenderMesh soft;   // PartId::Furniture
};

// Furnish every room of one storey whose floor is at y0. `seed` varies colours and mirrors per building.
void emitFurniture(FurnitureMeshes& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, uint32_t seed);

}  // namespace engine

#endif
