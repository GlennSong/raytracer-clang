#ifndef ENGINE_PROCGEN_CITY_ROOM_PLAN_H
#define ENGINE_PROCGEN_CITY_ROOM_PLAN_H

// ROOMS (skyscrapers v2 M7, Glenn 2026-09-15: "start building out interior
// spaces — offices and apartments, rooms and open floor plans"). A storey's
// floor plate above the lobby gets a ring of rooms along its outside walls
// and an open middle: OFFICES behind a curtain wall (glass-fronted rooms
// 4.5 m deep, the rest open plan), APARTMENTS behind masonry (drywall
// rooms 5.5 m deep, a door each into the hall). Pure in (storey plan,
// params, core, storey), like the core, so the streamed interior grows the
// same rooms every time.
//
//   +----------------------------------------+   the storey plan
//   | r | r | r | r | r | r | r | r | r | r |   rooms along an EVEN edge
//   |---+---+---+---+---+---+---+---+---+---|   (they take the corners)
//   | r |                               | r |
//   |---|        the open middle        |---|   rooms along an ODD edge
//   | r |          (the core sits       | r |   (between the corners)
//   |---|           in here)            |---|
//   | r | r | r | r | r | r | r | r | r | r |
//   +----------------------------------------+
//
// Room widths group the facade's window bays (the same bay rule the facade
// layout uses), so every partition lands on a pier or a mullion, never
// across a pane. A band stops short of the core's corridor ring and of the
// stair edge; every partition and front is 0.12 m thick with a collider.

#include "polygon.h"
#include "core_plan.h"
#include "shape_grammar.h"
#include <cstddef>
#include <vector>

namespace engine {

struct RoomWall {
    Vec2 a, b;           // world XZ, the wall's centre line
    Real doorAt = -1;    // parameter along a->b of a doorway's centre; < 0 = none
    bool glass = false;  // an office front: clear glass instead of drywall
};

struct Room {
    Poly2 rect;          // world XZ, CCW
    std::size_t edge;    // the plan edge it sits against
};

struct RoomPlan {
    bool office = false;
    std::vector<Room> rooms;
    std::vector<RoomWall> walls;
};

// The rooms of one storey. `inset` is the interior wall inset (the inner
// face of the exterior wall), `blankEdge` an edge to leave alone (the
// straight stair's), npos for none. Storey 0 (the lobby) gets no rooms.
RoomPlan roomPlan(const Poly2& storeyPlan, const BuildingParams& params, const CorePlan& core,
                  std::size_t blankEdge, Real inset, int storey);

// The geometry: drywall partitions and fronts (`drywall`), glass fronts
// (`glass`, the GlassClear part), doorways cut with their reveals, every
// wall into `colliderOut`.
struct RoomMeshes {
    RenderMesh drywall;
    RenderMesh glass;
};
void emitRooms(RoomMeshes& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, Real h,
               const Vec3& paint);

// The doorway every room front carries.
constexpr Real kRoomDoorW = 0.9;
constexpr Real kRoomDoorH = 2.05;
constexpr Real kRoomWallT = 0.12;

}  // namespace engine

#endif
