#ifndef ENGINE_PROCGEN_CITY_ROOM_PLAN_H
#define ENGINE_PROCGEN_CITY_ROOM_PLAN_H

// ROOMS (skyscrapers v2 M7, Glenn 2026-09-15). A storey's floor plate is
// divided by ONE of two topologies, chosen by the plate itself:
//
//   RING         a deep plate: rooms line the outside walls, the middle
//                stays open. Offices behind a curtain wall (glass fronts),
//                apartments behind masonry (drywall).
//   WHOLE FLOOR  a plate too small for a ring — a house, a cottage, one
//                rowhouse unit: the floor IS one dwelling. A hall crosses
//                it, picked up at the stair, with rooms either side and a
//                door from each into the hall.
//
//       RING                          WHOLE FLOOR
//   +---------------+             +--------------------+
//   | r | r | r | r |             | bed  | bed  | bath |
//   |---+-------+---|             |---=------=---------|   = doorways
//   | r |  open | r |             |  hall, with stair  |
//   |---+-------+---|             |------=-------=-----|
//   | r | r | r | r |             | living    | kitchen|
//   +---------------+             +--------------------+
//
// Two bands of a ring can only ever meet on a SHALLOW plate, and a shallow
// plate is a house — so the topology rule is also the overlap guard. A
// 10 x 8 m plate used to grow its rooms straight through each other
// (rooms_do_not_overlap_on_a_house_sized_plate); the ring now caps its band
// depth as well, so the invariant holds whatever the plate.
//
// Ring room widths group the facade's window bays (the same bay rule the
// facade layout uses), so a partition lands on a pier or a mullion, never
// across a pane. Pure in (storey plan, params, core, storey) like the core,
// so the streamed interior grows the same rooms every time.

#include "polygon.h"
#include "core_plan.h"
#include "shape_grammar.h"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace engine {

// How a storey's plate is divided.
enum class PlateTopology : uint8_t {
    Ring,        // rooms around an open middle (offices, flats)
    WholeFloor,  // the whole plate is one dwelling (houses, rowhouse units)
};

// What a room is FOR. The ring names its rooms by use only; the whole floor
// names them properly, which is what furniture and lighting read later.
enum class RoomKind : uint8_t { Office, Flat, Hall, Living, Kitchen, Bath, Bed };

struct RoomWall {
    Vec2 a, b;           // world XZ, the wall's centre line
    Real doorAt = -1;    // parameter along a->b of a doorway's centre; < 0 = none
    bool glass = false;  // an office front: clear glass instead of drywall
    bool accent = false; // this wall wears the building's interior finish
};

// WALL FINISHES (Glenn, 2026-09-15: "do something more interesting with the
// walls... what about architectural styles and keeping them the same inside
// and out"). One wall in a room wears the finish; the rest are the field
// colour. Slats and panelling stand PROUD of the wall face, so the wall
// itself is unchanged and nothing z-fights.
enum class WallFinishKind : uint8_t {
    Paint,     // nothing but the field colour
    Accent,    // the whole wall in a saturated colour (postmodern colour block)
    Slats,     // vertical timber battens: 45 mm wide, 25 mm proud, 90 mm pitch
    Wainscot,  // a dado band with a rail on top (tile, lacquer or painted timber)
    Masonry,   // the wall itself in brick or board-formed concrete
};

struct WallFinish {
    WallFinishKind kind = WallFinishKind::Paint;
    Vec3 base{0.87, 0.85, 0.80};    // the field colour every other wall takes
    Vec3 accent{0.45, 0.35, 0.28};  // slats, dado, brick, colour block
    PartId part = PartId::Interior; // the part the finish geometry rides
    Real bandH = 0.9;               // dado height (m)
};

// The interior finish a building's OWN FACADE implies. The style a recipe
// dressed the building with is not stored anywhere (architect.cpp `dress`
// applies it and drops it), but the wall part, the curtain-wall flag and the
// window head are its fingerprint — so the inside follows the outside
// without a new field, a cache version or a city regrow:
//
//   concrete            brutalist    board-formed concrete, monochrome
//   brick / dark brick  loft         one exposed brick wall
//   stucco + round head spanish      white plaster over a talavera tile dado
//   stucco + flat head  deco         tall lacquer dado, pale plaster above
//   painted siding      craftsman    painted timber wainscot with a rail
//   metal               industrial   dark timber slats
//   curtain wall        modern       walnut slat wall against white
//   anything else       postmodern   a colour-block accent wall
WallFinish interiorFinishFor(const BuildingParams& params);

struct Room {
    Poly2 rect;          // world XZ, CCW
    std::size_t edge = 0;   // the plan edge it sits against (ring only)
    RoomKind kind = RoomKind::Office;
};

struct RoomPlan {
    PlateTopology topology = PlateTopology::Ring;
    bool office = false;
    WallFinish finish;              // what an accent wall on this storey wears
    std::vector<Room> rooms;
    std::vector<RoomWall> walls;
};

// Which topology a plate takes. A plate with a core, or behind a curtain
// wall, is always a ring; a masonry plate whose short side is under
// kWholeFloorShortSide is one dwelling per floor.
PlateTopology plateTopologyFor(const Poly2& storeyPlan, const BuildingParams& params,
                               const CorePlan& core);

// The rooms of one storey. `inset` is the interior wall inset (the inner
// face of the exterior wall), `blankEdge` an edge to leave alone (the
// straight stair's), npos for none. `stairWell` is that stair's well, where
// the hall is picked up; `entranceEdge` keeps a partition off the front
// door. In RING topology storey 0 (the lobby) gets no rooms; in WHOLE FLOOR
// it does, because the ground floor of a house is where you live.
RoomPlan roomPlan(const Poly2& storeyPlan, const BuildingParams& params, const CorePlan& core,
                  std::size_t blankEdge, Real inset, int storey,
                  const Poly2& stairWell = Poly2{},
                  std::size_t entranceEdge = static_cast<std::size_t>(-1));

// WALKABILITY (Glenn, 2026-09-15: "it might help if there's an algorithm to
// determine if the entire floor is walkable which could be used for
// validation, and if not the entire interior needs to be rebuilt"). A coarse
// grid over the plate with the walls stamped in and the doorways left open,
// flooded from `entry`: true when every room is reachable. Walls are
// inflated by the player's radius, so a doorway that a capsule cannot fit
// through counts as closed. roomPlan runs this and simplifies the floor
// until it passes, so a blocked interior is never grown.
bool floorIsWalkable(const RoomPlan& rp, const Poly2& storeyPlan, const Vec2& entry,
                     const Poly2& stairWell = Poly2{});

// The geometry: drywall partitions and fronts (`drywall`), glass fronts
// (`glass`, the GlassClear part), doorways cut with their reveals, every
// wall into `colliderOut`.
struct RoomMeshes {
    RenderMesh drywall;
    RenderMesh glass;
    RenderMesh accent;   // the finish geometry; rides RoomPlan::finish.part
};
void emitRooms(RoomMeshes& out, RenderMesh* colliderOut, const RoomPlan& rp, Real y0, Real h,
               const Vec3& paint);

// The doorway every room front carries.
constexpr Real kRoomDoorW = 0.9;
constexpr Real kRoomDoorH = 2.05;
constexpr Real kRoomWallT = 0.12;
// A masonry plate narrower than this, with no core, is one dwelling a floor.
constexpr Real kWholeFloorShortSide = 13.0;

}  // namespace engine

#endif
