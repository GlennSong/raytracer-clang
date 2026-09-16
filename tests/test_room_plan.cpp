// ROOMS (skyscrapers v2 M7): offices and apartments along a storey's outside
// walls, partitions on the facade's bay lines, a door in every front, the
// core's corridor ring and the corners respected, colliders on everything.
#include "test_framework.h"

#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;

namespace {
BuildingParams tower(int floors, bool curtain) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = curtain;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 21;
    return p;
}
Real segLen(const RoomWall& w) { return (w.b - w.a).length(); }
}  // namespace

TEST_CASE(rooms_ring_the_plate_and_keep_the_core_clear) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 30}, {0, 30}};
    const BuildingParams p = tower(20, true);
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    CHECK(core.valid);
    const Real inset = std::max(p.wallThickness, Real(0.55));
    const RoomPlan rp = roomPlan(plan, p, core, static_cast<std::size_t>(-1), inset, 5);
    CHECK(rp.office);
    CHECK(rp.rooms.size() >= 12);
    // Every room sits inside the plan and outside the core's corridor ring.
    const Poly2 ring = core.rect();
    for (const Room& r : rp.rooms) {
        for (const Vec2& c : r.rect) {
            CHECK(pointInPolygon(plan, c));
            Real best = 1e9;
            for (const Vec2& q : ring) best = std::min(best, (c - q).length());
            (void)best;
            CHECK(!pointInPolygon(ring, c));
        }
    }
    // Office fronts are glass with a door; partitions are drywall.
    int glassFronts = 0, doors = 0;
    for (const RoomWall& w : rp.walls) {
        if (w.glass) ++glassFronts;
        if (w.doorAt >= 0) ++doors;
        CHECK(segLen(w) > 0.3);
    }
    CHECK(glassFronts == static_cast<int>(rp.rooms.size()));
    CHECK(doors == static_cast<int>(rp.rooms.size()));
    // The lobby gets none.
    CHECK(roomPlan(plan, p, core, static_cast<std::size_t>(-1), inset, 0).rooms.empty());
}

TEST_CASE(apartment_partitions_land_on_the_bay_lines) {
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 24}, {0, 24}};
    BuildingParams p = tower(6, false);
    p.core = 1;   // no core: the whole ring is rooms
    const Real inset = std::max(p.wallThickness, Real(0.55));
    const RoomPlan rp = roomPlan(plan, p, CorePlan{}, static_cast<std::size_t>(-1), inset, 2);
    CHECK(!rp.office);
    CHECK(rp.rooms.size() >= 8);
    // A side partition on edge 0 (y = 0, along +x) sits at x = 30 * b / bays.
    const int bays = std::max(1, static_cast<int>(std::lround(30.0 / p.bayWidth)));
    int sides = 0, onBay = 0;
    for (const RoomWall& w : rp.walls) {
        if (w.glass || w.doorAt >= 0) continue;                    // a partition
        if (std::fabs(w.a.y - inset) > 1e-3 || std::fabs(w.a.x - w.b.x) > 1e-6) continue;   // edge 0's, perpendicular
        ++sides;
        for (int b = 1; b < bays; ++b)
            if (std::fabs(w.a.x - 30.0 * b / bays) < 1e-3) ++onBay;
    }
    CHECK(sides >= 2);
    CHECK(onBay == sides);
    // The geometry: every wall reaches the collider, doorways are cut (a
    // point in a front's doorway at chest height is under no triangle of it).
    RoomMeshes rm;
    RenderMesh col;
    emitRooms(rm, &col, rp, 3.2, 3.2, Vec3(0.9, 0.9, 0.9));
    CHECK(!rm.drywall.indices.empty());
    CHECK(rm.glass.indices.empty());
    CHECK(col.indices.size() == rm.drywall.indices.size());
    const RoomWall* front = nullptr;
    for (const RoomWall& w : rp.walls) if (w.doorAt >= 0) { front = &w; break; }
    CHECK(front != nullptr);
    if (front) {
        const Vec2 dv = front->b - front->a;
        const Real L = dv.length();
        const Vec2 c2 = front->a + dv * (std::min(std::max(front->doorAt * L, kRoomDoorW * 0.5 + 0.3), L - kRoomDoorW * 0.5 - 0.3) / L);
        const Vec3 c(c2.x, 3.2 + 1.2, c2.y);
        bool covered = false;
        for (std::size_t i = 0; i + 2 < rm.drywall.indices.size(); i += 3) {
            const Vec3 a = rm.drywall.vertices[rm.drywall.indices[i]].position;
            const Vec3 b = rm.drywall.vertices[rm.drywall.indices[i + 1]].position;
            const Vec3 d = rm.drywall.vertices[rm.drywall.indices[i + 2]].position;
            // A vertical triangle of this front whose XZ footprint spans the door centre at this height.
            const Vec3 n = cross(b - a, d - a);
            if (std::fabs(n.y) > 0.5 * n.length()) continue;
            const Real minY = std::min({a.y, b.y, d.y}), maxY = std::max({a.y, b.y, d.y});
            if (c.y < minY || c.y > maxY) continue;
            const Real minX = std::min({a.x, b.x, d.x}) - 0.02, maxX = std::max({a.x, b.x, d.x}) + 0.02;
            const Real minZ = std::min({a.z, b.z, d.z}) - 0.02, maxZ = std::max({a.z, b.z, d.z}) + 0.02;
            if (c.x >= minX && c.x <= maxX && c.z >= minZ && c.z <= maxZ &&
                std::fabs(dot(normalize(n), c - a)) < kRoomWallT)
                covered = true;
        }
        CHECK(!covered);
    }
}

// A HOUSE: the whole floor is one dwelling. A hall crosses it at the stair,
// every room opens onto that hall, and the stair stays clear.
TEST_CASE(house_floor_is_a_hall_with_rooms_either_side) {
    const Poly2 plan = {{0, 0}, {11, 0}, {11, 8}, {0, 8}};
    BuildingParams p = tower(2, false);
    p.core = 1;
    const Real inset = std::max(p.wallThickness, Real(0.55));
    // A straight stair hugging the short edge, running into the plate.
    const Poly2 well = {{0.7, 2.4}, {1.9, 2.4}, {1.9, 5.6}, {0.7, 5.6}};
    const RoomPlan rp = roomPlan(plan, p, CorePlan{}, static_cast<std::size_t>(-1), inset, 1, well, 0);
    CHECK(rp.topology == PlateTopology::WholeFloor);
    std::printf("    [house] 11x8 upper floor: %zu rooms, %zu walls\n", rp.rooms.size(), rp.walls.size());
    CHECK(rp.rooms.size() >= 2);
    // One door per room, and every room inside the plate.
    std::size_t doors = 0;
    for (const RoomWall& w : rp.walls) if (w.doorAt >= 0) ++doors;
    CHECK(doors == rp.rooms.size());
    for (const Room& r : rp.rooms)
        for (const Vec2& c : r.rect) CHECK(pointInPolygon(plan, c));
    // The stair is in the hall, so no room sits on it.
    const Vec2 wc = centroid(well);
    for (const Room& r : rp.rooms) CHECK(!pointInPolygon(r.rect, wc));
    // Bedrooms upstairs, and nothing overlaps.
    bool bed = false;
    for (const Room& r : rp.rooms) if (r.kind == RoomKind::Bed) bed = true;
    CHECK(bed);
    for (std::size_t i = 0; i < rp.rooms.size(); ++i)
        for (std::size_t j = i + 1; j < rp.rooms.size(); ++j)
            CHECK(!pointInPolygon(rp.rooms[j].rect, centroid(rp.rooms[i].rect)));
}

// The ground floor of a house is living space; the ground floor of a tower
// is a lobby and keeps its rooms out.
TEST_CASE(a_house_ground_floor_has_rooms_a_tower_lobby_does_not) {
    const Poly2 house = {{0, 0}, {11, 0}, {11, 8}, {0, 8}};
    BuildingParams hp = tower(2, false);
    hp.core = 1;
    const RoomPlan ground = roomPlan(house, hp, CorePlan{}, static_cast<std::size_t>(-1), 0.55, 0);
    CHECK(ground.topology == PlateTopology::WholeFloor);
    CHECK(!ground.rooms.empty());
    bool living = false, kitchen = false;
    for (const Room& r : ground.rooms) {
        if (r.kind == RoomKind::Living) living = true;
        if (r.kind == RoomKind::Kitchen) kitchen = true;
    }
    CHECK(living);
    CHECK(kitchen);

    const Poly2 plate = {{0, 0}, {40, 0}, {40, 30}, {0, 30}};
    const BuildingParams tp = tower(20, true);
    const CorePlan core = coreFor(plate, tp, entranceEdgeFor(plate, tp));
    const RoomPlan lobby = roomPlan(plate, tp, core, static_cast<std::size_t>(-1), 0.55, 0);
    CHECK(lobby.topology == PlateTopology::Ring);
    CHECK(lobby.rooms.empty());
}

// The inside follows the outside: the facade's wall part, curtain-wall flag
// and window head are the fingerprint of the style the recipe dressed the
// building with, and the interior finish is read off them.
TEST_CASE(wall_finishes_follow_the_facade_style) {
    BuildingParams p = tower(4, false);
    p.core = 1;
    p.wallPart = PartId::Concrete;                    // brutalist
    CHECK(interiorFinishFor(p).kind == WallFinishKind::Masonry);
    CHECK(interiorFinishFor(p).part == PartId::Concrete);
    p.wallPart = PartId::Brick;                       // loft: PAINTED brick, not
    CHECK(interiorFinishFor(p).kind == WallFinishKind::Accent);   // the facade bake
    CHECK(interiorFinishFor(p).part == PartId::Interior);
    p.wallPart = PartId::Stucco;
    p.window.head = OpeningStyle::Head::Round;         // spanish: a tile dado
    const WallFinish sp = interiorFinishFor(p);
    CHECK(sp.kind == WallFinishKind::Wainscot);
    CHECK(sp.bandH < 1.0);
    p.window.head = OpeningStyle::Head::Flat;          // deco: a taller lacquer dado
    CHECK(interiorFinishFor(p).bandH > 1.0);
    const BuildingParams glass = tower(20, true);      // modern: a slat wall
    CHECK(interiorFinishFor(glass).kind == WallFinishKind::Slats);
    // A field colour is never dark: one lamp lights these rooms, and a
    // mid-tone wall would cave the room in (the paint census rule).
    for (PartId wp : {PartId::Concrete, PartId::Brick, PartId::Stucco, PartId::Siding, PartId::Metal}) {
        BuildingParams q = p;
        q.wallPart = wp;
        const WallFinish f = interiorFinishFor(q);
        CHECK(f.base.x >= 0.45);
        CHECK(f.base.y >= 0.45);
        CHECK(f.base.z >= 0.45);
    }
}

TEST_CASE(an_accent_wall_carries_its_finish_geometry) {
    const Poly2 plan = {{0, 0}, {11, 0}, {11, 8}, {0, 8}};
    BuildingParams p = tower(2, false);
    p.core = 1;
    p.wallPart = PartId::Metal;                        // slats
    const RoomPlan rp = roomPlan(plan, p, CorePlan{}, static_cast<std::size_t>(-1), 0.55, 1);
    CHECK(rp.finish.kind == WallFinishKind::Slats);
    int accents = 0;
    for (const RoomWall& w : rp.walls) if (w.accent) ++accents;
    std::printf("    [finish] %d accent walls of %zu, dado %.2f m\n", accents, rp.walls.size(),
                rp.finish.bandH);
    CHECK(accents >= 1);
    RoomMeshes rm;
    RenderMesh col;
    emitRooms(rm, &col, rp, 3.2, 3.2, rp.finish.base);
    CHECK(!rm.drywall.indices.empty());                // the walls
    CHECK(!rm.accent.indices.empty());                 // the battens on them
    // A 25 mm batten is not a collider: the wall behind it already stops you.
    CHECK(col.indices.size() == rm.drywall.indices.size());
}

// Glenn's walk, 2026-09-15: "I was immediately stopped by walls and couldn't
// get around it" and "rooms butt right up against the stairwell". Every
// house floor must be crossable from the front door, and the validator must
// actually catch a floor that is not.
TEST_CASE(a_house_floor_is_walkable_from_the_front_door) {
    const Poly2 plan = {{0, 0}, {11, 0}, {11, 8}, {0, 8}};
    BuildingParams p = tower(2, false);
    p.core = 1;
    const Poly2 well = {{0.7, 2.4}, {1.9, 2.4}, {1.9, 5.6}, {0.7, 5.6}};
    const Vec2 entry(5.5, 1.2);                     // just inside the front door
    for (int storey = 0; storey < 2; ++storey) {
        const RoomPlan rp = roomPlan(plan, p, CorePlan{}, static_cast<std::size_t>(-1), 0.55,
                                     storey, well, 0);
        std::printf("    [walk] storey %d: %zu rooms, %zu walls, walkable %d\n", storey,
                    rp.rooms.size(), rp.walls.size(), floorIsWalkable(rp, plan, entry, well) ? 1 : 0);
        // WITH THE WELL SOLID: you have to be able to walk ROUND the stair,
        // not through the hole it sits in.
        CHECK(floorIsWalkable(rp, plan, entry, well));
        // The stair keeps walking room: no room sits on the well.
        for (const Room& r : rp.rooms) CHECK(!pointInPolygon(r.rect, centroid(well)));
    }
}

// A hall whose only way past the stair is narrower than the player is a
// dead end, and the check must say so.
TEST_CASE(the_walk_check_catches_a_blocked_stair_passage) {
    const Poly2 plan = {{0, 0}, {11, 0}, {11, 8}, {0, 8}};
    // WALL TO WALL, edge to edge: a well that stops short of the plan leaves
    // a gap to walk round, and then the floor really is reachable.
    const Poly2 well = {{0.0, 3.0}, {11.0, 3.0}, {11.0, 5.0}, {0.0, 5.0}};
    RoomPlan rp;
    rp.topology = PlateTopology::WholeFloor;
    Room r;
    r.rect = {{0.6, 5.4}, {10.4, 5.4}, {10.4, 7.4}, {0.6, 7.4}};             // beyond it
    rp.rooms.push_back(r);
    RoomWall front;
    front.a = {0.6, 5.4};
    front.b = {10.4, 5.4};
    front.doorAt = 0.5;
    rp.walls.push_back(front);
    CHECK(floorIsWalkable(rp, plan, Vec2(5.5, 1.2)));                  // the hole is not modelled
    CHECK(!floorIsWalkable(rp, plan, Vec2(5.5, 1.2), well));           // ...and now it is
}

TEST_CASE(the_walk_check_catches_a_sealed_room) {
    const Poly2 plan = {{0, 0}, {11, 0}, {11, 8}, {0, 8}};
    RoomPlan rp;
    rp.topology = PlateTopology::WholeFloor;
    Room r;
    r.rect = {{6.0, 0.0}, {11.0, 0.0}, {11.0, 4.0}, {6.0, 4.0}};
    rp.rooms.push_back(r);
    // Seal it: two walls with no doorway, run corner to corner against the
    // plan's own edges. Walls that stop short leave a gap to walk round, and
    // the room is then not sealed at all.
    RoomWall w1; w1.a = {6.0, 0.0}; w1.b = {6.0, 4.0};
    RoomWall w2; w2.a = {6.0, 4.0}; w2.b = {11.0, 4.0};
    rp.walls.push_back(w1);
    rp.walls.push_back(w2);
    CHECK(!floorIsWalkable(rp, plan, Vec2(2.0, 2.0)));
    // Give it a door and it passes.
    rp.walls[0].doorAt = 0.5;
    CHECK(floorIsWalkable(rp, plan, Vec2(2.0, 2.0)));
}

// Glenn's walk: "there was a 5 story building I entered where the walls
// immediately jutted out of the building". A bounding box fits a rectangle
// and not a trapezoid, so the layout is clipped to the plan.
TEST_CASE(house_rooms_stay_inside_a_trapezoid_plan) {
    const Poly2 plan = {{0, 0}, {12, 0}, {10, 8}, {2, 8}};
    BuildingParams p = tower(2, false);
    p.core = 1;
    const RoomPlan rp = roomPlan(plan, p, CorePlan{}, static_cast<std::size_t>(-1), 0.55, 1);
    CHECK(rp.topology == PlateTopology::WholeFloor);
    std::printf("    [trapezoid] %zu rooms, %zu walls\n", rp.rooms.size(), rp.walls.size());
    for (const Room& r : rp.rooms)
        for (const Vec2& c : r.rect) CHECK(pointInPolygon(plan, c));
    for (const RoomWall& w : rp.walls) {
        CHECK(pointInPolygon(plan, w.a));
        CHECK(pointInPolygon(plan, w.b));
    }
}

// The facade brick bakes are world-scaled, so a course reads a foot tall
// indoors. Interior brick is paint until there is an interior-scale bake.
TEST_CASE(interior_brick_is_paint_not_the_facade_surface) {
    BuildingParams p = tower(3, false);
    p.wallPart = PartId::Brick;
    const WallFinish f = interiorFinishFor(p);
    CHECK(f.kind == WallFinishKind::Accent);
    CHECK(f.part == PartId::Interior);
    CHECK(f.accent.x >= 0.45);
    CHECK(f.accent.y >= 0.45);
    CHECK(f.accent.z >= 0.45);
}

TEST_CASE(streamed_storeys_carry_their_rooms) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 30}, {0, 30}};
    const BuildingParams p = tower(20, true);
    RenderMesh col;
    const BuildingMesh win = growInterior(plan, p, 0.0, &col, 4, 7);
    std::size_t clearGlass = 0;
    for (const RenderMesh& part : win.parts)
        if (part.materialIndex == static_cast<int>(PartId::GlassClear)) clearGlass += part.indices.size();
    CHECK(clearGlass > 0);   // the office fronts
}

// A HOUSE-sized plate: 10 x 8 m, no core. The band is a fixed depth per
// wall, so on a shallow plate opposite bands can meet in the middle or
// pass through each other. Rooms must never overlap and must stay inside
// the plate, whatever the plate's size.
TEST_CASE(rooms_do_not_overlap_on_a_house_sized_plate) {
    const Poly2 plan = {{0, 0}, {10, 0}, {10, 8}, {0, 8}};
    BuildingParams p = tower(2, false);
    p.core = 1;
    const Real inset = std::max(p.wallThickness, Real(0.55));
    const RoomPlan rp = roomPlan(plan, p, CorePlan{}, static_cast<std::size_t>(-1), inset, 1);
    std::printf("    [house] 10x8 plate: %zu rooms, %zu walls\n", rp.rooms.size(), rp.walls.size());
    for (const Room& r : rp.rooms) {
        for (const Vec2& c : r.rect) {
            CHECK(pointInPolygon(plan, c));
        }
    }
    for (std::size_t i = 0; i < rp.rooms.size(); ++i)
        for (std::size_t j = i + 1; j < rp.rooms.size(); ++j) {
            const Vec2 ci = centroid(rp.rooms[i].rect), cj = centroid(rp.rooms[j].rect);
            if (pointInPolygon(rp.rooms[j].rect, ci) || pointInPolygon(rp.rooms[i].rect, cj))
                std::printf("    [house] OVERLAP: room %zu centre (%.2f, %.2f) inside room %zu\n", i, ci.x, ci.y, j);
            CHECK(!pointInPolygon(rp.rooms[j].rect, ci));
            CHECK(!pointInPolygon(rp.rooms[i].rect, cj));
        }
}
