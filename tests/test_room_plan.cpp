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
