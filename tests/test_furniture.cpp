// THE FURNITURE KIT (buildings M4b): modelled pieces, one mesh per finish, drawn instanced; and the rooms they are
// placed in.
#include "test_framework.h"

#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/city/furniture.h"
#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include <cmath>
#include <cstdio>

using namespace engine;

// Every piece, in a spread of variants: geometry in at least one finish, every vertex finite and inside the
// footprint it declares (a little slack for handles and rounded edges), standing on the floor.
TEST_CASE(every_furniture_piece_is_modelled_inside_its_footprint) {
    for (int i = 0; i < kPieceCount; ++i) {
        for (uint32_t variant : {0u, 9u, 42u, 101u}) {
            const FurniturePiece& fp = furniturePiece(static_cast<Piece>(i), variant);
            std::size_t tris = 0;
            for (const RenderMesh& m : fp.mesh) {
                tris += m.indices.size() / 3;
                for (const Vertex& v : m.vertices) {
                    const Vec3 p = v.position;
                    CHECK(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z));
                    CHECK(std::fabs(p.x) <= fp.size.x * 0.5 + 0.08);
                    CHECK(p.z >= -0.06 && p.z <= fp.size.z + 0.08);
                    CHECK(p.y >= -0.01 && p.y <= fp.size.y + 0.08);
                }
            }
            CHECK(tris > 0);
            for (const RenderMesh& m : fp.mesh)
                for (const Vertex& v : m.vertices)
                    if (v.position.y > fp.size.y + 0.08 || std::fabs(v.position.x) > fp.size.x * 0.5 + 0.08 ||
                        v.position.z > fp.size.z + 0.08 || v.position.z < -0.06 || v.position.y < -0.01) {
                        std::printf("    [kit] OUTSIDE %s v%u at %.2f %.2f %.2f (size %.2f %.2f %.2f)\n",
                                    furniturePieceName(static_cast<Piece>(i)), variant, v.position.x, v.position.y,
                                    v.position.z, fp.size.x, fp.size.y, fp.size.z);
                        break;
                    }
            if (variant == 0u) std::printf("    [kit] %-15s %6zu tris\n", furniturePieceName(static_cast<Piece>(i)), tris);
        }
        Piece back;
        CHECK(furniturePieceByName(furniturePieceName(static_cast<Piece>(i)), back) && back == static_cast<Piece>(i));
    }
}

// An office floor is furnished: a desk, its chair and monitor in the rooms, every piece inside the plate, none in
// a doorway's swing, drawn as placements (not merged triangles).
TEST_CASE(office_rooms_are_furnished_with_placed_pieces) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 30}, {0, 30}};
    BuildingParams p;
    p.floors = 20; p.curtainWall = true; p.walkableGround = true; p.openDoorway = true; p.seed = 21;
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    const Real inset = std::max(p.wallThickness, Real(0.55));
    const RoomPlan rp = roomPlan(plan, p, core, static_cast<std::size_t>(-1), inset, 5);
    std::vector<PlacedPiece> placed;
    RenderMesh col;
    emitFurniture(placed, &col, rp, 25.0, p.seed);
    int desks = 0, chairs = 0;
    std::vector<Vec2> doors;
    for (const RoomWall& w : rp.walls)
        if (w.doorAt >= 0) doors.push_back(w.a + (w.b - w.a) * w.doorAt);
    for (const PlacedPiece& pp : placed) {
        if (pp.piece == static_cast<uint8_t>(Piece::Desk)) ++desks;
        if (pp.piece == static_cast<uint8_t>(Piece::OfficeChair)) ++chairs;
        const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
        CHECK(pointInPolygon(plan, at));
        CHECK(std::fabs(pp.xform.m[1][3] - 25.0) < 1.0);
        for (const Vec2& d : doors) CHECK((at - d).length() > 0.45);
    }
    CHECK(desks >= static_cast<int>(rp.rooms.size()) / 2);
    CHECK(chairs == desks);
    CHECK(col.indices.size() > 0);   // the desks and cabinets are solid
    std::printf("    [furnish] %zu rooms: %zu pieces, %d desks\n", rp.rooms.size(), placed.size(), desks);
}
