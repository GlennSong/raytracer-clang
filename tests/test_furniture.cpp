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

// WHOLE-FLOOR APARTMENTS (buildings B; Glenn: "make entire apartments out of the floor ... then naturally there
// would be hallway where the stairwell or elevators would be"). A masonry tower's typical floor: apartments, every
// one a hall, a bath and a living room at least, the rooms inside the plate and clear of each other, the floor
// walkable from the corridor -- and furnished: beds, closets with shelving, pictures on the walls.
TEST_CASE(a_residential_tower_floor_is_whole_apartments) {
    for (const Poly2& plan : {Poly2{{0, 0}, {44, 0}, {44, 32}, {0, 32}}, Poly2{{0, 0}, {36, 0}, {36, 36}, {0, 36}}}) {
        BuildingParams p;
        p.floors = 24; p.curtainWall = false; p.walkableGround = true; p.openDoorway = true; p.seed = 33;
        p.residential = true;
        const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
        CHECK(core.valid);
        const Real inset = std::max(p.wallThickness, Real(0.55));
        const RoomPlan rp = roomPlan(plan, p, core, static_cast<std::size_t>(-1), inset, 6);
        CHECK(rp.topology == PlateTopology::Apartments);
        int halls = 0, baths = 0, livings = 0, beds = 0, closets = 0, kitchens = 0;
        for (const Room& r : rp.rooms) {
            switch (r.kind) {
                case RoomKind::Hall: ++halls; break;
                case RoomKind::Bath: ++baths; break;
                case RoomKind::Living: ++livings; break;
                case RoomKind::Bed: ++beds; break;
                case RoomKind::Closet: ++closets; break;
                case RoomKind::Kitchen: ++kitchens; break;
                default: break;
            }
            for (const Vec2& v : r.rect) {
                const Vec2 c = centroid(r.rect);
                CHECK(pointInPolygon(plan, v + (c - v) * 0.01));
            }
        }
        std::printf("    [apts] %zu rooms: %d apartments, %d bedrooms, %d kitchens, %d closets\n", rp.rooms.size(), halls,
                    beds, kitchens, closets);
        CHECK(halls >= 6);
        CHECK(baths == halls);
        CHECK(livings == halls);
        CHECK(kitchens == halls);
        CHECK(beds >= halls);
        // No two rooms overlap (rectangles, 2 cm shrunk: touching is not overlapping).
        for (std::size_t i = 0; i < rp.rooms.size(); ++i)
            for (std::size_t j = i + 1; j < rp.rooms.size(); ++j) {
                Real lo1 = 1e9, hi1 = -1e9, lo2 = 1e9, hi2 = -1e9, lo3 = 1e9, hi3 = -1e9, lo4 = 1e9, hi4 = -1e9;
                for (const Vec2& v : rp.rooms[i].rect) { lo1 = std::min(lo1, v.x); hi1 = std::max(hi1, v.x); lo2 = std::min(lo2, v.y); hi2 = std::max(hi2, v.y); }
                for (const Vec2& v : rp.rooms[j].rect) { lo3 = std::min(lo3, v.x); hi3 = std::max(hi3, v.x); lo4 = std::min(lo4, v.y); hi4 = std::max(hi4, v.y); }
                const bool ov = lo1 < hi3 - 0.02 && lo3 < hi1 - 0.02 && lo2 < hi4 - 0.02 && lo4 < hi2 - 0.02;
                CHECK(!ov);
            }
        CHECK(floorIsWalkable(rp, plan, core.frame.toWorld({core.length * 0.5, -1.1})));
        std::vector<PlacedPiece> placed;
        RenderMesh col;
        emitFurniture(placed, &col, rp, 20.0, p.seed);
        int pbeds = 0, shelves = 0, pictures = 0;
        for (const PlacedPiece& pp : placed) {
            if (pp.piece == static_cast<uint8_t>(Piece::Bed)) ++pbeds;
            if (pp.piece == static_cast<uint8_t>(Piece::Shelving)) ++shelves;
            if (pp.piece == static_cast<uint8_t>(Piece::Picture)) ++pictures;
        }
        std::printf("    [apts] furnished: %zu pieces, %d beds, %d closets shelved, %d pictures\n", placed.size(), pbeds,
                    shelves, pictures);
        CHECK(pbeds >= halls);
        CHECK(shelves >= closets / 2);
        CHECK(pictures >= halls);
    }
}
