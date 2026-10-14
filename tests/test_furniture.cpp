// THE FURNITURE KIT (buildings M4b): modelled pieces, one mesh per finish, drawn instanced; and the rooms they are
// placed in.
#include "test_framework.h"

#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/city/furniture.h"
#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/trades.h"
#include <algorithm>
#include <map>
#include <string>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace engine;

#include "../src/engine/procgen/furniture_library.h"
#include "../src/engine/scripting/furniture_library_lua.h"
#include <fstream>
#include <sstream>

namespace {
std::string readScript(const char* name) {
    std::ifstream in(std::string(RT_SOURCE_DIR) + "/assets/scripts/" + name);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}
// the shipped library, with or without its room programs
FurnitureLibrary shippedFurniture(bool programs) {
    FurnitureLibrary lib;
    std::string err;
    const std::string src = readScript("furniture_library.lua") + (programs ? "\n" + readScript("furniture_rooms.lua") : "");
    const bool ok = loadFurnitureLibrarySource(src, lib, &err);
    if (!ok) std::printf("    furniture load: %s\n", err.c_str());
    CHECK(ok);
    return lib;
}
// The shipped library and room programs as the process-wide library for one test (rooms are furnished from it).
struct UseShippedFurniture {
    FurnitureLibrary saved = FurnitureLibrary::global();
    UseShippedFurniture() { FurnitureLibrary::global() = shippedFurniture(true); }
    ~UseShippedFurniture() { FurnitureLibrary::global() = saved; }
};
}  // namespace

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

// An office floor is furnished: workstations across the open plan and desks in the corner offices, every piece
// inside the plate, none in a doorway's swing, drawn as placements (not merged triangles).
TEST_CASE(office_rooms_are_furnished_with_placed_pieces) {
    UseShippedFurniture shipped;   // rooms are furnished from furniture_rooms.lua
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 30}, {0, 30}};
    BuildingParams p;
    p.floors = 20; p.curtainWall = true; p.walkableGround = true; p.openDoorway = true; p.seed = 21;
    const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
    const Real inset = std::max(p.wallThickness, Real(0.55));
    const RoomPlan rp = roomPlan(plan, p, core, static_cast<std::size_t>(-1), inset, 5);
    std::vector<PlacedPiece> placed;
    RenderMesh col;
    emitFurniture(placed, &col, rp, 25.0, p.seed);
    int desks = 0, chairs = 0, seats = 0, receptions = 0;
    std::vector<Vec2> doors;
    for (const RoomWall& w : rp.walls)
        if (w.doorAt >= 0) doors.push_back(w.a + (w.b - w.a) * w.doorAt);
    for (const PlacedPiece& pp : placed) {
        if (pp.piece == static_cast<uint8_t>(Piece::Desk)) { ++desks; ++seats; }
        if (pp.piece == static_cast<uint8_t>(Piece::OfficeChair)) ++chairs;
        if (pp.piece == static_cast<uint8_t>(Piece::ReceptionDesk)) ++receptions;
        if (pp.piece == static_cast<uint8_t>(Piece::DeskPod)) seats += ((pp.variant >> 5) & 4u) ? 4 : 6;      // the open plan (buildings C)
        if (pp.piece == static_cast<uint8_t>(Piece::Cubicle)) seats += 1;
        const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
        CHECK(pointInPolygon(plan, at));
        CHECK(std::fabs(pp.xform.m[1][3] - 25.0) < 1.0);
        for (const Vec2& d : doors) CHECK((at - d).length() > 0.45);
    }
    CHECK(seats >= 28);   // a 40 x 30 m office floor seats a team (~30 cubicles, ~90 at benches), not a desk a room
    CHECK(receptions == 1);
    CHECK(chairs == desks + receptions);   // a chair at every desk, and the receptionist's
    CHECK(col.indices.size() > 0);   // the desks and cabinets are solid
    std::printf("    [furnish] %zu rooms: %zu pieces, %d workstations\n", rp.rooms.size(), placed.size(), seats);
}

// WHOLE-FLOOR APARTMENTS (buildings B; Glenn: "make entire apartments out of the floor ... then naturally there
// would be hallway where the stairwell or elevators would be"). A masonry tower's typical floor: apartments, every
// one a hall, a bath and a living room at least, the rooms inside the plate and clear of each other, the floor
// walkable from the corridor -- and furnished: beds, closets with shelving, pictures on the walls.
TEST_CASE(a_residential_tower_floor_is_whole_apartments) {
    UseShippedFurniture shipped;   // rooms are furnished from furniture_rooms.lua
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

// THE WALK-UP (Glenn, 2026-10-01: "I didn't see the apartments ... some of them look like dorms"): a five-storey
// residential building with no lift core. Its floor is apartments off a corridor from the stair, every one a hall,
// a bath, a kitchen and a living room, walkable from the stair.
TEST_CASE(a_walkup_floor_is_apartments_off_the_stair) {
    for (const Poly2& plan : {Poly2{{0, 0}, {26, 0}, {26, 15}, {0, 15}}, Poly2{{0, 0}, {30, 0}, {30, 10.5}, {0, 10.5}}}) {
        BuildingParams p;
        p.floors = 5; p.curtainWall = false; p.walkableGround = true; p.openDoorway = true; p.seed = 7;
        p.residential = true; p.core = 1;   // no lift: a stair
        const std::size_t entrance = entranceEdgeFor(plan, p);
        const InteriorLayout il = interiorLayout(plan, p, entrance);
        CHECK(il.hasStair);
        const CorePlan core = coreFor(plan, p, entrance);
        CHECK(!core.valid);
        const Real inset = std::max(p.wallThickness, Real(0.55));
        const RoomPlan rp = roomPlan(plan, p, core, il.edge, inset, 2, il.well, entrance);
        int halls = 0, baths = 0, kitchens = 0, livings = 0;
        for (const Room& r : rp.rooms) {
            if (r.kind == RoomKind::Hall) ++halls;
            if (r.kind == RoomKind::Bath) ++baths;
            if (r.kind == RoomKind::Kitchen) ++kitchens;
            if (r.kind == RoomKind::Living) ++livings;
        }
        std::printf("    [walkup] %.0f x %.0f: %d apartments, %zu rooms\n", plan[1].x, plan[2].y, halls, rp.rooms.size());
        CHECK(rp.topology == PlateTopology::Apartments);
        CHECK(halls >= 2);
        CHECK(baths == halls);
        CHECK(kitchens == halls);
        CHECK(livings == halls);
        CHECK(floorIsWalkable(rp, plan, centroid(il.well), il.well));
    }
}

// SHOPS (Glenn, 2026-10-01: "I'm still waiting to see these small shops ... I'd also like to see that with smaller
// buildings"): a four-storey building with a storefront on a 30 m street face. Its ground storey has shops: each
// with its own street door (a "shopdoor" attach, after the building's "entrance"), a room behind the shopfront,
// furnished for its trade -- a counter in every one.
TEST_CASE(a_storefront_ground_floor_is_shops_with_their_own_doors) {
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 14}, {0, 14}};
    BuildingParams p;
    p.floors = 3; p.groundRetail = true; p.walkableGround = true; p.openDoorway = true; p.seed = 12;
    p.residential = true; p.core = 1;
    const BuildingMesh ext = growPlanBuilding(plan, p, 0.0, FacadeDetail::Full);
    int entrances = 0, shopDoors = 0;
    bool entranceFirst = false;
    for (const AttachPoint& ap : ext.attaches) {
        if (ap.tag == "entrance") { if (shopDoors == 0) entranceFirst = true; ++entrances; }
        if (ap.tag == "shopdoor") ++shopDoors;
    }
    std::printf("    [shops] %d entrance, %d shop doors\n", entrances, shopDoors);
    CHECK(entrances == 1);
    CHECK(shopDoors >= 2);
    (void)entranceFirst;
    RenderMesh col;
    const BuildingMesh in = growInterior(plan, p, 0.0, &col, 0, 1);
    int counters = 0;
    for (const PlacedPiece& pp : in.furniture)
        if (pp.piece == static_cast<uint8_t>(Piece::ShopCounter)) ++counters;
    std::printf("    [shops] ground floor: %zu pieces, %d counters\n", in.furniture.size(), counters);
    CHECK(counters >= 2);
}

// BIG-BOX STORES (Glenn, 2026-10-01: "Big box stores like Costco or Bestbuy"). Each chain: one door on the front
// (+z), blank walls but for it, the lit sign over it; inside, checkout lanes inside the doors with the doorway
// itself clear, the chain's own stock on the sales floor, racking in the stockroom.
TEST_CASE(a_big_box_store_has_its_sign_checkouts_and_stock) {
    const Poly2 plan = {{-40, -28}, {40, -28}, {40, 28}, {-40, 28}};
    for (int chain = 1; chain <= 4; ++chain) {
        BuildingRecipe rec = architectBigBox(static_cast<uint32_t>(chain * 7));
        BuildingParams p = rec.params;
        CHECK(rec.massing == BuildingRecipe::Massing::BigBox);
        dressBigBox(p, chain, p.seed);
        CHECK(p.bigBox == chain);
        p.openDoorway = true;
        p.faceDir = Vec3(0, 0, 1);
        const BuildingMesh bm = growPlanBuilding(plan, p);
        int fronts = 0;
        Vec2 door(0, 0);
        for (const AttachPoint& ap : bm.attaches)
            if (ap.tag == "entrance") { ++fronts; door = Vec2(ap.position.x, ap.position.z); CHECK(ap.normal.z > 0.9); }
        CHECK(fronts == 1);
        int signLit = 0, sideGlass = 0;
        for (const RenderMesh& part : bm.parts) {
            for (const Vertex& v : part.vertices) {
                if (part.materialIndex == static_cast<int>(PartId::LitBand) && v.position.z > 28 && v.position.y > 5) ++signLit;
                if ((part.materialIndex == static_cast<int>(PartId::Glass) ||
                     part.materialIndex == static_cast<int>(PartId::GlassLit)) && std::fabs(v.normal.x) > 0.9) ++sideGlass;
            }
        }
        CHECK(signLit > 0);
        CHECK(sideGlass == 0);
        RenderMesh col;
        const BuildingMesh in = growInterior(plan, p, 0.0, &col, 0, -1);
        int checkouts = 0, stock = 0, racks = 0, inDoorway = 0;
        const Piece own = (chain == 1 || chain == 3) ? Piece::PalletRack : Piece::Gondola;
        for (const PlacedPiece& pp : in.furniture) {
            const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
            if (pp.piece == static_cast<uint8_t>(Piece::Checkout)) ++checkouts;
            if (pp.piece == static_cast<uint8_t>(own) && at.y > -16 && at.y < 18) ++stock;
            if (pp.piece == static_cast<uint8_t>(Piece::PalletRack) && at.y < -16) ++racks;
            if ((at - door).length() < 3.0 && pp.piece != static_cast<uint8_t>(Piece::CeilingLight)) ++inDoorway;   // lights hang overhead
        }
        std::printf("    [big box] chain %d: %zu pieces, %d checkouts, %d of its stock, %d stockroom racks\n", chain,
                    in.furniture.size(), checkouts, stock, racks);
        CHECK(checkouts >= 10);
        CHECK(stock >= 20);
        CHECK(racks >= 8);
        CHECK(inDoorway == 0);
        CHECK(!col.indices.empty());
    }
}


// THE ROOM PROGRAMS (M3b): home and office rooms are furnished from furniture_rooms.lua, every piece picked by what
// it is. (Before the hand-written cases were deleted, the programs matched them exactly: 180 floors, 37,560 pieces,
// none different.) Without the programs those rooms stay empty -- the furniture is data -- and with them every
// kind gets its pieces: desks and chairs, beds between nightstands, sofas, kitchens, bathrooms, office pods.
TEST_CASE(room_programs_furnish_every_home_and_office_room) {
    const FurnitureLibrary withPrograms = shippedFurniture(true), without = shippedFurniture(false);
    CHECK(withPrograms.programCount() >= 10);
    CHECK(without.programCount() == 0);
    const FurnitureLibrary saved = FurnitureLibrary::global();
    struct Case { Poly2 plan; bool residential, curtain; int core; };
    const Case cases[] = {
        {{{0, 0}, {40, 0}, {40, 30}, {0, 30}}, false, true, 0},
        {{{0, 0}, {34, 0}, {34, 26}, {0, 26}}, false, false, 0},
        {{{0, 0}, {44, 0}, {44, 32}, {0, 32}}, true, false, 0},
        {{{0, 0}, {36, 0}, {36, 36}, {0, 36}}, true, false, 0},
        {{{0, 0}, {26, 0}, {26, 15}, {0, 15}}, true, false, 1},
        {{{0, 0}, {30, 0}, {30, 10.5}, {0, 10.5}}, true, false, 1},
    };
    std::array<int, kPieceCount> with{}, bare{};
    int plans = 0;
    for (const Case& c : cases)
        for (uint32_t seed = 1; seed <= 6; ++seed)
            for (int floor : {1, 3, 6}) {
                BuildingParams p;
                p.floors = c.core == 1 ? 5 : 20; p.curtainWall = c.curtain; p.walkableGround = true; p.openDoorway = true;
                p.seed = seed * 7919u; p.residential = c.residential; p.core = c.core;
                const std::size_t entrance = entranceEdgeFor(c.plan, p);
                const Real inset = std::max(p.wallThickness, Real(0.55));
                RoomPlan rp;
                if (c.core == 1) {
                    const InteriorLayout il = interiorLayout(c.plan, p, entrance);
                    if (!il.hasStair) continue;
                    rp = roomPlan(c.plan, p, coreFor(c.plan, p, entrance), il.edge, inset, std::min(floor, 4), il.well, entrance);
                } else {
                    const CorePlan core = coreFor(c.plan, p, entrance);
                    if (!core.valid) continue;
                    rp = roomPlan(c.plan, p, core, static_cast<std::size_t>(-1), inset, floor);
                }
                ++plans;
                for (int pass = 0; pass < 2; ++pass) {
                    std::vector<PlacedPiece> out;
                    RenderMesh col;
                    FurnitureLibrary::global() = pass == 0 ? withPrograms : without;
                    emitFurniture(out, &col, rp, 20.0, p.seed + floor, 22.8);
                    for (const PlacedPiece& pp : out) ++(pass == 0 ? with : bare)[pp.piece];
                }
            }
    FurnitureLibrary::global() = saved;
    auto n = [&](const std::array<int, kPieceCount>& t, Piece pc) { return t[static_cast<std::size_t>(pc)]; };
    std::printf("    [programs] %d floors: desks %d, office chairs %d, beds %d, nightstands %d, wardrobes %d, sofas %d, "
                "sinks %d, toilets %d, pods %d, cubicles %d, meeting tables %d\n", plans, n(with, Piece::Desk),
                n(with, Piece::OfficeChair), n(with, Piece::Bed), n(with, Piece::Nightstand), n(with, Piece::Wardrobe),
                n(with, Piece::Sofa), n(with, Piece::KitchenSink), n(with, Piece::Toilet), n(with, Piece::DeskPod),
                n(with, Piece::Cubicle), n(with, Piece::MeetingTable));
    CHECK(plans > 30);
    for (Piece pc : {Piece::Desk, Piece::OfficeChair, Piece::Bed, Piece::Wardrobe, Piece::Sofa, Piece::KitchenSink,
                     Piece::Toilet, Piece::Bathtub, Piece::MeetingTable})
        CHECK(n(with, pc) > 0);
    CHECK(n(with, Piece::Nightstand) == 2 * n(with, Piece::Bed));       // a nightstand each side of every bed
    CHECK(n(with, Piece::OfficeChair) == n(with, Piece::Desk) + n(with, Piece::ReceptionDesk));   // a chair at every desk
    CHECK(n(with, Piece::DeskPod) + n(with, Piece::Cubicle) > 0);
    for (Piece pc : {Piece::Desk, Piece::Bed, Piece::Sofa, Piece::Toilet, Piece::DeskPod, Piece::Cubicle})
        CHECK(n(bare, pc) == 0);                                          // the furniture is data
}

// A PICK BY TAGS chooses by what fits: bedroom storage is a nightstand in a nightstand's slot and a wardrobe in a
// wardrobe's; a new piece described with the right family and tags is picked with no code.
TEST_CASE(a_tag_pick_takes_the_largest_described_piece_that_fits_the_slot) {
    const FurnitureLibrary lib = shippedFurniture(true);
    FurnPick store;
    store.family = "storage";
    store.tags = {"bedroom"};
    CHECK(lib.pick(store, 0.5, 0.5, 1) == Piece::Nightstand);
    CHECK(lib.pick(store, 1.2, 0.6, 1) == Piece::Wardrobe);
    FurnPick seat;
    seat.family = "seating";
    seat.tags = {"living"};
    CHECK(lib.pick(seat, 2.16, 1.0, 1) == Piece::Sofa);
    CHECK(lib.pick(seat, 0.84, 0.84, 1) == Piece::LoungeChair);
    FurnPick none;
    none.family = "seating";
    none.tags = {"no_such_room"};
    CHECK(lib.pick(none, 3, 3, 1) == Piece::Count);
    // a new office chair, described: an office's chair slot now has a choice (the largest that fits)
    FurnitureLibrary more = lib;
    FurnitureAsset extra;
    extra.piece = Piece::LoungeChair;
    extra.family = "seating";
    extra.tags = {"living", "lobby", "study", "office"};
    more.set(extra);
    FurnPick office;
    office.family = "seating";
    office.tags = {"office"};
    CHECK(lib.pick(office, 0.7, 0.7, 1) == Piece::OfficeChair);
    CHECK(more.pick(office, 0.9, 0.9, 1) == Piece::LoungeChair);   // it fits, and it is bigger
    CHECK(more.pick(office, 0.7, 0.7, 1) == Piece::OfficeChair);   // ...but not in a desk chair's slot
}

// THE UNIVERSITY'S FLOORS (the campus, milestone 1): a teaching hall's ground floor holds lecture halls and
// classrooms, its upper floors classrooms and labs; the library a reading room below and stacks above; a residence
// hall dorm rooms, shared baths and a lounge -- every room off the corridor (the floor walks from the stair), each
// furnished by its program: tiered lecture rows, pupils' desks, lab benches, reading tables, back-to-back stacks,
// two single beds a dorm room.
TEST_CASE(campus_buildings_have_their_rooms_and_furniture) {
    UseShippedFurniture shipped;
    const Poly2 plans[3] = {{{0, 0}, {52, 0}, {52, 22}, {0, 22}}, {{0, 0}, {40, 0}, {40, 24}, {0, 24}},
                            {{0, 0}, {54, 0}, {54, 15}, {0, 15}}};
    std::array<int, kPieceCount> n{};
    std::array<int, 32> kinds{};
    for (int campus = 1; campus <= 3; ++campus)
        for (int storey : {0, 1, 2}) {
            BuildingParams p;
            p.floors = 4; p.campus = static_cast<uint8_t>(campus); p.core = 1; p.walkableGround = true;
            p.openDoorway = true; p.seed = 41;
            const Poly2& plan = plans[campus - 1];
            const std::size_t entrance = entranceEdgeFor(plan, p);
            const InteriorLayout il = interiorLayout(plan, p, entrance);
            CHECK(il.hasStair);
            if (!il.hasStair) continue;
            const Real inset = std::max(p.wallThickness, Real(0.55));
            const RoomPlan rp = roomPlan(plan, p, coreFor(plan, p, entrance), il.edge, inset, storey, il.well, entrance,
                                         il.stairFoot);
            CHECK(!rp.rooms.empty());
            // every room is reached from the floor just before the stair's first step, and that floor -- and the
            // flight's foot -- is the stair hall's, not inside a room (Glenn: "The stairwells are inaccessible": the hall
            // was cut round the well only, the hole over the flight's top, and the foot stood behind its wall)
            CHECK(floorIsWalkable(rp, plan, il.stairFoot - il.stairDir * 1.0, il.well));
            for (const Vec2& q : {il.stairFoot, il.stairFoot - il.stairDir * 1.0})
                for (const Room& r : rp.rooms) {
                    Poly2 rr = r.rect;
                    ensureCCW(rr);
                    if (pointInPolygon(rr, q)) {
                        std::printf("    stair foot inside a room: campus %d storey %d at (%.2f, %.2f)\n", campus, storey, q.x, q.y);
                        CHECK(false);
                    }
                }
            for (const Room& r : rp.rooms) ++kinds[static_cast<std::size_t>(r.kind)];
            std::vector<PlacedPiece> out;
            RenderMesh col;
            emitFurniture(out, &col, rp, 0.0, p.seed + storey, 3.2);
            for (const PlacedPiece& pp : out) ++n[pp.piece];
        }
    auto k = [&](RoomKind r) { return kinds[static_cast<std::size_t>(r)]; };
    auto c = [&](Piece pc) { return n[static_cast<std::size_t>(pc)]; };
    std::printf("    [campus] rooms: lecture %d, classroom %d, lab %d, reading %d, stacks %d, dorm %d, bath %d, lounge %d | "
                "pieces: lecture rows %d, school desks %d, lab benches %d, reading tables %d, bookcases %d, single beds %d\n",
                k(RoomKind::Lecture), k(RoomKind::Classroom), k(RoomKind::Lab), k(RoomKind::Reading), k(RoomKind::Stacks),
                k(RoomKind::Dorm), k(RoomKind::Bath), k(RoomKind::Living), c(Piece::LectureRow), c(Piece::SchoolDesk),
                c(Piece::LabBench), c(Piece::ReadingTable), c(Piece::Bookcase), c(Piece::SingleBed));
    for (RoomKind r : {RoomKind::Lecture, RoomKind::Classroom, RoomKind::Lab, RoomKind::Reading, RoomKind::Stacks,
                       RoomKind::Dorm, RoomKind::Bath, RoomKind::Living})
        CHECK(k(r) > 0);
    for (Piece pc : {Piece::LectureRow, Piece::Lectern, Piece::SchoolDesk, Piece::SchoolChair, Piece::LabBench,
                     Piece::ReadingTable, Piece::Bookcase, Piece::SingleBed, Piece::Whiteboard})
        CHECK(c(pc) > 0);
    CHECK(c(Piece::SchoolChair) >= c(Piece::SchoolDesk));   // every desk has its chair (dorm desks too)
    CHECK(c(Piece::SingleBed) == 2 * k(RoomKind::Dorm));    // two singles a dorm room
}

// WALL ART ON WALLS (Glenn, 2026-10-03: "mirrors on the wall and paintings ... need to be hung in places on the wall
// that have room. Some of them in the residential areas I noticed were hung in doorways or over windows"): every
// picture and whiteboard is against an interior wall -- never the window wall (the facade, not a partition) or an
// open side -- and clear of that wall's doorway.
TEST_CASE(pictures_hang_on_real_walls_clear_of_doors) {
    UseShippedFurniture shipped;
    int hung = 0, offWall = 0, inDoor = 0;
    auto check = [&](const RoomPlan& rp, const std::vector<PlacedPiece>& out) {
        for (const PlacedPiece& pp : out) {
            if (pp.piece != static_cast<uint8_t>(Piece::Picture) && pp.piece != static_cast<uint8_t>(Piece::Whiteboard)) continue;
            ++hung;
            const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
            bool against = false, door = false;
            for (const RoomWall& w : rp.walls) {
                const Vec2 d = w.b - w.a;
                const Real L = d.length();
                if (L < 1e-6) continue;
                const Real t = dot(at - w.a, d) / (L * L);
                if (t < -0.01 || t > 1.01) continue;
                const Vec2 q = w.a + d * t;
                if ((q - at).length() > kRoomWallT * 0.5 + 0.08) continue;
                against = true;
                if (w.doorAt >= 0 && std::fabs(t - w.doorAt) * L < kRoomDoorW * 0.5 + 0.3) door = true;
            }
            if (!against) ++offWall;
            if (door) ++inDoor;
        }
    };
    for (const Poly2& plan : {Poly2{{0, 0}, {44, 0}, {44, 32}, {0, 32}}, Poly2{{0, 0}, {36, 0}, {36, 36}, {0, 36}}})
        for (uint32_t seed = 1; seed <= 6; ++seed) {
            BuildingParams p;
            p.floors = 24; p.walkableGround = true; p.openDoorway = true; p.seed = seed * 31; p.residential = true;
            const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
            if (!core.valid) continue;
            const RoomPlan rp = roomPlan(plan, p, core, static_cast<std::size_t>(-1), std::max(p.wallThickness, Real(0.55)), 4);
            std::vector<PlacedPiece> out;
            RenderMesh col;
            emitFurniture(out, &col, rp, 0.0, p.seed, 3.0);
            check(rp, out);
        }
    for (int campus = 1; campus <= 3; ++campus)
        for (int storey : {0, 1}) {
            const Poly2 plan = {{0, 0}, {52, 0}, {52, 22}, {0, 22}};
            BuildingParams p;
            p.floors = 4; p.campus = static_cast<uint8_t>(campus); p.core = 1; p.walkableGround = true; p.openDoorway = true; p.seed = 9;
            const std::size_t e = entranceEdgeFor(plan, p);
            const InteriorLayout il = interiorLayout(plan, p, e);
            if (!il.hasStair) continue;
            const RoomPlan rp = roomPlan(plan, p, coreFor(plan, p, e), il.edge, std::max(p.wallThickness, Real(0.55)), storey,
                                         il.well, e, il.stairFoot);
            std::vector<PlacedPiece> out;
            RenderMesh col;
            emitFurniture(out, &col, rp, 0.0, p.seed, 3.2);
            check(rp, out);
        }
    std::printf("    [wall art] %d hung, %d off any wall, %d in a doorway\n", hung, offWall, inDoor);
    CHECK(hung > 20);
    CHECK(offWall == 0);
    CHECK(inDoor == 0);
}

// PEOPLE INSIDE (campus milestone 4): the teaching hall's lecture hall seats its students on the rows -- each body on
// a seat of its row's riser (the spot raised by its tier), no two on one seat, a lecturer standing at the lectern
// facing the hall -- and at night the residence hall's students lie in its beds.
#include "../src/engine/interior_occupants.h"
TEST_CASE(students_sit_in_the_lecture_rows_and_sleep_in_the_dorm_beds) {
    UseShippedFurniture shipped;
    const FurnitureLibrary& lib = FurnitureLibrary::global();
    auto setFor = [&](int campus, const Poly2& plan, int storey) {
        BuildingParams p;
        p.floors = 4; p.campus = static_cast<uint8_t>(campus); p.core = 1; p.walkableGround = true;
        p.openDoorway = true; p.seed = 41;
        const std::size_t entrance = entranceEdgeFor(plan, p);
        const InteriorLayout il = interiorLayout(plan, p, entrance);
        const Real inset = std::max(p.wallThickness, Real(0.55));
        const RoomPlan rp = roomPlan(plan, p, coreFor(plan, p, entrance), il.edge, inset, storey, il.well, entrance,
                                     il.stairFoot);
        std::vector<PlacedPiece> out;
        RenderMesh col;
        emitFurniture(out, &col, rp, 0.0, p.seed + storey, 3.2);
        Interactables set;
        for (const PlacedPiece& pp : out)
            if (lib.interactive(static_cast<Piece>(pp.piece)))
                set.pieces.push_back({pp.piece, interactXform(lib.find(static_cast<Piece>(pp.piece)), pp.xform, pp.variant), 0});
        return set;
    };
    // the teaching hall's ground storey: the lecture hall is there (campus_buildings_have_their_rooms_and_furniture)
    Interactables hall = setFor(1, {{0, 0}, {52, 0}, {52, 22}, {0, 22}}, 0);
    OccupantPlan cls;
    cls.people = 40; cls.lecturer = true; cls.seed = 7;
    const std::vector<Occupant> occ = planOccupants(hall, lib, cls);
    int rowSeats = 0, raised = 0, standing = 0, shared = 0, faceHall = 0;
    std::vector<uint32_t> held(hall.pieces.size(), 0);
    for (const Occupant& o : occ) {
        if (held[o.piece] & o.spots) ++shared;
        held[o.piece] |= o.spots;
        const InteractPiece& ip = hall.pieces[o.piece];
        const Vec3 at(o.at.m[0][3], o.at.m[1][3], o.at.m[2][3]);
        if (static_cast<Piece>(ip.piece) == Piece::LectureRow) {
            ++rowSeats;
            const Real seatTop = piecePoint(ip.xform, Vec3(0, 0.48, 0)).y;   // already raised by the tier
            if (std::fabs(at.y - seatTop) < 1e-6 && at.y > 0.6) ++raised;
            // seated facing +z of the row (toward the board), like the row's own seats
            const Vec3 f = pieceDir(o.at, Vec3(0, 0, 1)), rowF = pieceDir(ip.xform, Vec3(0, 0, 1));
            if (f.x * rowF.x + f.z * rowF.z > 0.95) ++faceHall;
        }
        if (o.pose == Occupant::Pose::Stand) {
            ++standing;
            CHECK(static_cast<Piece>(ip.piece) == Piece::Lectern);
        }
    }
    std::printf("    [class] %zu placed of 40: %d in the lecture rows (%d up on a tier, %d facing the board), %d at the lectern, "
                "%d shared\n", occ.size(), rowSeats, raised, faceHall, standing, shared);
    CHECK(static_cast<int>(occ.size()) == 40);
    CHECK(standing == 1);
    CHECK(shared == 0);
    CHECK(rowSeats >= 20);
    CHECK(raised >= 5);
    CHECK(faceHall == rowSeats);
    // a seat the player holds is left alone
    {
        Interactables h2 = hall;
        for (InteractPiece& ip : h2.pieces) ip.taken = ~0u;
        CHECK(planOccupants(h2, lib, cls).empty());
    }

    // the residence hall at night: in bed, on their backs, along the bed
    Interactables dorm = setFor(3, {{0, 0}, {54, 0}, {54, 15}, {0, 15}}, 1);
    OccupantPlan night;
    night.people = 12; night.night = true; night.seed = 3;
    int lying = 0, alongBed = 0, onMattress = 0;
    for (const Occupant& o : planOccupants(dorm, lib, night)) {
        if (o.pose != Occupant::Pose::Lie) continue;
        ++lying;
        const InteractPiece& ip = dorm.pieces[o.piece];
        CHECK(static_cast<Piece>(ip.piece) == Piece::SingleBed);
        // the body's up (+y) runs along the bed toward the pillow (-z); the whole 1.8 m body on the mattress (z 0.07
        // to 2.01), its back on it
        const Vec3 up = pieceDir(o.at, Vec3(0, 1, 0)), toPillow = pieceDir(ip.xform, Vec3(0, 0, -1));
        if (up.x * toPillow.x + up.y * toPillow.y + up.z * toPillow.z > 0.99) ++alongBed;
        const Vec3 crown = piecePoint(o.at, Vec3(0, 0.9, 0)), soles = piecePoint(o.at, Vec3(0, -0.9, 0));
        const Mat4 inv = ip.xform.inverse();
        const Vec3 cz = piecePoint(inv, crown), sz = piecePoint(inv, soles);
        const Vec3 mid(o.at.m[0][3], o.at.m[1][3], o.at.m[2][3]);
        if (cz.z > 0.07 && sz.z < 2.01 && std::fabs(mid.y - (piecePoint(ip.xform, Vec3(0, 0.5, 0)).y + 0.13)) < 1e-6)
            ++onMattress;
    }
    std::printf("    [dorm] %d asleep, %d along the bed, %d on the mattress\n", lying, alongBed, onMattress);
    CHECK(lying == 12);
    CHECK(alongBed == lying);
    CHECK(onMattress == lying);
}

// A HALL WITH A NOTCH (Glenn: "one of the school buildings has the auditorium and classrooms clipped through the back
// wall and half outside"): metro_v2_test's second teaching hall is a 36 x 10.6 m bar with a 12 m wide, 3.7 m deep
// recess in one long side. Every room and every piece of furniture stays inside the plan, and it still has its
// lecture hall.
TEST_CASE(a_campus_hall_with_a_notch_keeps_its_rooms_inside) {
    UseShippedFurniture shipped;
    Poly2 plan = {{-576.20, 403.99}, {-546.73, 425.65}, {-552.98, 434.16}, {-554.99, 432.68},
                  {-552.81, 429.70}, {-562.66, 422.46}, {-564.85, 425.44}, {-582.46, 412.50}};
    for (Vec2& q : plan) q = q - Vec2(-564.0, 419.0);
    Poly2 ccw = plan;
    ensureCCW(ccw);
    // inside, or within a wall's thickness of the plan's edge
    auto inside = [&](const Vec2& q) {
        if (pointInPolygon(ccw, q)) return true;
        for (std::size_t i = 0; i < ccw.size(); ++i) {
            const Vec2 a = ccw[i], b = ccw[(i + 1) % ccw.size()], ab = b - a;
            const Real t = std::max(Real(0), std::min(Real(1), dot(q - a, ab) / ab.lengthSquared()));
            if ((a + ab * t - q).length() < 0.15) return true;
        }
        return false;
    };
    int rooms = 0, roomsOut = 0, pieces = 0, piecesOut = 0, lecture = 0;
    for (int storey : {0, 1, 2}) {
        BuildingParams p;
        p.floors = 3; p.campus = 1; p.core = 1; p.walkableGround = true; p.openDoorway = true; p.seed = 41;
        const std::size_t entrance = entranceEdgeFor(plan, p);
        const InteriorLayout il = interiorLayout(plan, p, entrance);
        if (!il.hasStair) continue;
        const Real inset = std::max(p.wallThickness, Real(0.55));
        const RoomPlan rp = roomPlan(plan, p, coreFor(plan, p, entrance), il.edge, inset, storey, il.well, entrance,
                                     il.stairFoot);
        for (const Room& r : rp.rooms) {
            ++rooms;
            lecture += r.kind == RoomKind::Lecture;
            bool out = false;
            for (const Vec2& q : r.rect) out = out || !inside(q);
            for (std::size_t i = 0; i < r.rect.size() && !out; ++i)   // the notch's corners never inside a room
                for (const Vec2& v : ccw) {
                    Poly2 rr = r.rect;
                    ensureCCW(rr);
                    const Vec2 c = centroid(rr);
                    if (pointInPolygon(rr, v + (c - v) * 0.02)) out = true;
                }
            roomsOut += out;
        }
        std::vector<PlacedPiece> outP;
        RenderMesh col;
        emitFurniture(outP, &col, rp, 0.0, p.seed + storey, 3.2);
        for (const PlacedPiece& pp : outP) {
            ++pieces;
            const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
            piecesOut += !inside(at);
        }
    }
    std::printf("    [notch] %d rooms (%d lecture), %d reaching outside; %d pieces, %d outside\n", rooms, lecture, roomsOut,
                pieces, piecesOut);
    CHECK(rooms >= 3);   // the notch sits near one end: one long run a floor, the lecture hall on the ground
    CHECK(roomsOut == 0);
    CHECK(piecesOut == 0);
    CHECK(lecture == 1);
}

// THE QUAD DOOR (Glenn: the campus doors): a campus hall's back door, in the middle of its rear wall, opens onto a
// passage from the corridor -- never into a classroom, a dorm room or the stacks -- and from it the stair is walked to.
TEST_CASE(a_campus_halls_quad_door_opens_onto_a_passage_to_the_corridor) {
    UseShippedFurniture shipped;
    // three plain halls (their stair hall often lands mid-floor, where the door is anyway) and two real ones off
    // metro_v2_test: the teaching hall with a recess in one long side, and a hall trimmed at one end
    Poly2 notched = {{-576.20, 403.99}, {-546.73, 425.65}, {-552.98, 434.16}, {-554.99, 432.68},
                     {-552.81, 429.70}, {-562.66, 422.46}, {-564.85, 425.44}, {-582.46, 412.50}};
    for (Vec2& q : notched) q = q - Vec2(-564.0, 419.0);
    const Poly2 plans[5] = {{{0, 0}, {52, 0}, {52, 11}, {0, 11}}, {{0, 0}, {40, 0}, {40, 24}, {0, 24}},
                            {{0, 0}, {54, 0}, {54, 15}, {0, 15}}, notched,
                            {{0, 0}, {40, 0}, {44, 4}, {44, 11}, {0, 11}}};
    const int kinds[5] = {1, 2, 3, 1, 1};
    int checked = 0;
    for (int pi = 0; pi < 5; ++pi)
    for (int face : {-1, 1}) {   // the street on one side (the quad on the other), and the other way round
        const int campus = kinds[pi];
        BuildingParams p;
        p.floors = 4; p.campus = static_cast<uint8_t>(campus); p.core = 1; p.walkableGround = true;
        p.openDoorway = true; p.seed = 41; p.backDoor = true;
        p.faceDir = Vec3(0, 0, static_cast<Real>(face));
        const Poly2& plan = plans[pi];
        const std::size_t entrance = entranceEdgeFor(plan, p);
        const InteriorLayout il = interiorLayout(plan, p, entrance);
        CHECK(il.hasStair);
        if (!il.hasStair) continue;
        const Real inset = std::max(p.wallThickness, Real(0.55));
        const RoomPlan rp = roomPlan(plan, p, coreFor(plan, p, entrance), il.edge, inset, 0, il.well, entrance, il.stairFoot);
        if (rp.rooms.empty()) continue;   // an open floor (no campus plan fits): nothing to open into
        Vec2 mid, outward;
        CHECK(campusQuadDoor(plan, p, entrance, mid, outward));
        const Vec2 q = mid - outward * (inset + 0.6);   // a step inside the door
        // and the building really has it: a "backdoor" attach on that wall, at that spot
        const BuildingMesh bm = growPlanBuilding(plan, p, 0.0);
        bool door = false;
        for (const AttachPoint& ap : bm.attaches)
            if (ap.tag == "backdoor" && (Vec2(ap.position.x, ap.position.z) - mid).length() < 0.6) door = true;
        if (!door) {
            std::printf("    plan %d face %d: no back door grown at (%.1f, %.1f); attaches:", pi, face, mid.x, mid.y);
            for (const AttachPoint& ap : bm.attaches) std::printf(" %s(%.1f,%.1f)", ap.tag.c_str(), ap.position.x, ap.position.z);
            std::printf("\n");
        }
        CHECK(door);
        bool inRoom = false;
        for (const Room& r : rp.rooms) {
            Poly2 rr = r.rect;
            ensureCCW(rr);
            if (pointInPolygon(rr, q)) { inRoom = true; std::printf("    plan %d face %d: the quad door opens into a room (kind %d)\n", pi, face, static_cast<int>(r.kind)); }
        }
        CHECK(!inRoom);
        CHECK(floorIsWalkable(rp, plan, q, il.well));
        ++checked;
    }
    std::printf("    [quad door] %d halls checked\n", checked);
    CHECK(checked >= 8);
}

// THE LABS (behaviour plan, step 3): a teaching hall's labs are worked standing -- students at the benches' fronts,
// facing them, on their own spots -- like the lecture rows are sat in.
TEST_CASE(students_stand_at_the_lab_benches) {
    UseShippedFurniture shipped;
    const FurnitureLibrary& lib = FurnitureLibrary::global();
    // the teaching hall's upper storey has labs (campus_buildings_have_their_rooms_and_furniture)
    const Poly2 plan = {{0, 0}, {52, 0}, {52, 22}, {0, 22}};
    int standing = 0, atBench = 0, facing = 0, benches = 0;
    for (int storey : {1, 2}) {
        BuildingParams p;
        p.floors = 4; p.campus = 1; p.core = 1; p.walkableGround = true; p.openDoorway = true; p.seed = 41;
        const std::size_t entrance = entranceEdgeFor(plan, p);
        const InteriorLayout il = interiorLayout(plan, p, entrance);
        const Real inset = std::max(p.wallThickness, Real(0.55));
        const RoomPlan rp = roomPlan(plan, p, coreFor(plan, p, entrance), il.edge, inset, storey, il.well, entrance, il.stairFoot);
        std::vector<PlacedPiece> out;
        RenderMesh col;
        emitFurniture(out, &col, rp, 0.0, p.seed + storey, 3.2);
        Interactables set;
        for (const PlacedPiece& pp : out)
            if (lib.interactive(static_cast<Piece>(pp.piece)))
                set.pieces.push_back({pp.piece, interactXform(lib.find(static_cast<Piece>(pp.piece)), pp.xform, pp.variant), 0});
        for (const InteractPiece& ip : set.pieces) benches += static_cast<Piece>(ip.piece) == Piece::LabBench;
        OccupantPlan cls;
        cls.people = 400; cls.seed = 5;   // more than the storey's places: every one is taken
        for (const Occupant& o : planOccupants(set, lib, cls)) {
            if (o.pose != Occupant::Pose::Stand) continue;
            ++standing;
            const InteractPiece& ip = set.pieces[o.piece];
            if (static_cast<Piece>(ip.piece) != Piece::LabBench) continue;
            ++atBench;
            // in front of the bench (its +z), facing it (-z)
            const Vec3 at(o.at.m[0][3], o.at.m[1][3], o.at.m[2][3]);
            const Mat4 inv = ip.xform.inverse();
            const Vec3 local = piecePoint(inv, at);
            const Vec3 f = pieceDir(inv, pieceDir(o.at, Vec3(0, 0, 1)));
            if (local.z > 0.75 && f.z < -0.9) ++facing;
        }
    }
    std::printf("    [labs] %d benches, %d standing, %d at a bench, %d in front facing it\n", benches, standing, atBench, facing);
    CHECK(benches > 0);
    CHECK(atBench == benches * 3);
    CHECK(facing == atBench);
}

// EVERY TRADE'S SHOP LOOKS LIKE ITS TRADE (storefronts plan, stage 5; Glenn: "What about the interior does it look
// like a shop would look?"). Storefront buildings across seeds until every trade (trades.h) has turned up; each
// furniture piece of the ground floor is counted to the shop unit whose stretch of the facade it stands behind. Prints
// each trade's fit-out; holds that every trade's shop has something in it, and that the ones that serve have a counter.
// A TRADE'S ROOM IS FINISHED AS ONE (Glenn: "white walls for a bar? Yikes"): a bar's and a club's walls, floor and
// ceiling -- the room's interior surfaces, inside its rectangle -- are dark, a grocer's light; none of them is the
// building's own field paint.
// A TRADE TAKES ITS OWN WIDTH, A STREET ITS OWN MIX (~/.claude/plans/nightlife-and-malls.md): every unit is within
// its trade's bays (a club 3-5), a residential street has no clubs, a nightlife strip is mostly bars, restaurants
// and clubs.
TEST_CASE(shop_units_take_their_trades_width_and_their_streets_mix) {
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 16}, {0, 16}};
    std::map<int, std::map<int, int>> byMix;   // mix -> trade -> units
    int outOfRange = 0;
    for (uint8_t mixK : {uint8_t(0), uint8_t(1), uint8_t(2), uint8_t(3)})
        for (uint32_t seed = 1; seed < 160; ++seed) {
            BuildingParams p;
            p.floors = 2; p.groundRetail = true; p.walkableGround = true; p.openDoorway = true; p.seed = seed;
            p.shopMix = mixK;
            for (const ShopFront& f : shopFrontsOf(plan, p)) {
                ++byMix[mixK][f.trade];
                const TradeInfo* t = tradeById(f.trade);
                const int bays = static_cast<int>(std::lround((f.b - f.a).length() / p.bayWidth));
                if (t && (bays < t->minBays || bays > t->maxBays + 1)) ++outOfRange;   // (+1: a lone end bay joins)
            }
        }
    auto share = [&](int m, std::initializer_list<int> ids) {
        int n = 0, all = 0;
        for (const auto& kv : byMix[m]) { all += kv.second; for (int id : ids) if (kv.first == id) n += kv.second; }
        return all ? double(n) / all : 0.0;
    };
    std::printf("    [mix] everyday nightlife share %.2f, nightlife %.2f, mall boutiques+electronics %.2f; clubs: %d everyday, "
                "%d nightlife; out of range %d\n", share(0, {13, 14, 15}), share(2, {13, 14, 15}), share(3, {2, 4}),
                byMix[0][15], byMix[2][15], outOfRange);
    CHECK(byMix[0][15] == 0);
    CHECK(byMix[2][15] > 0);
    CHECK(share(2, {13, 14, 15}) > 0.6);
    CHECK(share(0, {13, 14, 15}) < 0.3);
    CHECK(share(3, {2, 4}) > 0.3);
    CHECK(outOfRange == 0);
}

// THE INDOOR MALL (~/.claude/plans/nightlife-and-malls.md stage 2): its units front the concourse (indoors, with
// signs' fascias), each furnished as its trade; the anchor is stocked and the food court has its tables.
TEST_CASE(an_indoor_mall_is_a_concourse_of_shops) {
    BuildingRecipe rec = architectMall(1234u);
    BuildingParams p = rec.params;
    p.faceDir = Vec3(0, 0, -1);
    const Poly2 plan = {{0, 0}, {120, 0}, {120, 48}, {0, 48}};   // the front (z = 0) faces -z
    const std::vector<ShopFront> fronts = shopFrontsOf(plan, p);
    const BuildingMesh in = growInterior(plan, p, 3.0, nullptr, 0, 1);
    int furnished = 0, stocked = 0, tables = 0;
    for (const ShopFront& f : fronts) {
        CHECK(f.indoor);
        CHECK(f.fasciaY1 > f.fasciaY0 + 0.2);
        const Vec2 d = normalize(f.b - f.a);
        const Real len = (f.b - f.a).length();
        bool any = false;
        for (const PlacedPiece& pp : in.furniture) {
            if (pp.piece == static_cast<uint8_t>(Piece::CeilingLight)) continue;
            const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
            const Real along = dot(at - f.a, d), depth = dot(f.a - at, f.n);
            if (along > 0.2 && along < len - 0.2 && depth > 0 && depth < 14.0) { any = true; break; }
        }
        if (any) ++furnished;
    }
    for (const PlacedPiece& pp : in.furniture) {
        const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
        if (at.x < 30 && (pp.piece == static_cast<uint8_t>(Piece::Gondola) || pp.piece == static_cast<uint8_t>(Piece::ClothesRack))) ++stocked;
        if (at.x > 92 && pp.piece == static_cast<uint8_t>(Piece::BistroTable)) ++tables;
    }
    std::printf("    [mall] %zu units, %d furnished; anchor %d pieces of stock, food court %d tables\n", fronts.size(), furnished,
                stocked, tables);
    CHECK(fronts.size() >= 8);
    CHECK(furnished >= static_cast<int>(fronts.size()) - 1);
    CHECK(stocked > 4);
    CHECK(tables > 4);
}

// THE PARKING GARAGE YOU CAN DRIVE INTO (~/.claude/plans/nightlife-and-malls.md stage 4; Glenn: "make those parking
// structures actually enterable with ramps for cars to park there"): cast straight down through its collider. Every
// level is solid deck; every ramp starts flush with its deck and ends flush with the next; over every ramp a storey's
// headroom; every stall stands on deck; and the portal is open (a car fits through at ground level).
namespace {
// the surfaces straight under (x, z), highest first
std::vector<Real> surfacesAt(const std::vector<Vec3>& v, const std::vector<uint32_t>& idx, Real x, Real z) {
    std::vector<Real> hits;
    for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
        const Vec3 &a = v[idx[i]], &b = v[idx[i + 1]], &c = v[idx[i + 2]];
        const Real d = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
        if (std::fabs(d) < 1e-9) continue;   // a vertical face
        const Real l1 = ((b.z - c.z) * (x - c.x) + (c.x - b.x) * (z - c.z)) / d;
        const Real l2 = ((c.z - a.z) * (x - c.x) + (a.x - c.x) * (z - c.z)) / d;
        const Real l3 = 1 - l1 - l2;
        if (l1 < -1e-6 || l2 < -1e-6 || l3 < -1e-6) continue;
        hits.push_back(l1 * a.y + l2 * b.y + l3 * c.y);
    }
    std::sort(hits.rbegin(), hits.rend());
    hits.erase(std::unique(hits.begin(), hits.end(), [](Real p, Real q) { return std::fabs(p - q) < 0.02; }), hits.end());
    return hits;
}
}  // namespace

TEST_CASE(a_parking_garage_can_be_driven_to_the_roof) {
    BuildingRecipe rec;
    CHECK(architectRecipeByName("parking_garage", 7u, 0.3, 32.0, 1600.0, rec));
    BuildingParams p = rec.params;
    p.floors = 3;
    p.faceDir = Vec3(0, 0, -1);
    const Poly2 plan = {{0, 0}, {52, 0}, {52, 32}, {0, 32}};   // the street along z = 0
    const Real baseY = 10.0;
    std::vector<Vec3> v;
    std::vector<uint32_t> idx;
    CHECK(garageColliderOf(plan, p, baseY, v, idx));
    const std::vector<ParkingStall> stalls = garageStallsOf(plan, p);
    Vec2 portal, outDir;
    CHECK(garagePortalOf(plan, p, portal, outDir));
    std::printf("    [garage] %zu collider tris, %zu stalls, portal at %.1f %.1f facing %.2f %.2f\n", idx.size() / 3,
                stalls.size(), portal.x, portal.y, outDir.x, outDir.y);
    CHECK(stalls.size() > 40);
    CHECK(outDir.y < -0.9);   // out to the street
    // every stall on a deck at its height
    int floating = 0;
    for (const ParkingStall& st : stalls) {
        bool on = false;
        for (Real h : surfacesAt(v, idx, st.at.x, st.at.y)) on = on || std::fabs(h - (baseY + st.y)) < 0.06;
        if (!on) ++floating;
    }
    CHECK(floating == 0);
    // THE DRIVE: through the portal, then up: walk the ramp strip's centre line (along x) at every level and check
    // the surface climbs continuously from deck to deck with headroom above it
    const Real gh = p.groundHeight, fh = p.floorHeight;
    std::vector<Real> levels = {0.03};
    for (int k = 0; k <= p.floors; ++k) levels.push_back(gh + k * fh);
    // find the strip: the column of x where, mid-garage, there is a surface between level 0 and 1
    int ramps = 0, cramped = 0, gaps = 0;
    for (Real z = 1.0; z < 31.0; z += 0.5) {
        const std::vector<Real> mid = surfacesAt(v, idx, 26.0, z);
        bool between = false;
        for (Real h : mid) between = between || (h > baseY + 0.5 && h < baseY + gh - 0.5);
        if (!between) continue;
        // a ramp line at this z: step along x and follow each ramp up
        // follow the surface along +x from each deck: a car's wheels climb at most 0.35 m between samples
        for (std::size_t k = 0; k + 1 < levels.size(); ++k) {
            Real prev = baseY + levels[k];
            bool started = false;
            for (Real x = 0.5; x < 51.5; x += 0.5) {
                Real next = -1;
                for (Real h : surfacesAt(v, idx, x, z))
                    if (h >= prev - 0.05 && h <= prev + 0.35) next = std::max(next, h);
                if (next < 0) { if (started) ++gaps; continue; }
                if (next > prev + 0.02) started = true;
                if (next > baseY + levels[k] + 0.2 && next < baseY + levels[k + 1] - 0.2)   // on the ramp: its headroom
                    for (Real h : surfacesAt(v, idx, x, z))
                        if (h > next + 0.3 && h - next < 2.2) ++cramped;
                prev = next;
            }
            if (std::fabs(prev - (baseY + levels[k + 1])) < 0.06) ++ramps;
        }
        break;
    }
    std::printf("    [garage] ramps climbing deck to deck: %d of %zu; steps %d; cramped samples %d\n", ramps,
                levels.size() - 1, gaps, cramped);
    CHECK(ramps == static_cast<int>(levels.size()) - 1);
    CHECK(gaps == 0);
    CHECK(cramped == 0);
    // the portal: open at car height (no wall within 1.0 m above the ground there)
    int blocked = 0;
    for (std::size_t i = 0; i + 2 < idx.size(); i += 3) {
        const Vec3 &a = v[idx[i]], &b = v[idx[i + 1]], &c = v[idx[i + 2]];
        const Vec3 cen = (a + b + c) * (1.0 / 3.0);
        if ((Vec2(cen.x, cen.z) - portal).length() < 2.5 && cen.y > baseY + 0.2 && cen.y < baseY + 1.0) ++blocked;
    }
    CHECK(blocked == 0);
}

// THE AMERICAN MALL (~/.claude/plans/malls-v2-and-managed-ground.md; Glenn: "multiple entrances and exits at the
// different wings ... a two story mall with shops on the second floor ... escalators"): a cross of wings off a court.
// Shops on BOTH levels, every one signed and furnished; the upper level open over the concourses and the court; the
// escalators a walkable climb from floor to floor; a balustrade wherever the upper floor stops at an opening.
TEST_CASE(an_american_mall_has_two_levels_of_shops_and_escalators) {
    BuildingRecipe rec = architectMallWings(99u);
    BuildingParams p = rec.params;
    p.faceDir = Vec3(0, 0, -1);
    p.mallArms = 0xF;
    p.mallCourt = Vec2(0, 0);
    const Real a = 20, A = 100;
    // the cross, CCW from above (x right, z up the page is +v: away from the street at -z)
    const Poly2 plan = {{a, -a}, {A, -a}, {A, a}, {a, a}, {a, 80}, {-a, 80}, {-a, a}, {-A, a}, {-A, -a}, {-a, -a},
                        {-a, -60}, {a, -60}};
    const std::vector<ShopFront> fronts = shopFrontsOf(plan, p);
    int upper = 0;
    for (const ShopFront& f : fronts) { CHECK(f.indoor); CHECK(f.fasciaY1 > f.fasciaY0 + 0.2); if (f.fasciaY0 > 5.0) ++upper; }
    RenderMesh col;
    const BuildingMesh in = growInterior(plan, p, 0.0, &col, 0, 2);
    int furnished = 0;
    for (const ShopFront& f : fronts) {
        const Vec2 d = normalize(f.b - f.a);
        const Real len = (f.b - f.a).length();
        const Real lvl = f.fasciaY0 > 5.0 ? p.groundHeight : 0.0;
        for (const PlacedPiece& pp : in.furniture) {
            if (pp.piece == static_cast<uint8_t>(Piece::CeilingLight)) continue;
            const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
            const Real along = dot(at - f.a, d), depth = dot(f.a - at, f.n);
            if (along > 0.2 && along < len - 0.2 && depth > 0 && depth < 14.5 && std::fabs(pp.xform.m[1][3] - lvl) < 1.5) {
                ++furnished;
                break;
            }
        }
    }
    std::vector<Vec3> cv;
    std::vector<uint32_t> ci;
    for (const Vertex& v : col.vertices) cv.push_back(v.position);
    ci = col.indices;
    // the escalator: from the court floor up the +u axis at v = -2.4, the collider climbs to the upper floor
    const Real yU = p.groundHeight + 0.05;
    Real prev = 0.07;
    int steps = 0;
    bool reached = false;
    for (Real x = 7.0 - 9.6 + 0.2; x <= 7.0 + 1.0; x += 0.25) {
        Real next = -1;
        for (Real h : surfacesAt(cv, ci, x, -2.4)) if (h >= prev - 0.05 && h <= prev + 0.3) next = std::max(next, h);
        if (next < 0) { ++steps; continue; }
        prev = next;
        if (std::fabs(prev - yU) < 0.1) reached = true;
    }
    // the atria: no floor at the upper level over a concourse's middle, nor the court's
    const bool openCourt = [&] { for (Real h : surfacesAt(cv, ci, 0.0, 3.0)) if (std::fabs(h - yU) < 0.1) return false; return true; }();
    const bool openWing = [&] { for (Real h : surfacesAt(cv, ci, 40.0, 0.0)) if (std::fabs(h - yU) < 0.1) return false; return true; }();
    const bool floorBeside = [&] { for (Real h : surfacesAt(cv, ci, 40.0, 4.5)) if (std::fabs(h - yU) < 0.1) return true; return false; }();
    std::printf("    [mall] %zu shops (%d upstairs), %d furnished; escalator reaches the upper floor %d (gaps %d); court open %d, "
                "wing atrium open %d, floor beside it %d\n", fronts.size(), upper, furnished, reached ? 1 : 0, steps,
                openCourt ? 1 : 0, openWing ? 1 : 0, floorBeside ? 1 : 0);
    CHECK(fronts.size() >= 40);
    CHECK(upper >= 20);
    CHECK(furnished >= static_cast<int>(fronts.size()) - 2);
    CHECK(reached);
    CHECK(steps == 0);
    CHECK(openCourt);
    CHECK(openWing);
    CHECK(floorBeside);
}

// A JAPANESE TOWER (Glenn: "malls like this in the skyscrapers like in Japan. Also in Japan there are restaurants at the
// top of skyscrapers"): its department-store floors are fitted as shops, its top floor as restaurants and bars, and it
// carries the store's vertical board and the restaurant floor's name.
TEST_CASE(a_tower_has_a_department_store_and_restaurants_on_top) {
    BuildingRecipe rec;
    CHECK(architectRecipeByName("glass_tower", 5u, 0.9, 30.0, 900.0, rec));
    BuildingParams p = rec.params;
    p.floors = 20;
    p.storeFloors = 3;
    p.dineFloors = 1;
    p.faceDir = Vec3(0, 0, -1);
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 30}, {0, 30}};
    auto piecesOn = [&](int k) {
        std::map<std::string, int> n;
        const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, k, k + 1);
        for (const PlacedPiece& pp : in.furniture) ++n[furniturePieceName(static_cast<Piece>(pp.piece))];
        return n;
    };
    // the restaurant floor: the top of the main plate (a setback crown's floors are mostly core)
    int dineK = -1;
    for (int k = 20; k >= 1 && dineK < 0; --k) {
        const auto n = piecesOn(k);
        if (n.count("bar_counter") || n.count("kitchen_pass")) dineK = k;
    }
    std::printf("    [tower] restaurant floor at storey %d\n", dineK);
    const auto store = piecesOn(1), dine = piecesOn(std::max(dineK, 1)), office = piecesOn(10);
    if (std::getenv("RT_TOWER_DEBUG"))
        for (int k = 1; k <= 21; ++k) {
            const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, k, k + 1);
            std::printf("      storey %d: %zu pieces, %zu parts\n", k, in.furniture.size(), in.parts.size());
        }
    std::vector<MallSign> signs;
    CHECK(towerSignsOf(plan, p, signs));
    std::printf("    [tower] storey 1: clothes racks %d, counters %d; storey 20: bar counters %d, bistro tables %d, kitchen "
                "passes %d; storey 10 desks %d; %zu signs\n", store.count("clothes_rack") ? store.at("clothes_rack") : 0,
                store.count("shop_counter") ? store.at("shop_counter") : 0, dine.count("bar_counter") ? dine.at("bar_counter") : 0,
                dine.count("bistro_table") ? dine.at("bistro_table") : 0, dine.count("kitchen_pass") ? dine.at("kitchen_pass") : 0,
                office.count("desk") ? office.at("desk") : 0, signs.size());
    CHECK(store.count("clothes_rack") > 0);
    CHECK(dine.count("bar_counter") > 0 || dine.count("kitchen_pass") > 0);
    CHECK(signs.size() == 2);
}

TEST_CASE(a_bar_is_not_painted_like_an_office) {
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 14}, {0, 14}};
    int bars = 0, light = 0;
    for (uint32_t seed = 1; seed < 400 && bars < 6; ++seed) {
        BuildingParams p;
        p.floors = 3; p.groundRetail = true; p.walkableGround = true; p.openDoorway = true; p.seed = seed;
        p.residential = true; p.core = 1;
        const std::vector<ShopRoomRect> rooms = shopRoomsOf(plan, p, 5.0);
        const BuildingMesh in = growInterior(plan, p, 5.0, nullptr, 0, 1);
        for (const ShopRoomRect& r : rooms) {
            if (r.trade != 14 && r.trade != 15) continue;
            Real lum = 0;
            int n = 0;
            for (const RenderMesh& m : in.parts) {
                if (m.materialIndex != static_cast<int>(PartId::Interior)) continue;
                for (const Vertex& v : m.vertices) {
                    if (!pointInPolygon(r.rect, Vec2(v.position.x, v.position.z))) continue;
                    lum += 0.3 * v.color.x + 0.59 * v.color.y + 0.11 * v.color.z;
                    ++n;
                }
            }
            if (n < 8) continue;
            ++bars;
            if (lum / n > 0.35) ++light;
            std::printf("    [finish] %s seed %u: %d interior vertices, mean paint %.2f\n", r.trade == 14 ? "bar" : "club",
                        seed, n, lum / n);
        }
    }
    CHECK(bars > 0);
    CHECK(light == 0);
}

TEST_CASE(every_trade_is_furnished_as_its_trade) {
    const Poly2 plan = {{0, 0}, {30, 0}, {30, 14}, {0, 14}};
    std::map<int, std::map<std::string, int>> byTrade;
    std::map<int, int> units;
    for (uint32_t seed = 1; seed < 400 && static_cast<int>(units.size()) < tradeCount(); ++seed) {
        BuildingParams p;
        p.floors = 3; p.groundRetail = true; p.walkableGround = true; p.openDoorway = true; p.seed = seed;
        p.residential = true; p.core = 1;
        const std::vector<ShopFront> fronts = shopFrontsOf(plan, p);
        if (fronts.empty()) continue;
        RenderMesh col;
        // (off the datum: the shops' dice once hashed the storey's height, and the signs -- grown at 0 -- named a
        // different trade from the room behind them)
        const BuildingMesh in = growInterior(plan, p, 23.7, &col, 0, 1);
        // every room is the trade of the front it stands behind (the bug: a pharmacy's sign over a bookshop's room)
        const std::vector<ShopRoomRect> rooms = shopRoomsOf(plan, p, 23.7);
        for (const ShopRoomRect& r : rooms) {
            const Vec2 c = (r.rect[0] + r.rect[1]) * 0.5;   // the middle of its front edge (rects start there)
            const ShopFront* behind = nullptr;
            Real bestDepth = 1e9;
            for (const ShopFront& f : fronts) {
                const Vec2 d = normalize(f.b - f.a);
                const Real along = dot(c - f.a, d), depth = dot(f.a - c, f.n);
                if (along < 0 || along > (f.b - f.a).length() || depth < 0 || depth > bestDepth) continue;
                behind = &f;
                bestDepth = depth;
            }
            CHECK(behind != nullptr && behind->trade == r.trade);
        }
        for (const ShopRoomRect& r : rooms) ++units[r.trade];
        for (std::size_t i = 0; i < rooms.size(); ++i)   // no two shops share floor (a + at a corner)
            for (std::size_t j = i + 1; j < rooms.size(); ++j)
            {
                Poly2 both = rooms[i].rect, b = rooms[j].rect;
                if (signedArea(b) < 0) std::reverse(b.begin(), b.end());
                for (std::size_t k = 0; k < b.size() && both.size() >= 3; ++k) {
                    const Vec2 ed = b[(k + 1) % b.size()] - b[k], out(ed.y, -ed.x);   // CCW: outward is right
                    both = clipHalfPlane(both, out, dot(out, b[k]));
                }
                CHECK(both.size() < 3 || area(both) < 0.05);
            }
        // and furnished as that trade: each piece counts for the room it stands in
        for (const PlacedPiece& pp : in.furniture) {
            if (pp.piece == static_cast<uint8_t>(Piece::CeilingLight)) continue;
            const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
            for (const ShopRoomRect& r : rooms)
                if (pointInPolygon(r.rect, at)) { ++byTrade[r.trade][furniturePieceName(static_cast<Piece>(pp.piece))]; break; }
        }
    }
    // what makes each one read as its trade (a room with none of these is a different shop behind the sign)
    const std::map<int, std::vector<std::string>> signature = {
        {0, {"bistro_table", "shop_counter"}}, {1, {"gondola"}}, {2, {"clothes_rack"}}, {3, {"bookcase"}},
        {4, {"tv_unit"}}, {5, {"wall_shelf"}}, {6, {"display_case"}}, {13, {"kitchen_pass", "bistro_table"}},
        {14, {"bar_counter", "back_bar", "bar_stool"}}, {15, {"dj_booth", "bar_counter"}}};
    int empty = 0, noCounter = 0, unlike = 0;
    for (int ti = 0; ti < tradeCount(); ++ti) {
        const TradeInfo& t = tradeAt(ti);
        std::string line;
        int pieces = 0;
        for (const auto& kv : byTrade[t.id]) { line += " " + kv.first + "x" + std::to_string(kv.second); pieces += kv.second; }
        std::printf("    [trade] %-11s %2d units:%s\n", t.name, units[t.id], line.c_str());
        if (units[t.id] > 0 && pieces == 0) ++empty;
        if (units[t.id] > 0 && byTrade[t.id]["shop_counter"] == 0 && t.id != 1 && t.id < 13) ++noCounter;
        if (auto it = signature.find(t.id); it != signature.end() && units[t.id] > 0)
            for (const std::string& pc : it->second)
                if (byTrade[t.id][pc] == 0) { ++unlike; std::printf("    [trade] %s has no %s\n", t.name, pc.c_str()); }
    }
    CHECK(unlike == 0);
    CHECK(static_cast<int>(units.size()) == tradeCount());   // every trade turned up
    CHECK(empty == 0);
    CHECK(noCounter == 0);
}

// SHOP STAFF (Glenn, 2026-10-09: "Ulu Coffee has the open sign lit up but nobody is there manning the restaurant" /
// "Sora Sushi in the indoor mall is closed but two people are sitting in there"): a shop unit's people go on ITS
// pieces only -- its staff behind the counter (the counter's staff side, -z), its customers on its chairs, never one
// behind the counter -- and a closed unit next door with nobody in stays empty.
TEST_CASE(shop_staff_stand_behind_their_own_counter_and_nobody_sits_in_a_closed_shop) {
    UseShippedFurniture shipped;
    const FurnitureLibrary& lib = FurnitureLibrary::global();
    Interactables set;
    auto add = [&](Piece pc, Real x, Real z, int unit) {
        const Mat4 xf = Mat4::translate(x, 0, z);
        set.pieces.push_back({static_cast<uint8_t>(pc), interactXform(lib.find(pc), xf, 0), 0, static_cast<int16_t>(unit)});
    };
    add(Piece::ShopCounter, 0, 0, 0);    // the cafe: a counter, two chairs
    add(Piece::BistroChair, 0, 3, 0);
    add(Piece::BistroChair, 1.2, 3, 0);
    add(Piece::BarStool, 10, 3, 1);      // the sushi bar next door: two stools
    add(Piece::BarStool, 10.6, 3, 1);
    CHECK(set.pieces.size() == 5);
    std::vector<uint8_t> cafe{0, 1, 0}, sushi{0, 0, 1};   // [unit + 1]
    OccupantPlan open;
    open.people = 3; open.staff = 2; open.units = &cafe; open.seed = 3;
    const std::vector<Occupant> occ = planOccupants(set, lib, open);
    int behind = 0, seated = 0, elsewhere = 0;
    for (const Occupant& o : occ) {
        const InteractPiece& ip = set.pieces[o.piece];
        if (ip.unit != 0) ++elsewhere;
        if (o.pose == Occupant::Pose::Stand && static_cast<Piece>(ip.piece) == Piece::ShopCounter && o.at.m[2][3] < 0) ++behind;
        if (o.pose == Occupant::Pose::Sit && static_cast<Piece>(ip.piece) == Piece::BistroChair) ++seated;
    }
    std::printf("    [cafe] %zu placed: %d behind the counter, %d at the tables, %d in another unit\n", occ.size(), behind, seated, elsewhere);
    CHECK(occ.size() == 3);
    CHECK(behind == 2);
    CHECK(seated == 1);
    CHECK(elsewhere == 0);
    // more staff than places behind the counter: the rest are out the back, not on the customers' chairs
    OccupantPlan crew = open;
    crew.people = 5; crew.staff = 5;
    CHECK(planOccupants(set, lib, crew).size() == 2);
    // customers only: on the chairs, never behind the counter
    OccupantPlan custom = open;
    custom.people = 4; custom.staff = 0;
    for (const Occupant& o : planOccupants(set, lib, custom)) CHECK(o.pose == Occupant::Pose::Sit);
    // the closed sushi bar with nobody in it: nobody drawn there
    OccupantPlan shut;
    shut.people = 0; shut.units = &sushi;
    CHECK(planOccupants(set, lib, shut).empty());
}

// THE OUTSIDE DOORS AND WINDOWS (Glenn, 2026-10-13: "a tv sitting in front of the front doorway or a shelf in a store
// blocking the front entrance"; "a picture or mirror shouldn't be placed over a window but a desk or sofa could be
// placed by the window"). Across the city's recipes, every storey grown: no piece stands in the way in through a street,
// shop or back door, and nothing taller than low furniture stands against a window or hangs over one.
// (RT_FURNITURE_BLIND_OPENINGS=1 grows them as before, for the counts.)
TEST_CASE(furniture_keeps_out_of_the_outside_doors_and_off_the_windows) {
    UseShippedFurniture shipped;
    const char* recipes[] = {"cottage", "bungalow", "craftsman_house", "modern_house", "villa", "yard_house",
                             "oldtown_house", "walkup_homes", "apartments", "terrace_condo", "loft_block", "brick_shop",
                             "corner_shop", "commercial_block", "office_midrise", "office_slab", "hotel", "condo_tower",
                             "pencil_tower", "glass_tower", "art_deco_tower", "tower_on_base", "podium_tower",
                             "industrial_office", "civic_hall", "bank", "cinema", "strip_mall", "supermarket", "glass_slab"};
    const Real sizes[][2] = {{10, 8}, {14, 11}, {22, 14}, {30, 20}, {40, 26}};
    long pieces = 0, inDoor = 0, beforeGlass = 0, overGlass = 0, doors = 0, windows = 0;
    std::map<std::string, int> what;
    for (const char* rn : recipes)
        for (uint32_t seed = 1; seed <= 3; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                BuildingParams p = rc.params;
                p.openDoorway = true;
                const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
                const std::size_t ee = entranceEdgeFor(plan, p);
                const int top = std::min<int>(static_cast<int>(sps.size()), 4);   // the ground and the first floors
                const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, 0, top);
                for (int k = 0; k < top; ++k) {
                    const StoreyPlan& sp = sps[static_cast<std::size_t>(k)];
                    const std::vector<WallOpening> ops = storeyOpenings(sp.plan, p, k, sp.y0, sp.h, ee);
                    for (const WallOpening& o : ops) (o.door ? doors : windows)++;
                    for (const PlacedPiece& pp : in.furniture) {
                        const Real y = pp.xform.m[1][3] - sp.y0;
                        if (y > 0.1 && static_cast<Piece>(pp.piece) != Piece::Picture && static_cast<Piece>(pp.piece) != Piece::Whiteboard)
                            continue;   // on another piece (a desk's monitor): it stands on what stands on the floor
                        if (y < -0.05 || y > sp.h - 0.3) continue;   // another storey's
                        const Piece pc = static_cast<Piece>(pp.piece);
                        if (pc == Piece::CeilingLight || pc == Piece::Rug || pc == Piece::DanceFloor) continue;
                        if (pc == Piece::CinemaScreen) continue;   // (blacks out the windows behind it, by design)
                        if (FurnitureLibrary::global().find(pc) && FurnitureLibrary::global().find(pc)->family == "goods") continue;
                        ++pieces;
                        if (std::getenv("RT_OPENINGS_WHY")) ++what[std::string("all: ") + furniturePieceName(pc)];
                        const FurniturePiece& kit = furniturePiece(pc, pp.variant);
                        const Vec2 c(pp.xform.m[0][3], pp.xform.m[2][3]);
                        const Vec2 X(pp.xform.m[0][0], pp.xform.m[2][0]), Z(pp.xform.m[0][2], pp.xform.m[2][2]);
                        const Real hw = kit.size.x * 0.5, topY = y + kit.size.y;
                        for (const WallOpening& o : ops) {
                            const Vec2 d = normalize(o.b - o.a);
                            Vec2 in2(-d.y, d.x);   // into the building
                            if (!pointInPolygon(sp.plan, (o.a + o.b) * 0.5 + in2 * 0.5)) in2 = in2 * -1.0;
                            const Real len = (o.b - o.a).length();
                            // the piece's footprint in the opening's frame: along it, and in from the wall's face
                            Real s0 = 1e9, s1 = -1e9, n0 = 1e9, n1 = -1e9;
                            for (const Vec2& q : {c - X * hw, c + X * hw, c - X * hw + Z * kit.size.z, c + X * hw + Z * kit.size.z}) {
                                s0 = std::min(s0, dot(q - o.a, d)); s1 = std::max(s1, dot(q - o.a, d));
                                n0 = std::min(n0, dot(q - o.a, in2)); n1 = std::max(n1, dot(q - o.a, in2));
                            }
                            if (s1 <= 0.1 || s0 >= len - 0.1) continue;   // not across it (flush to its frame is fine)
                            if (o.door) {
                                // the leaf and a metre in: the way in
                                if (n0 < 1.0 && n1 > -0.1 && y < 0.5) { ++inDoor; ++what[std::string("door: ") + furniturePieceName(pc)];
                                    if (std::getenv("RT_OPENINGS_WHY") && inDoor < 15) std::printf("      DOOR %s %gx%g s%u k%d %s n %.2f..%.2f s %.2f..%.2f len %.2f\n", rn, sz[0], sz[1], seed, k, furniturePieceName(pc), n0, n1, s0, s1, len); }
                            } else if (n0 < 0.25 && n1 > -0.1 && topY > std::max(o.sill, Real(0.9)) + 0.3) {
                                const bool hung = pc == Piece::Picture || pc == Piece::Whiteboard;
                                ++(hung ? overGlass : beforeGlass);
                                if (std::getenv("RT_OPENINGS_WHY") && beforeGlass < 15 && !hung) std::printf("      GLASS %s %gx%g s%u k%d %s n %.2f..%.2f s %.2f..%.2f len %.2f sill %.2f top %.2f\n", rn, sz[0], sz[1], seed, k, furniturePieceName(pc), n0, n1, s0, s1, len, o.sill, topY);
                                ++what[std::string(hung ? "over glass: " : "before glass: ") + furniturePieceName(pc)];
                            }
                        }
                    }
                }
            }
    std::printf("    [openings] %ld doors, %ld windows; %ld pieces: %ld in a doorway, %ld tall before a window, %ld hung "
                "over one\n", doors, windows, pieces, inDoor, beforeGlass, overGlass);
    for (const auto& [k, n] : what) std::printf("      %4d  %s\n", n, k.c_str());
    CHECK(doors > 50 && windows > 500 && pieces > 2000);   // (the spread reaches them)
    CHECK(inDoor == 0);
    CHECK(beforeGlass == 0);
    CHECK(overGlass == 0);
}

// EVERY TOWER FLOOR IS FURNISHED (Glenn, 2026-10-13: "some floors in skyscrapers are still empty"). A setback tier keeps
// the core sized for its base, so the floor round it narrows past the ring's rooms and the office's bands; those floors
// grew bare -- 62% of tower_on_base's, a fifth of the art deco, stepped and sky exposure towers'. They are a row of
// desks at the glass now (room_plan.cpp deskRowPlan), and an office floor's open zones are furnished with or without a
// partition; a residential tower's is its residents' lounge. What may stay bare: the plant floors, and a crown whose
// core leaves only a corridor round it -- a 14 m crown on a 24 m lot round the smallest core there is (one lift, two
// stairs: 2.3-2.5 m a side). Measured: 1525 floors here, 28 bare (62% of tower_on_base's floors and a fifth of three
// other kinds' were bare before).
// PARTITIONS (Glenn, the same day: "no office partitions in the office?"): a strip deep enough for an office and the
// corridor behind it is a row of glass-fronted private offices (officeRow), so no desk row is that deep any more.
TEST_CASE(tower_floors_are_furnished_up_their_setbacks) {
    UseShippedFurniture shipped;
    const char* recipes[] = {"glass_tower", "art_deco_tower", "stepped_tower", "drum_tower", "podium_tower",
                             "pagoda_tower", "condo_tower", "sky_exposure_tower", "tower_on_base",
                             "tapered_glass_tower", "pencil_tower"};
    const Real sizes[][2] = {{24, 24}, {30, 30}, {40, 26}, {48, 34}};
    long allFloors = 0, allBare = 0, deskRows = 0, deepRows = 0;
    for (const char* rn : recipes) {
        int floors = 0, bare = 0;
        for (uint32_t seed = 1; seed <= 2; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                BuildingParams p = rc.params;
                p.openDoorway = true;
                const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
                const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
                for (int k = 1; k < static_cast<int>(sps.size()); k += 2) {   // every other floor, up every tier
                    if (mechanicalStorey(p, k)) continue;
                    ++floors;
                    const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, k, k + 1);
                    long real = 0;
                    for (const PlacedPiece& pp : in.furniture) if (pp.piece != static_cast<uint8_t>(Piece::CeilingLight)) ++real;
                    if (real == 0) ++bare;
                    const RoomPlan rp = roomPlan(sps[static_cast<std::size_t>(k)].plan, p, core, static_cast<std::size_t>(-1),
                                                 std::max(p.wallThickness, Real(0.55)), k);   // (interiorInset)
                    for (const Room& r : rp.rooms) if (r.kind == RoomKind::DeskRow) { ++deskRows; break; }
                    for (const Room& r : rp.rooms)
                        if (r.kind == RoomKind::DeskRow && r.rect.size() == 4 && (r.rect[3] - r.rect[0]).length() >= 5.0 &&
                            (r.rect[1] - r.rect[0]).length() >= 2.4)
                            ++deepRows;   // deep enough for offices (kCorridorRing + an office's 2.8 m)
                    if (real == 0 && std::getenv("RT_BARE_WHY")) {
                        const OBB2 ob = orientedBoundingBox(sps[static_cast<std::size_t>(k)].plan);
                        std::printf("      bare %s %gx%g s%u k%d plate %.1fx%.1f core %d %.1fx%.1f rooms %zu walls %zu topo %d lights %zu\n", rn, sz[0], sz[1], seed, k,
                                    2 * ob.half[0], 2 * ob.half[1], core.valid, core.length, core.depth, rp.rooms.size(), rp.walls.size(), (int)rp.topology, in.furniture.size());
                    }
                }
            }
        std::printf("    [towers] %-20s %3d floors, %2d bare\n", rn, floors, bare);
        CHECK(floors > 0);
        CHECK(bare * 100 <= floors * 10);   // (the 24 m lots' crowns)
        allFloors += floors;
        allBare += bare;
    }
    std::printf("    [towers] %ld floors, %ld bare; %ld desk-row floors; %ld desk rows deep enough for offices\n", allFloors,
                allBare, deskRows, deepRows);
    CHECK(allBare * 100 <= allFloors * 3);
    CHECK(deskRows > 0);
    CHECK(deepRows == 0);
}

// FLATS ROUND THE CORE (Glenn, 2026-10-13: "if you're showing me apartments then we need more walls for the separate
// apartments and then separate rooms"). A residential crown too slim for apartmentPlan's corridor and bands was the
// residents' lounge (14 pencil-tower floors in this spread) or a ring of rooms -- named offices behind a glass tower's
// curtain wall (6). It is two flats off the lift lobby now (room_plan.cpp wrapFlatsPlan), each a hall, a kitchen, a
// living room, a bedroom and its bath, or a studio. Every residential tower floor here is flats: walkable from the
// lobby, every one with its kitchen and its bath, and no lounge and no office anywhere.
TEST_CASE(slim_residential_crowns_are_flats_round_the_core) {
    const char* recipes[] = {"pencil_tower", "condo_tower"};
    const Real sizes[][2] = {{24, 24}, {30, 30}, {40, 26}, {48, 34}, {36, 20}};
    long floors = 0, wrapped = 0, lounge = 0, offices = 0, noKitchen = 0, noBath = 0, blocked = 0;
    for (const char* rn : recipes)
        for (uint32_t seed = 1; seed <= 6; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                const BuildingParams& p = rc.params;
                if (!p.residential) continue;
                const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
                const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
                if (!core.valid) continue;
                for (int k = 1; k < static_cast<int>(sps.size()); ++k) {
                    if (mechanicalStorey(p, k)) continue;
                    const Poly2& sp = sps[static_cast<std::size_t>(k)].plan;
                    const RoomPlan rp = roomPlan(sp, p, core, static_cast<std::size_t>(-1), std::max(p.wallThickness, Real(0.55)), k);
                    ++floors;
                    int kitchens = 0, baths = 0, halls = 0, bathRooms = 0;
                    bool hasLounge = false, hasOffice = false;
                    for (const Room& r : rp.rooms) {
                        hasLounge = hasLounge || r.kind == RoomKind::Lounge;
                        hasOffice = hasOffice || r.kind == RoomKind::Office;
                        if (r.kind == RoomKind::Kitchen || r.kind == RoomKind::Flat) ++kitchens;
                        if (r.kind == RoomKind::Bath || r.kind == RoomKind::Flat) ++baths;
                        if (r.kind == RoomKind::Hall) ++halls;
                        if (r.kind == RoomKind::Bath) ++bathRooms;
                    }
                    lounge += hasLounge;
                    offices += hasOffice;
                    if (std::getenv("RT_CROWN_WHY") && !rp.rooms.empty() && (rp.rooms[0].kind == RoomKind::Lounge || rp.rooms[0].kind == RoomKind::Office)) {
                        const OBB2 ob = orientedBoundingBox(sp);
                        std::printf("      %s %gx%g s%u k%d: plate n%zu %.1fx%.1f, core %.1fx%.1f, topo %d, first room %d\n", rn, sz[0], sz[1], seed, k,
                                    sp.size(), 2 * ob.half[0], 2 * ob.half[1], core.length, core.depth, (int)rp.topology, (int)rp.rooms[0].kind);
                    }
                    if (rp.topology != PlateTopology::Apartments) continue;
                    if (halls == 2 && bathRooms == 2) ++wrapped;   // (apartmentPlan's floors carry more than two)
                    if (kitchens == 0) ++noKitchen;
                    if (baths == 0) ++noBath;
                    if (!floorIsWalkable(rp, sp, core.frame.toWorld({core.length * 0.5, -1.1}))) ++blocked;
                }
            }
    std::printf("    [crowns] %ld residential tower floors: %ld wrapped round the core; %ld lounge floors, %ld with offices; "
                "%ld without a kitchen, %ld without a bath, %ld not walkable\n", floors, wrapped, lounge, offices,
                noKitchen, noBath, blocked);
    CHECK(floors > 1000);
    CHECK(wrapped >= 15);
    CHECK(lounge == 0);
    CHECK(offices == 0);
    CHECK(noKitchen == 0);
    CHECK(noBath == 0);
    CHECK(blocked == 0);
}

// WORKPLACES ARE OFFICES (Glenn, 2026-10-13: "offices can have meeting rooms, small kitchens, lounge space, reception
// desk, offices and cubes and open floor plans"). A masonry office tower, a bank, a civic hall, the floors over a
// commercial block were furnished as rings of studio flats -- a bed, two nightstands and a sofa in every room, an empty
// middle -- because only a curtain wall marked a floor as offices (room_plan.cpp). They are the office floor now
// (BuildingParams::officeUse): private offices along the windows or corner offices, cubicles or pods inside, and
// against the corridor a reception facing the lifts, meeting rooms, a kitchenette and a breakout lounge. A building
// with only a stair lays it round the stair. A hotel and a hospital keep their rooms. Measured on these 177 workplaces'
// 852 upper floors (officeUse forced off for the before): beds 7690 -> 0, desks 4595 -> 16775, meeting tables 142 ->
// 1024, reception desks 32 -> 213 (the glass towers had the office floor already, without its reception).
TEST_CASE(workplaces_are_furnished_as_offices) {
    UseShippedFurniture shipped;
    const Real sizes[][2] = {{48, 34}, {36, 24}, {30, 20}};
    std::array<int, kPieceCount> n{};
    int workplaces = 0, floors = 0, hotelBeds = 0;
    for (int ri = 0; ri < architectRecipeCount(); ++ri) {
        const std::string rn = architectRecipeName(ri);
        for (uint32_t seed = 1; seed <= 2; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                BuildingParams p = rc.params;
                p.openDoorway = true;
                const bool hotel = rn == "hotel";
                if (!p.officeUse && !hotel) continue;
                const int top = std::min(p.floors + 1, 9);
                if (top < 2) continue;
                ++workplaces;
                const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, 1, top);
                floors += top - 1;
                for (const PlacedPiece& pp : in.furniture) {
                    if (hotel) { if (pp.piece == static_cast<uint8_t>(Piece::Bed)) ++hotelBeds; continue; }
                    ++n[pp.piece];
                }
            }
    }
    auto c = [&](Piece pc) { return n[static_cast<std::size_t>(pc)]; };
    std::printf("    [workplaces] %d buildings, %d upper floors: beds %d, nightstands %d, desks %d, cubicles %d, pods %d, "
                "meeting tables %d, kitchenette sinks %d, reception desks %d, sofas %d; the hotel's beds %d\n", workplaces,
                floors, c(Piece::Bed), c(Piece::Nightstand), c(Piece::Desk), c(Piece::Cubicle), c(Piece::DeskPod),
                c(Piece::MeetingTable), c(Piece::KitchenSink), c(Piece::ReceptionDesk), c(Piece::Sofa), hotelBeds);
    CHECK(workplaces > 40);
    CHECK(c(Piece::Bed) == 0);
    CHECK(c(Piece::Nightstand) == 0);
    CHECK(c(Piece::Desk) > 300);
    CHECK(c(Piece::Cubicle) + c(Piece::DeskPod) > 300);
    CHECK(c(Piece::MeetingTable) > 50);
    CHECK(c(Piece::KitchenSink) > 20);
    CHECK(c(Piece::ReceptionDesk) > 20);
    CHECK(hotelBeds > 100);
}

// CIVIC BUILDINGS HAVE THEIR OWN ROOMS (Glenn, 2026-10-13: "keep going"). Off the campus, a school, a library, a museum
// and a university were rings of studio flats too -- a bed, two nightstands and a sofa in every room. They are
// campusPlan's corridor floors now (BuildingParams::civicRooms): classrooms and labs (and the university's lecture
// hall), the library's reading room and stacks, the museum's galleries (display cases, pictures, a bench); their bath
// rooms are restrooms, toilets and basins, not a bathtub.
TEST_CASE(civic_buildings_have_their_own_rooms) {
    UseShippedFurniture shipped;
    const char* recipes[] = {"school", "library", "museum", "university"};
    const Real sizes[][2] = {{36, 24}, {30, 20}, {48, 30}};
    std::map<std::string, std::array<int, kPieceCount>> by;
    for (const char* rn : recipes)
        for (uint32_t seed = 1; seed <= 3; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                BuildingParams p = rc.params;
                p.openDoorway = true;
                const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, 0, p.floors + 1);
                auto& n = by[rn];
                for (const PlacedPiece& pp : in.furniture) ++n[pp.piece];
            }
    auto c = [&](const char* rn, Piece pc) { return by[rn][static_cast<std::size_t>(pc)]; };
    for (const char* rn : recipes)
        std::printf("    [civic] %-10s beds %d, nightstands %d, bathtubs %d; school desks %d, lab benches %d, lecture rows %d, "
                    "reading tables %d, bookcases %d, display cases %d, pictures %d, toilets %d\n", rn, c(rn, Piece::Bed),
                    c(rn, Piece::Nightstand), c(rn, Piece::Bathtub), c(rn, Piece::SchoolDesk), c(rn, Piece::LabBench),
                    c(rn, Piece::LectureRow), c(rn, Piece::ReadingTable), c(rn, Piece::Bookcase), c(rn, Piece::DisplayCase),
                    c(rn, Piece::Picture), c(rn, Piece::Toilet));
    for (const char* rn : recipes) {
        CHECK(c(rn, Piece::Bed) == 0);
        CHECK(c(rn, Piece::Nightstand) == 0);
        CHECK(c(rn, Piece::Bathtub) == 0);
    }
    CHECK(c("school", Piece::SchoolDesk) > 200 && c("school", Piece::LabBench) > 0);
    CHECK(c("university", Piece::LectureRow) > 0 && c("university", Piece::SchoolDesk) > 200);
    CHECK(c("library", Piece::Bookcase) > 100 && c("library", Piece::ReadingTable) > 20);
    CHECK(c("museum", Piece::DisplayCase) > 50 && c("museum", Piece::Picture) > 50);
}

// CHAMFERED OFFICE FLOORS (the same day): the tapered glass tower's upper plates cut their corners on the diagonal --
// eight corners -- and the office floor was rectangles only, so they fell to a ring of offices round an empty middle
// (105 of 488 floors under 60% covered). The office floor lays them in the rectangle the long walls stand on, its
// bands kept clear of the cut corners: every chamfered office floor here is the office floor, and every room stands
// inside its plate.
TEST_CASE(chamfered_office_floors_take_the_office_floor) {
    const Real sizes[][2] = {{24, 24}, {30, 30}, {40, 26}, {48, 34}};
    int chamfered = 0, officeFloor = 0, outside = 0;
    for (uint32_t seed = 1; seed <= 3; ++seed)
        for (const auto& sz : sizes) {
            const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
            BuildingRecipe rc;
            if (!architectRecipeByName("tapered_glass_tower", seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
            const BuildingParams& p = rc.params;
            const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
            const CorePlan core = coreFor(plan, p, entranceEdgeFor(plan, p));
            if (!core.valid) continue;
            for (int k = 1; k < static_cast<int>(sps.size()); k += 2) {
                const Poly2& sp = sps[static_cast<std::size_t>(k)].plan;
                if (sp.size() <= 4 || mechanicalStorey(p, k)) continue;
                ++chamfered;
                const RoomPlan rp = roomPlan(sp, p, core, static_cast<std::size_t>(-1), std::max(p.wallThickness, Real(0.55)), k);
                bool laid = false;
                for (const Room& r : rp.rooms) {
                    laid = laid || r.kind == RoomKind::OpenPlan || r.kind == RoomKind::DeskRow;
                    for (const Vec2& c : r.rect) if (!pointInPolygon(sp, c + (centroid(r.rect) - c) * 0.01)) ++outside;
                }
                officeFloor += laid;
            }
        }
    std::printf("    [chamfered] %d chamfered office floors: %d laid as the office floor; %d room corners outside\n",
                chamfered, officeFloor, outside);
    CHECK(chamfered > 50);
    CHECK(officeFloor * 100 >= chamfered * 90);
    CHECK(outside == 0);
}

// THE HOSPITAL (Glenn, 2026-10-13: "Do hospitals. I didn't even know we had them"): a city's one hospital was a ring of
// studio flats (a double bed, two nightstands, a sofa in each room) over an empty ground floor. Its floors are
// hospitalPlan's now: upstairs, wards along the windows with hospital beds, a nurses' station on every ward floor, a
// staff room, restrooms and stores; on the ground, the lobby (an information desk, waiting seats) and examination rooms.
TEST_CASE(the_hospital_has_wards_a_lobby_and_examination_rooms) {
    UseShippedFurniture shipped;
    const Real sizes[][2] = {{48, 34}, {40, 28}, {36, 24}};
    std::array<int, kPieceCount> n{};
    int hospitals = 0, wardFloors = 0, stations = 0, lobbies = 0, blocked = 0;
    for (uint32_t seed = 1; seed <= 4; ++seed)
        for (const auto& sz : sizes) {
            const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
            BuildingRecipe rc;
            if (!architectRecipeByName("hospital", seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
            BuildingParams p = rc.params;
            p.openDoorway = true;
            const std::vector<StoreyPlan> sps = storeyPlans(plan, p);
            const std::size_t ee = entranceEdgeFor(plan, p);
            const CorePlan core = coreFor(plan, p, ee);
            if (!core.valid) continue;
            ++hospitals;
            const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, 0, p.floors + 1);
            for (const PlacedPiece& pp : in.furniture) ++n[pp.piece];
            for (int k = 0; k < static_cast<int>(sps.size()); ++k) {
                if (mechanicalStorey(p, k)) continue;
                const RoomPlan rp = roomPlan(sps[static_cast<std::size_t>(k)].plan, p, core, static_cast<std::size_t>(-1),
                                             std::max(p.wallThickness, Real(0.55)), k, Poly2{}, ee);
                bool station = false, lobby = false, ward = false;
                for (const Room& r : rp.rooms) {
                    station = station || r.kind == RoomKind::NurseStation;
                    lobby = lobby || r.kind == RoomKind::Waiting;
                    ward = ward || r.kind == RoomKind::Ward;
                }
                if (k == 0) lobbies += lobby;
                else { wardFloors += ward; stations += station; }
                if (!floorIsWalkable(rp, sps[static_cast<std::size_t>(k)].plan, core.frame.toWorld({core.length * 0.5, -1.1}))) ++blocked;
            }
        }
    auto c = [&](Piece pc) { return n[static_cast<std::size_t>(pc)]; };
    std::printf("    [hospital] %d hospitals: %d lobbies, %d ward floors, %d nurses' stations, %d floors not walkable; "
                "hospital beds %d, beds %d, sofas %d, exam couches %d, waiting seats %d, toilets %d, bathtubs %d\n",
                hospitals, lobbies, wardFloors, stations, blocked, c(Piece::HospitalBed), c(Piece::Bed), c(Piece::Sofa),
                c(Piece::ExamCouch), c(Piece::WaitingSeats), c(Piece::Toilet), c(Piece::Bathtub));
    CHECK(hospitals >= 8);
    CHECK(lobbies == hospitals);
    CHECK(wardFloors > 0 && stations == wardFloors);
    CHECK(blocked == 0);
    CHECK(c(Piece::HospitalBed) > 300);
    CHECK(c(Piece::Bed) == 0 && c(Piece::Sofa) == 0 && c(Piece::Bathtub) == 0);
    CHECK(c(Piece::ExamCouch) > 50 && c(Piece::WaitingSeats) > 50 && c(Piece::Toilet) > 0);
}

// THE CIVIC HALLS (Glenn, 2026-10-13, on the list of buildings standing empty or fitted as a cafe-bar: "I like your
// idea. Do that"). A church was empty; a cinema and a market hall were furnished as a cafe-bar; a police station's,
// a courthouse's and a capitol's ground floors were empty. Now: the church's nave (pews facing the altar), the
// cinema's foyer and auditoriums (raked rows, a screen each), the market hall's stalls, the police station's lobby,
// cells and interview rooms, the courthouse's courtrooms (the bench, the jury box, counsel, the public benches) and
// the capitol's council chamber.
TEST_CASE(civic_halls_are_furnished_for_what_they_are) {
    UseShippedFurniture shipped;
    const char* recipes[] = {"church", "cinema", "market_hall", "police", "courthouse", "capitol"};
    const Real sizes[][2] = {{36, 24}, {30, 20}, {44, 28}};
    std::map<std::string, std::array<int, kPieceCount>> by;
    for (const char* rn : recipes)
        for (uint32_t seed = 1; seed <= 3; ++seed)
            for (const auto& sz : sizes) {
                const Poly2 plan = {{0, 0}, {sz[0], 0}, {sz[0], sz[1]}, {0, sz[1]}};
                BuildingRecipe rc;
                if (!architectRecipeByName(rn, seed, 1.0, sz[1], sz[0] * sz[1], rc)) continue;
                BuildingParams p = rc.params;
                p.openDoorway = true;
                const BuildingMesh in = growInterior(plan, p, 0.0, nullptr, 0, 1);   // the ground floor
                for (const PlacedPiece& pp : in.furniture) ++by[rn][pp.piece];
            }
    auto c = [&](const char* rn, Piece pc) { return by[rn][static_cast<std::size_t>(pc)]; };
    std::printf("    [halls] church: pews %d, altars %d | cinema: rows %d, screens %d, ticket desks %d | market: stalls %d "
                "| police: cell bunks %d, interview tables %d, front desks %d | courthouse: benches %d, jury boxes %d, "
                "pews %d | capitol: daises %d | bars in the cinema and the market %d, beds outside the cells %d\n",
                c("church", Piece::Pew), c("church", Piece::Altar), c("cinema", Piece::CinemaRow),
                c("cinema", Piece::CinemaScreen), c("cinema", Piece::ReceptionDesk), c("market_hall", Piece::MarketStall),
                c("police", Piece::SingleBed), c("police", Piece::MeetingTable), c("police", Piece::ReceptionDesk),
                c("courthouse", Piece::JudgeBench), c("courthouse", Piece::JuryBox), c("courthouse", Piece::Pew),
                c("capitol", Piece::JudgeBench), c("cinema", Piece::BarCounter) + c("market_hall", Piece::BarCounter),
                c("police", Piece::Bed) + c("courthouse", Piece::Bed) + c("capitol", Piece::Bed));
    CHECK(c("church", Piece::Pew) > 300 && c("church", Piece::Altar) >= 9);
    CHECK(c("cinema", Piece::CinemaRow) > 100 && c("cinema", Piece::CinemaScreen) >= 9);
    CHECK(c("market_hall", Piece::MarketStall) > 100);
    CHECK(c("police", Piece::SingleBed) > 20 && c("police", Piece::ReceptionDesk) >= 5);
    CHECK(c("courthouse", Piece::JudgeBench) >= 9 && c("courthouse", Piece::JuryBox) > 0);
    CHECK(c("capitol", Piece::JudgeBench) >= 5);
    CHECK(c("cinema", Piece::BarCounter) + c("market_hall", Piece::BarCounter) == 0);
}
