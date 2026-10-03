// THE FURNITURE KIT (buildings M4b): modelled pieces, one mesh per finish, drawn instanced; and the rooms they are
// placed in.
#include "test_framework.h"

#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/city/furniture.h"
#include "../src/engine/procgen/city/room_plan.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/architect.h"
#include <cmath>
#include <cstdio>

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
    int desks = 0, chairs = 0, seats = 0;
    std::vector<Vec2> doors;
    for (const RoomWall& w : rp.walls)
        if (w.doorAt >= 0) doors.push_back(w.a + (w.b - w.a) * w.doorAt);
    for (const PlacedPiece& pp : placed) {
        if (pp.piece == static_cast<uint8_t>(Piece::Desk)) { ++desks; ++seats; }
        if (pp.piece == static_cast<uint8_t>(Piece::OfficeChair)) ++chairs;
        if (pp.piece == static_cast<uint8_t>(Piece::DeskPod)) seats += ((pp.variant >> 5) & 4u) ? 4 : 6;      // the open plan (buildings C)
        if (pp.piece == static_cast<uint8_t>(Piece::Cubicle)) seats += 1;
        const Vec2 at(pp.xform.m[0][3], pp.xform.m[2][3]);
        CHECK(pointInPolygon(plan, at));
        CHECK(std::fabs(pp.xform.m[1][3] - 25.0) < 1.0);
        for (const Vec2& d : doors) CHECK((at - d).length() > 0.45);
    }
    CHECK(seats >= 28);   // a 40 x 30 m office floor seats a team (~30 cubicles, ~90 at benches), not a desk a room
    CHECK(chairs == desks);
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
    CHECK(n(with, Piece::OfficeChair) == n(with, Piece::Desk));         // a chair at every desk
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
