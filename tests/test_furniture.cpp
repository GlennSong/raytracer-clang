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
