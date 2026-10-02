// THE FURNITURE LIBRARY'S INTERACTIONS (M1; Glenn, 2026-10-01: "sit for chair type objects and lay for bed ... lay
// on sofas and benches"): the shipped descriptions are sane against the kit's own pieces, occupancy is per spot,
// and the reach query offers what a person standing there would expect.
#include "test_framework.h"

#include "../src/engine/interaction.h"
#include "../src/engine/interact_broker.h"
#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/furniture_library.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/city_lots.h"
#include "../src/engine/scripting/furniture_library_lua.h"
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace engine;

namespace {

FurnitureLibrary shippedLibrary() {
    std::ifstream f(std::string(RT_SOURCE_DIR) + "/assets/scripts/furniture_library.lua");
    std::stringstream ss;
    ss << f.rdbuf();
    FurnitureLibrary lib;
    std::string err;
    const bool ok = loadFurnitureLibrarySource(ss.str(), lib, &err);
    if (!ok) std::printf("    [library] %s\n", err.c_str());
    CHECK(ok);
    return lib;
}

Mat4 at(Real x, Real y, Real z) { return Mat4::translate(x, y, z); }

}  // namespace

// Every described seat or bed: its spots on the piece, its eye above the body, its exit off the piece on the floor.
TEST_CASE(the_furniture_library_describes_its_pieces_sanely) {
    const FurnitureLibrary lib = shippedLibrary();
    CHECK(lib.size() >= 6);
    for (Piece p : {Piece::OfficeChair, Piece::DiningChair, Piece::LoungeChair, Piece::Sofa, Piece::Bed})
        CHECK(lib.interactive(p));
    CHECK(lib.find(Piece::Sofa)->verbs.size() == 4);   // three seats and lying down
    for (int i = 0; i < kPieceCount; ++i) {
        const FurnitureAsset* a = lib.find(static_cast<Piece>(i));
        if (!a) continue;
        const Vec3 sz = furniturePiece(static_cast<Piece>(i), 0).size;
        for (const FurnSpot& s : a->spots) {
            CHECK(std::fabs(s.at.x) <= sz.x * 0.5 + 0.05);
            CHECK(s.at.z >= 0 && s.at.z <= sz.z + 0.05);
            CHECK(s.at.y > 0.3 && s.at.y <= sz.y);
        }
        for (const FurnVerb& v : a->verbs) {
            Real top = 0;
            for (std::size_t k = 0; k < a->spots.size(); ++k) if (v.spots & (1u << k)) top = std::max(top, a->spots[k].at.y);
            CHECK(v.eye.y > top + (v.verb == Verb::Sit ? 0.5 : 0.15));
            CHECK(std::fabs(v.exit.y) < 1e-6);
            CHECK(std::fabs(v.exit.x) > sz.x * 0.5 || v.exit.z > sz.z);   // off the piece
        }
    }
}

// A sofa seats three or lets one lie down: occupancy is per spot.
TEST_CASE(sitting_on_the_sofa_leaves_the_other_seats_but_not_room_to_lie) {
    const FurnitureLibrary lib = shippedLibrary();
    Interactables set;
    set.pieces.push_back({static_cast<uint8_t>(Piece::Sofa), at(0, 0, 0), 0});
    set.refreshBounds();
    const Entity e;
    const std::vector<std::pair<Entity, const Interactables*>> sets = {{e, &set}};
    // standing in front of the sofa, looking at it
    InteractChoice c = findInteraction(sets, lib, Vec3(0, 0, 1.4), Vec3(0, 0, -1));
    CHECK(c.valid());
    const FurnitureAsset* a = lib.find(Piece::Sofa);
    CHECK(c.primary >= 0 && a->verbs[static_cast<std::size_t>(c.primary)].verb == Verb::Sit);
    CHECK(c.secondary >= 0 && a->verbs[static_cast<std::size_t>(c.secondary)].verb == Verb::Lie);
    Seated s;
    CHECK(takeInteraction(set, e, c.piece, c.primary, lib, s));
    CHECK(s.eye.y > 1.0);
    c = findInteraction(sets, lib, Vec3(0.6, 0, 1.4), Vec3(0, 0, -1));
    CHECK(c.valid());
    CHECK(c.primary >= 0 && c.primary != static_cast<int>(s.verb));   // another cushion
    CHECK(c.secondary < 0);                                          // no lying down past someone
    releaseInteraction(&set, s);
    c = findInteraction(sets, lib, Vec3(0, 0, 1.4), Vec3(0, 0, -1));
    CHECK(c.secondary >= 0);
}

// A bed's primary is lying down; out of reach or behind you, nothing is offered.
TEST_CASE(a_bed_offers_lying_down_and_nothing_is_offered_out_of_reach) {
    const FurnitureLibrary lib = shippedLibrary();
    Interactables set;
    set.pieces.push_back({static_cast<uint8_t>(Piece::Bed), at(10, 3, 0), 0});
    set.refreshBounds();
    const std::vector<std::pair<Entity, const Interactables*>> sets = {{Entity{}, &set}};
    InteractChoice c = findInteraction(sets, lib, Vec3(11.3, 3, 1.1), Vec3(-1, 0, 0));
    CHECK(c.valid());
    CHECK(lib.find(Piece::Bed)->verbs[static_cast<std::size_t>(c.primary)].verb == Verb::Lie);
    CHECK(!findInteraction(sets, lib, Vec3(16, 3, 1), Vec3(-1, 0, 0)).valid());     // too far
    CHECK(!findInteraction(sets, lib, Vec3(11.6, 3, 1.1), Vec3(1, 0, 0)).valid());  // looking away
    CHECK(!findInteraction(sets, lib, Vec3(11.3, 7, 1.1), Vec3(-1, 0, 0)).valid()); // a floor up
}

// A typo in the data is an error, not a silently missing seat.
TEST_CASE(a_malformed_furniture_library_is_refused) {
    FurnitureLibrary lib;
    std::string err;
    CHECK(!loadFurnitureLibrarySource(
        "furniture_library = { sofa = { spots = { { id = 'a', at = {0,0.5,0.5} } }, verbs = { { verb = 'sit', spots = { 'b' } } } } }",
        lib, &err));
    CHECK(err.find("unknown spot") != std::string::npos);
    CHECK(!loadFurnitureLibrarySource("furniture_library = { sofaa = { } }", lib, &err));
    CHECK(!loadFurnitureLibrarySource(
        "furniture_library = { sofa = { spots = { { id = 'a', at = {0,0.5,0.5} } }, verbs = { { verb = 'dance', spots = { 'a' } } } } }",
        lib, &err));
}

// THE BROKER'S CHOICE (Glenn, 2026-10-01: "E sounds like it's a universal interaction button ... highlight things
// close to you or if you're in a volume"): of the offers in reach, the one you look at wins over a nearer one beside
// you; a volume you stand in needs no look; an exclusive offer hides the rest; a wall hides what is behind it.
TEST_CASE(the_interaction_broker_focuses_what_you_look_at) {
    const Vec3 feet(0, 0, 0), eye(0, 1.6, 0);
    auto offer = [](const char* name, Vec3 at) {
        InteractOffer o;
        o.provider = "test";
        o.name = name;
        o.tap = "use";
        o.anchor = at;
        return o;
    };
    std::vector<InteractOffer> offers = {offer("near_left", Vec3(-0.9, 0.6, -0.4)), offer("ahead", Vec3(0, 0.6, -1.6)),
                                         offer("behind", Vec3(0, 0.6, 1.5)), offer("far", Vec3(0, 0.6, -6))};
    std::vector<RankedOffer> r = rankInteractions(offers, feet, eye, Vec3(0, -0.4, -1));
    CHECK(r.size() == 1);   // behind you, out of reach, and 66 degrees off to the side are not candidates
    CHECK(offers[r[0].index].name == std::string("ahead"));
    r = rankInteractions(offers, feet, eye, Vec3(-1, -0.6, -0.3));
    CHECK(!r.empty() && offers[r[0].index].name == std::string("near_left"));
    // BEHIND and below, looking a little down: a 3D cone would take it (the seat sits far below the eye); facing
    // does not (the live shot that focused a sofa behind the player)
    {
        const std::vector<InteractOffer> behind = {offer("behind_low", Vec3(0.7, 0.55, 0.65))};
        CHECK(rankInteractions(behind, feet, eye, Vec3(0, -0.45, -1)).empty());
    }
    // a volume you stand in: no look needed, and it outranks a point beside it
    InteractOffer lift = offer("lift", Vec3(0, 1.4, 3));
    lift.inVolume = true;
    offers.push_back(lift);
    r = rankInteractions(offers, feet, eye, Vec3(1, 0, 0));
    CHECK(!r.empty() && offers[r[0].index].name == std::string("lift"));
    // exclusive: only it
    InteractOffer stand = offer("stand", eye);
    stand.exclusive = true;
    offers.push_back(stand);
    r = rankInteractions(offers, feet, eye, Vec3(0, 0, -1));
    CHECK(r.size() == 1 && offers[r[0].index].name == std::string("stand"));
    offers.pop_back();
    // a wall: whatever lies past z = -1 is hidden
    auto wall = [](const Vec3& e, const Vec3& a, void*) { return e.z > -1.0 && a.z < -1.0; };
    r = rankInteractions(offers, feet, eye, Vec3(0, -0.4, -1), wall, nullptr);
    for (const RankedOffer& k : r) CHECK(offers[k.index].name != std::string("ahead"));
}

TEST_CASE(the_interaction_marker_projects_onto_the_screen) {
    Real sx = 0, sy = 0;
    // straight ahead: the centre
    CHECK(projectToScreen(Vec3(0, 0, -5), Vec3(0, 0, 0), Vec3(0, 0, -1), Vec3(0, 1, 0), 60, 16.0 / 9.0, 1600, 900, sx, sy));
    CHECK(std::fabs(sx - 800) < 1e-6 && std::fabs(sy - 450) < 1e-6);
    // up and to the right: right of centre, above it
    CHECK(projectToScreen(Vec3(1, 1, -5), Vec3(0, 0, 0), Vec3(0, 0, -1), Vec3(0, 1, 0), 60, 16.0 / 9.0, 1600, 900, sx, sy));
    CHECK(sx > 800 && sy < 450);
    // behind: not on screen
    CHECK(!projectToScreen(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 0, -1), Vec3(0, 1, 0), 60, 16.0 / 9.0, 1600, 900, sx, sy));
}

// THE TOILET (Glenn: "Don't forget the toilet. Lol you should be able to sit on that"). A real apartment floor, grown
// as the city grows it: every bathroom's toilet is in the floor's interactive set, and standing in front of one,
// looking at it, the offer is to sit.
TEST_CASE(you_can_sit_on_the_toilet) {
    const FurnitureLibrary lib = shippedLibrary();
    const Poly2 plan = {{0, 0}, {44, 0}, {44, 32}, {0, 32}};
    BuildingParams p;
    p.floors = 24; p.curtainWall = false; p.walkableGround = true; p.openDoorway = true; p.seed = 33;
    p.residential = true;
    const BuildingMesh bm = growInterior(plan, p, 0.0, nullptr, 6, 7);
    Interactables set;
    for (const PlacedPiece& pp : bm.furniture)
        if (lib.interactive(static_cast<Piece>(pp.piece))) set.pieces.push_back({pp.piece, pp.xform, 0});
    set.refreshBounds();
    int toilets = 0, offered = 0;
    for (std::size_t i = 0; i < set.pieces.size(); ++i) {
        if (set.pieces[i].piece != static_cast<uint8_t>(Piece::Toilet)) continue;
        ++toilets;
        const Mat4& m = set.pieces[i].xform;
        // standing in front of the bowl, on its floor
        const Vec3 front = piecePoint(m, Vec3(0, 0, 1.2));
        int prim = -1, sec = -1;
        Vec3 at;
        Real d = 0;
        if (pieceVerbs(set, static_cast<uint32_t>(i), lib, front, 1.8, prim, sec, at, d) && prim >= 0 &&
            lib.find(Piece::Toilet)->verbs[static_cast<std::size_t>(prim)].verb == Verb::Sit)
            ++offered;
    }
    std::printf("    [toilet] %d toilets on the floor, %d offer a seat\n", toilets, offered);
    CHECK(toilets > 0);
    CHECK(offered == toilets);
}

// OUTDOOR PIECES (M2): the lot pass places a bench by its back's centre and the way it faces; the transform must
// turn the piece's front (+z) that way, keep it right-handed (a mirrored frame turns a piece inside out), and put
// its seats in front of where it stands.
TEST_CASE(an_outdoor_piece_faces_where_it_was_placed) {
    const FurnitureLibrary lib = shippedLibrary();
    for (const Vec2 face : {Vec2(1, 0), Vec2(0, -1), Vec2(-0.6, 0.8)}) {
        OutdoorPiece op;
        op.piece = static_cast<uint8_t>(Piece::Bench);
        op.at = Vec2(10, 20);
        op.yaw = std::atan2(face.x, face.y);
        const Mat4 m = outdoorPieceXform(op, 3.0);
        const Vec3 Z = pieceDir(m, Vec3(0, 0, 1)), X = pieceDir(m, Vec3(1, 0, 0));
        CHECK(std::fabs(Z.x - face.x) < 1e-9 && std::fabs(Z.z - face.y) < 1e-9);
        CHECK(dot(cross(X, Vec3(0, 1, 0)), Z) > 0.99);
        const Vec3 seat = piecePoint(m, lib.find(Piece::Bench)->spots[1].at);
        CHECK(std::fabs(seat.y - 3.47) < 1e-6);
        CHECK((seat.x - 10) * face.x + (seat.z - 20) * face.y > 0.3);   // in front of its back
    }
}
