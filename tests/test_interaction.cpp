// THE FURNITURE LIBRARY'S INTERACTIONS (M1; Glenn, 2026-10-01: "sit for chair type objects and lay for bed ... lay
// on sofas and benches"): the shipped descriptions are sane against the kit's own pieces, occupancy is per spot,
// and the reach query offers what a person standing there would expect.
#include "test_framework.h"

#include "../src/engine/interaction.h"
#include "../src/engine/procgen/furniture_kit.h"
#include "../src/engine/procgen/furniture_library.h"
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
