#include "test_framework.h"

#include "../src/engine/procgen/city/parcel_block.h"
#include <cmath>
#include <set>

using namespace engine;

namespace {
bool near(Real a, Real b, Real tol) { return std::fabs(a - b) <= tol; }
}  // namespace

TEST_CASE(parcel_block_conserves_area) {
    // Every square metre of the block is accounted for: a lot, open space, or
    // the road margin. Area silently going missing is the defect the old
    // parceller was measured to have (41% court + 31% unaccounted).
    ProgramSet set = residentialPrograms();
    ParcelParams params;
    Shape2 block = rectShape(0, 0, 90, 70);
    ParcelledBlock r = parcelBlock(block, set, params, true, 0.3, StreetClass::Street, 5);

    const Real margin = area(block) - area(r.parcellable);
    const Real accounted = r.lotArea() + r.openArea() + margin;
    CHECK(near(accounted, area(block), area(block) * 0.02));
    CHECK(r.lotArea() > 0);
}

TEST_CASE(parcel_lots_do_not_overlap_and_stay_in_the_block) {
    ProgramSet set = residentialPrograms();
    ParcelParams params;
    Shape2 block = rectShape(0, 0, 100, 64);
    ParcelledBlock r = parcelBlock(block, set, params, true, 0.3, StreetClass::Street, 9);
    CHECK(r.lots.size() >= 4);

    for (std::size_t i = 0; i < r.lots.size(); ++i) {
        std::vector<Shape2> spill =
            shapeBool({r.lots[i].shape}, {block}, BoolOp::Subtract);
        Real out = 0;
        for (const Shape2& s : spill) out += area(s);
        CHECK(out < 0.5);
        for (std::size_t j = i + 1; j < r.lots.size(); ++j) {
            std::vector<Shape2> ov = shapeBool({r.lots[i].shape},
                                               {r.lots[j].shape}, BoolOp::Intersect);
            Real a = 0;
            for (const Shape2& s : ov) a += area(s);
            CHECK(a < 0.5);
        }
    }
}

// THE INVERSION: no lot ships that fails its program's minimum. That is the
// whole point — there is nothing left downstream to reject.
TEST_CASE(parcel_no_lot_ships_that_fails_its_program) {
    for (std::uint32_t seed = 1; seed <= 10; ++seed) {
        ProgramSet set = residentialPrograms();
        ParcelParams params;
        Shape2 block = rectShape(0, 0, 70 + seed * 7, 55 + seed * 3);
        ParcelledBlock r = parcelBlock(block, set, params, true, 0.3,
                                       StreetClass::Street, seed);
        for (const ParcelledLot& lot : r.lots) {
            CHECK(lot.program >= 0);
            CHECK(lotFitsProgram(lot.tags, set.programs[lot.program]));
        }
    }
}

// Land that can carry nothing becomes open space BY DESIGN, not by rejection.
TEST_CASE(parcel_unbuildable_land_becomes_open_space) {
    ProgramSet set = residentialPrograms();
    ParcelParams params;
    // A block too small for any residential program at all.
    Shape2 tiny = rectShape(0, 0, 12, 9);
    ParcelledBlock r = parcelBlock(tiny, set, params, true, 0.3, StreetClass::Street, 3);
    CHECK(r.lots.empty());
    CHECK(!r.openSpace.empty());
    CHECK(r.openArea() > 0);
}

TEST_CASE(parcel_every_lot_has_street_frontage_or_is_a_court) {
    ProgramSet set = residentialPrograms();
    ParcelParams params;
    Shape2 block = rectShape(0, 0, 110, 80);
    ParcelledBlock r = parcelBlock(block, set, params, true, 0.3, StreetClass::Street, 4);
    for (const ParcelledLot& lot : r.lots) {
        Real front = 0;
        int court = 0;
        for (std::size_t i = 0; i < lot.shape.outer.size(); ++i) {
            const EdgeTag t = lot.shape.outer.edges[i].tag;
            if (t == EdgeTag::Street)
                front += (lot.shape.outer.end(i) - lot.shape.outer.start(i)).length();
            if (t == EdgeTag::Court) ++court;
        }
        // A frontage-first split keeps every emitted lot on a street. A
        // landlocked region has no eligible program and never becomes a lot.
        CHECK(front > 1.0 || court > 0);
    }
}

TEST_CASE(parcel_tags_a_lot_street_rear_and_sides) {
    // The reference is the parcellable region — see tagLotEdges' contract.
    Shape2 block = rectShape(0, 0, 60, 40);
    for (Edge2& e : block.outer.edges) e.tag = EdgeTag::Street;
    // A lot cut from the south strip of the block.
    Shape2 lot = rectShape(10, 0, 24, 18);
    tagLotEdges(lot, block, 9.0);
    int street = 0, rear = 0, side = 0, party = 0;
    for (const Edge2& e : lot.outer.edges) {
        if (e.tag == EdgeTag::Street) ++street;
        if (e.tag == EdgeTag::Rear) ++rear;
        if (e.tag == EdgeTag::Side) ++side;
        if (e.tag == EdgeTag::Party) ++party;
    }
    CHECK(street == 1);                 // only y = 0 lies on the block boundary
    CHECK(rear == 1);                   // the far edge
    CHECK(side + party == 2);
    CHECK(lot.outer.edges[0].tag == EdgeTag::Street);
    CHECK(lot.outer.edges[2].tag == EdgeTag::Rear);
}

// A terrace's side walls are PARTY walls — blank by construction, so a row of
// houses cannot look into next door's bedroom.
TEST_CASE(parcel_narrow_lots_get_party_walls) {
    Shape2 block = rectShape(0, 0, 60, 40);
    for (Edge2& e : block.outer.edges) e.tag = EdgeTag::Street;
    Shape2 narrow = rectShape(10, 0, 16, 18);      // 6 m frontage
    tagLotEdges(narrow, block, 9.0);
    int party = 0;
    for (const Edge2& e : narrow.outer.edges) if (e.tag == EdgeTag::Party) ++party;
    CHECK(party == 2);

    Shape2 wide = rectShape(10, 0, 34, 18);        // 24 m frontage
    tagLotEdges(wide, block, 9.0);
    party = 0;
    int side = 0;
    for (const Edge2& e : wide.outer.edges) {
        if (e.tag == EdgeTag::Party) ++party;
        if (e.tag == EdgeTag::Side) ++side;
    }
    CHECK(party == 0);
    CHECK(side == 2);
}

// §17.6: a rim block is not a special case, it is a TAG. The coarse grain falls
// out of the eligible programs' larger minimums, with no branch anywhere.
TEST_CASE(parcel_rim_blocks_come_out_coarser) {
    ParcelParams params;
    Shape2 block = rectShape(0, 0, 180, 140);

    ProgramSet fine = residentialPrograms();
    ParcelledBlock a = parcelBlock(block, fine, params, true, 0.2,
                                   StreetClass::Street, 11);
    ProgramSet coarse = rimPrograms();
    ParcelledBlock b = parcelBlock(block, coarse, params, false, 0.2,
                                   StreetClass::Street, 11);

    CHECK(!a.lots.empty());
    CHECK(!b.lots.empty());
    const Real aMean = a.lotArea() / a.lots.size();
    const Real bMean = b.lotArea() / b.lots.size();
    CHECK(bMean > aMean * 1.8);         // campus parcels are far bigger
    CHECK(b.lots.size() < a.lots.size());
}

TEST_CASE(parcel_is_deterministic) {
    Shape2 block = rectShape(0, 0, 96, 72);
    ParcelParams params;
    ProgramSet a = residentialPrograms(), b = residentialPrograms();
    ParcelledBlock x = parcelBlock(block, a, params, true, 0.3, StreetClass::Street, 77);
    ParcelledBlock y = parcelBlock(block, b, params, true, 0.3, StreetClass::Street, 77);
    CHECK(x.lots.size() == y.lots.size());
    for (std::size_t i = 0; i < x.lots.size(); ++i) {
        CHECK(near(area(x.lots[i].shape), area(y.lots[i].shape), 1e-9));
        CHECK(x.lots[i].program == y.lots[i].program);
    }
}

// The quota survives block subdivision: one cathedral in a downtown block, not
// one per lot that happens to qualify.
TEST_CASE(parcel_quotas_hold_across_a_whole_block) {
    ProgramSet set = downtownPrograms();
    ParcelParams params;
    Shape2 block = rectShape(0, 0, 220, 180);
    ParcelledBlock r = parcelBlock(block, set, params, true, 0.9,
                                   StreetClass::Arterial, 6);
    int cathedrals = 0;
    for (const ParcelledLot& lot : r.lots)
        if (lot.program >= 0 && set.programs[lot.program].name == "cathedral")
            ++cathedrals;
    CHECK(cathedrals <= 1);
}
