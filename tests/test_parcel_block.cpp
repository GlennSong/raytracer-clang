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
    BlockParcelParams params;
    Shape2 block = rectShape(0, 0, 90, 70);
    ParcelledBlock r = parcelBlock(block, set, params, true, 0.3, StreetClass::Street, 5);

    const Real margin = area(block) - area(r.parcellable);
    const Real accounted = r.lotArea() + r.openArea() + r.laneArea() + margin;
    CHECK(near(accounted, area(block), area(block) * 0.02));
    CHECK(r.lotArea() > 0);
}

TEST_CASE(parcel_lots_do_not_overlap_and_stay_in_the_block) {
    ProgramSet set = residentialPrograms();
    BlockParcelParams params;
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
        BlockParcelParams params;
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
    BlockParcelParams params;
    // A block too small for any residential program at all.
    Shape2 tiny = rectShape(0, 0, 12, 9);
    ParcelledBlock r = parcelBlock(tiny, set, params, true, 0.3, StreetClass::Street, 3);
    CHECK(r.lots.empty());
    CHECK(!r.openSpace.empty());
    CHECK(r.openArea() > 0);
}

// Every lot has a WAY IN — a street or a lane. A mews lot reached only from a
// back lane is legitimate (that is what lanes are for); a lot reached from
// nothing is the defect.
TEST_CASE(parcel_every_lot_has_access) {
    BlockParcelParams params;
    for (std::uint32_t seed = 1; seed <= 6; ++seed) {
        ProgramSet set = residentialPrograms();
        Shape2 block = rectShape(0, 0, 110 + seed * 11, 80 + seed * 14);
        ParcelledBlock r = parcelBlock(block, set, params, true, 0.3,
                                       StreetClass::Street, seed);
        CHECK(!r.lots.empty());
        for (const ParcelledLot& lot : r.lots) {
            Real access = 0;
            int court = 0;
            for (std::size_t i = 0; i < lot.shape.outer.size(); ++i) {
                const EdgeTag t = lot.shape.outer.edges[i].tag;
                if (t == EdgeTag::Street || t == EdgeTag::Lane)
                    access += (lot.shape.outer.end(i) -
                               lot.shape.outer.start(i)).length();
                if (t == EdgeTag::Court) ++court;
            }
            CHECK(access > 1.0 || court > 0);
        }
    }
}

// The tagging bug worth a test of its own: an inherited LANE tag must survive
// the rear/side pass. When it did not, a strip between two lanes lost all its
// frontage and shipped as a single 273 m slab instead of a row of lots.
TEST_CASE(parcel_an_inherited_lane_tag_survives_tagging) {
    Shape2 subBlock = rectShape(0, 0, 120, 40);
    subBlock.outer.edges[0].tag = EdgeTag::Lane;    // y = 0 is a service lane
    subBlock.outer.edges[1].tag = EdgeTag::Street;
    subBlock.outer.edges[2].tag = EdgeTag::Lane;    // y = 40 is another
    subBlock.outer.edges[3].tag = EdgeTag::Street;

    Shape2 lot = rectShape(20, 0, 44, 40);
    tagLotEdges(lot, subBlock, 9.0);
    int lanes = 0;
    Real laneLen = 0;
    for (std::size_t i = 0; i < lot.outer.size(); ++i)
        if (lot.outer.edges[i].tag == EdgeTag::Lane) {
            ++lanes;
            laneLen += (lot.outer.end(i) - lot.outer.start(i)).length();
        }
    CHECK(lanes == 2);
    CHECK(near(laneLen, 48, 0.01));
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
    BlockParcelParams params;
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
    BlockParcelParams params;
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
    BlockParcelParams params;
    Shape2 block = rectShape(0, 0, 220, 180);
    ParcelledBlock r = parcelBlock(block, set, params, true, 0.9,
                                   StreetClass::Arterial, 6);
    int cathedrals = 0;
    for (const ParcelledLot& lot : r.lots)
        if (lot.program >= 0 && set.programs[lot.program].name == "cathedral")
            ++cathedrals;
    CHECK(cathedrals <= 1);
}


// --- lanes and passages ----------------------------------------------------

// A block deeper than its programs can serve from the perimeter gets a service
// lane. Without one, a frontage-first parceller rings the outside and strands
// the middle as a green nobody can reach.
TEST_CASE(parcel_deep_blocks_get_a_service_lane) {
    ProgramSet set = residentialPrograms();
    BlockParcelParams params;
    // 150 m deep: far more than two lots back to back.
    Shape2 deep = rectShape(0, 0, 140, 150);
    ParcelledBlock r = parcelBlock(deep, set, params, true, 0.3, StreetClass::Street, 8);
    int service = 0;
    for (const BlockLane& l : r.lanes) if (!l.passage) ++service;
    CHECK(service >= 1);
    CHECK(r.laneArea() > 0);

    // A SHALLOW block needs none — the perimeter reaches everything.
    Shape2 shallow = rectShape(0, 0, 140, 44);
    ParcelledBlock t = parcelBlock(shallow, set, params, true, 0.3,
                                   StreetClass::Street, 8);
    int shallowLanes = 0;
    for (const BlockLane& l : t.lanes) if (!l.passage) ++shallowLanes;
    CHECK(shallowLanes == 0);
}

// The invariant the lanes exist to guarantee.
TEST_CASE(parcel_every_shared_green_is_reachable) {
    BlockParcelParams params;
    for (std::uint32_t seed = 1; seed <= 8; ++seed) {
        ProgramSet set = residentialPrograms();
        Shape2 block = rectShape(0, 0, 110 + seed * 9, 120 + seed * 6);
        ParcelledBlock r = parcelBlock(block, set, params, true, 0.3,
                                       StreetClass::Street, seed);
        CHECK(r.openSpaceIsReachable());
    }
}

// What a lane actually buys on a deep block. Once the cutter refuses to strand
// a half, a lane-less block stops producing green — it produces LOTS 150 m
// DEEP instead, which is just as wrong and harder to see. The lane is what
// makes the interior reachable at a sane lot depth.
TEST_CASE(parcel_lanes_keep_lots_from_becoming_absurdly_deep) {
    ProgramSet a = residentialPrograms(), b = residentialPrograms();
    Shape2 block = rectShape(0, 0, 150, 150);

    BlockParcelParams withLanes;
    BlockParcelParams without;
    without.lanes = false;
    without.passageWidth = 0;

    ParcelledBlock on = parcelBlock(block, a, withLanes, true, 0.3,
                                    StreetClass::Street, 12);
    ParcelledBlock off = parcelBlock(block, b, without, true, 0.3,
                                     StreetClass::Street, 12);
    CHECK(on.lots.size() > off.lots.size());

    auto deepest = [](const ParcelledBlock& p) {
        Real d = 0;
        for (const ParcelledLot& l : p.lots)
            d = std::max(d, std::min(l.tags.inscribedW, l.tags.inscribedD));
        return d;
    };
    CHECK(deepest(on) < deepest(off));
    // Both stay reachable; the lane version simply has a better grain.
    CHECK(on.openSpaceIsReachable());
    CHECK(off.openSpaceIsReachable());
}

// The reason the depth split has to check access: a split whose back half is
// landlocked converts buildable ground into stranded green, which is the whole
// complaint about big blocks.
TEST_CASE(parcel_a_split_never_strands_a_half) {
    ProgramSet set = residentialPrograms();
    BlockParcelParams params;
    for (std::uint32_t seed = 1; seed <= 8; ++seed) {
        Shape2 block = rectShape(0, 0, 120 + seed * 8, 130 + seed * 7);
        ParcelledBlock r = parcelBlock(block, set, params, true, 0.3,
                                       StreetClass::Street, seed);
        // Green is now a rounding error, not most of the block.
        CHECK(r.openArea() < area(block) * 0.12);
        CHECK(r.lotArea() > area(block) * 0.5);
    }
}

// A lot reached only from a lane is a MEWS: buildable, but its lane face is a
// back face, never a show frontage — so it must not read as a corner site.
TEST_CASE(parcel_a_lane_grants_access_but_not_cornerness) {
    Shape2 block = rectShape(0, 0, 60, 40);
    for (Edge2& e : block.outer.edges) e.tag = EdgeTag::Street;
    Shape2 mews = rectShape(10, 10, 26, 26);
    // Two perpendicular LANE edges: access on two sides, but not a corner shop.
    mews.outer.edges[0].tag = EdgeTag::Lane;
    mews.outer.edges[1].tag = EdgeTag::Lane;
    mews.outer.edges[2].tag = EdgeTag::Rear;
    mews.outer.edges[3].tag = EdgeTag::Side;
    LotTags t = measureLot(mews, {}, StreetClass::Lane, true, 0.2);
    CHECK(t.laneWidth > 30);
    CHECK(near(t.frontWidth, 0, 1e-9));
    CHECK(t.shape == LotShape::MidBlock);       // NOT a corner
    // Access still makes it buildable.
    LotProgram cottage;
    for (const LotProgram& p : residentialPrograms().programs)
        if (p.name == "cottage") cottage = p;
    CHECK(lotFitsProgram(t, cottage));
}

TEST_CASE(parcel_lanes_are_deterministic) {
    Shape2 block = rectShape(0, 0, 130, 140);
    BlockParcelParams params;
    ProgramSet a = residentialPrograms(), b = residentialPrograms();
    ParcelledBlock x = parcelBlock(block, a, params, true, 0.3, StreetClass::Street, 21);
    ParcelledBlock y = parcelBlock(block, b, params, true, 0.3, StreetClass::Street, 21);
    CHECK(x.lanes.size() == y.lanes.size());
    CHECK(near(x.laneArea(), y.laneArea(), 1e-9));
}
