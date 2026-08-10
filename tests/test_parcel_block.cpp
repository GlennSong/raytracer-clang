#include "test_framework.h"

#include "../src/engine/procgen/city/parcel_block.h"
#include "../src/engine/procgen/city/site_plan.h"
#include <algorithm>
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
    // Lot DEPTH is now bounded by the row stationing whether or not a lane is
    // cut — a block is a whole number of rows of the depth its programs are
    // built at, so there is no leftover row to be absurdly deep. (This check
    // used to read `deepest(on) < deepest(off)`, on the older cutter where
    // peeling one row off the front left the remainder as one deep back lot
    // and only a lane could break it up. The lane still earns its keep on the
    // count above; it is no longer what bounds the depth.)
    Real rowDepth = 0;
    for (const LotProgram& p : residentialPrograms().programs)
        if (p.quota <= 0) rowDepth = std::max(rowDepth, p.targetDepth());
    CHECK(deepest(on) < rowDepth * 1.2);
    CHECK(deepest(off) < rowDepth * 1.2);
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

// --- grain: the size a lot comes out at ------------------------------------

namespace {

// The median lot area, which is what "the grain" means for a block.
Real medianLotArea(const ParcelledBlock& pb) {
    std::vector<Real> a;
    for (const ParcelledLot& l : pb.lots) a.push_back(area(l.shape));
    if (a.empty()) return 0;
    std::sort(a.begin(), a.end());
    return a[a.size() / 2];
}

// The shortest side of the biggest building envelope the block produced — the
// number that decides whether a plan can grow WINGS rather than being a box.
Real widestPlate(const ParcelledBlock& pb, ProgramSet& mix) {
    Real best = 0;
    for (const ParcelledLot& l : pb.lots) {
        const int pi = mix.bestFor(l.tags);
        if (pi < 0) continue;
        SitePlan sp = layoutSite(l.shape, l.tags, mix.programs[pi], 5);
        for (const ZoneArea& z : sp.zones) {
            if (z.zone != Zone::Building) continue;
            for (const Shape2& s : z.parts) {
                Vec2 lo, hi;
                bounds(s, lo, hi);
                best = std::max(best, std::min(hi.x - lo.x, hi.y - lo.y));
            }
        }
    }
    return best;
}

}  // namespace

// THE BUG THIS EXISTS FOR: a recursive cutter that splits near the middle can
// only ever produce lots of `frontage / 2^k`, so the grain is an artefact of
// where the halving cascade happens to stop. Measured before programs carried
// a target: a 110 m downtown block gave 2100 m2 tower plates and a 150 m one
// gave 1040 m2 — the BIGGER block gave the SMALLER lots, and whether a tower
// had room for a shaped plan was luck.
//
// The invariant is not monotonicity (a block is not obliged to divide evenly);
// it is that the lot comes out at the size the PROGRAM asked for, whatever
// block it is handed. Stationing the cuts on whole target-width lots is what
// makes that true.
TEST_CASE(parcel_lot_size_follows_the_program_not_the_block) {
    BlockParcelParams params;
    Real smallest = 1e30, largest = 0;
    for (Real side : {Real(100), Real(120), Real(150), Real(190), Real(240)}) {
        ProgramSet mix = downtownPrograms();
        ParcelledBlock pb = parcelBlock(rectShape(0, 0, side, side), mix, params,
                                        true, 0.9, StreetClass::Street, 11);
        const Real med = medianLotArea(pb);
        CHECK(med > 1500);
        // Whatever the block, downtown land ends up carrying a plate wide
        // enough for a plan with wings — 30 m is where plan_grammar's
        // `radial-wings` stops being able to grow one.
        CHECK(widestPlate(pb, mix) >= 30.0);
        smallest = std::min(smallest, med);
        largest = std::max(largest, med);
    }
    // The whole spread across every block size, against the 2x INVERSION this
    // replaced (110 m -> 2100 m2, 150 m -> 1040 m2).
    CHECK(largest / smallest < 1.6);
}

// The biggest program in a mix has to be REACHABLE. A one-sided scale penalty
// decays every candidate by the same factor, so it changes the magnitudes and
// never the order — the highest base weight then wins at every size and the
// skyscraper program is dead weight in the list. Measured before the fit was
// made symmetric: `glass_tower` lost to `office_tower` on a 1600 m2 parcel and
// on a 48 000 m2 one alike.
TEST_CASE(parcel_the_largest_program_wins_the_largest_land) {
    ProgramSet mix = downtownPrograms();
    auto bestOn = [&](Real side) {
        Shape2 s = rectShape(0, 0, side, side);
        for (Edge2& e : s.outer.edges) e.tag = EdgeTag::Street;
        const LotTags t = measureLot(s, {}, StreetClass::Street, true, 0.9);
        const int i = mix.bestFor(t);
        return i >= 0 ? mix.programs[i].name : std::string("-");
    };
    // A modest plate is an office tower; a whole block is a skyscraper site.
    CHECK(bestOn(40) == "office_tower");
    CHECK(bestOn(200) == "glass_tower");
}

// A block at an angle to the axes is the normal case, not a special one: the
// generator's streets follow arterials, not the X axis. Depth was measured off
// an axis-aligned bounding box, which is deeper than an angled block actually
// is, so the same land parcelled differently depending on its rotation.
TEST_CASE(parcel_grain_survives_a_rotated_block) {
    BlockParcelParams params;
    auto grainOf = [&](Real deg) {
        const Real c = std::cos(deg * 3.14159265358979 / 180.0);
        const Real sn = std::sin(deg * 3.14159265358979 / 180.0);
        Poly2 ring;
        for (const Vec2& v : {Vec2(0, 0), Vec2(150, 0), Vec2(150, 150), Vec2(0, 150)})
            ring.push_back(Vec2(v.x * c - v.y * sn, v.x * sn + v.y * c));
        ProgramSet mix = downtownPrograms();
        ParcelledBlock pb = parcelBlock(shapeFromPoly(ring), mix, params, true,
                                        0.9, StreetClass::Street, 11);
        return medianLotArea(pb);
    };
    const Real flat = grainOf(0);
    CHECK(flat > 0);
    for (Real deg : {Real(17), Real(31), Real(45)}) {
        const Real turned = grainOf(deg);
        CHECK(turned > flat * 0.6);
        CHECK(turned < flat * 1.7);
    }
}

// The other half of the same question: a downtown lot has to be big enough to
// carry a plan with wings, and a neighbourhood lot has to stay small. One
// parceller, one rule, opposite answers — because the PROGRAMS differ, not
// because anything branches on district.
TEST_CASE(parcel_downtown_builds_tower_plates_and_terraces_stay_tight) {
    BlockParcelParams params;
    ProgramSet down = downtownPrograms();
    ParcelledBlock city = parcelBlock(rectShape(0, 0, 120, 120), down, params,
                                      true, 0.9, StreetClass::Street, 3);
    // 30 m is the width a plan needs before `radial-wings` will grow wings at
    // all (plan_grammar's kMinWing); a plate under it can only be a box.
    CHECK(widestPlate(city, down) >= 30.0);
    CHECK(medianLotArea(city) > 1200);

    ProgramSet res = residentialPrograms();
    ParcelledBlock hood = parcelBlock(rectShape(0, 0, 120, 120), res, params,
                                      true, 0.2, StreetClass::Street, 3);
    CHECK(medianLotArea(hood) < 500);
    CHECK(hood.lots.size() > city.lots.size() * 4);
}

// The block a district needs is DERIVED from what it intends to build, not
// chosen. If this inverts, the road layer is laying the wrong grain and no
// amount of parcelling downstream can recover the land.
TEST_CASE(parcel_block_grain_follows_the_programs) {
    const BlockGrain down = blockGrainFor(downtownPrograms());
    const BlockGrain com = blockGrainFor(commercialPrograms());
    const BlockGrain res = blockGrainFor(residentialPrograms());
    const BlockGrain rim = blockGrainFor(rimPrograms());
    CHECK(down.depth > com.depth);
    CHECK(com.depth > res.depth);
    CHECK(rim.depth > down.depth);
    // Deep enough for two rows of what it is for, and no deeper.
    CHECK(res.depth > 60 && res.depth < 90);
    CHECK(down.depth > 110 && down.depth < 160);
    // A block face carries several lots, so it reads as a block.
    CHECK(down.length >= down.depth);
}
