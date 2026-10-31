// CIRCULATION (circulation.h, issue #105 step 3): shafts, banks and zones as words of their own; today's core as
// the first preset, read back into the CorePlan the emitters take; the shafts as cells of the space graph.
#include "test_framework.h"

#include "../src/engine/procgen/city/architect.h"
#include "../src/engine/procgen/city/circulation.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

using namespace engine;

namespace {
BuildingParams towerParams(int floors) {
    BuildingParams p;
    p.floors = floors;
    p.curtainWall = true;
    p.walkableGround = true;
    p.openDoorway = true;
    p.seed = 77;
    return p;
}
bool sameFrame(const SiteFrame& a, const SiteFrame& b) {
    return a.origin.x == b.origin.x && a.origin.y == b.origin.y && a.u.x == b.u.x && a.u.y == b.u.y && a.v.x == b.v.x && a.v.y == b.v.y;
}
bool sameShaft(const CoreShaft& a, const Shaft& b) {
    return sameFrame(a.frame, b.frame) && a.width == b.w && a.depth == b.d && a.doorX == b.doorU && a.doorWidth == b.doorW &&
           a.doorHeight == b.doorH;
}
int count(const CirculationPlan& c, Shaft::Kind k) {
    int n = 0;
    for (const Shaft& s : c.shafts) n += s.kind == k;
    return n;
}
}  // namespace

TEST_CASE(core_preset_lays_out_todays_core_bit_for_bit) {
    // The preset IS the core: every float of every shaft the same as the CorePlan read back from it, on a square
    // plate, a tall tower, a setback stack and a slab -- the plans the old corePlan was proven on.
    struct Case { Poly2 plan; int floors; bool setback; };
    const Case cases[] = {
        {{{0, 0}, {40, 0}, {40, 40}, {0, 40}}, 30, false},
        {{{0, 0}, {30, 0}, {30, 30}, {0, 30}}, 60, false},
        {{{0, 0}, {36, 0}, {36, 44}, {0, 44}}, 40, true},
        {{{-10, -30}, {10, -30}, {10, 30}, {-10, 30}}, 12, false},
        {{{0, 0}, {12, 0}, {12, 12}, {0, 12}}, 6, false},   // too small: nothing fits
    };
    for (const Case& cs : cases) {
        BuildingParams p = towerParams(cs.floors);
        p.stairPlace = 1;   // the classic core by name (left to the seed, a tall tower may stand its banks facing)
        p.liftBanks = 1;
        if (cs.setback) {
            p.envelope = BuildingParams::Envelope::StreetWallSetback;
            p.baseFloors = 5; p.setback1 = 5; p.stepFloors = 10; p.stepDepth = 3; p.towerFrac = 0.35; p.towerFloor = 20;
        }
        const std::size_t e = entranceEdgeFor(cs.plan, p);
        const std::vector<MassTier> tiers = massStack(cs.plan, p);
        const CirculationPlan c = corePreset(cs.plan, tiers, p, e);
        const CorePlan core = corePlan(cs.plan, tiers, p, e);
        CHECK(c.valid == core.valid);
        if (!c.valid) { CHECK(c.shafts.empty()); CHECK(c.banks.empty()); continue; }
        CHECK(sameFrame(c.frame, core.frame));
        CHECK(c.length == core.length && c.depth == core.depth);
        CHECK(count(c, Shaft::Kind::Stair) == 2);
        CHECK(count(c, Shaft::Kind::Hoistway) == static_cast<int>(core.hoistways.size()));
        CHECK(count(c, Shaft::Kind::Service) == static_cast<int>(core.services.size()));
        std::size_t si = 0, hi = 0, vi = 0;
        for (const Shaft& s : c.shafts) {
            if (s.kind == Shaft::Kind::Stair) CHECK(sameShaft(core.stairs[si++].shaft, s));
            else if (s.kind == Shaft::Kind::Hoistway) CHECK(sameShaft(core.hoistways[hi++], s));
            else if (s.kind == Shaft::Kind::Service) CHECK(sameShaft(core.services[vi++], s));
            CHECK(s.k0 == 0 && s.k1 == cs.floors);   // storey 0 is the ground
        }
        // One bank of every hoistway facing the entrance edge; one zone over every storey; no sky lobby.
        CHECK(c.banks.size() == 1);
        CHECK(c.banks[0].shafts.size() == core.hoistways.size());
        CHECK(c.banks[0].lobbyEdge == e);
        for (int i : c.banks[0].shafts) CHECK(c.shafts[static_cast<std::size_t>(i)].kind == Shaft::Kind::Hoistway && c.shafts[static_cast<std::size_t>(i)].bank == 0);
        CHECK(c.zones.size() == 1 && c.zones[0].bank == 0 && c.zones[0].k0 == 0 && c.zones[0].k1 == cs.floors);
        CHECK(c.skyLobby == -1);
        for (const Shaft& s : c.shafts)
            for (int k = 0; k <= cs.floors; ++k) CHECK(c.serves(s, k) == (s.doorW > 0));
        std::printf("    [preset] %d floors: %zu shafts, %zu hoistways, service %zu, block %.2f x %.2f\n", cs.floors, c.shafts.size(),
                    core.hoistways.size(), core.services.size(), c.length, c.depth);
    }
}

TEST_CASE(core_for_reads_the_preset) {
    // coreFor is the one call the grammar, the lot pass and the runtime share: it reads circulationFor.
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(30);
    const std::size_t e = entranceEdgeFor(plan, p);
    const CirculationPlan c = circulationFor(plan, p, e);
    const CorePlan a = coreFor(plan, p, e), b = asCorePlan(c);
    CHECK(a.valid && b.valid);
    CHECK(a.hoistways.size() == b.hoistways.size() && a.stairs.size() == b.stairs.size());
    for (std::size_t i = 0; i < a.hoistways.size(); ++i) CHECK(sameFrame(a.hoistways[i].frame, b.hoistways[i].frame));
    BuildingParams low = towerParams(3);   // no core under four floors
    CHECK(!circulationFor(plan, low, e).valid);
    CHECK(!coreFor(plan, low, e).valid);
}

TEST_CASE(shafts_become_cells_with_a_door_on_every_storey_they_serve) {
    // addTo: one Stair / Lift cell per shaft up the building, a portal from every storey's hall, the stairs on into
    // the roof when the core reaches it. The adapter's own cells come from this call now.
    const Poly2 plan = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const BuildingParams p = towerParams(30);
    const std::size_t e = entranceEdgeFor(plan, p);
    CirculationPlan c = circulationFor(plan, p, e);
    CHECK(c.valid);
    const SpaceGraph g = spaceGraphOf(plan, p, 0.0);
    const int nS = g.storeys;
    CHECK(nS == 31);   // the ground and thirty floors
    int stairCells = 0, liftCells = 0;
    for (const Cell& cell : g.cells) { stairCells += cell.kind == CellKind::Stair; liftCells += cell.kind == CellKind::Lift; }
    CHECK(stairCells == 2);
    CHECK(liftCells == count(c, Shaft::Kind::Hoistway));
    // Every lift cell: nS Lift portals, one per storey, each from that storey's hall.
    for (std::size_t i = 0; i < g.cells.size(); ++i) {
        const Cell& cell = g.cells[i];
        if (cell.kind != CellKind::Lift && cell.kind != CellKind::Stair) continue;
        std::vector<int> seen(static_cast<std::size_t>(nS + 1), 0);
        int roofDoors = 0;
        for (const Portal& po : g.portals) {
            if (po.a != static_cast<int>(i) && po.b != static_cast<int>(i)) continue;
            const int other = po.a == static_cast<int>(i) ? po.b : po.a;
            if (other == g.roof) { ++roofDoors; continue; }
            CHECK(po.kind == (cell.kind == CellKind::Lift ? Portal::Kind::Lift : Portal::Kind::Stair));
            CHECK(g.cells[static_cast<std::size_t>(other)].kind == CellKind::Hall);
            CHECK(po.storey >= 0 && po.storey < nS);
            seen[static_cast<std::size_t>(po.storey)] += 1;
        }
        for (int k = 0; k < nS; ++k) CHECK(seen[static_cast<std::size_t>(k)] == 1);
        CHECK(roofDoors == (cell.kind == CellKind::Stair && coreReachesRoof(p) ? 1 : 0));
        CHECK(cell.k0 == 0);
        CHECK(cell.k1 == (cell.kind == CellKind::Stair && coreReachesRoof(p) ? nS : nS - 1));
        if (cell.kind == CellKind::Stair) CHECK(cell.floor.kind == FloorSurface::Kind::Stepped);
    }
    // The same shafts handed to a bare graph set their cells.
    SpaceGraph bare;
    bare.storeys = 2;
    std::vector<StoreyPlan> storeys = storeyPlans(plan, p);
    std::vector<int> hallOf;
    for (int k = 0; k < static_cast<int>(storeys.size()); ++k) {
        Cell h;
        h.kind = CellKind::Hall;
        h.plan = storeys[static_cast<std::size_t>(k)].plan;
        h.k0 = h.k1 = k;
        bare.cells.push_back(h);
        hallOf.push_back(k);
    }
    c.addTo(bare, storeys, 0.0, hallOf, false);
    for (const Shaft& s : c.shafts) {
        if (s.kind == Shaft::Kind::Service) { CHECK(s.cell == -1); continue; }
        CHECK(s.cell >= 0 && s.cell < static_cast<int>(bare.cells.size()));
        CHECK(bare.cells[static_cast<std::size_t>(s.cell)].kind == (s.kind == Shaft::Kind::Stair ? CellKind::Stair : CellKind::Lift));
    }
    CHECK(unreachableCells(g).empty());
}

TEST_CASE(stairs_and_lifts_placed_apart_are_walkable_base_to_top) {
    // STEP 3b (Glenn, 2026-10-15: "disconnect them and place stairs and elevators separately"): a stairwell at each end
    // of the long axis, two banks facing across a lift lobby, and both -- each a block of its own inside every tier,
    // clear of the others; the floors planned round them all and the building walkable from the street to every cell,
    // the stairwells at the ends included. On the recipes whose floors are zoned: a hotel, an office tower, flats.
    struct Case { const char* recipe; Poly2 plan; };
    const Case cases[] = {
        {"hotel", {{-26, -10}, {26, -10}, {26, 10}, {-26, 10}}},
        {"glass_tower", {{-30, -24}, {30, -24}, {30, 24}, {-30, 24}}},
        {"apartments", {{-24, -11}, {24, -11}, {24, 11}, {-24, 11}}},
    };
    struct Lay { uint8_t stairs, lifts; };
    const Lay lays[] = {{2, 1}, {1, 2}, {2, 2}};
    int apart = 0, wantApart = 0;
    for (const Case& cs : cases) {
        for (const Lay& lay : lays) {
            BuildingRecipe rc;
            const OBB2 ob = orientedBoundingBox(cs.plan);
            CHECK(architectRecipeByName(cs.recipe, 11, 1.0, 2 * std::min(ob.half[0], ob.half[1]), area(cs.plan), rc));
            BuildingParams p = rc.params;
            p.openDoorway = true;
            p.core = 2;
            p.stairPlace = lay.stairs;
            p.liftBanks = lay.lifts;
            const std::size_t e = entranceEdgeFor(cs.plan, p);
            const CirculationPlan c = circulationFor(cs.plan, p, e);
            CHECK(c.valid);
            if (!c.valid) continue;
            const CorePlan core = asCorePlan(c);
            const bool ends = lay.stairs == 2, facing = lay.lifts == 2 && count(c, Shaft::Kind::Hoistway) >= 2;
            const bool isApart = c.blocks.size() > 1;
            apart += isApart;
            wantApart += ends || facing;   // (a one-lift building has no second bank to face the first)
            CHECK(c.blocks.size() == static_cast<std::size_t>((ends ? 2 : 0) + (facing ? 2 : 1)));
            CHECK(core.blocks.size() == c.blocks.size());
            CHECK(count(c, Shaft::Kind::Stair) == 2);
            if (ends) {   // the stairwells far apart: across most of the top tier's long axis (every tier holds them)
                const Shaft* a = c.stairs(0);
                const Shaft* b = c.stairs(1);
                CHECK(a && b);
                if (a && b) {
                    const Real sep = (centroid(a->rect()) - centroid(b->rect())).length();
                    const OBB2 tb = orientedBoundingBox(massStack(cs.plan, p).back().plan);
                    CHECK(sep > 0.6 * 2 * std::max(tb.half[0], tb.half[1]));
                    CHECK(a->block != b->block && a->block > 0 && b->block > 0);
                }
            }
            // every block inside every tier, no two overlapping, every shaft inside its block
            const std::vector<MassTier> tiers = massStack(cs.plan, p);
            for (std::size_t i = 0; i < c.blocks.size(); ++i) {
                const Poly2 r = c.blocks[i].rect();
                for (const Vec2& q : r)
                    for (const MassTier& t : tiers) CHECK(pointInPolygon(t.plan, q));
                for (std::size_t j = i + 1; j < c.blocks.size(); ++j) {
                    const Poly2 o = c.blocks[j].rect();
                    for (const Vec2& q : o) CHECK(!pointInPolygon(r, q));
                    CHECK(!pointInPolygon(r, centroid(o)));
                }
            }
            for (const Shaft& s : c.shafts) {
                CHECK(s.block >= 0 && s.block < static_cast<int>(c.blocks.size()));
                const Poly2 r = offsetPlanPublic(c.blocks[static_cast<std::size_t>(s.block)].rect(), -0.01);
                for (const Vec2& q : s.rect()) CHECK(pointInPolygon(r, q));
            }
            // the gate: from the street to every cell, the stairs and lifts placed apart included; no room over a block
            const SpaceGraph g = spaceGraphOf(cs.plan, p, 0.0);
            CHECK(g.modelled);
            const std::vector<int> un = unreachableCells(g);
            CHECK(un.empty());
            int stairCells = 0, liftCells = 0, rooms = 0;
            for (const Cell& cell : g.cells) {
                stairCells += cell.kind == CellKind::Stair;
                liftCells += cell.kind == CellKind::Lift;
                if (cell.kind != CellKind::Room) continue;
                ++rooms;
                for (const Poly2& br : core.rects()) {
                    CHECK(!pointInPolygon(br, centroid(cell.plan)));
                    CHECK(!pointInPolygon(cell.plan, centroid(br)));
                }
            }
            CHECK(stairCells == 2);
            CHECK(liftCells == count(c, Shaft::Kind::Hoistway));
            if (const char* dir = std::getenv("RT_BLUEPRINTS")) {   // the sheets, to look at (RT_BLUEPRINTS=<dir>)
                char name[512];
                std::snprintf(name, sizeof name, "%s/apart_%s_stairs%d_lifts%d.svg", dir, cs.recipe, lay.stairs, lay.lifts);
                if (FILE* f = std::fopen(name, "w")) {
                    const std::string svg = blueprintSvg(g, std::string(cs.recipe) + (lay.stairs == 2 ? ", stairs at the ends" : "") +
                                                                (lay.lifts == 2 ? ", lifts facing" : ""), un);
                    std::fwrite(svg.data(), 1, svg.size(), f);
                    std::fclose(f);
                }
            }
            std::printf("    [apart] %s stairs %d lifts %d: %zu blocks, %d hoistways, %d rooms, %zu unreachable\n", cs.recipe,
                        lay.stairs, lay.lifts, c.blocks.size(), liftCells, rooms, un.size());
        }
    }
    CHECK(apart == wantApart && apart >= 7);
}

// Glenn (2026-10-28): "we should still be able to have elevator cores ... variety when placing elevator shafts for
// different buildings either as variety or because the floor space is too small." Left to the seed, a square
// tower of 30 floors gets the classic core or facing banks, a slab of flats the classic core or stairs at its ends,
// a square mid-rise always the classic core; named, a layout is what every seed gets.
TEST_CASE(the_seed_varies_the_layout_and_a_named_one_is_kept) {
    const Poly2 sq = {{0, 0}, {40, 0}, {40, 40}, {0, 40}};
    const Poly2 slab = {{0, 0}, {48, 0}, {48, 22}, {0, 22}};
    auto tally = [](const Poly2& plan, BuildingParams p, int out[4]) {
        for (uint32_t seed = 1; seed <= 200; ++seed) {
            p.seed = seed;
            const CorePlan c = coreFor(plan, p, entranceEdgeFor(plan, p));
            if (!c.valid) continue;
            const bool facing = c.hoistways.size() >= 2 && c.blocks.size() >= 2 && [&] {
                int lifts = 0;
                for (std::size_t bi = 0; bi < c.blocks.size(); ++bi) {
                    bool has = false;
                    for (const CoreShaft& h : c.hoistways) has = has || h.block == static_cast<int>(bi);
                    lifts += has;
                }
                return lifts >= 2;
            }();
            bool ends = false;   // a stairwell in a block of its own
            for (const CoreStair& st : c.stairs) {
                bool lifts = false;
                for (const CoreShaft& h : c.hoistways) lifts = lifts || st.shaft.block == h.block;
                ends = ends || !lifts;
            }
            ++out[(ends ? 1 : 0) + (facing ? 2 : 0)];
        }
    };
    int tower[4] = {}, flats[4] = {}, mid[4] = {}, named[4] = {};
    tally(sq, towerParams(30), tower);
    BuildingParams fp = towerParams(10);
    fp.residential = true;
    tally(slab, fp, flats);
    tally(sq, towerParams(6), mid);
    BuildingParams np = towerParams(30);
    np.stairPlace = 1;
    np.liftBanks = 1;
    tally(sq, np, named);
    std::printf("    [layouts] tower 30: classic %d facing %d; flats slab: classic %d ends %d; mid-rise classic %d; named classic %d\n",
                tower[0], tower[2], flats[0], flats[1], mid[0], named[0]);
    CHECK(tower[0] > 80 && tower[2] > 40 && tower[1] == 0 && tower[3] == 0);
    CHECK(flats[0] > 50 && flats[1] > 50 && flats[2] == 0 && flats[3] == 0);
    CHECK(mid[0] == 200);
    CHECK(named[0] == 200);
}

TEST_CASE(facing_banks_keep_a_service_block_behind_each) {
    // Two banks facing with their stairs beside them: each row is a stairwell deep, so each has a closed block behind
    // its hoistways, and the CorePlan the walls are built from holds both (it once kept the last one only, leaving a
    // sealed void behind bank A that the space graph had and the mesh didn't).
    const Poly2 sq = {{0, 0}, {48, 0}, {48, 48}, {0, 48}};
    const BuildingParams p = towerParams(30);
    const CirculationPlan c = apartPreset(sq, {MassTier{sq, 0}}, p, entranceEdgeFor(sq, p), false, true);
    CHECK(c.valid && c.banks.size() == 2);
    CHECK(count(c, Shaft::Kind::Service) == 2);
    const CorePlan core = asCorePlan(c);
    CHECK(core.services.size() == 2);
    std::size_t vi = 0;
    for (const Shaft& s : c.shafts)
        if (s.kind == Shaft::Kind::Service) {
            CHECK(sameShaft(core.services[vi], s));
            CHECK(core.services[vi].block == s.block);
            ++vi;
        }
    CHECK(core.services.size() == 2 && core.services[0].block != core.services[1].block);
}

TEST_CASE(a_layout_too_wide_for_its_lifts_gives_way_before_a_lift_is_dropped) {
    // A 30-floor plate 21 x 12 m: three lifts fit in the classic core, but facing banks of three need ~18 m across
    // the lobby. A tower named facing takes the classic core with all three, not facing banks shrunk to one lift.
    const Poly2 plate = {{0, 0}, {21, 0}, {21, 12}, {0, 12}};
    BuildingParams p = towerParams(30);
    p.liftBanks = 2;
    const std::vector<MassTier> tiers = {MassTier{plate, 0}};
    const std::size_t e = entranceEdgeFor(plate, p);
    CHECK(!apartPreset(plate, tiers, p, e, false, true, 3).valid);   // the case: facing at three doesn't fit...
    CHECK(corePreset(plate, tiers, p, e, 3).valid);                   // ...the classic core at three does
    const CirculationPlan c = circulationOf(plate, tiers, p, e);
    CHECK(c.valid && count(c, Shaft::Kind::Hoistway) == 3);
    CHECK(c.blocks.size() == 1 && c.banks.size() == 1);
    // and where nothing takes three, the most that fits: a 14 x 12 m plate holds fewer
    const Poly2 small = {{0, 0}, {14, 0}, {14, 12}, {0, 12}};
    const CirculationPlan s = circulationOf(small, {MassTier{small, 0}}, p, entranceEdgeFor(small, p));
    CHECK(s.valid && count(s, Shaft::Kind::Hoistway) >= 1 && count(s, Shaft::Kind::Hoistway) < 3);
}
