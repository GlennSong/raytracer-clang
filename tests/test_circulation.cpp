// CIRCULATION (circulation.h, issue #105 step 3): shafts, banks and zones as words of their own; today's core as
// the first preset, read back into the CorePlan the emitters take; the shafts as cells of the space graph.
#include "test_framework.h"

#include "../src/engine/procgen/city/circulation.h"
#include "../src/engine/procgen/city/core_plan.h"
#include "../src/engine/procgen/city/shape_grammar.h"
#include "../src/engine/procgen/city/space_graph.h"

#include <cmath>
#include <cstdio>
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
        CHECK(count(c, Shaft::Kind::Service) == (core.hasService ? 1 : 0));
        std::size_t si = 0, hi = 0;
        for (const Shaft& s : c.shafts) {
            if (s.kind == Shaft::Kind::Stair) CHECK(sameShaft(core.stairs[si++].shaft, s));
            else if (s.kind == Shaft::Kind::Hoistway) CHECK(sameShaft(core.hoistways[hi++], s));
            else if (s.kind == Shaft::Kind::Service) CHECK(sameShaft(core.service, s));
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
        std::printf("    [preset] %d floors: %zu shafts, %zu hoistways, service %d, block %.2f x %.2f\n", cs.floors, c.shafts.size(),
                    core.hoistways.size(), core.hasService ? 1 : 0, c.length, c.depth);
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
