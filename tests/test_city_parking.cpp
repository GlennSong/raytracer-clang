// Curbside parking (roads-v2.1 R6b, plan 4d phase 1): marked parallel bays
// mid-link on at-grade Local streets, seeded ~half full with scenery cars;
// an arriving driver claims a free bay on its arrival link (the old grass
// verge remains the fallback), and pulling out frees it. Gates, per the
// plan: bays never crowd junction mouths, arrivals really park and depart,
// scenery bays are never stolen, and no bay ever holds two cars.
#include "test_framework.h"

#include "../src/apps/citysim/city_sim.h"
#include "city_test_util.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;
using namespace citysim;

namespace {

// A wide 4-way cross (10 m carriageways, 100 m arms) — wide and long enough
// to carry curbside bays, unlike the 8 m test fixtures (which must stay
// bay-free so the older flow gates keep their geometry).
NavGraph wideCross() {
    RoadGraph g;
    g.nodes = { { Vec2(0, 0) }, { Vec2(0, 100) }, { Vec2(0, -100) },
                { Vec2(100, 0) }, { Vec2(-100, 0) } };
    g.edges = {
        RoadEdge{ 0, 1, 10, RoadClass::Local, 0 },
        RoadEdge{ 0, 2, 10, RoadClass::Local, 0 },
        RoadEdge{ 0, 3, 10, RoadClass::Local, 0 },
        RoadEdge{ 0, 4, 10, RoadClass::Local, 0 },
    };
    return buildNavGraph(g);
}

}  // namespace

TEST_CASE(parking_bays_stay_clear_of_junction_mouths) {
    NavGraph nav = wideCross();
    CitySim sim;
    sim.build(nav, 0, 0, 5);
    const auto& bays = sim.parkingBays();
    CHECK(!bays.empty());

    int scenery = 0;
    for (const CitySim::ParkingBay& b : bays) {
        // Clear of every junction node (the mouth machinery owns that zone).
        for (int ni = 0; ni < nav.nodeCount(); ++ni)
            CHECK((b.pos - nav.nodes[ni]).length() >= 12.0);
        // Hugs the right curb of its own link: lateral offset just inside
        // the carriageway edge, never in the travel lane or the sidewalk.
        const Vec2 dir = nav.direction(b.link);
        const Vec2 rel = b.pos - nav.nodes[nav.links[b.link].from];
        const Real lat = rel.x * dir.y - rel.y * dir.x;   // + = right of dir
        CHECK(lat > 2.9);
        CHECK(lat < 5.0);
        if (b.occupant == CitySim::kBayScenery) ++scenery;
    }
    const Real frac = Real(scenery) / Real(bays.size());
    std::printf("[parking] bays=%zu sceneryFrac=%.2f\n", bays.size(), frac);
    CHECK(frac > 0.3);
    CHECK(frac < 0.8);
}

TEST_CASE(parking_arrivals_claim_bays_and_departures_free_them) {
    NavGraph nav = wideCross();
    CitySim sim;
    sim.build(nav, 10, 0, 9);   // scheduled day: commutes arrive + rest

    long parkTicks = 0, freedAfterPark = 0;
    bool sceneryStolen = false, doubleParked = false, wrongPose = false;
    std::vector<int> lastOccupant(sim.parkingBays().size(), -1);
    for (std::size_t i = 0; i < sim.parkingBays().size(); ++i)
        lastOccupant[i] = sim.parkingBays()[i].occupant;

    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.5);
        const auto& ag = sim.agents();
        const auto& bays = sim.parkingBays();
        std::vector<int> users(bays.size(), 0);
        for (std::size_t ai = 0; ai < ag.size(); ++ai) {
            const Agent& a = ag[ai];
            if (a.parkedBay < 0) continue;
            ++parkTicks;
            ++users[a.parkedBay];
            const CitySim::ParkingBay& b = bays[a.parkedBay];
            if (b.occupant != static_cast<int>(ai)) doubleParked = true;
            if ((a.pos - b.pos).length() > 0.5) wrongPose = true;
            if (a.moving) wrongPose = true;
        }
        for (std::size_t bi = 0; bi < bays.size(); ++bi) {
            if (users[bi] > 1) doubleParked = true;
            if (lastOccupant[bi] == CitySim::kBayScenery &&
                bays[bi].occupant != CitySim::kBayScenery)
                sceneryStolen = true;
            // A bay that held an agent and is free again = a real departure.
            if (lastOccupant[bi] >= 0 && bays[bi].occupant == -1)
                ++freedAfterPark;
            lastOccupant[bi] = bays[bi].occupant;
        }
    }
    std::printf("[parking] parkTicks=%ld departures=%ld\n", parkTicks,
                freedAfterPark);
    CHECK(parkTicks > 0);          // drivers really rest in bays
    CHECK(freedAfterPark > 0);     // and pull out again
    CHECK(!sceneryStolen);
    CHECK(!doubleParked);
    CHECK(!wrongPose);
}
