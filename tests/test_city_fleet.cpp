#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cmath>
#include <set>

using namespace engine;
using namespace citysim;

// The composable vehicle fleet (ADR-0061 Phase 4): NPC cars are built from a
// shared body table (dimensions + type), so they come in real sizes — sedans,
// hatchbacks, SUVs, pickups, a van, a box truck — and car-following keeps a gap
// scaled to those lengths, so a longer body never packs tighter than a sedan.

namespace {

// Scenarios (city_test_util.h). This file's straight road is NARROW (5 m → a
// single lane): same-direction cars queue in one line with NO junction and NO
// merges — car-following is the only thing that separates them, so the
// length-aware follow gap can be checked cleanly.
using citytest::cross4;
NavGraph narrowRoad(Real len) { return citytest::straightRoad(len, 5.0); }

}  // namespace

TEST_CASE(fleet_offers_varied_bodies_including_large_ones) {
    CHECK(vehicleFleetSize() >= 6);
    std::set<int> types;
    Real sedanLen = 0, maxLen = 0;
    bool hasBoxTruck = false;
    for (int s = 0; s < vehicleFleetSize(); ++s) {
        const VehicleBody& b = vehicleFleetBody(s);
        types.insert(static_cast<int>(b.type));
        if (b.type == VehicleType::Sedan) sedanLen = b.length;
        if (b.type == VehicleType::BoxTruck) hasBoxTruck = true;
        maxLen = std::max(maxLen, b.length);
        CHECK(b.length > 0 && b.width > 0 && b.height > 0);
    }
    CHECK(types.size() >= 4);         // a genuinely mixed fleet
    CHECK(hasBoxTruck);               // includes a big vehicle
    CHECK(maxLen > sedanLen);         // ...that is longer than the player's sedan
}

TEST_CASE(fleet_slot_wraps_and_is_stable) {
    // A slot beyond the table wraps back into it (drivers index by vehicle number).
    const VehicleBody& a = vehicleFleetBody(0);
    const VehicleBody& b = vehicleFleetBody(vehicleFleetSize());
    CHECK(a.length == b.length);
    CHECK(a.type == b.type);
    // Negative wraps too (defensive).
    const VehicleBody& c = vehicleFleetBody(-vehicleFleetSize());
    CHECK(c.length == a.length);
}

TEST_CASE(drivers_receive_the_fleet_bodies) {
    NavGraph nav = cross4(50.0);
    CitySim sim;
    const int cars = 24;   // > fleet size, so the table is covered and then wraps
    sim.build(nav, cars, 0, 5);

    CHECK(static_cast<int>(sim.vehicles().size()) == cars);
    std::set<int> distinctLen;
    Real maxLen = 0;
    for (std::size_t v = 0; v < sim.vehicles().size(); ++v) {
        const SimVehicle& sv = sim.vehicles()[v];
        const VehicleBody& body = vehicleFleetBody(static_cast<int>(v));
        // The car wears exactly the body the fleet assigns to its slot.
        CHECK(sv.length == body.length);
        CHECK(sv.width == body.width);
        CHECK(sv.height == body.height);
        CHECK(sv.type == body.type);
        distinctLen.insert(static_cast<int>(std::lround(sv.length * 100)));
        maxLen = std::max(maxLen, sv.length);
    }
    CHECK(distinctLen.size() >= 3);   // not all identical
    CHECK(maxLen > 4.2);              // some vehicle is bigger than the sedan
}

TEST_CASE(mixed_fleet_cars_keep_length_aware_gaps) {
    // On a straight single-lane road, same-direction cars queue in one line with no
    // junction or merge, so the length-aware follow gap is the ONLY thing keeping
    // them apart. With the box truck (slot 11) in the fleet, adjacent bodies must
    // never drive through one another: the along-lane centre distance between a car
    // and the one directly ahead stays at least the sum of their half-lengths.
    NavGraph nav = narrowRoad(300.0);
    CitySim sim;
    sim.build(nav, 16, 0, 17);   // 16 > 12 → the box truck is among the fleet

    long sampled = 0, overlaps = 0;
    Real worstPenetration = 0;
    bool sawBigVehicle = false;

    auto lenOf = [&](const Agent& a) -> Real {
        if (a.vehicle >= 0 && a.vehicle < static_cast<int>(sim.vehicles().size()))
            return sim.vehicles()[a.vehicle].length;
        return 4.2;
    };

    for (int i = 0; i < 8000; ++i) {
        sim.step(0.1, 0.5);
        const auto& ag = sim.agents();
        for (std::size_t p = 0; p < ag.size(); ++p) {
            const Agent& A = ag[p];
            if (A.mode != Agent::Mode::Driver || !A.moving) continue;
            if (lenOf(A) > 6.0) sawBigVehicle = true;
            // Only judge an actively-DRIVING follower. Cars start a trip stacked at
            // their shared origin node (all at distOnLeg 0) and sit there until the
            // leader pulls away — a spawn transient, not a car driving through
            // another, so we skip near-stationary cars (as the cross_node test does).
            if (A.speed < 0.5) continue;
            if (A.leg >= static_cast<int>(A.route.links.size())) continue;
            // Find B: the car directly AHEAD of A in the same link + lane (smallest
            // positive distOnLeg gap). That leader is who A must not overrun.
            const Agent* leader = nullptr;
            Real best = 1e9;
            for (std::size_t q = 0; q < ag.size(); ++q) {
                if (q == p) continue;
                const Agent& B = ag[q];
                if (B.mode != Agent::Mode::Driver || !B.moving) continue;
                if (B.leg >= static_cast<int>(B.route.links.size())) continue;
                if (A.route.links[A.leg] != B.route.links[B.leg] || A.lane != B.lane) continue;
                Real d = B.distOnLeg - A.distOnLeg;
                if (d > 0 && d < best) { best = d; leader = &B; }
            }
            if (!leader) continue;
            Real bodyGap = 0.5 * (lenOf(A) + lenOf(*leader));
            ++sampled;
            if (best < bodyGap) {
                ++overlaps;
                worstPenetration = std::max(worstPenetration, bodyGap - best);
            }
        }
    }
    CHECK(sampled > 0);
    CHECK(sawBigVehicle);                        // trucks really drove
    // Following keeps bodies apart: overlaps are rare (a small discrete-step touch)
    // and never deep — no car is ever buried inside the one ahead.
    CHECK(overlaps * 100 < sampled);             // < 1% of samples
    CHECK(worstPenetration < 1.0);
}

TEST_CASE(released_driver_stops_being_driven) {
    // When the player commandeers a car (ADR-0062), the sim releases that agent so
    // its ghost no longer moves and can't fight the now player-driven physical car.
    NavGraph nav = narrowRoad(300.0);
    CitySim sim;
    sim.build(nav, 8, 0, 3);
    // Run until some driver is actually moving, then release it.
    int target = -1;
    for (int i = 0; i < 4000 && target < 0; ++i) {
        sim.step(0.1, 0.5);
        for (std::size_t k = 0; k < sim.agents().size(); ++k)
            if (sim.agents()[k].mode == Agent::Mode::Driver && sim.agents()[k].moving) {
                target = static_cast<int>(k);
                break;
            }
    }
    CHECK(target >= 0);
    sim.releaseDriver(target);
    Vec2 frozen = sim.agents()[target].pos;
    CHECK(!sim.agents()[target].moving);
    // Its pose never changes again, however long the sim runs.
    for (int i = 0; i < 500; ++i) {
        sim.step(0.1, 0.5);
        CHECK(sim.agents()[target].pos.x == frozen.x);
        CHECK(sim.agents()[target].pos.y == frozen.y);
    }
}

TEST_CASE(fleet_assignment_is_deterministic) {
    NavGraph nav = cross4(50.0);
    CitySim a, b;
    a.build(nav, 20, 0, 909);
    b.build(nav, 20, 0, 909);
    bool same = true;
    for (std::size_t v = 0; v < a.vehicles().size(); ++v)
        if (a.vehicles()[v].length != b.vehicles()[v].length ||
            a.vehicles()[v].type != b.vehicles()[v].type)
            same = false;
    CHECK(same);
}

// THE DRIVER GETS OUT (the city's next ten #6; Glenn: "When you steal a car, its driver just vanishes. They should
// get out and stand there."): ejectDriver puts the driver on the pavement beside the car, on foot, still a live agent
// -- not released -- who never gets the stolen car back and walks on when its brain next decides.
TEST_CASE(a_driver_whose_car_is_taken_gets_out_and_walks_on) {
    NavGraph nav = citytest::cityNav(800.0, 80.0, 5);
    CitySim sim;
    sim.build(nav, 30, 40, 5);
    int target = -1;
    for (int i = 0; i < 6000 && target < 0; ++i) {
        sim.step(0.1, 0.5);
        for (std::size_t k = 0; k < sim.agents().size(); ++k) {
            const Agent& a = sim.agents()[k];
            if (a.mode == Agent::Mode::Driver && a.moving && a.vehicle >= 0 && a.car == a.vehicle) { target = static_cast<int>(k); break; }
        }
    }
    CHECK(target >= 0);
    if (target < 0) return;
    const int car = sim.agents()[static_cast<std::size_t>(target)].vehicle;
    const Vec2 was = sim.agents()[static_cast<std::size_t>(target)].pos;
    CHECK(sim.ejectDriver(target));
    const Agent& a = sim.agents()[static_cast<std::size_t>(target)];
    CHECK(a.mode == Agent::Mode::Pedestrian && a.vehicle < 0 && !a.released);
    const Real stood = (a.pos - was).length();
    CHECK(stood < 12.0);   // beside the car, on the pavement
    CHECK(sim.vehicles()[static_cast<std::size_t>(car)].offStreet);
    CHECK(!sim.ejectDriver(target));        // not a driver any more
    Real walked = 0;
    Vec2 last = a.pos;
    bool remounted = false;
    for (int i = 0; i < 20000; ++i) {
        sim.step(0.1, 0.5);
        const Agent& b = sim.agents()[static_cast<std::size_t>(target)];
        if (b.vehicle == car || b.mode == Agent::Mode::Driver) remounted = true;
        walked += (b.pos - last).length();
        last = b.pos;
    }
    std::printf("    [ejected] stood %.1f m from the car's line, then walked %.0f m on foot\n", stood, walked);
    CHECK(!remounted);
    CHECK(walked > 20.0);
}

// THE DEPOT'S PULL-INS ARE STAGGERED (the city's next ten #6): the buses of a 5:30-23:30 service finish over the last
// 75 minutes, a quarter of an hour apart, and pull out over the first 40 -- not all at once (the island's 40 buses
// queued at the depot gates at 23:30).
TEST_CASE(buses_finish_their_service_day_over_an_hour_not_at_once) {
    CitySim sim;
    sim.setBusService(5.5, 23.5);
    std::set<int> ends, starts;
    for (int i = 0; i < 40; ++i) {
        const Real e = sim.busOwnEnd(i), s = sim.busOwnStart(i);
        CHECK(e <= 23.5 + 1e-9 && e >= 22.25 - 1e-9);
        CHECK(s >= 5.5 - 1e-9 && s <= 6.15);
        ends.insert(static_cast<int>(std::lround(e * 4)));
        starts.insert(static_cast<int>(std::lround(s * 8)));
    }
    std::printf("    [depot] %zu different finishing times, %zu pull-out times among 40 buses\n", ends.size(), starts.size());
    CHECK(ends.size() >= 5);
    CHECK(starts.size() >= 5);
}
