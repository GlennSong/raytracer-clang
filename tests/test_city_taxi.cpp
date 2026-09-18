#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"

#include <cmath>

using namespace engine;
using namespace citysim;

// A CAB THAT ACTUALLY DRIVES A FARE (Glenn: "we should make buses and taxis and
// Lyfts for npcs to get around the city"). Dispatch and RideBook are each tested
// alone; this is the one that says the whole chain works — hail, match, drive to
// the pickup, board, drive to the destination, set down.
//
// It asserts the JOURNEY, not just the flags: a cab that teleports its fare, or
// reports a delivery without moving, passes a flag check and fails this. That
// failure mode is not hypothetical — an earlier elevator probe reported a clean
// ride on a cab that never left the lobby.

namespace {

Real dist(Vec2 a, Vec2 b) {
    const Real dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

TEST_CASE(taxi_collects_a_fare_and_delivers_it) {
    NavGraph nav = citytest::cityNav(600.0, 120.0, 7);
    CitySim sim;
    sim.setWander(true);          // no schedules: everyone just moves
    sim.build(nav, 6, 6, 21);
    const int cab = 0;            // [0, drivers) are drivers
    const int rider = 6;          // [drivers, ...) are pedestrians
    CHECK(sim.agents().size() > static_cast<std::size_t>(rider));

    sim.setTaxi(cab, true);
    CHECK(sim.isTaxi(cab));
    CHECK(!sim.isTaxi(1));

    // Pick two far-apart nodes so the ride is a real journey, not a nudge.
    int pickup = 0, drop = 0;
    Real best = 0;
    for (std::size_t i = 0; i < nav.nodes.size(); ++i)
        for (std::size_t j = i + 1; j < nav.nodes.size(); ++j) {
            const Real d = dist(nav.nodes[i], nav.nodes[j]);
            if (d > best) { best = d; pickup = static_cast<int>(i); drop = static_cast<int>(j); }
        }
    CHECK(best > 200.0);

    CHECK(sim.hail(rider, pickup, drop));
    CHECK(sim.dispatch().waiting() == 1);
    CHECK(sim.awaitingRide(rider));
    // Hailing PARKS the rider: they wait to be collected instead of walking on.
    CHECK(!sim.agents()[static_cast<std::size_t>(rider)].moving);

    bool matched = false, boarded = false, delivered = false;
    Vec2 carriedFrom(0, 0);
    Real carriedDistance = 0;
    for (int tick = 0; tick < 40000 && !delivered; ++tick) {
        sim.step(0.05, 0.0);      // no clock advance: the day must not interfere
        const Vec2 riderPos = sim.agents()[static_cast<std::size_t>(rider)].pos;
        if (!matched && sim.dispatch().fareOf(cab)) matched = true;
        if (!boarded && sim.riding(rider)) {
            boarded = true;
            carriedFrom = riderPos;
            // Boarding happens AT the pickup, not wherever the rider stood.
            CHECK(dist(riderPos, nav.nodes[static_cast<std::size_t>(pickup)]) < 40.0);
        }
        if (boarded) {
            carriedDistance = std::max(carriedDistance, dist(riderPos, carriedFrom));
            if (!sim.riding(rider)) delivered = true;
        }
    }

    CHECK(matched);        // the free cab took the waiting hail
    CHECK(boarded);        // and actually collected them
    CHECK(delivered);      // and set them down again
    // THE JOURNEY: they were carried a real distance, and ended up near the
    // destination they asked for rather than anywhere the cab happened to stop.
    CHECK(carriedDistance > 100.0);
    CHECK(dist(sim.agents()[static_cast<std::size_t>(rider)].pos,
               nav.nodes[static_cast<std::size_t>(drop)]) < 60.0);
    // The cab is free again, and the rider is back on their own feet.
    CHECK(sim.dispatch().fareOf(cab) == nullptr);
    CHECK(!sim.riding(rider));
    CHECK(!sim.awaitingRide(rider));
}

TEST_CASE(taxi_carries_its_passenger_rather_than_leaving_them_behind) {
    // While aboard, the rider's position IS the cab's — they move together, and
    // the rider contributes no walking of their own.
    NavGraph nav = citytest::cityNav(600.0, 120.0, 3);
    CitySim sim;
    sim.setWander(true);
    sim.build(nav, 6, 6, 5);
    const int cab = 0, rider = 6;
    sim.setTaxi(cab, true);

    int pickup = 0, drop = 0;
    Real best = 0;
    for (std::size_t i = 0; i < nav.nodes.size(); ++i)
        for (std::size_t j = i + 1; j < nav.nodes.size(); ++j) {
            const Real d = dist(nav.nodes[i], nav.nodes[j]);
            if (d > best) { best = d; pickup = static_cast<int>(i); drop = static_cast<int>(j); }
        }
    CHECK(sim.hail(rider, pickup, drop));

    long sampled = 0;
    Real worstGap = 0;
    for (int tick = 0; tick < 40000; ++tick) {
        sim.step(0.05, 0.0);
        if (!sim.riding(rider)) { if (sampled > 0) break; continue; }
        const Vec2 r = sim.agents()[static_cast<std::size_t>(rider)].pos;
        const Vec2 c = sim.agents()[static_cast<std::size_t>(cab)].pos;
        worstGap = std::max(worstGap, dist(r, c));
        ++sampled;
    }
    CHECK(sampled > 20);       // there really was a ride to sample
    CHECK(worstGap < 0.01);    // and the rider never trailed the cab
}

TEST_CASE(walkers_hail_cabs_on_their_own) {
    // The whole chain with NOBODY driving it from outside: some drivers work as
    // cabs, some walkers facing a long trip hail one, and rides happen. If this
    // passes only because the test called hail(), it is worthless — so it never
    // calls hail() at all.
    NavGraph nav = citytest::cityNav(900.0, 120.0, 9);
    CitySim sim;
    sim.build(nav, 40, 80, 13);
    sim.setTaxiFraction(0.25);
    sim.setHailPolicy(0.6, 250.0);

    int cabs = 0;
    for (std::size_t i = 0; i < sim.agents().size(); ++i)
        if (sim.isTaxi(static_cast<int>(i))) ++cabs;
    CHECK(cabs > 0);
    CHECK(cabs < 40);            // a QUARTER of the drivers, not all of them

    long everHailed = 0, everRode = 0;
    std::vector<char> rodeOnce(sim.agents().size(), 0);
    std::vector<char> hailedOnce(sim.agents().size(), 0);
    for (int tick = 0; tick < 24000; ++tick) {
        sim.step(0.05, 0.4);     // let the clock run: schedules drive departures
        for (std::size_t i = 0; i < sim.agents().size(); ++i) {
            const int ai = static_cast<int>(i);
            if (!hailedOnce[i] && sim.awaitingRide(ai)) { hailedOnce[i] = 1; ++everHailed; }
            if (!rodeOnce[i] && sim.riding(ai)) { rodeOnce[i] = 1; ++everRode; }
        }
    }
    CHECK(everHailed > 0);       // walkers decided to hail, unprompted
    CHECK(everRode > 0);         // and cabs actually collected them
}
