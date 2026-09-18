#include "../src/apps/citysim/city_bus.h"
#include "city_test_util.h"
#include "test_framework.h"

#include <cmath>

using namespace engine;
using namespace citysim;

// The bus NETWORK, on its own: routes derived from node positions, with no
// graph, no sim and no clock. Deriving rather than authoring is the whole point
// — a route written into a level file would name nodes that the next city
// regeneration moves or deletes — so these tests are mostly about the derivation
// being sane on an arbitrary set of points, and identical every time.

namespace {

// Routes are pathfound over STREETS now, so the fixture is a real grid city
// rather than a bag of points -- a network that cannot route has no routes.
NavGraph gridCity(Real extent = 500.0, Real cell = 120.0, uint32_t seed = 4) {
    return citytest::cityNav(extent, cell, seed);
}

Real dist(Vec2 a, Vec2 b) {
    const Real dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

TEST_CASE(bus_network_lays_out_the_routes_it_was_asked_for) {
    BusNetwork net;
    CHECK(net.empty());
    net.build(gridCity(), 3, 6, 11);
    CHECK(net.routeCount() == 3);
    for (int r = 0; r < net.routeCount(); ++r) {
        const BusRoute& route = net.route(r);
        CHECK(route.valid());
        CHECK(route.stops.size() >= 3);   // spacing drives the count now
        // Every stop sits ON a node, and no stop repeats within a route.
        std::vector<int> seen;
        for (const BusStop& s : route.stops) {
            CHECK(s.node >= 0);
            for (int p : seen) CHECK(p != s.node);
            seen.push_back(s.node);
        }
    }
}

TEST_CASE(bus_stops_spread_across_the_city_instead_of_clumping) {
    // Farthest-point sampling is the reason this holds: a route whose stops all
    // landed in one district would be useless, and is what a naive "pick N
    // random nodes" would produce often enough to matter.
    BusNetwork net;
    net.build(gridCity(), 1, 8, 5);
    const BusRoute& route = net.route(0);

    Real spanX = 0, spanY = 0;
    for (const BusStop& a : route.stops)
        for (const BusStop& b : route.stops) {
            spanX = std::max(spanX, std::fabs(a.pos.x - b.pos.x));
            spanY = std::max(spanY, std::fabs(a.pos.y - b.pos.y));
        }
    // Covers most of the map in both directions, not a corner of it.
    CHECK(spanX > 250.0);
    CHECK(spanY > 250.0);
}

TEST_CASE(bus_network_is_reproducible_and_routes_differ) {
    const NavGraph nav = gridCity();
    BusNetwork a, b;
    a.build(nav, 3, 5, 99);
    b.build(nav, 3, 5, 99);
    CHECK(a.routeCount() == b.routeCount());
    for (int r = 0; r < a.routeCount(); ++r) {
        CHECK(a.route(r).stops.size() == b.route(r).stops.size());
        for (std::size_t i = 0; i < a.route(r).stops.size(); ++i)
            CHECK(a.route(r).stops[i].node == b.route(r).stops[i].node);
    }
    // Two routes of the same network are not the same loop.
    bool anyDifferent = false;
    for (std::size_t i = 0; i < a.route(0).stops.size(); ++i)
        if (a.route(0).stops[i].node != a.route(1).stops[i].node) anyDifferent = true;
    CHECK(anyDifferent);
}

TEST_CASE(bus_network_refuses_impossible_layouts) {
    BusNetwork net;
    net.build(NavGraph{}, 2, 4, 1);               // no city
    CHECK(net.empty());
    net.build(gridCity(), 0, 4, 1);               // no routes asked for
    CHECK(net.empty());
    net.build(gridCity(), 2, 1, 1);               // a "route" of one stop
    CHECK(net.empty());
    CHECK(net.nearestStop(0, Vec2(0, 0)) == -1);
    CHECK(!net.planTrip(Vec2(0, 0), Vec2(100, 100), 200.0).valid());
}

TEST_CASE(bus_trip_is_planned_only_when_the_bus_actually_helps) {
    BusNetwork net;
    net.build(gridCity(), 2, 8, 3);

    // Across the city: a bus should be worth taking.
    const BusTrip far = net.planTrip(Vec2(-400, -400), Vec2(400, 400), 250.0);
    CHECK(far.valid());
    CHECK(far.fromStop != far.toStop);
    const BusRoute& r = net.route(far.route);
    // Both ends are within the walk bound the caller allowed.
    CHECK(dist(Vec2(-400, -400), r.stops[static_cast<std::size_t>(far.fromStop)].pos) <= 250.0);
    CHECK(dist(Vec2(400, 400), r.stops[static_cast<std::size_t>(far.toStop)].pos) <= 250.0);

    // A few steps down the street: walking beats waiting, so no trip.
    CHECK(!net.planTrip(Vec2(0, 0), Vec2(25, 0), 250.0).valid());
    // Nothing within the walk bound: no trip rather than a silly one.
    CHECK(!net.planTrip(Vec2(-400, -400), Vec2(400, 400), 5.0).valid());
}

TEST_CASE(bus_waiting_lists_are_per_stop_and_ascending) {
    BusNetwork net;
    net.build(gridCity(), 1, 8, 7);
    BusTrip t;
    t.route = 0; t.fromStop = 2; t.toStop = 4;

    CHECK(net.waitFor(40, t));
    CHECK(net.waitFor(3, t));
    CHECK(net.waitFor(17, t));
    CHECK(!net.waitFor(3, t));            // already waiting
    CHECK(net.waitingCount() == 3);
    CHECK(net.tripOf(17) != nullptr);
    CHECK(net.tripOf(99) == nullptr);

    const std::vector<int> at2 = net.waitingAt(0, 2);
    CHECK(at2.size() == 3);
    for (std::size_t i = 1; i < at2.size(); ++i) CHECK(at2[i - 1] < at2[i]);
    CHECK(at2[0] == 3);

    CHECK(net.waitingAt(0, 5).empty());   // nobody at that stop
    net.stopWaiting(3);
    CHECK(net.waitingCount() == 2);
    CHECK(net.tripOf(3) == nullptr);

    BusTrip bad;
    CHECK(!net.waitFor(5, bad));          // malformed
    t.route = 9;
    CHECK(!net.waitFor(6, t));            // no such route
}

// --- and now the whole thing, in a running city ----------------------------
#include "../src/apps/citysim/city_sim.h"

TEST_CASE(buses_drive_their_loop_and_carry_riders) {
    // NOBODY drives this from outside: no test calls waitFor or boards anyone.
    // Walkers choose a bus themselves when one saves them walking, walk to the
    // stop, wait, ride, and get off. Asserting the JOURNEY — a rider carried a
    // real distance by a vehicle that is actually a bus — because a flag check
    // would pass on a bus that never moved.
    NavGraph nav = citytest::cityNav(900.0, 120.0, 4);
    CitySim sim;
    sim.build(nav, 24, 60, 17);
    // HEADWAY, not mechanism, is what makes this test pass or fail: 4 buses on
    // 8-stop loops completed barely ONE lap in the old 30000 ticks, so a rider
    // who started waiting after their stop was served never saw another bus.
    // More stops (more walkers are within range of one), more buses per route
    // (shorter headway) and a longer run.
    sim.setBuses(2, 12, 8, 260.0);

    CHECK(!sim.buses().empty());
    CHECK(sim.buses().routeCount() == 2);
    int busCount = 0;
    for (std::size_t i = 0; i < sim.agents().size(); ++i)
        if (sim.isBus(static_cast<int>(i))) ++busCount;
    CHECK(busCount == 8);

    // Where every bus starts, so we can prove they went somewhere.
    std::vector<Vec2> busStart(sim.agents().size(), Vec2(0, 0));
    for (std::size_t i = 0; i < sim.agents().size(); ++i)
        busStart[i] = sim.agents()[i].pos;

    long everWaited = 0, everRode = 0;
    Real farthestCarried = 0, farthestBusDrove = 0;
    std::vector<char> waitedOnce(sim.agents().size(), 0);
    std::vector<char> rodeOnce(sim.agents().size(), 0);
    std::vector<Vec2> boardedAt(sim.agents().size(), Vec2(0, 0));

    for (int tick = 0; tick < 90000; ++tick) {
        sim.step(0.05, 0.4);
        for (std::size_t i = 0; i < sim.agents().size(); ++i) {
            const int ai = static_cast<int>(i);
            if (sim.isBus(ai))
                farthestBusDrove = std::max(farthestBusDrove,
                                            dist(sim.agents()[i].pos, busStart[i]));
            if (!waitedOnce[i] && sim.buses().tripOf(ai)) { waitedOnce[i] = 1; ++everWaited; }
            const int drv = sim.rides().driverOf(ai);
            if (drv >= 0 && sim.isBus(drv)) {
                if (!rodeOnce[i]) { rodeOnce[i] = 1; ++everRode; boardedAt[i] = sim.agents()[i].pos; }
                farthestCarried = std::max(farthestCarried,
                                           dist(sim.agents()[i].pos, boardedAt[i]));
            }
        }
    }

    CHECK(farthestBusDrove > 100.0);   // the buses actually drove their loops
    CHECK(everWaited > 0);             // walkers chose a bus, unprompted
    CHECK(everRode > 0);               // and one of them got on
    CHECK(farthestCarried > 50.0);     // and was carried, not just flagged
    CHECK(sim.busStopsServed() > 50);        // the loops really were driven
    CHECK(sim.busBoardAttempts() > 0);       // and buses OFFERED rides
    CHECK(sim.busBoardRefused() == 0);       // every offer was taken up
    // EVIDENCE for the fix above: buses really do meet stops they cannot route
    // to, so the skip is load-bearing rather than defensive. Before it, this
    // was the stall that kept everRode at zero.
    std::printf("    [bus] skipped %ld, SERVED %ld stops, waited %ld, rode %ld, carried %.0f m\n",
                sim.busSkippedLegs(), sim.busStopsServed(), everWaited, everRode, farthestCarried);
    std::printf("    [bus] board attempts %ld, refused %ld\n",
                sim.busBoardAttempts(), sim.busBoardRefused());
}
