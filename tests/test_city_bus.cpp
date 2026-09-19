#include "../src/apps/citysim/city_bus.h"
#include "city_test_util.h"
#include "test_framework.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

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
    // Riders weigh the WAIT: half a headway. One bus a route is a wait of
    // half a lap, which no cross-town trip beats on foot; six is a service.
    net.setFleet({6, 6});

    // Across the city: a bus should be worth taking.
    const BusTrip far = net.planTrip(Vec2(-400, -400), Vec2(400, 400), 250.0);
    CHECK(far.valid());
    // Stop here on failure: an invalid trip is route -1, and indexing it is
    // undefined -- it read as "4 checks failed" on one run and killed the
    // whole suite with a segfault on the next three.
    if (!far.valid()) return;
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
    // AFTER THE RIDE: where each rider got off, and how far from there they
    // are over the next few seconds. Their next trip used to depart from the
    // stop they BOARDED at (restNode was never moved), so a rider stepped off
    // the bus and reappeared a kilometre back where they started.
    std::vector<int> wasOnBus(sim.agents().size(), 0);
    std::vector<Vec2> alightedAt(sim.agents().size(), Vec2(0, 0));
    std::vector<int> sinceAlight(sim.agents().size(), -1);
    long alightings = 0, snappedBack = 0;
    Real worstSnap = 0;

    for (int tick = 0; tick < 90000; ++tick) {
        sim.step(0.05, 0.4);
        for (std::size_t i = 0; i < sim.agents().size(); ++i) {
            const int ai = static_cast<int>(i);
            {
                const int d = sim.rides().driverOf(ai);
                const bool onBus = d >= 0 && sim.isBus(d);
                if (wasOnBus[i] && !onBus) {
                    alightedAt[i] = sim.agents()[i].pos;
                    sinceAlight[i] = 0;
                    ++alightings;
                }
                wasOnBus[i] = onBus ? 1 : 0;
                if (sinceAlight[i] >= 0) {
                    const Real away = dist(sim.agents()[i].pos, alightedAt[i]);
                    // 3 s at walking pace is ~4 m; 60 m is only reachable by a jump.
                    if (away > 60.0) {
                        ++snappedBack;
                        worstSnap = std::max(worstSnap, away);
                        sinceAlight[i] = -1;
                    } else if (++sinceAlight[i] > 60) {
                        sinceAlight[i] = -1;
                    }
                }
            }
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
    std::printf("    [bus] %ld alightings; %ld riders jumped away from where they got "
                "off (worst %.0f m)\n", alightings, snappedBack, worstSnap);
    CHECK(alightings > 0);
    CHECK(snappedBack == 0);
}


// ROUTES SPREAD OUT. The legs between hubs are pathfound, and a fastest-path
// search sends every one of them down the same few arterials: metro's four
// loops drove 23.6 km on 8.5 km of street. Each street a route has used costs
// the next route more, so the share of a route's streets that some other route
// also drives stays a minority.
TEST_CASE(bus_routes_spread_across_streets_instead_of_sharing_them) {
    const NavGraph nav = gridCity(600.0, 100.0, 6);
    BusNetwork net;
    net.build(nav, 3, 10, 21);
    CHECK(net.routeCount() == 3);
    auto key = [&](int a, int b) {
        const long long n = nav.nodeCount();
        return a < b ? a * n + b : b * n + a;
    };
    std::vector<std::vector<long long>> streets(static_cast<std::size_t>(net.routeCount()));
    for (int r = 0; r < net.routeCount(); ++r) {
        const std::vector<int>& p = net.route(r).pathNodes;
        CHECK(p.size() >= 4);
        for (std::size_t k = 0; k + 1 < p.size(); ++k)
            streets[static_cast<std::size_t>(r)].push_back(key(p[k], p[k + 1]));
    }
    Real sharedTotal = 0;
    for (int r = 0; r < net.routeCount(); ++r) {
        int shared = 0;
        for (long long st : streets[static_cast<std::size_t>(r)]) {
            bool other = false;
            for (int q = 0; q < net.routeCount() && !other; ++q) {
                if (q == r) continue;
                for (long long s2 : streets[static_cast<std::size_t>(q)])
                    if (s2 == st) { other = true; break; }
            }
            shared += other ? 1 : 0;
        }
        const Real frac = static_cast<Real>(shared) /
                          static_cast<Real>(std::max<std::size_t>(1, streets[static_cast<std::size_t>(r)].size()));
        std::printf("    [spread] route %d: %.0f%% of its streets shared\n", r, 100.0 * frac);
        sharedTotal += frac;
    }
    CHECK(sharedTotal / net.routeCount() < 0.5);
    CHECK(net.streetShare(nav) > 0.0);
}

// A BUS STARTS ON ITS ROUTE, AND ITS ROUTE'S BUSES START APART. They began
// wherever their drivers were, aimed at stops 0..m-1 -- a route's fleet bunched
// over its first few stops, some a kilometre off their own loop. Now bus k of
// m stands at stop k*N/m, in its vehicle, ready to leave.
TEST_CASE(buses_start_at_their_own_stops_evenly_spaced) {
    NavGraph nav = citytest::cityNav(900.0, 120.0, 4);
    CitySim sim;
    sim.build(nav, 24, 60, 17);
    sim.setBuses(2, 12, 8, 260.0);
    const BusNetwork& net = sim.buses();
    CHECK(net.routeCount() == 2);
    for (int r = 0; r < net.routeCount(); ++r) {
        const BusRoute& route = net.route(r);
        const int n = static_cast<int>(route.stops.size());
        std::vector<int> starts;
        for (int i = 0; i < static_cast<int>(sim.agents().size()); ++i) {
            if (sim.busRouteOf(i) != r) continue;
            const Agent& a = sim.agents()[static_cast<std::size_t>(i)];
            CHECK(a.vehicle >= 0);                       // at the wheel
            const int at = (sim.busNextStopOf(i) - 1 + n) % n;
            starts.push_back(at);
            // Standing at that stop (idlePose puts it at the kerb, not the node).
            CHECK(dist(a.pos, route.stops[static_cast<std::size_t>(at)].pos) < 25.0);
        }
        CHECK(starts.size() == 4);
        std::sort(starts.begin(), starts.end());
        // Evenly spread: consecutive start stops are about N/m apart.
        for (std::size_t k = 1; k < starts.size(); ++k)
            CHECK(starts[k] - starts[k - 1] >= n / 4 - 1);
    }
}


// A BUS STOPS AT A STOP. It used to chain straight into the next leg at speed:
// riders boarded in passing and a player could never get on, since boarding
// needs a standing bus (Glenn: "they should stop and give people time to get on
// and off"). Every served stop now holds the bus still, doors open, for at
// least the base dwell.
TEST_CASE(a_bus_stands_at_each_stop_long_enough_to_board) {
    NavGraph nav = citytest::cityNav(900.0, 120.0, 4);
    CitySim sim;
    sim.build(nav, 24, 60, 17);
    sim.setBuses(2, 12, 4, 260.0);
    const Real dt = 1.0 / 30.0;
    // Timed from the moment a bus SERVES a stop (its next-stop index moves on)
    // to the moment it pulls away. Red lights near a stop do not count: the
    // clock only starts at a service.
    const std::size_t N = sim.agents().size();
    std::vector<int> lastNext(N, -2);
    std::vector<Real> held(N, -1);          // -1 = not timing
    std::vector<Real> shortest(N, 1e9);
    int services = 0;
    Real nearestStand = 1e9, farthestStand = 0;
    for (int t = 0; t < 30 * 240; ++t) {
        sim.step(dt);
        for (int i = 0; i < static_cast<int>(N); ++i) {
            if (sim.busRouteOf(i) < 0) continue;
            const std::size_t k = static_cast<std::size_t>(i);
            const Agent& a = sim.agents()[k];
            const int next = sim.busNextStopOf(i);
            if (lastNext[k] != -2 && next != lastNext[k]) {
                held[k] = 0;
                ++services;
                // WHERE it stands: back from the corner, on the street it came
                // in on -- not on the node, where buses of other routes leaving
                // by the same street were placed on top of it.
                const BusRoute& route = sim.buses().route(sim.busRouteOf(i));
                const Vec2 node = route.stops[static_cast<std::size_t>(lastNext[k])].pos;
                nearestStand = std::min(nearestStand, dist(a.pos, node));
                farthestStand = std::max(farthestStand, dist(a.pos, node));
            }
            lastNext[k] = next;
            if (held[k] < 0) continue;
            if (a.speed <= 0.5) held[k] += dt;
            else { shortest[k] = std::min(shortest[k], held[k]); held[k] = -1; }
        }
    }
    Real worst = 1e9;
    for (std::size_t k = 0; k < N; ++k) worst = std::min(worst, shortest[k]);
    std::printf("    [dwell] %d stop services; shortest stand after a service %.1f s; "
                "a bus stood %.1f..%.1f m from the stop's node\n",
                services, worst, nearestStand, farthestStand);
    CHECK(services >= 8);
    CHECK(worst >= 9.5);
    CHECK(nearestStand >= 5.0);
    CHECK(farthestStand <= 40.0);   // AT the stop it served, not somewhere else
}

// BUSES AT A SHARED STOP DO NOT PILE INTO EACH OTHER. With a dwell, buses of
// three routes arriving at a hub on three different streets all stood on the
// node itself -- inside one another, in the middle of the junction -- and none
// ever left (metro, 2026-09-18). A bus now stands short of the junction on
// the street it came in on, so none should share a spot, and none should stand
// still for longer than a dwell plus a signal cycle.
TEST_CASE(buses_sharing_hub_stops_neither_overlap_nor_deadlock) {
    NavGraph nav = citytest::cityNav(900.0, 120.0, 4);
    CitySim sim;
    sim.build(nav, 24, 60, 17);
    sim.setBuses(3, 12, 12, 260.0);
    const Real dt = 1.0 / 30.0;
    const std::size_t N = sim.agents().size();
    std::vector<int> fleet;
    for (int i = 0; i < static_cast<int>(N); ++i)
        if (sim.busRouteOf(i) >= 0) fleet.push_back(i);
    std::vector<Real> still(N, 0), longestStill(N, 0);
    Real overlapFor = 0, worstOverlap = 0;
    for (int t = 0; t < 30 * 360; ++t) {
        sim.step(dt);
        bool overlapping = false;
        for (std::size_t x = 0; x < fleet.size(); ++x) {
            const Agent& a = sim.agents()[static_cast<std::size_t>(fleet[x])];
            const std::size_t k = static_cast<std::size_t>(fleet[x]);
            still[k] = a.speed <= 0.1 ? still[k] + dt : 0;
            longestStill[k] = std::max(longestStill[k], still[k]);
            for (std::size_t y = x + 1; y < fleet.size(); ++y) {
                const Agent& b = sim.agents()[static_cast<std::size_t>(fleet[y])];
                if (dist(a.pos, b.pos) < 3.0) overlapping = true;
            }
        }
        overlapFor = overlapping ? overlapFor + dt : 0;
        worstOverlap = std::max(worstOverlap, overlapFor);
    }
    Real worstStill = 0;
    for (int i : fleet) worstStill = std::max(worstStill, longestStill[static_cast<std::size_t>(i)]);
    std::printf("    [hub] %zu buses, %ld stops served; longest overlap %.1f s, "
                "longest standstill %.1f s\n",
                fleet.size(), sim.busStopsServed(), worstOverlap, worstStill);
    CHECK(sim.busStopsServed() >= 30);
    CHECK(worstOverlap < 5.0);
    CHECK(worstStill < 100.0);
}

// THE RIDE HAS TO BEAT THE WALK (Glenn: "Is there some algorithm that ... takes
// bus routing into account to help them get across the city faster?"). A loop
// only runs one way, so a stop "near both ends" can still mean riding almost
// the whole loop round. The choice is door-to-door time; with a single bus a
// route the wait alone sinks it.
TEST_CASE(bus_trip_is_chosen_by_door_to_door_time) {
    BusNetwork net;
    net.build(gridCity(), 2, 8, 3);
    net.setFleet({6, 6});
    const Vec2 from(-400, -400), to(400, 400);
    const BusTrip t = net.planTrip(from, to, 250.0);
    CHECK(t.valid());
    if (!t.valid()) return;
    const BusRoute& r = net.route(t.route);
    const Real walk = dist(from, to) * 1.25 / 1.4;
    const Real bus = dist(from, r.stops[static_cast<std::size_t>(t.fromStop)].pos) * 1.25 / 1.4 +
                     net.waitSeconds(t.route) + net.rideSeconds(t.route, t.fromStop, t.toStop) +
                     dist(r.stops[static_cast<std::size_t>(t.toStop)].pos, to) * 1.25 / 1.4;
    std::printf("    [choice] walk %.0f s, bus %.0f s (ride %.0f m of a %.0f m loop)\n", walk,
                bus, net.rideMetres(t.route, t.fromStop, t.toStop), r.loopLength);
    CHECK(bus < walk - 60.0);
    // A ride is measured FORWARD round the loop: stop b to stop a is the rest
    // of the loop, not the way back.
    CHECK(std::fabs(net.rideMetres(t.route, t.fromStop, t.toStop) +
                    net.rideMetres(t.route, t.toStop, t.fromStop) - r.loopLength) < 1e-6);
    // One bus a route: waiting half a lap is not worth it.
    net.setFleet({1, 1});
    CHECK(!net.planTrip(from, to, 250.0).valid());
}
