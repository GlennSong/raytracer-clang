#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

#include <array>
#include <cmath>

using namespace engine;
using namespace citysim;

// The driver finite state machine (ADR-0061 Phase 3): each step a car labels what
// is governing it — Cruising (free), Following (a leader in its lane), Yielding
// (a person in its vision cone), Turning (a bend at the coming node), or Waiting
// (held at a red). These tests drive scenarios that should each surface a state
// and assert the label appears (and that a driver never wears a pedestrian state).

namespace {

// Scenarios (city_test_util.h): straightRoad gives a car a clear open stretch to
// reach free speed, so it must report Cruising; on cross4 routes between arms
// bend 90 deg through a signalled centre and many cars share the approaches — so
// Turning, Waiting (red) and Following all occur over a busy run.
using citytest::cross4;
using citytest::straightRoad;

// Which behaviour states any DRIVER wore at least once over `steps`.
std::array<bool, static_cast<int>(Agent::State::Count)> observeDriverStates(
    CitySim& sim, int steps) {
    std::array<bool, static_cast<int>(Agent::State::Count)> seen{};
    for (int i = 0; i < steps; ++i) {
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents())
            if (a.mode == Agent::Mode::Driver && a.moving)
                seen[static_cast<int>(a.state)] = true;
    }
    return seen;
}

bool sawState(const std::array<bool, static_cast<int>(Agent::State::Count)>& s,
              Agent::State st) {
    return s[static_cast<int>(st)];
}

}  // namespace

TEST_CASE(lone_driver_cruises_the_open_road) {
    NavGraph nav = straightRoad(160.0);
    CitySim sim;
    sim.build(nav, 1, 0, 3);

    auto seen = observeDriverStates(sim, 4000);
    CHECK(sawState(seen, Agent::State::Cruising));   // reached free flow on the clear stretch
    // A driver must never wear a pedestrian-only state.
    CHECK(!sawState(seen, Agent::State::Walking));
    CHECK(!sawState(seen, Agent::State::Avoiding));
    // Nothing was ever in front of it: no leader, no signal, no person.
    CHECK(!sawState(seen, Agent::State::Following));
    CHECK(!sawState(seen, Agent::State::Waiting));
    CHECK(!sawState(seen, Agent::State::Yielding));
}

TEST_CASE(driver_yields_to_a_person_in_its_path) {
    NavGraph nav = straightRoad(160.0);
    CitySim sim;
    sim.build(nav, 1, 0, 3);

    bool yielded = false;
    for (int i = 0; i < 4000 && !yielded; ++i) {
        // Once the car is rolling, drop a person ~8 m dead ahead in its lane; the
        // car should see it in its vision cone and brake — the Yielding state.
        const Agent& car = sim.agents().front();
        if (car.moving) {
            Vec2 ahead(car.pos.x + car.heading.x * 8.0, car.pos.y + car.heading.y * 8.0);
            sim.setExternalObstacles({ ahead });
        }
        sim.step(0.1, 0.5);
        const Agent& c = sim.agents().front();
        if (c.mode == Agent::Mode::Driver && c.moving && c.state == Agent::State::Yielding)
            yielded = true;
    }
    CHECK(yielded);
}

TEST_CASE(a_car_stops_behind_the_players_stopped_car_not_into_it) {
    // Glenn, 2026-10-02: "If my car stops the car behind me just rams me." The player's car reached the sim as a
    // PERSON: seen only inside an 18 m cone (too late from 50 km/h) and held short by a person's 4 m measured to the
    // car's CENTRE -- the follower's nose ended inside it. A vehicle obstacle is now seen down the lane far enough
    // to stop, and held short by both bodies' lengths.
    NavGraph nav = straightRoad(600.0);
    CitySim sim;
    sim.build(nav, 1, 0, 3);
    const Real half = 2.2;   // the player's car
    bool placed = false;
    Vec2 stopped(0, 0);
    Real topSpeed = 0, closest = 1e9, ownHalf = 0;
    for (int i = 0; i < 6000; ++i) {
        const Agent& car = sim.agents().front();
        if (!placed && car.moving && car.speed > 7.0) {
            // the player's car, stopped dead in this lane 40 m ahead
            stopped = Vec2(car.pos.x + car.heading.x * 40.0, car.pos.y + car.heading.y * 40.0);
            placed = true;
            ownHalf = sim.fleetBody(car.vehicle).length * 0.5;
        }
        if (placed) sim.setExternalObstacles({ stopped }, { half });
        sim.step(0.05, 0.5);
        const Agent& c = sim.agents().front();
        topSpeed = std::max(topSpeed, c.speed);
        if (placed) closest = std::min(closest, (c.pos - stopped).length());
    }
    std::printf("    [rear-end] top %.1f m/s, closest centre-to-centre %.2f m (bodies %.2f m)\n", topSpeed, closest,
                ownHalf + half);
    CHECK(placed);
    CHECK(closest > ownHalf + half + 0.5);   // bumper to bumper, never into it
    CHECK(closest < ownHalf + half + 4.0);   // and it did pull up behind, not stop a block away

    // A car in the NEXT lane over (4 m across) is passed, not braked for
    CitySim sim2;
    sim2.build(nav, 1, 0, 3);
    bool placed2 = false;
    Vec2 beside(0, 0), dir0(0, 0);
    Real slowestAlongside = 1e9;
    for (int i = 0; i < 6000; ++i) {
        const Agent& car = sim2.agents().front();
        if (!placed2 && car.moving && car.speed > 7.0) {
            const Vec2 side(-car.heading.y, car.heading.x);
            beside = Vec2(car.pos.x + car.heading.x * 40.0 + side.x * 4.0, car.pos.y + car.heading.y * 40.0 + side.y * 4.0);
            dir0 = car.heading;
            placed2 = true;
        }
        if (placed2) sim2.setExternalObstacles({ beside }, { half });
        sim2.step(0.05, 0.5);
        const Agent& c = sim2.agents().front();
        // the first pass only: back from the road's end it drives in THAT lane, and should stop
        if (placed2 && c.heading.x * dir0.x + c.heading.y * dir0.y > 0.5 && (c.pos - beside).length() < 8.0)
            slowestAlongside = std::min(slowestAlongside, c.speed);
    }
    std::printf("    [next lane] slowest passing it %.1f m/s\n", slowestAlongside);
    CHECK(slowestAlongside > 5.0 && slowestAlongside < 1e8);
}

TEST_CASE(lane_changes_ease_turn_into_the_change_and_never_overlap) {
    // Glenn, 2026-10-02: "Lane changing for simulated cars is wonky." The glide was a constant-rate slide with the
    // nose pointing straight down the lane (a crab), and the changer left its old lane's follow chain the instant it
    // decided. Now: the sideways rate eases in from zero, the heading turns into the change, and a car mid-change is
    // in both lanes' chains -- so no two same-way cars ever share space.
    RoadGraph g;   // an arterial: two lanes each way (lanes come from the class, not the width)
    g.nodes = { {Vec2(0, 0)}, {Vec2(900, 0)} };
    g.edges = { RoadEdge{0, 1, 16, RoadClass::Arterial, 0} };
    NavGraph nav = buildNavGraph(g);
    CHECK(nav.links[0].lanes >= 2);
    CitySim sim;
    sim.build(nav, 24, 0, 41);
    int changes = 0, yawed = 0, overlaps = 0;
    Real worstJerk = 0;   // the largest one-step jump in sideways rate (lanes/s)
    std::vector<Real> lastVel(sim.agents().size(), 0.0);
    std::vector<int> lastLane(sim.agents().size(), -1);
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.05, 0.5);
        const auto& ag = sim.agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            const Agent& a = ag[k];
            if (a.mode != Agent::Mode::Driver || !a.moving || a.leg >= static_cast<int>(a.route.links.size())) continue;
            if (lastLane[k] >= 0 && a.lane != lastLane[k]) ++changes;
            lastLane[k] = a.lane;
            worstJerk = std::max(worstJerk, std::fabs(a.laneVel - lastVel[k]));
            lastVel[k] = a.laneVel;
            if (std::fabs(a.laneVel) > 0.3 && a.speed > 3.0) {
                const Vec2 d = nav.direction(a.route.links[a.leg]);
                const Real sinA = d.x * a.heading.y - d.y * a.heading.x;
                if (std::fabs(sinA) > std::sin(1.0 * 3.14159265 / 180.0)) ++yawed;
            }
            for (std::size_t j = k + 1; j < ag.size(); ++j) {
                const Agent& b = ag[j];
                if (b.mode != Agent::Mode::Driver || !b.moving) continue;
                if (a.heading.x * b.heading.x + a.heading.y * b.heading.y < 0.7) continue;
                const Real dx = b.pos.x - a.pos.x, dy = b.pos.y - a.pos.y;
                const Real along = std::fabs(a.heading.x * dx + a.heading.y * dy);
                const Real lat = std::fabs(a.heading.y * dx - a.heading.x * dy);
                const Real reach = 0.5 * (sim.fleetBody(a.vehicle).length + sim.fleetBody(b.vehicle).length) - 0.3;
                if (lat < 1.5 && along < reach) ++overlaps;
            }
        }
    }
    std::printf("    [lanes] %d changes, %d steps yawed into a change, worst sideways-rate step %.3f lanes/s, %d overlaps\n",
                changes, yawed, worstJerk, overlaps);
    CHECK(changes > 0);
    CHECK(yawed > 0);              // the nose turns into the change
    CHECK(worstJerk < 0.2);        // eased: the rate builds over steps (the old slide jumped 0 -> 0.55 in one)
    CHECK(overlaps == 0);
}

TEST_CASE(busy_traffic_shows_following_turning_and_waiting) {
    NavGraph nav = cross4(60.0);
    CitySim sim;
    sim.build(nav, 20, 0, 17);   // packed approaches + a signalled junction

    auto seen = observeDriverStates(sim, 12000);
    CHECK(sawState(seen, Agent::State::Cruising));    // some car ran free
    CHECK(sawState(seen, Agent::State::Following));   // some car queued behind a leader
    CHECK(sawState(seen, Agent::State::Turning));     // some car arced through the cross
    CHECK(sawState(seen, Agent::State::Waiting));     // some car held at a red
}

TEST_CASE(driver_fsm_is_deterministic) {
    NavGraph nav = cross4(60.0);
    CitySim a, b;
    a.build(nav, 16, 0, 909);
    b.build(nav, 16, 0, 909);
    bool same = true;
    for (int i = 0; i < 3000; ++i) {
        a.step(0.1, 0.5);
        b.step(0.1, 0.5);
        for (std::size_t k = 0; k < a.agents().size(); ++k)
            if (a.agents()[k].state != b.agents()[k].state) same = false;
    }
    CHECK(same);
}
