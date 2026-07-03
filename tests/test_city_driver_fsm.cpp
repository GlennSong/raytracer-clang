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
