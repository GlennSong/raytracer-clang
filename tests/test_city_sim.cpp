#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

using namespace engine;
using namespace citysim;

namespace {
// This file's grid town (city_test_util.h cityNav, historical dimensions/seed).
NavGraph cityNav() { return citytest::cityNav(200, 50, 7); }
}  // namespace

TEST_CASE(city_build_possession) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 20, 10, 1234);
    CHECK(sim.agents().size() == 30u);
    CHECK(sim.vehicles().size() == 20u);   // one car per driver, none for pedestrians

    int drivers = 0, peds = 0;
    for (const Agent& a : sim.agents()) {
        if (a.mode == Agent::Mode::Driver) {
            ++drivers;
            CHECK(a.vehicle >= 0);                       // a driver possesses a car
        } else {
            ++peds;
            CHECK(a.vehicle == -1);                      // a pedestrian has none
        }
    }
    CHECK(drivers == 20);
    CHECK(peds == 10);

    // Possession is two-way: each car's driver points back at an agent that owns it.
    for (std::size_t v = 0; v < sim.vehicles().size(); ++v) {
        int d = sim.vehicles()[v].driver;
        CHECK(d >= 0);
        CHECK(sim.agents()[d].vehicle == static_cast<int>(v));
    }
}

TEST_CASE(city_possessed_car_tracks_its_driver) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 25, 0, 99);
    bool anyMoved = false;
    for (int i = 0; i < 4000; ++i) sim.step(0.1, 0.3);
    for (const Agent& a : sim.agents()) {
        if (a.mode != Agent::Mode::Driver || a.vehicle < 0) continue;
        const SimVehicle& v = sim.vehicles()[a.vehicle];
        // The car mirrors the agent that drives it.
        CHECK_APPROX(v.pos.x, a.pos.x, 1e-9);
        CHECK_APPROX(v.pos.y, a.pos.y, 1e-9);
        if ((a.pos - nav.nodes[a.home]).length() > 5.0) anyMoved = true;
    }
    CHECK(anyMoved);   // at least one driver actually drove its car somewhere
}

TEST_CASE(city_player_agent_is_not_auto_driven) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 10, 0, 7);
    sim.setPlayerControlled(0, true);
    Vec2 start = sim.agents()[0].pos;
    bool othersMoved = false;
    for (int i = 0; i < 4000; ++i) sim.step(0.1, 0.3);
    // The player agent stays put (its brain is the host, not the sim).
    CHECK_APPROX(sim.agents()[0].pos.x, start.x, 1e-9);
    CHECK_APPROX(sim.agents()[0].pos.y, start.y, 1e-9);
    for (std::size_t i = 1; i < sim.agents().size(); ++i)
        if ((sim.agents()[i].pos - start).length() > 5.0) othersMoved = true;
    CHECK(othersMoved);   // the AI crowd still drives
}

TEST_CASE(city_drivers_commute) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 40, 0, 55);
    bool atWork = false;
    for (int i = 0; i < 6000 && !atWork; ++i) {
        sim.step(0.1, 0.3);
        for (const Agent& a : sim.agents())
            if (a.activity == Agent::Activity::AtWork) atWork = true;
    }
    CHECK(atWork);
}

TEST_CASE(city_is_deterministic) {
    NavGraph nav = cityNav();
    CitySim a, b;
    a.build(nav, 20, 20, 4242);
    b.build(nav, 20, 20, 4242);
    for (int i = 0; i < 2500; ++i) { a.step(0.1, 0.3); b.step(0.1, 0.3); }
    bool same = a.agents().size() == b.agents().size();
    for (std::size_t i = 0; i < a.agents().size() && same; ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
}
