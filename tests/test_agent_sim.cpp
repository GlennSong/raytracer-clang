#include "test_framework.h"

#include "../src/engine/ai/agent_sim.h"
#include "../src/engine/procgen/city/road_network.h"

using namespace engine;

namespace {

// A deterministic, connected city grid to drive agents over.
NavGraph cityNav() {
    GridRoadParams p;
    p.extent = 200;
    p.cellSize = 50;
    p.dropout = 0.0;
    p.seed = 7;
    return buildNavGraph(gridRoads(p));
}

}  // namespace

TEST_CASE(agent_build_populates) {
    NavGraph nav = cityNav();
    CHECK(nav.nodeCount() > 0);
    AgentSim sim;
    sim.build(nav, 20, 10, 1234);
    CHECK(sim.agents().size() == 30u);
    // Everyone starts at home, before the morning commute.
    int atHome = 0;
    for (const Agent& a : sim.agents())
        if (a.activity == AgentActivity::AtHome) ++atHome;
    CHECK(atHome == 30);
}

TEST_CASE(agent_sim_is_deterministic) {
    NavGraph nav = cityNav();
    AgentSim a, b;
    a.build(nav, 25, 15, 4242);
    b.build(nav, 25, 15, 4242);
    for (int i = 0; i < 3000; ++i) {
        a.step(0.1, 0.3);
        b.step(0.1, 0.3);
    }
    CHECK(a.timeOfDay() == b.timeOfDay());
    CHECK(a.agents().size() == b.agents().size());
    bool identical = true;
    for (std::size_t i = 0; i < a.agents().size(); ++i) {
        const Agent& pa = a.agents()[i];
        const Agent& pb = b.agents()[i];
        if (pa.pos.x != pb.pos.x || pa.pos.y != pb.pos.y) identical = false;
        if (pa.activity != pb.activity) identical = false;
        if (pa.leg != pb.leg) identical = false;
    }
    CHECK(identical);
}

TEST_CASE(agent_commutes_and_arrives) {
    NavGraph nav = cityNav();
    AgentSim sim;
    sim.build(nav, 40, 0, 99);
    bool anyMoved = false;
    bool anyAtWork = false;
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.3);
        for (const Agent& a : sim.agents()) {
            if (a.moving) anyMoved = true;
            if (a.activity == AgentActivity::AtWork) anyAtWork = true;
        }
    }
    CHECK(anyMoved);     // agents leave home once the clock passes departWork
    CHECK(anyAtWork);    // and at least one completes its commute
}

TEST_CASE(car_following_cap_ramps) {
    // Far ahead → free speed; at/under the bumper gap → full stop; in between →
    // linear ramp.
    CHECK_APPROX(carFollowingCap(10.0, 100.0, 5.0, 14.0), 10.0, 1e-9);  // clear road
    CHECK_APPROX(carFollowingCap(10.0, 14.0, 5.0, 14.0), 10.0, 1e-9);   // at slowZone
    CHECK_APPROX(carFollowingCap(10.0, 5.0, 5.0, 14.0), 0.0, 1e-9);     // at minGap
    CHECK_APPROX(carFollowingCap(10.0, 3.0, 5.0, 14.0), 0.0, 1e-9);     // inside minGap
    // Midpoint gap 9.5 → (9.5-5)/(14-5) = 0.5 → half speed.
    CHECK_APPROX(carFollowingCap(10.0, 9.5, 5.0, 14.0), 5.0, 1e-9);
}

TEST_CASE(car_following_keeps_commute_flowing) {
    // Dense traffic on a small grid still completes commutes (no gridlock from
    // the car-following cap).
    NavGraph nav = cityNav();
    AgentSim sim;
    sim.build(nav, 60, 0, 314);
    bool anyAtWork = false;
    for (int i = 0; i < 6000 && !anyAtWork; ++i) {
        sim.step(0.1, 0.3);
        for (const Agent& a : sim.agents())
            if (a.activity == AgentActivity::AtWork) anyAtWork = true;
    }
    CHECK(anyAtWork);
}

TEST_CASE(idle_agents_park_off_the_centerline) {
    NavGraph nav = cityNav();
    AgentSim sim;
    sim.build(nav, 30, 30, 55);   // everyone starts idle at home
    bool carOffset = false, pedOffset = false;
    for (const Agent& a : sim.agents()) {
        Real d = (a.pos - nav.nodes[a.home]).length();
        if (a.kind == AgentKind::Car && d > 0.5) carOffset = true;
        if (a.kind == AgentKind::Pedestrian && d > 0.5) pedOffset = true;
    }
    CHECK(carOffset);   // parked cars pull off the road node (to the curb)
    CHECK(pedOffset);   // idle pedestrians wait off the road node (on the sidewalk)
}

TEST_CASE(agent_clock_wraps) {
    NavGraph nav = cityNav();
    AgentSim sim;
    sim.build(nav, 1, 0, 1);
    for (int i = 0; i < 2000; ++i) sim.step(0.1, 0.5);
    CHECK(sim.timeOfDay() >= 0.0);
    CHECK(sim.timeOfDay() < 24.0);
}
