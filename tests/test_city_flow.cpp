#include "test_framework.h"

#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

using namespace engine;
using namespace citysim;

namespace {

// A 4-way cross with long arms: routes between arms all cross the centre
// junction, so a busy run piles cars into one conflict point — exactly where a
// naive "brake for everything you see" rule deadlocks.
NavGraph cross4(Real arm) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(0, arm)}, {Vec2(0, -arm)},
                {Vec2(arm, 0)}, {Vec2(-arm, 0)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},
        RoadEdge{0, 2, 8, RoadClass::Local, 0},
        RoadEdge{0, 3, 8, RoadClass::Local, 0},
        RoadEdge{0, 4, 8, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

// Count how many times any car completes a trip (arrives at work or home) over
// the run. A gridlocked sim produces almost none; flowing traffic produces many.
int countArrivals(CitySim& sim, int steps) {
    std::vector<Agent::Activity> prev;
    for (const Agent& a : sim.agents()) prev.push_back(a.activity);
    int arrivals = 0;
    for (int i = 0; i < steps; ++i) {
        sim.step(0.1, 0.5);
        const auto& ag = sim.agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            if (ag[k].mode != Agent::Mode::Driver) continue;
            bool nowArrived = ag[k].activity == Agent::Activity::AtWork ||
                              ag[k].activity == Agent::Activity::AtHome;
            bool wasMoving = prev[k] == Agent::Activity::Commuting ||
                             prev[k] == Agent::Activity::Returning;
            if (wasMoving && nowArrived) ++arrivals;
            prev[k] = ag[k].activity;
        }
    }
    return arrivals;
}

}  // namespace

TEST_CASE(busy_junction_does_not_gridlock) {
    NavGraph nav = cross4(50.0);
    CitySim sim;
    sim.build(nav, 24, 0, 17);   // 24 cars funnelling through one junction

    int arrivals = countArrivals(sim, 12000);
    // Over ~6 in-world days, two-dozen commuters crossing one signal should
    // complete many trips. A snarl (cars braking for each other forever) yields
    // a handful at most; healthy flow yields dozens.
    CHECK(arrivals > 40);
}

TEST_CASE(cars_do_not_freeze_for_oncoming_traffic) {
    // A single two-way street: cars run both directions. The old wide cone made
    // every car brake for oncoming traffic 3.5 m to the side; here they must keep
    // moving past each other.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(120, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    NavGraph nav = buildNavGraph(g);   // two-way -> opposing links

    CitySim sim;
    sim.build(nav, 12, 0, 5);

    // Watch the busiest stretch: at most steps SOME car is moving at a healthy
    // clip. If oncoming traffic froze everyone, this stays false for long runs.
    int movingSteps = 0, sampled = 0;
    for (int i = 0; i < 4000; ++i) {
        sim.step(0.1, 0.5);
        bool anyFast = false;
        bool anyCommuting = false;
        for (const Agent& a : sim.agents()) {
            if (a.activity == Agent::Activity::Commuting ||
                a.activity == Agent::Activity::Returning) anyCommuting = true;
            if (a.moving && a.speed > 3.0) anyFast = true;
        }
        if (anyCommuting) { ++sampled; if (anyFast) ++movingSteps; }
    }
    CHECK(sampled > 0);
    // Whenever cars are out commuting, traffic is moving the vast majority of the
    // time (not frozen nose-to-nose with oncoming cars).
    CHECK(movingSteps > sampled * 9 / 10);
}
