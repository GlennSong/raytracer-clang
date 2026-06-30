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

TEST_CASE(cars_keep_to_the_right_side_of_the_road) {
    // A wide two-way street: with a fixed lane offset a car would hug the
    // centreline on a wide road (ambiguous side); the width-relative spacing keeps
    // each car centred in its own half. Verify every moving car sits on the RIGHT
    // of its travel direction and inside the carriageway, in both directions.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(140, 0)} };
    g.edges = { RoadEdge{0, 1, 14, RoadClass::Arterial, 0} };   // 14 m -> 2 lanes/side
    NavGraph nav = buildNavGraph(g);

    CitySim sim;
    sim.build(nav, 14, 0, 33);
    const NavGraph& G = *sim.graph();

    bool sawEast = false, sawWest = false;
    int checks = 0;
    for (int i = 0; i < 4000; ++i) {
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents()) {
            if (a.mode != Agent::Mode::Driver || !a.moving) continue;
            if (a.leg >= static_cast<int>(a.route.links.size())) continue;
            int li = a.route.links[a.leg];
            Real L = G.links[li].length;
            Real t = L > 1e-9 ? a.distOnLeg / L : 0.0;
            Vec2 dir = G.direction(li);
            Vec2 right(dir.y, -dir.x);
            Vec2 base = G.pointOnLink(li, t);
            Real lateral = (a.pos.x - base.x) * right.x + (a.pos.y - base.y) * right.y;
            Real halfWidth = G.links[li].width * 0.5;
            CHECK(lateral > 0.0);                  // on the right of travel
            CHECK(lateral <= halfWidth + 1e-6);    // and inside the carriageway
            ++checks;
            if (dir.x > 0.5) sawEast = true;
            if (dir.x < -0.5) sawWest = true;
        }
    }
    CHECK(checks > 0);
    CHECK(sawEast);   // both travel directions present...
    CHECK(sawWest);   // ...each correctly on its own right-hand side
}

TEST_CASE(cars_stop_for_the_player_standing_in_the_road) {
    // A straight two-way street. The "player" stands in the eastbound lane; cars
    // driving up behind must hold short, not drive through them.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(120, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    NavGraph nav = buildNavGraph(g);

    CitySim sim;
    sim.build(nav, 10, 0, 71);
    const Vec2 player(60.0, -2.0);   // mid-road, in the eastbound (right-hand) lane

    Real minDist = 1e9;
    bool approached = false;
    for (int i = 0; i < 4000; ++i) {
        sim.setExternalObstacles({ player });   // host injects the live player each step
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents()) {
            if (a.mode != Agent::Mode::Driver || !a.moving) continue;
            if (a.heading.x < 0.5) continue;     // only the lane heading toward the player
            Real dx = a.pos.x - player.x, dz = a.pos.y - player.y;
            Real d = std::sqrt(dx * dx + dz * dz);
            minDist = std::min(minDist, d);
            if (a.pos.x < player.x && d < 12.0) approached = true;
        }
    }
    CHECK(approached);          // a car really did come up behind the player...
    CHECK(minDist > 2.0);       // ...and never ran them over (held short)
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
