#include "test_framework.h"

#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

using namespace engine;
using namespace citysim;

namespace {

// A 4-way cross with long arms: centre node 0 is a junction; arm tips 1..4.
// buildNavGraph makes each edge two-way, so routes between arms cross the centre.
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

// Is agent `a` halted (speed ~0) at a red light's stop line? A walker holds at
// the corner; a car's line is SET BACK before the painted crosswalk band
// (junction mouth + band + margin + half its body — device round 3), so its
// window is correspondingly wider.
bool stoppedAtRed(const CitySim& sim, const NavGraph& nav, const Agent& a) {
    if (!a.moving || a.leg >= static_cast<int>(a.route.links.size())) return false;
    int li = a.route.links[a.leg];
    if (!nav.isJunction(nav.links[li].to)) return false;
    Real distToEnd = nav.links[li].length - a.distOnLeg;
    Real window = (a.mode == Agent::Mode::Driver) ? 18.0 : 3.0;
    return distToEnd < window && a.speed < 0.5 &&
           const_cast<CitySim&>(sim).signals().stateForLink(li) == SignalState::Red;
}

}  // namespace

TEST_CASE(city_cars_stop_for_red_and_still_commute) {
    NavGraph nav = cross4(40.0);
    CitySim sim;
    sim.build(nav, 16, 0, 3);

    int redStops = 0;
    bool anyAtWork = false;
    for (int i = 0; i < 8000; ++i) {
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents()) {
            if (a.activity == Agent::Activity::AtWork) anyAtWork = true;
            if (a.mode == Agent::Mode::Driver && stoppedAtRed(sim, nav, a)) ++redStops;
        }
    }
    // Junction easing alone only crawls (~4 m/s); a full halt at the line proves
    // the signal stop fired. And the green phase still clears traffic (no gridlock).
    CHECK(redStops > 0);
    CHECK(anyAtWork);
}

TEST_CASE(city_pedestrians_wait_at_signals) {
    NavGraph nav = cross4(30.0);
    CitySim sim;
    sim.build(nav, 0, 20, 9);   // pedestrians only
    int pedWaits = 0;
    bool anyArrived = false;
    for (int i = 0; i < 8000; ++i) {
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents()) {
            if (a.activity == Agent::Activity::AtWork) anyArrived = true;
            if (a.mode == Agent::Mode::Pedestrian && stoppedAtRed(sim, nav, a)) ++pedWaits;
        }
    }
    CHECK(pedWaits > 0);    // pedestrians hold at the curb for the signal
    CHECK(anyArrived);      // and still reach their destination
}

TEST_CASE(city_signals_keep_determinism) {
    NavGraph nav = cross4(40.0);
    CitySim a, b;
    a.build(nav, 12, 8, 2024);
    b.build(nav, 12, 8, 2024);
    for (int i = 0; i < 3000; ++i) { a.step(0.1, 0.5); b.step(0.1, 0.5); }
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
}

TEST_CASE(signals_have_an_all_red_clearance_between_phases) {
    // Browser round: cross traffic used to get green the instant the other
    // phase's yellow ended, meeting late turners inside the box. The cycle now
    // holds BOTH phases red for a clearance interval so the junction drains.
    NavGraph nav = cross4(40.0);
    SignalController sc;
    sc.build(nav);
    // Two PERPENDICULAR approaches into the centre: one per phase.
    int la = -1, lb = -1;
    for (int li = 0; li < nav.linkCount(); ++li) {
        if (nav.links[li].to != 0) continue;
        if (nav.links[li].from == 1) la = li;   // north arm approach
        if (nav.links[li].from == 3) lb = li;   // east arm approach
    }
    CHECK(la >= 0);
    CHECK(lb >= 0);

    bool bothRed = false, aGreen = false, bGreen = false, bothGreen = false;
    for (int i = 0; i < 800; ++i) {   // 80 s: several full cycles
        sc.update(0.1);
        SignalState sa = sc.stateForLink(la), sb = sc.stateForLink(lb);
        bothRed |= (sa == SignalState::Red && sb == SignalState::Red);
        aGreen |= sa == SignalState::Green;
        bGreen |= sb == SignalState::Green;
        bothGreen |= (sa == SignalState::Green && sb == SignalState::Green);
    }
    CHECK(bothRed);     // the clearance interval exists
    CHECK(aGreen);      // both phases still get their turn
    CHECK(bGreen);
    CHECK(!bothGreen);  // perpendicular arms are never green together
}
