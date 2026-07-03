#include "test_framework.h"

#include "../src/apps/citysim/traffic_signal.h"
#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/procgen/city/road_network.h"

using namespace engine;
using namespace citysim;

namespace {

// A 4-way cross: centre node 0 (a junction) with N/S/E/W arms.
NavGraph cross() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(0, 10)}, {Vec2(0, -10)},
                {Vec2(10, 0)}, {Vec2(-10, 0)} };
    g.edges = {
        RoadEdge{1, 0, 8, RoadClass::Local, 0},   // N -> centre
        RoadEdge{2, 0, 8, RoadClass::Local, 0},   // S -> centre
        RoadEdge{3, 0, 8, RoadClass::Local, 0},   // E -> centre
        RoadEdge{4, 0, 8, RoadClass::Local, 0},   // W -> centre
    };
    return buildNavGraph(g);
}

// Find an incoming link to the junction (node 0) approaching along ~axis.
int linkApproaching(const NavGraph& nav, Real dirx, Real dirz) {
    for (int i = 0; i < nav.linkCount(); ++i) {
        if (nav.links[i].to != 0) continue;
        Vec2 d = nav.direction(i);
        if (d.x * dirx + d.y * dirz > 0.9) return i;
    }
    return -1;
}

}  // namespace

TEST_CASE(signal_open_road_is_always_green) {
    // A straight road, no junction → no signals.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(10, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    NavGraph nav = buildNavGraph(g);
    SignalController sig;
    sig.build(nav);
    for (int i = 0; i < nav.linkCount(); ++i) {
        CHECK(!sig.hasSignal(i));
        CHECK(sig.stateForLink(i) == SignalState::Green);
    }
}

TEST_CASE(signal_junction_arms_get_signals) {
    NavGraph nav = cross();
    SignalController sig;
    sig.build(nav);
    int ns = linkApproaching(nav, 0, -1);   // travelling -Z toward centre (from N)
    int ew = linkApproaching(nav, -1, 0);   // travelling -X toward centre (from E)
    CHECK(ns >= 0);
    CHECK(ew >= 0);
    CHECK(sig.hasSignal(ns));
    CHECK(sig.hasSignal(ew));
}

TEST_CASE(signal_perpendicular_arms_never_both_green) {
    NavGraph nav = cross();
    SignalController sig;
    sig.build(nav, 8.0, 2.0);
    int ns = linkApproaching(nav, 0, -1);
    int ew = linkApproaching(nav, -1, 0);
    // Sample the whole 20s cycle; N-S and E-W must never be green simultaneously.
    bool everBothGreen = false;
    SignalController s = sig;
    for (int step = 0; step < 200; ++step) {
        if (s.stateForLink(ns) == SignalState::Green &&
            s.stateForLink(ew) == SignalState::Green)
            everBothGreen = true;
        s.update(0.1);
    }
    CHECK(!everBothGreen);
}

TEST_CASE(signal_cycles_through_states) {
    NavGraph nav = cross();
    SignalController sig;
    sig.build(nav, 4.0, 1.0);
    int ns = linkApproaching(nav, 0, -1);
    bool sawGreen = false, sawYellow = false, sawRed = false;
    for (int step = 0; step < 200; ++step) {
        switch (sig.stateForLink(ns)) {
            case SignalState::Green: sawGreen = true; break;
            case SignalState::Yellow: sawYellow = true; break;
            case SignalState::Red: sawRed = true; break;
        }
        sig.update(0.1);
    }
    CHECK(sawGreen);
    CHECK(sawYellow);
    CHECK(sawRed);
}
