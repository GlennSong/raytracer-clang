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

namespace {

// A skewed X: all four bearings within +/-45 deg of the E-W axis — the exact
// geometry that made the OLD bearing-parity phases put all four arms in one
// bin (metropolis drive: "the lights on all four sides turned green at the
// same time and all cars then moved").
NavGraph skewedX() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)},
                {Vec2(96, 26)},  {Vec2(-96, -26)},   // ~15 deg pair
                {Vec2(-96, 26)}, {Vec2(96, -26)} };  // ~165 deg pair
    g.edges = {
        RoadEdge{1, 0, 8, RoadClass::Local, 0},
        RoadEdge{2, 0, 8, RoadClass::Local, 0},
        RoadEdge{3, 0, 8, RoadClass::Local, 0},
        RoadEdge{4, 0, 8, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

NavGraph fiveArm() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)},  {Vec2(80, 6)},  {Vec2(-78, -14)},
                {Vec2(18, 80)}, {Vec2(-30, 74)}, {Vec2(24, -78)} };
    g.edges = {
        RoadEdge{1, 0, 8, RoadClass::Local, 0},
        RoadEdge{2, 0, 8, RoadClass::Local, 0},
        RoadEdge{3, 0, 8, RoadClass::Local, 0},
        RoadEdge{4, 0, 8, RoadClass::Local, 0},
        RoadEdge{5, 0, 8, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

// Two approaches CONFLICT unless near-parallel or near-opposing (straight-
// through streams cross the box) — the controller's own model, restated
// independently for the property test.
bool conflicts(const NavGraph& nav, int a, int b) {
    Vec2 da = nav.direction(a), db = nav.direction(b);
    const Real crossv = da.x * db.y - da.y * db.x;
    return std::fabs(crossv) >= 0.342;   // sin(20 deg)
}

void checkNoConflictingGreens(const NavGraph& nav, const char* tag) {
    SignalController sc;
    sc.build(nav, 6.0, 1.5, 1.0);
    std::vector<int> in;
    for (int i = 0; i < nav.linkCount(); ++i)
        if (nav.links[i].to == 0 && sc.hasSignal(i)) in.push_back(i);
    CHECK(in.size() >= 4u);
    std::vector<char> served(in.size(), 0);
    for (double t = 0; t < 240.0; t += 0.25) {
        sc.update(0.25);
        for (std::size_t i = 0; i < in.size(); ++i) {
            const SignalState si = sc.stateForLink(in[i]);
            if (si == SignalState::Green) served[i] = 1;
            if (si == SignalState::Red) continue;
            for (std::size_t j = i + 1; j < in.size(); ++j) {
                const SignalState sj = sc.stateForLink(in[j]);
                if (sj == SignalState::Red) continue;
                // Both moving: they must not conflict.
                if (conflicts(nav, in[i], in[j])) {
                    std::printf("[sig] %s: links %d & %d both moving\n", tag,
                                in[i], in[j]);
                    CHECK(false);
                    return;
                }
            }
        }
    }
    for (std::size_t i = 0; i < served.size(); ++i)
        CHECK(served[i] == 1);          // every approach greens each cycle
}

}  // namespace

TEST_CASE(signals_conflicting_approaches_never_move_together) {
    // The invariant the old bearing-parity model could not keep: at NO time
    // may two conflicting approaches both be moving — on the orthodox 4-way,
    // the metropolis all-one-band skew X, and a five-arm.
    NavGraph c = cross();
    checkNoConflictingGreens(c, "cross");
    NavGraph sx = skewedX();
    checkNoConflictingGreens(sx, "skewX");
    NavGraph fa = fiveArm();
    checkNoConflictingGreens(fa, "five");
}

TEST_CASE(signals_walk_window_stops_everything) {
    // Every full cycle ends in a WALK window: all approaches red so
    // pedestrians own the box (S8's kerb logic waits for it at signalled
    // crossings).
    NavGraph nav = cross();
    SignalController sc;
    sc.build(nav, 6.0, 1.5, 1.0);
    bool sawWalk = false;
    for (double t = 0; t < 120.0; t += 0.25) {
        sc.update(0.25);
        if (!sc.walkWindowAt(0)) continue;
        sawWalk = true;
        for (int i = 0; i < nav.linkCount(); ++i)
            if (nav.links[i].to == 0 && sc.hasSignal(i))
                CHECK(sc.stateForLink(i) == SignalState::Red);
    }
    CHECK(sawWalk);
}
