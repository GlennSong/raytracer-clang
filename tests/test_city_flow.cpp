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

TEST_CASE(agents_never_teleport_even_with_unroutable_pairs) {
    // Two road components with NO edge between them, ~200 m apart. An agent whose
    // home and work land in different components has no route; the old code
    // teleported it to the goal (it "disappeared and reappeared"). Now it must
    // never jump: every step's displacement stays within normal travel.
    RoadGraph g;
    g.nodes = {
        {Vec2(0, 0)}, {Vec2(40, 0)}, {Vec2(40, 40)}, {Vec2(0, 40)},        // A
        {Vec2(400, 0)}, {Vec2(440, 0)}, {Vec2(440, 40)}, {Vec2(400, 40)},  // B (far)
    };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0}, RoadEdge{1, 2, 8, RoadClass::Local, 0},
        RoadEdge{2, 3, 8, RoadClass::Local, 0}, RoadEdge{3, 0, 8, RoadClass::Local, 0},
        RoadEdge{4, 5, 8, RoadClass::Local, 0}, RoadEdge{5, 6, 8, RoadClass::Local, 0},
        RoadEdge{6, 7, 8, RoadClass::Local, 0}, RoadEdge{7, 4, 8, RoadClass::Local, 0},
    };
    NavGraph nav = buildNavGraph(g);

    CitySim sim;
    sim.build(nav, 16, 16, 99);
    std::vector<Vec2> prev;
    for (const Agent& a : sim.agents()) prev.push_back(a.pos);

    Real maxStep = 0;
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.6);
        const auto& ag = sim.agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            Real dx = ag[k].pos.x - prev[k].x, dy = ag[k].pos.y - prev[k].y;
            maxStep = std::max(maxStep, std::sqrt(dx * dx + dy * dy));
            prev[k] = ag[k].pos;
        }
    }
    // A component-to-component teleport (the old disappear/reappear bug) would be
    // hundreds of metres; real motion is a couple of metres per 0.1 s step. (A
    // small lateral hop of up to ~one road-width can still happen at a trip
    // boundary where an agent changes which side it rests on — separate polish,
    // well under this bound.)
    CHECK(maxStep < 20.0);
}

TEST_CASE(pedestrians_react_and_walk_around_each_other) {
    // Reactive walkers (ADR-0060): they see neighbours in a vision cone and step
    // aside to go around them, their bodies never overlap, they enter the Avoiding
    // state when they see someone, they still reach their goals, and it's
    // deterministic.
    NavGraph nav = cross4(40.0);
    CitySim a, b;
    a.build(nav, 0, 30, 55);   // pedestrians only
    b.build(nav, 0, 30, 55);

    Real minPedDist = 1e9;
    bool anyArrived = false, anyAvoided = false;
    for (int i = 0; i < 8000; ++i) {
        a.step(0.1, 0.5);
        b.step(0.1, 0.5);
        const auto& ag = a.agents();
        for (std::size_t p = 0; p < ag.size(); ++p) {
            if (ag[p].mode != Agent::Mode::Pedestrian) continue;
            if (ag[p].activity == Agent::Activity::AtWork) anyArrived = true;
            if (ag[p].state == Agent::State::Avoiding) anyAvoided = true;
            if (!ag[p].moving || ag[p].speed < 0.3) continue;   // actually walking
            for (std::size_t q = p + 1; q < ag.size(); ++q) {
                if (ag[q].mode != Agent::Mode::Pedestrian || !ag[q].moving || ag[q].speed < 0.3)
                    continue;
                Real dx = ag[p].pos.x - ag[q].pos.x, dy = ag[p].pos.y - ag[q].pos.y;
                minPedDist = std::min(minPedDist, std::sqrt(dx * dx + dy * dy));
            }
        }
    }
    // Bodies (0.5 m wide) never overlap — they go around, not through.
    CHECK(minPedDist > 0.45);
    CHECK(anyAvoided);          // walkers really did react to someone they saw
    CHECK(anyArrived);          // and still reached their destinations
    bool same = true;           // deterministic
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
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

TEST_CASE(cross_node_following_keeps_cars_off_each_other) {
    // Car-following now spans nodes: a car crossing a junction/bend keeps its
    // distance to the car ahead on its continuing route, so followers rarely pile
    // through their leader. On this single-junction stress test (all traffic funnels
    // through one node) genuine body overlap is rare; a real grid spreads far
    // thinner. We assert it's rare (not zero — a brief one-step touch can still
    // happen at a merge, a known discrete-step limitation), traffic flows, and the
    // sim is deterministic.
    NavGraph nav = cross4(50.0);
    CitySim a, b;
    a.build(nav, 24, 0, 17);
    b.build(nav, 24, 0, 17);

    long overlapSteps = 0, sampled = 0;
    int arrivals = 0;
    std::vector<Agent::Activity> prev;
    for (const Agent& ag : a.agents()) prev.push_back(ag.activity);

    for (int i = 0; i < 12000; ++i) {
        a.step(0.1, 0.5);
        b.step(0.1, 0.5);
        const auto& ag = a.agents();
        // Arrivals (count over ALL drivers — an arrived car has moving == false).
        for (std::size_t k = 0; k < ag.size(); ++k) {
            if (ag[k].mode != Agent::Mode::Driver) continue;
            bool arrived = ag[k].activity == Agent::Activity::AtWork ||
                           ag[k].activity == Agent::Activity::AtHome;
            bool wasMoving = prev[k] == Agent::Activity::Commuting ||
                             prev[k] == Agent::Activity::Returning;
            if (wasMoving && arrived) ++arrivals;
            prev[k] = ag[k].activity;
        }
        // Overlap among actively-driving cars.
        bool overlap = false, anyMoving = false;
        for (std::size_t p = 0; p < ag.size(); ++p) {
            if (ag[p].mode != Agent::Mode::Driver || !ag[p].moving || ag[p].speed < 0.5) continue;
            anyMoving = true;
            for (std::size_t q = p + 1; q < ag.size(); ++q) {
                if (ag[q].mode != Agent::Mode::Driver || !ag[q].moving || ag[q].speed < 0.5) continue;
                Real dx = ag[p].pos.x - ag[q].pos.x, dy = ag[p].pos.y - ag[q].pos.y;
                if (std::sqrt(dx * dx + dy * dy) < 1.5) overlap = true;   // bodies overlapping
            }
        }
        if (anyMoving) { ++sampled; if (overlap) ++overlapSteps; }
    }
    CHECK(sampled > 0);
    CHECK(arrivals > 40);                          // traffic keeps flowing (no gridlock)
    CHECK(overlapSteps < sampled / 10);            // genuine overlap in well under 10% of steps
    bool same = true;                              // and the sim stays deterministic
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
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
