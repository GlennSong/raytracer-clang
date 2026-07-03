#include "test_framework.h"

#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cmath>
#include <vector>

using namespace engine;
using namespace citysim;

// The planner<->physics contract for AI-driven cars (ADR-0062): the CitySim ghost
// PLANS; the physical car follows the plan closed-loop. These cover the two sim
// hooks the bridge stands on — the route exported as a pursuit lane path, and the
// tether that stops a ghost outrunning the car it plans for.

namespace {

NavGraph straightRoad(Real len) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(len, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    return buildNavGraph(g);
}

// Step until some driver agent is moving; returns its index (or -1).
int firstMovingDriver(CitySim& sim, int maxSteps = 4000) {
    for (int i = 0; i < maxSteps; ++i) {
        sim.step(0.1, 0.5);
        for (std::size_t k = 0; k < sim.agents().size(); ++k)
            if (sim.agents()[k].mode == Agent::Mode::Driver && sim.agents()[k].moving)
                return static_cast<int>(k);
    }
    return -1;
}

}  // namespace

TEST_CASE(lane_path_traces_the_route_at_lane_offset) {
    NavGraph nav = straightRoad(200.0);
    CitySim sim;
    sim.build(nav, 6, 0, 3);
    int i = firstMovingDriver(sim);
    CHECK(i >= 0);
    CHECK(sim.agents()[i].trips >= 1);   // a trip started (the bridge keys off this)

    std::vector<Vec2> path = sim.lanePath(i);
    CHECK(path.size() >= 2u);
    Real total = 0;
    for (std::size_t k = 0; k + 1 < path.size(); ++k) {
        Vec2 d = path[k + 1] - path[k];
        Real seg = d.length();
        CHECK(seg > 1e-6);       // strictly ordered, no duplicate points
        CHECK(seg < 3.5);        // sampled at ~the requested step
        total += seg;
    }
    // Spans the route: the road is 200 m, so a route along it is a long path.
    CHECK(total > 100.0);
    // Every point rides the agent's LANE — off the centreline (right-hand side)
    // but on the carriageway (within the half-width).
    for (const Vec2& p : path) {
        CHECK(std::fabs(p.y) > 0.2);     // not on the centreline
        CHECK(std::fabs(p.y) < 4.0);     // inside the 8 m carriageway
    }
}

TEST_CASE(lane_path_is_empty_without_a_route) {
    NavGraph nav = straightRoad(200.0);
    CitySim sim;
    sim.build(nav, 4, 0, 3);   // nobody stepped: no trips yet
    for (std::size_t k = 0; k < sim.agents().size(); ++k)
        if (!sim.agents()[k].moving)
            CHECK(sim.lanePath(static_cast<int>(k)).empty());
    CHECK(sim.lanePath(-1).empty());
    CHECK(sim.lanePath(9999).empty());
}

TEST_CASE(tethered_ghost_waits_for_its_car) {
    NavGraph nav = straightRoad(300.0);
    CitySim sim;
    sim.build(nav, 6, 0, 7);
    int i = firstMovingDriver(sim);
    CHECK(i >= 0);

    // Anchor the ghost where it stands with a short leash: however long the sim
    // runs, it never gets further than the leash (its "car" never catches up).
    Vec2 anchor = sim.agents()[i].pos;
    sim.setAgentTether(i, anchor, 5.0);
    for (int s = 0; s < 400; ++s) {
        sim.step(0.1, 0.5);
        Vec2 d = sim.agents()[i].pos - anchor;
        CHECK(d.length() < 5.6);   // leash + one step of slack
    }
    // Held, not finished: the ghost reports Waiting while the leash binds.
    CHECK(sim.agents()[i].moving);
    CHECK(sim.agents()[i].state == Agent::State::Waiting);

    // The car "catches up" (leash cleared): the ghost drives on.
    sim.clearAgentTether(i);
    Real before = (sim.agents()[i].pos - anchor).length();
    for (int s = 0; s < 300 && sim.agents()[i].moving; ++s) sim.step(0.1, 0.5);
    Real after = (sim.agents()[i].pos - anchor).length();
    CHECK(after > before + 3.0);   // moved well beyond the old leash
}

TEST_CASE(agents_have_distinct_personal_paces) {
    // Personality (ADR-0062): each agent holds its own fraction of the nominal
    // speed, derived from its brain seed — varied, bounded, and reproducible.
    NavGraph nav = straightRoad(200.0);
    CitySim a, b;
    a.build(nav, 12, 6, 42);
    b.build(nav, 12, 6, 42);

    std::vector<Real> factors;
    for (std::size_t k = 0; k < a.agents().size(); ++k) {
        Real f = a.agents()[k].speedFactor;
        CHECK(f >= 0.85 && f <= 1.15);                    // bounded around nominal
        CHECK(f == b.agents()[k].speedFactor);            // same seed -> same driver
        bool fresh = true;
        for (Real g : factors)
            if (std::fabs(f - g) < 1e-6) fresh = false;
        if (fresh) factors.push_back(f);
    }
    CHECK(factors.size() >= 6u);   // a genuinely mixed crowd, not lockstep
}

TEST_CASE(near_junction_covers_the_box_and_only_the_box) {
    // A 4-way cross at the origin, 8 m roads: the box test drives don't-block-
    // the-box and spawn placement, so it must be tight — inside the node's
    // half-width (+margin) yes, mid-link no.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(0, 60)}, {Vec2(0, -60)},
                {Vec2(60, 0)}, {Vec2(-60, 0)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0}, RoadEdge{0, 2, 8, RoadClass::Local, 0},
        RoadEdge{0, 3, 8, RoadClass::Local, 0}, RoadEdge{0, 4, 8, RoadClass::Local, 0},
    };
    CitySim sim;
    sim.build(buildNavGraph(g), 2, 0, 3);
    CHECK(sim.nearJunction(Vec2(0, 0)));            // dead centre
    CHECK(sim.nearJunction(Vec2(3, 1)));            // inside the half-width
    CHECK(sim.nearJunction(Vec2(0, 5.5), 2.0));     // margin extends the box
    CHECK(!sim.nearJunction(Vec2(0, 30)));          // mid-link: open road
    CHECK(!sim.nearJunction(Vec2(0, 58)));          // a dead-end is not a junction
}

TEST_CASE(walkers_keep_a_wide_berth_around_the_player) {
    // The player (an external obstacle) gets a hard clearance like the poles do,
    // but wider — a ped ghost can never brush through them.
    NavGraph nav = straightRoad(200.0);
    CitySim sim;
    sim.build(nav, 0, 24, 11);
    // Park the "player" on the sidewalk line the peds walk (right side, +y).
    Vec2 player = nav.sidewalkPoint(0, 0.5);
    sim.setExternalObstacles({ player });
    Real minDist = 1e9;
    for (int i = 0; i < 4000; ++i) {
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents())
            if (a.mode == Agent::Mode::Pedestrian && a.moving)
                minDist = std::min(minDist, (a.pos - player).length());
    }
    CHECK(minDist >= 1.1 - 1e-3);   // the wide berth held (kPlayerClearance)
    CHECK(minDist < 3.0);           // and someone really did pass nearby
}

TEST_CASE(agents_commit_to_decisions_between_thinks) {
    // Think cadence (ADR-0062): the reactive decision (leanTarget) may only
    // change on the agent's slow think clock — between thinks it is COMMITTED,
    // however often the sim ticks. Re-deciding every tick is the "wigging out".
    NavGraph nav = straightRoad(200.0);
    CitySim sim;
    sim.setThinkPeriod(0.4);
    sim.build(nav, 0, 20, 11);   // a crowd of walkers (plenty to react to)

    // Warm up until several peds are moving.
    for (int i = 0; i < 2000; ++i) sim.step(0.05, 0.5);

    std::vector<Real> lastTarget(sim.agents().size(), 0.0);
    std::vector<int> changes(sim.agents().size(), 0);
    std::vector<int> movingTicks(sim.agents().size(), 0);
    for (std::size_t k = 0; k < sim.agents().size(); ++k)
        lastTarget[k] = sim.agents()[k].leanTarget;

    const int ticks = 200;                       // 10 s at dt 0.05
    for (int i = 0; i < ticks; ++i) {
        sim.step(0.05, 0.5);
        for (std::size_t k = 0; k < sim.agents().size(); ++k) {
            const Agent& a = sim.agents()[k];
            if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
            ++movingTicks[k];
            if (a.leanTarget != lastTarget[k]) ++changes[k];
            lastTarget[k] = a.leanTarget;
        }
    }
    bool anyMoved = false;
    for (std::size_t k = 0; k < sim.agents().size(); ++k) {
        if (movingTicks[k] < 50) continue;
        anyMoved = true;
        // At dt 0.05 and a 0.4 s think period, thinks land every 8 ticks: the
        // committed target may change at most ~once per 8 ticks (+ slack), never
        // per-tick.
        CHECK(changes[k] <= movingTicks[k] / 4);
    }
    CHECK(anyMoved);
}

TEST_CASE(tether_preserves_determinism) {
    NavGraph nav = straightRoad(300.0);
    CitySim a, b;
    a.build(nav, 8, 0, 11);
    b.build(nav, 8, 0, 11);
    for (int s = 0; s < 500; ++s) {
        // The same tether call sequence on both sims (the host owns whatever
        // nondeterminism it feeds; identical feeds -> identical sims).
        if (s == 100) { a.setAgentTether(0, Vec2(5, 2), 6.0); b.setAgentTether(0, Vec2(5, 2), 6.0); }
        if (s == 300) { a.clearAgentTether(0); b.clearAgentTether(0); }
        a.step(0.1, 0.5);
        b.step(0.1, 0.5);
    }
    bool same = true;
    for (std::size_t k = 0; k < a.agents().size(); ++k)
        if (a.agents()[k].pos.x != b.agents()[k].pos.x ||
            a.agents()[k].pos.y != b.agents()[k].pos.y)
            same = false;
    CHECK(same);
}
