// THREE-TIER TRAFFIC (P4) — the V (virtual far) tier's truth gates:
//   (a) determinism: two identical runs with a scripted player path crossing
//       the tier boundaries end byte-identical after ~10k fixed steps;
//   (b) conservation: no agent is created or destroyed by promotion/demotion;
//   (c) travel truth: a V agent's arrival time on a known route lands within
//       tolerance of the same agent driven as a full K agent;
//   (d) no teleport: a promotion hands K the exact lane pose the V state held
//       (bounded step displacement, and the pose sits ON the lane polyline);
//   (e) tiering-off default: with the switch off — or no player centre fed —
//       states match the untouched sim exactly (this is what protects every
//       existing ratchet baseline).
// Plus a deterministic 1000-agent stress case (no wall-clock asserts).
#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

#include <chrono>
#include <cmath>
#include <cstdio>

using namespace engine;
using namespace citysim;

namespace {

// Exact-equality state comparison (ADR-0002: identical fixed-update sequences
// must produce identical states — compared with ==, not a tolerance).
int agentMismatches(const CitySim& A, const CitySim& B) {
    const auto& a = A.agents();
    const auto& b = B.agents();
    if (a.size() != b.size()) return 1 << 20;
    int bad = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const Agent& x = a[i];
        const Agent& y = b[i];
        const bool same =
            x.uid == y.uid && x.mode == y.mode && x.state == y.state &&
            x.tier == y.tier && x.activity == y.activity && x.goal == y.goal &&
            x.goalHours == y.goalHours && x.restNode == y.restNode &&
            x.arrivedLink == y.arrivedLink && x.leg == y.leg &&
            x.distOnLeg == y.distOnLeg && x.speed == y.speed &&
            x.elevation == y.elevation && x.grade == y.grade &&
            x.deckY == y.deckY && x.lane == y.lane && x.laneF == y.laneF &&
            x.laneTimer == y.laneTimer && x.trips == y.trips &&
            x.parkedBay == y.parkedBay && x.crashTimer == y.crashTimer &&
            x.crashCount == y.crashCount && x.holdTimer == y.holdTimer &&
            x.thinkTimer == y.thinkTimer && x.leanTarget == y.leanTarget &&
            x.lateralOffset == y.lateralOffset && x.brain == y.brain &&
            x.speedFactor == y.speedFactor && x.pos.x == y.pos.x &&
            x.pos.y == y.pos.y && x.heading.x == y.heading.x &&
            x.heading.y == y.heading.y && x.moving == y.moving &&
            x.vHold == y.vHold && x.vLastTick == y.vLastTick &&
            x.route.links == y.route.links;
        if (!same) ++bad;
    }
    return bad;
}

int countTier(const CitySim& sim, Agent::Tier t) {
    int n = 0;
    for (const Agent& a : sim.agents())
        if (a.tier == t) ++n;
    return n;
}

// Distance from p to the polyline `path` (min over segments).
Real distToPolyline(const std::vector<Vec2>& path, Vec2 p) {
    if (path.empty()) return 1e30;
    Real best = 1e30;
    for (std::size_t k = 0; k + 1 < path.size(); ++k) {
        const Vec2 s = path[k], e = path[k + 1];
        const Vec2 d = e - s;
        const Real l2 = d.lengthSquared();
        Real t = l2 > 1e-12 ? dot(p - s, d) / l2 : 0.0;
        t = t < 0 ? 0 : (t > 1 ? 1 : t);
        best = std::min(best, (s + d * t - p).length());
    }
    if (path.size() == 1) best = (path[0] - p).length();
    return best;
}

// A straight run with two degree-3 (unsignalled) tee junctions on the way —
// the travel-truth route: long enough to measure, junctions to model.
NavGraph lineWithTees() {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)},     {Vec2(150, 0)},  {Vec2(300, 0)},
                {Vec2(450, 0)},   {Vec2(150, -70)}, {Vec2(300, 70)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},
        RoadEdge{1, 2, 8, RoadClass::Local, 0},
        RoadEdge{2, 3, 8, RoadClass::Local, 0},
        RoadEdge{1, 4, 8, RoadClass::Local, 0},
        RoadEdge{2, 5, 8, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

}  // namespace

// (a) + (b): a scripted player path sweeps the bubble corner-to-corner across
// a wandering town; two independent sims must stay byte-identical for 10k
// fixed steps while promotions AND demotions churn, and the population is
// conserved throughout.
TEST_CASE(tier_v_determinism_and_conservation) {
    const NavGraph nav = citytest::cityNav(700, 130, 5);
    CitySim simA, simB;
    for (CitySim* s : {&simA, &simB}) {
        s->build(nav, 50, 30, 42);
        s->setWander(true);
        s->tieringEnabled = true;
    }
    const int steps = 10000;
    const Real dt = 1.0 / 60.0;
    const int total = static_cast<int>(simA.agents().size());
    CHECK(total == 80);
    for (int i = 0; i < steps; ++i) {
        const Real t = static_cast<Real>(i) / steps;
        const Vec2 c(-650.0 + 1300.0 * t, -650.0 + 1300.0 * t);
        simA.setTierCenter(c);
        simB.setTierCenter(c);
        simA.step(dt, 0.05);
        simB.step(dt, 0.05);
        if (i % 1000 == 999) {
            // Conservation: every agent is exactly one of V/K, none lost.
            CHECK(static_cast<int>(simA.agents().size()) == total);
            CHECK(countTier(simA, Agent::Tier::V) +
                      countTier(simA, Agent::Tier::K) == total);
        }
    }
    const int bad = agentMismatches(simA, simB);
    std::printf("    [tierV det] mismatches=%d promos=%ld demos=%ld K=%d V=%d\n",
                bad, simA.tierPromotions(), simA.tierDemotions(),
                countTier(simA, Agent::Tier::K), countTier(simA, Agent::Tier::V));
    CHECK(bad == 0);
    CHECK(simA.seconds() == simB.seconds());
    CHECK(simA.tierPromotions() == simB.tierPromotions());
    CHECK(simA.tierDemotions() == simB.tierDemotions());
    // The path genuinely crossed tier boundaries — both transitions fired.
    CHECK(simA.tierPromotions() > 0);
    CHECK(simA.tierDemotions() > 0);
    // Vehicles are conserved too (a demotion strips render/proxy, never the car).
    CHECK(static_cast<int>(simA.vehicles().size()) == 50);
}

// (e): the tiering-off default. An untouched sim, a sim with the switch on but
// NO player centre, and a sim fed a centre with the switch OFF must all march
// in lockstep — this is the guard that keeps every pre-P4 ratchet baseline
// (test_city_generated.cpp et al) meaningful.
TEST_CASE(tier_v_off_matches_legacy) {
    const NavGraph nav = citytest::cityNav(400, 110, 7);
    CitySim plain, switchOnNoCentre, centreNoSwitch;
    for (CitySim* s : {&plain, &switchOnNoCentre, &centreNoSwitch}) {
        s->build(nav, 30, 20, 9);
        s->setWander(true);
    }
    switchOnNoCentre.tieringEnabled = true;   // no centre fed: must stay all K
    const Real dt = 1.0 / 60.0;
    for (int i = 0; i < 600; ++i) {
        centreNoSwitch.setTierCenter(Vec2(0, 0));   // fed, but switch is off
        plain.step(dt, 0.05);
        switchOnNoCentre.step(dt, 0.05);
        centreNoSwitch.step(dt, 0.05);
    }
    CHECK(agentMismatches(plain, switchOnNoCentre) == 0);
    CHECK(agentMismatches(plain, centreNoSwitch) == 0);
    CHECK(countTier(switchOnNoCentre, Agent::Tier::V) == 0);
    CHECK(countTier(centreNoSwitch, Agent::Tier::V) == 0);
    CHECK(switchOnNoCentre.tierDemotions() == 0);
}

// (c): travel truth. The same driver runs its commute once as a full K agent
// and once as a far V agent (player parked 100 km away); the modelled trip —
// class speed shaped by fixed junction dwells — must land within 25% of the
// integrated one. The V arrival is quantized to its ~1 s coarse tick, hence
// the small absolute allowance on top.
TEST_CASE(tier_v_travel_truth) {
    const NavGraph nav = lineWithTees();
    Real duration[2] = {0, 0};
    for (int run = 0; run < 2; ++run) {
        CitySim sim;
        sim.build(nav, 1, 0, 21);
        if (run == 1) {
            sim.tieringEnabled = true;
            sim.setTierCenter(Vec2(1e5, 1e5));   // far: the driver demotes at once
        }
        const Agent& a = sim.agents()[0];
        CHECK(a.home != a.work);   // seed 21 picks a commutable pair; guard it
        const Real dt = 1.0 / 60.0;
        Real t0 = -1, t1 = -1;
        for (int i = 0; i < 60 * 400 && t1 < 0; ++i) {
            sim.step(dt, 0.05);
            if (t0 < 0 && a.moving) t0 = sim.seconds();
            else if (t0 >= 0 && !a.moving) t1 = sim.seconds();
        }
        CHECK(t0 >= 0);
        CHECK(t1 > t0);
        duration[run] = t1 - t0;
        if (run == 1) {
            CHECK(sim.tierDemotions() >= 1);
            CHECK(sim.agents()[0].tier == Agent::Tier::V);
        }
    }
    std::printf("    [tierV travel] K=%.1fs V=%.1fs (route home->work)\n",
                duration[0], duration[1]);
    CHECK(std::fabs(duration[1] - duration[0]) <= 0.25 * duration[0] + 2.0);
}

// (d): no teleport at promotion. Park the player far out until the town has
// demoted, then stand them mid-town: every V->K flip must hand over a pose
// within one coarse tick's modelled travel of the last V pose, and a moving
// promotee must sit ON its own lane polyline — never snapped to a node, a
// park pose, or its destination.
TEST_CASE(tier_v_no_teleport_on_promotion) {
    const NavGraph nav = citytest::cityNav(500, 120, 3);
    CitySim sim;
    sim.build(nav, 40, 20, 4);
    sim.setWander(true);
    sim.tieringEnabled = true;
    const Real dt = 1.0 / 60.0;
    sim.setTierCenter(Vec2(2500, 2500));   // far: everyone demotes
    for (int i = 0; i < 300; ++i) sim.step(dt, 0.05);
    CHECK(sim.tierDemotions() > 0);
    CHECK(countTier(sim, Agent::Tier::V) > 0);

    const int n = static_cast<int>(sim.agents().size());
    std::vector<Vec2> prevPos(n);
    std::vector<Agent::Tier> prevTier(n);
    int flips = 0, farFlips = 0, offLane = 0;
    sim.setTierCenter(Vec2(0, 0));   // the player appears mid-town
    for (int i = 0; i < 600; ++i) {
        for (int k = 0; k < n; ++k) {
            prevPos[k] = sim.agents()[k].pos;
            prevTier[k] = sim.agents()[k].tier;
        }
        sim.step(dt, 0.05);
        for (int k = 0; k < n; ++k) {
            const Agent& a = sim.agents()[k];
            if (prevTier[k] != Agent::Tier::V || a.tier != Agent::Tier::K)
                continue;
            ++flips;
            // Bounded by one coarse tick of modelled travel (~1 s at class
            // speed) plus this step's K advance — a snap to a node/park pose
            // would measure tens to hundreds of metres.
            const Real moved = (a.pos - prevPos[k]).length();
            if (moved > 30.0) ++farFlips;
            if (a.moving) {
                const Real off = distToPolyline(sim.lanePath(k), a.pos);
                if (off > 4.5) ++offLane;   // 4.5: corner-cut Bezier vs polyline
            }
        }
    }
    std::printf("    [tierV promo] flips=%d farFlips=%d offLane=%d\n", flips,
                farFlips, offLane);
    CHECK(flips > 0);       // the bubble actually promoted people
    CHECK(farFlips == 0);   // no teleport
    CHECK(offLane == 0);    // exact lane pose at handoff
}

// Perf-shape guard: 1000 agents under a live bubble stay deterministic and a
// fixed-step run completes headlessly. Deliberately NO wall-clock assert
// (ADR-0002 — timing is reported, never gated).
TEST_CASE(tier_v_stress_deterministic) {
    const NavGraph nav = citytest::cityNav(900, 150, 9);
    const auto wall0 = std::chrono::steady_clock::now();
    CitySim simA, simB;
    for (CitySim* s : {&simA, &simB}) {
        s->build(nav, 600, 400, 77);
        s->setWander(true);
        s->tieringEnabled = true;
        s->setTierCenter(Vec2(0, 0));
    }
    const Real dt = 1.0 / 60.0;
    for (int i = 0; i < 300; ++i) {
        simA.step(dt, 0.05);
        simB.step(dt, 0.05);
    }
    const auto wall1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(wall1 - wall0).count();
    std::printf("    [tierV stress] 1000 agents x300 steps x2 sims: %.0f ms "
                "(K=%d V=%d, promos=%ld demos=%ld)\n",
                ms, countTier(simA, Agent::Tier::K),
                countTier(simA, Agent::Tier::V), simA.tierPromotions(),
                simA.tierDemotions());
    CHECK(agentMismatches(simA, simB) == 0);
    CHECK(countTier(simA, Agent::Tier::V) > 0);   // the bubble engaged
    CHECK(countTier(simA, Agent::Tier::K) > 0);
}

// ---- Tier::D, dormancy (P5) -------------------------------------------------

// OFF MUST BE INVISIBLE. Same shape as tier_v_off_matches_legacy: with the
// switch off, or with no player centre fed, dormancy must not exist — no agent
// reaches Tier::D and the sim marches exactly as the V/K sim does.
TEST_CASE(dormancy_off_matches_tier_v) {
    const NavGraph nav = citytest::cityNav(400, 110, 7);
    CitySim plain, switchOnNoCentre, centreNoSwitch;
    for (CitySim* s : {&plain, &switchOnNoCentre, &centreNoSwitch}) {
        s->build(nav, 30, 20, 9);
        s->setWander(true);
        s->tieringEnabled = true;
    }
    switchOnNoCentre.dormancyEnabled = true;   // no centre fed: must stay awake
    centreNoSwitch.dormancyEnabled = false;
    const Real dt = 1.0 / 60.0;
    for (int i = 0; i < 600; ++i) {
        plain.setTierCenter(Vec2(0, 0));
        centreNoSwitch.setTierCenter(Vec2(0, 0));
        plain.step(dt, 0.05);
        switchOnNoCentre.step(dt, 0.05);
        centreNoSwitch.step(dt, 0.05);
    }
    CHECK(agentMismatches(plain, centreNoSwitch) == 0);
    CHECK(plain.dormantAgents() == 0);
    CHECK(switchOnNoCentre.dormantAgents() == 0);
    CHECK(centreNoSwitch.dormantAgents() == 0);
}

// Agents beyond the dormant radius stop being simulated, and come back when the
// player returns — rebuilt from their schedule, not resumed from a stale pose.
TEST_CASE(dormancy_sleeps_the_far_field_and_wakes_it_back) {
    const NavGraph nav = citytest::cityNav(1600, 120, 3);
    CitySim sim;
    sim.build(nav, 200, 100, 11);
    sim.tieringEnabled = true;
    sim.dormancyEnabled = true;
    sim.dormantRadius = 600.0;         // tight, so the fixture actually reaches it
    sim.dormantResumeRadius = 500.0;

    // Sit in one corner: the far side of the city should go dormant.
    for (int i = 0; i < 1200; ++i) {
        sim.setTierCenter(Vec2(-700, -700));
        sim.step(1.0 / 60.0, 0.05);
    }
    const int dormant = sim.dormantAgents();
    std::printf("    [dormancy] %d of %zu agents dormant, %d sleeps / %d wakes\n",
                dormant, sim.agents().size(), sim.dormancies(), sim.wakes());
    CHECK(dormant > 0);
    CHECK(sim.dormancies() > 0);

    // Walk to the other corner: they must come back.
    for (int i = 0; i < 1200; ++i) {
        sim.setTierCenter(Vec2(700, 700));
        sim.step(1.0 / 60.0, 0.05);
    }
    std::printf("    [dormancy] after crossing: %d dormant, %d wakes\n",
                sim.dormantAgents(), sim.wakes());
    CHECK(sim.wakes() > 0);
    // Nobody is lost: every agent still exists and holds a sane tier.
    CHECK(sim.agents().size() == 300u);
}

// DORMANCY STRIPS THE AGENT, NEVER THE VEHICLE. A parked car keeps its pose
// while its owner does not exist — otherwise a car the player is looking at
// would move (or vanish) the moment its owner slept, and the whole "you can
// steal a parked car" premise depends on the car outliving the agent.
TEST_CASE(dormant_agents_do_not_move_their_parked_cars) {
    const NavGraph nav = citytest::cityNav(1600, 120, 3);
    CitySim sim;
    sim.build(nav, 150, 0, 21);
    sim.tieringEnabled = true;
    sim.dormantRadius = 600.0;
    sim.dormantResumeRadius = 500.0;

    // SETTLE FIRST, with dormancy OFF. A dormant agent does not drive anywhere,
    // so switching it on before the city has run leaves nothing parked to test
    // — the far field simply freezes at its starting pose.
    sim.dormancyEnabled = false;
    for (int i = 0; i < 9000; ++i) {
        sim.setTierCenter(Vec2(-700, -700));
        sim.step(0.1, 0.05);
    }
    // Now sleep the far field.
    sim.dormancyEnabled = true;
    for (int i = 0; i < 600; ++i) {
        sim.setTierCenter(Vec2(-700, -700));
        sim.step(0.1, 0.05);
    }

    // Snapshot the cars whose owner is dormant RIGHT NOW. A car whose owner is
    // awake may legitimately be driven away and re-parked somewhere else; the
    // invariant is only about the ones nobody is simulating.
    std::vector<int> ownerOf(sim.vehicles().size(), -1);
    for (std::size_t i = 0; i < sim.agents().size(); ++i)
        if (sim.agents()[i].car >= 0) ownerOf[sim.agents()[i].car] = static_cast<int>(i);

    {   // diagnostic: why are there (or aren't there) parked cars to watch?
        int driverless = 0, dormant = 0, dormantWithParkedCar = 0, peds = 0;
        for (std::size_t v = 0; v < sim.vehicles().size(); ++v)
            if (sim.vehicles()[v].driver < 0) ++driverless;
        for (const Agent& a : sim.agents()) {
            if (a.tier == Agent::Tier::D) ++dormant;
            if (a.mode == Agent::Mode::Pedestrian) ++peds;
            if (a.tier == Agent::Tier::D && a.car >= 0 &&
                sim.vehicles()[a.car].driver < 0) ++dormantWithParkedCar;
        }
        std::printf("    [dormancy] %d driverless cars, %d dormant agents, "
                    "%d on foot, %d dormant-with-parked-car\n",
                    driverless, dormant, peds, dormantWithParkedCar);
    }

    std::vector<int> watch;
    std::vector<Vec2> before;
    for (std::size_t v = 0; v < sim.vehicles().size(); ++v) {
        const int o = ownerOf[v];
        if (o < 0 || sim.vehicles()[v].driver >= 0) continue;
        if (sim.agents()[o].tier != Agent::Tier::D) continue;
        watch.push_back(static_cast<int>(v));
        before.push_back(sim.vehicles()[v].pos);
    }

    // Keep the player put so nothing wakes, and run on.
    for (int i = 0; i < 3000; ++i) {
        sim.setTierCenter(Vec2(-700, -700));
        sim.step(0.1, 0.05);
    }

    int held = 0;
    for (std::size_t k = 0; k < watch.size(); ++k) {
        const int v = watch[k];
        if (sim.agents()[ownerOf[v]].tier != Agent::Tier::D) continue;  // woke: fair game
        ++held;
        CHECK((sim.vehicles()[v].pos - before[k]).length() < 1e-9);
    }
    std::printf("    [dormancy] %d cars of still-dormant owners held their pose\n",
                held);
    CHECK(held > 0);   // the invariant must actually have been exercised
}
