#include "test_framework.h"
#include <cstdio>

#include "model_scene.h"
#include "../src/apps/citysim/city_sim.h"

using namespace engine;
using namespace citysim;

// The scenes are road files built as a level's roads are (model_scene.h, #306 piece 6 W2).

TEST_CASE(agents_never_teleport_even_with_unroutable_pairs) {
    // Two road components with NO road between them, 400 m apart. An agent whose
    // home and work land in different components has no route; the old code
    // teleported it to the goal (it "disappeared and reappeared"). Now it must
    // never jump: every step's displacement stays within normal travel.
    // (W2: each component a cross of 40 m arms, A at (0, 0) and B at (400, 0); the hand-made map's were two 40 m
    // squares, whose bends make no junction, and a road file with none gets no lane level.)
    RoadGraph g;
    for (const Real cx : {0.0, 400.0}) {
        const int c = static_cast<int>(g.nodes.size());
        g.nodes.push_back({Vec2(cx, 0)});
        for (const Vec2 d : {Vec2(40, 0), Vec2(-40, 0), Vec2(0, 40), Vec2(0, -40)}) {
            g.nodes.push_back({Vec2(cx, 0) + d});
            g.edges.push_back(RoadEdge{c, static_cast<int>(g.nodes.size()) - 1, 8, RoadClass::Local, 0});
        }
    }
    const citytest::ModelScene& s = citytest::graphScene("two_crosses", g);
    CHECK(s.problems.empty());

    CitySim sim;
    s.build(sim, 32, 99);
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
    // Reactive walkers (ADR-0061): they see neighbours in a vision cone and step
    // aside to go around them, their bodies never overlap, they enter the Avoiding
    // state when they see someone, they still reach their goals, and it's
    // deterministic.
    const citytest::ModelScene& s = citytest::crossScene(40.0);
    CitySim a, b;
    s.build(a, 30, 55);
    s.build(b, 30, 55);

    Real minPedDist = 1e9, maxLeanJump = 0;
    bool anyArrived = false, anyAvoided = false;
    std::vector<Real> prevLean(a.agents().size(), 0.0);
    std::vector<bool> wasMoving(a.agents().size(), false);
    for (int i = 0; i < 8000; ++i) {
        a.step(0.1, 0.5);
        b.step(0.1, 0.5);
        const auto& ag = a.agents();
        for (std::size_t p = 0; p < ag.size(); ++p) {
            if (ag[p].activity == Agent::Activity::AtWork) anyArrived = true;
            if (ag[p].state == Agent::State::Avoiding) anyAvoided = true;
            // The sideways lean only ever changes by a bounded amount per step
            // (continuous steer, never a discrete pop) while the ped keeps walking.
            if (ag[p].moving && wasMoving[p])
                maxLeanJump = std::max(maxLeanJump, std::fabs(ag[p].lateralOffset - prevLean[p]));
            prevLean[p] = ag[p].lateralOffset;
            wasMoving[p] = ag[p].moving;
            if (!ag[p].moving || ag[p].speed < 0.3) continue;   // actually walking
            for (std::size_t q = p + 1; q < ag.size(); ++q) {
                if (!ag[q].moving || ag[q].speed < 0.3)
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
    CHECK(maxLeanJump < 0.17);  // ~kPedLateralRate * dt: continuous, never a pop
    bool same = true;           // deterministic
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
}

TEST_CASE(tether_held_pedestrian_ghost_does_not_drift) {
    // Review fix (H1): the reactive lean is pos += offset on top of a re-anchored
    // pose. A tether-HELD ghost skipped advance() (no re-anchor) but still got the
    // lean re-applied every tick — a pinned walker's ghost slid sideways without
    // bound. Pin a walking ped's tether far behind it and hold for 30 s: the
    // ghost must stay put (waiting), not wander off the map.
    const citytest::ModelScene& s = citytest::straightScene(200.0);
    CitySim sim;
    s.build(sim, 6, 9);     // several walkers so the cone SEES neighbours
                            // (a non-zero lean is what used to accumulate)
    // Walk until someone is properly en route.
    int target = -1;
    for (int i = 0; i < 4000 && target < 0; ++i) {
        sim.step(0.1, 0.5);
        const auto& ag = sim.agents();
        for (int k = 0; k < static_cast<int>(ag.size()); ++k)
            if (ag[k].moving && ag[k].speed > 0.5) { target = k; break; }
    }
    CHECK(target >= 0);

    // Pin its body anchor far behind: the hold engages immediately and stays.
    Vec2 heldAt = sim.agents()[target].pos;
    Vec2 anchor(heldAt.x - 50.0, heldAt.y - 50.0);
    Real maxDrift = 0;
    for (int i = 0; i < 300; ++i) {
        sim.setAgentTether(target, anchor, 5.0);
        sim.step(0.1, 0.5);
        maxDrift = std::max(maxDrift, (sim.agents()[target].pos - heldAt).length());
    }
    CHECK(maxDrift < 1.0);   // held means held — the bug drifted metres per tick
}

// WALKERS IN FILES (Glenn: "all the agents ... on top of one another like it's some spawn point"). A tower door's
// strollers went down one line 1 m in from the kerb, each held behind the next: single file at a third of a walking
// pace, a heap at the door. On a wide pavement they keep up to three files, each its own following chain; on a narrow
// one (and where the sim knows no pavement) one file, as before.
TEST_CASE(walkers_keep_files_across_a_wide_pavement) {
    const citytest::ModelScene& s = citytest::gridScene(600.0, 120.0, 5);
    CitySim sim;
    sim.setWander(true);
    s.build(sim, 300, 11);
    int byFile[3] = {0, 0, 0};
    for (const Agent& a : sim.agents()) ++byFile[std::clamp(sim.walkerFile(a), 0, 2)];
    CHECK(byFile[0] == 300);   // no pavement known: the one line by the kerb

    sim.setJunctionPad(5.0);   // the island's 5 m pavements
    int files[3] = {0, 0, 0};
    for (const Agent& a : sim.agents()) ++files[std::clamp(sim.walkerFile(a), 0, 2)];
    std::printf("    [files] %d / %d / %d of 300\n", files[0], files[1], files[2]);
    for (int f = 0; f < 3; ++f) CHECK(files[f] > 60);
    // ...0.9 m apart, the outermost 2.8 m in: on a 5 m pavement, short of the shopfronts
    Real most = 0;
    for (const Agent& a : sim.agents()) most = std::max(most, sim.walkerVerge(a));
    CHECK(std::fabs(most - 2.8) < 1e-6);

    sim.setJunctionPad(3.0);   // a narrow pavement: two files
    int narrow = 0;
    for (const Agent& a : sim.agents()) narrow = std::max(narrow, sim.walkerFile(a));
    CHECK(narrow == 1);

    // where they walk: their own file across the street's walk (W3: measured against the walk's own line, the sidewalk's
    // middle; a file is its distance in from the kerb, walkerVerge, held within the walk)
    sim.setJunctionPad(5.0);
    for (int t = 0; t < 200; ++t) sim.step(0.05, 0.0);
    int checked = 0, onFile = 0;
    for (const Agent& a : sim.agents()) {
        if (!CitySim::onRoute(a) || a.roadSide == 0) continue;
        const engine::roads::WalkStep& s = a.route.steps[static_cast<std::size_t>(a.leg)];
        if (sim.roads()->laneLevel.walks[s.walk.index].street == engine::kNoIndex) continue;   // on a street's stretch
        const Real along = std::fabs(s.to - s.from);
        if (along < 30 || a.distOnLeg < 12 || a.distOnLeg > along - 12) continue;   // clear of the corners
        Vec2 dir(1, 0);
        const Vec2 P = sim.routePoint(a, a.leg, a.distOnLeg, &dir);
        const Vec2 right(dir.y, -dir.x);
        const Vec2 away = a.roadSide > 0 ? right * -1.0 : right;
        const Real half = s.walk.index < sim.roads()->laneLevel.walks.size() ? sim.roads()->laneLevel.walks[s.walk.index].width * 0.5 : 0;
        const Real room = std::max(Real(0), half - CitySim::kWalkerHalfAcross);
        const Real file = std::clamp(sim.walkerVerge(a) - half, -room, room);
        ++checked;
        onFile += std::fabs(dot(a.pos - P, away) - file) < 1.7;   // (less the lean)
    }
    std::printf("    [files] %d of %d walkers mid-link on their file\n", onFile, checked);
    CHECK(checked > 30);
    CHECK(onFile * 10 >= checked * 8);
}
