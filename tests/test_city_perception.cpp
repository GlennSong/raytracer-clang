#include "test_framework.h"

#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/ai/perception.h"
#include "../src/engine/procgen/city/road_network.h"

using namespace engine;
using namespace citysim;

namespace {
NavGraph cityNav() {
    GridRoadParams p;
    p.extent = 180;
    p.cellSize = 45;
    p.dropout = 0.0;
    p.seed = 4;
    return buildNavGraph(gridRoads(p));
}
}  // namespace

TEST_CASE(perfect_agents_make_no_faults) {
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 20, 20, 11);
    sim.setPerceptionReliability(1.0);
    for (int i = 0; i < 3000; ++i) sim.step(0.1, 0.4);
    CHECK(sim.faults() == 0);   // a perfect driver never misses
}

TEST_CASE(imperfect_agents_make_faults_deterministically) {
    NavGraph nav = cityNav();
    CitySim a, b;
    a.build(nav, 25, 0, 808);
    b.build(nav, 25, 0, 808);
    a.setPerceptionReliability(0.7);
    b.setPerceptionReliability(0.7);
    for (int i = 0; i < 3000; ++i) { a.step(0.1, 0.4); b.step(0.1, 0.4); }
    CHECK(a.faults() > 0);            // imperfect drivers do miss sometimes
    CHECK(a.faults() == b.faults());  // and reproducibly (same seed -> same faults)
    // Full state stays deterministic with faults in play.
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
}

// Average speed of moving cars that have a pedestrian ahead in their vision cone.
// With perception ON the car brakes for that pedestrian, so the average is lower.
static double avgSpeedWithPedAhead(CitySim& sim, double reliability) {
    sim.setPerceptionReliability(reliability);
    double sum = 0;
    long count = 0;
    for (int step = 0; step < 4000; ++step) {
        sim.step(0.1, 0.4);
        const auto& ag = sim.agents();
        for (const Agent& c : ag) {
            if (c.mode != Agent::Mode::Driver || !c.moving) continue;
            VisionCone cone{c.pos, c.heading, 25.0, 0.9};
            bool pedAhead = false;
            for (const Agent& p : ag) {
                if (p.mode != Agent::Mode::Pedestrian) continue;
                if (sees(cone, p.pos) && forwardDistance(cone, p.pos) > 0) {
                    pedAhead = true;
                    break;
                }
            }
            if (pedAhead) { sum += c.speed; ++count; }
        }
    }
    return count > 0 ? sum / static_cast<double>(count) : -1.0;
}

TEST_CASE(cars_slow_down_for_pedestrians_they_see) {
    NavGraph nav = cityNav();
    CitySim perceptive, blind;
    perceptive.build(nav, 25, 40, 321);
    blind.build(nav, 25, 40, 321);
    double withPerception = avgSpeedWithPedAhead(perceptive, 1.0);
    double withoutPerception = avgSpeedWithPedAhead(blind, 0.0);   // never perceives
    CHECK(withPerception >= 0.0);    // cars did encounter pedestrians ahead
    CHECK(withoutPerception >= 0.0);
    // Seeing a pedestrian ahead makes a car measurably slower than a blind one.
    CHECK(withPerception < withoutPerception);
}
