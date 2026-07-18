#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/ai/perception.h"
#include "../src/engine/procgen/city/road_network.h"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace engine;
using namespace citysim;

namespace {
// This file's grid town (city_test_util.h cityNav, historical dimensions/seed).
NavGraph cityNav() { return citytest::cityNav(180, 45, 4); }
}  // namespace

TEST_CASE(pedestrians_walk_around_static_poles) {
    // Signal poles are handed to the sim as static obstacles; a walker must steer
    // around them and never end up standing inside one. Place a pole at the mid-
    // point of every sidewalk so any walking ped meets one, then assert no ped
    // centre ever comes closer than the pole clearance — and that at least one got
    // pushed right up against a pole (so the avoidance was actually exercised).
    constexpr Real kPoleClearance = 0.7;   // mirror of the sim constant
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 0, 30, 11);   // pedestrians only

    std::vector<Vec2> poles;
    for (int li = 0; li < nav.linkCount(); ++li) poles.push_back(nav.sidewalkPoint(li, 0.5));
    sim.setStaticObstacles(poles);

    Real minDist = 1e9;
    for (int i = 0; i < 4000; ++i) {
        sim.step(0.1, 0.4);
        for (const Agent& a : sim.agents()) {
            if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
            for (const Vec2& o : poles) {
                Real dx = a.pos.x - o.x, dy = a.pos.y - o.y;
                minDist = std::min(minDist, std::sqrt(dx * dx + dy * dy));
            }
        }
    }
    CHECK(minDist >= kPoleClearance - 1e-3);   // never standing inside a pole
    CHECK(minDist <= kPoleClearance + 0.6);    // a ped really did brush past one
}

TEST_CASE(static_obstacle_avoidance_is_deterministic) {
    NavGraph nav = cityNav();
    CitySim a, b;
    a.build(nav, 0, 24, 77);
    b.build(nav, 0, 24, 77);
    std::vector<Vec2> poles;
    for (int li = 0; li < nav.linkCount(); ++li) poles.push_back(nav.sidewalkPoint(li, 0.4));
    a.setStaticObstacles(poles);
    b.setStaticObstacles(poles);
    for (int i = 0; i < 2500; ++i) { a.step(0.1, 0.4); b.step(0.1, 0.4); }
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y)
            same = false;
    CHECK(same);
}

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

TEST_CASE(cars_remember_a_person_after_losing_sight) {
    // Object permanence at the sim level (ADR-0063): a car that saw a person in
    // its lane keeps yielding to the MEMORY of them for a few seconds after they
    // vanish from sight — then the track fades and the car resumes. A snapshot-
    // driven car would floor it the very frame the person disappeared.
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(160, 0)} };
    g.edges = { RoadEdge{0, 1, 8, RoadClass::Local, 0} };
    NavGraph nav = buildNavGraph(g);
    CitySim sim;
    sim.build(nav, 1, 0, 3);   // one lone driver, an empty straight road

    // Once the car rolls, freeze a person 12 m dead ahead in its lane.
    bool placed = false;
    Vec2 person;
    for (int i = 0; i < 4000 && !placed; ++i) {
        sim.step(0.1, 0.5);
        const Agent& c = sim.agents().front();
        if (c.moving) {
            person = Vec2(c.pos.x + c.heading.x * 12.0, c.pos.y + c.heading.y * 12.0);
            sim.setExternalObstacles({ person });
            placed = true;
        }
    }
    CHECK(placed);

    // The car walls up short of the person and holds.
    bool stopped = false;
    for (int i = 0; i < 600 && !stopped; ++i) {
        sim.step(0.1, 0.5);
        const Agent& c = sim.agents().front();
        Real dx = c.pos.x - person.x, dy = c.pos.y - person.y;
        if (c.moving && c.speed < 0.05 && std::sqrt(dx * dx + dy * dy) < 6.0)
            stopped = true;
    }
    CHECK(stopped);

    // The person vanishes. The car must NOT sail through their spot immediately —
    // memory still holds a confident track there, so it creeps up and walls a few
    // metres short — but once the track fades it must drive on through.
    sim.setExternalObstacles({});
    int steps = 0;
    const int kMax = 200;   // 20 s ceiling
    bool passed = false;
    while (steps < kMax && !passed) {
        sim.step(0.1, 0.5);
        ++steps;
        const Agent& c = sim.agents().front();
        Real ahead = (person.x - c.pos.x) * c.heading.x + (person.y - c.pos.y) * c.heading.y;
        if (ahead < -1.0) passed = true;   // a metre beyond where they stood
    }
    Real heldSeconds = steps * 0.1;
    CHECK(passed);                       // recovered — the ghost didn't pin it forever
    CHECK(heldSeconds >= 2.0);           // remembered: held off well after the loss
    CHECK(heldSeconds <= 8.0);           // ...but forgot in time and moved on
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

TEST_CASE(pedestrians_never_stand_inside_cars) {
    // Cars are obstacles to walkers (device: "pedestrians get stuck if they bump
    // up against a car"): the think scan considers the closest point on each
    // car's rectangle and a hard push-out squeezes a walker out of the footprint
    // along the shallowest axis, so it slides around the body instead of
    // pinning. Invariant: after any step, no MOVING walker's centre is inside
    // any car's (slightly inflated) footprint.
    NavGraph nav = cityNav();
    CitySim sim;
    sim.build(nav, 14, 20, 9);   // cars AND pedestrians
    Real worstPen = 0;
    for (int i = 0; i < 3000; ++i) {
        sim.step(0.1, 0.4);
        for (const Agent& a : sim.agents()) {
            if (a.mode != Agent::Mode::Pedestrian || !a.moving) continue;
            for (const SimVehicle& v : sim.vehicles()) {
                Vec2 f = v.heading, r(v.heading.y, -v.heading.x);
                Vec2 rel(a.pos.x - v.pos.x, a.pos.y - v.pos.y);
                Real lx = std::fabs(rel.x * f.x + rel.y * f.y);
                Real ly = std::fabs(rel.x * r.x + rel.y * r.y);
                Real px = v.length * 0.5 + 0.30 - lx;
                Real py = v.width * 0.5 + 0.30 - ly;
                if (px > 0 && py > 0) worstPen = std::max(worstPen, std::min(px, py));
            }
        }
    }
    std::printf("[pen] worstPen=%.3f\n", worstPen);
    CHECK(worstPen < 0.10);   // never meaningfully inside a car body
}
