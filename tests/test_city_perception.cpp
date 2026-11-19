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
    sim.build(nav, 30, 11);

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
    a.build(nav, 24, 77);
    b.build(nav, 24, 77);
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
