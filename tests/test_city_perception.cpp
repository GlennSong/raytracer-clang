#include "test_framework.h"

#include "model_scene.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/ai/perception.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

using namespace engine;
using namespace citysim;

namespace {
// This file's town: the grid town (city_test_util.h gridCity, its historical size and seed) as a road file, built as a
// level's roads are and walked on its road model's walks (model_scene.h, #306).
const citytest::ModelScene& town() {
    const std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
    const citytest::ModelScene& s = citytest::gridScene(180, 45, 4);
    std::printf("    [scene] grid town 180/45/4: %zu walks; %.2f s%s%s\n", s.model->laneLevel.walks.size(),
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(),
                s.problems.empty() ? "" : "; ", s.problems.c_str());
    return s;
}
// A pole a fraction `t` along every sidewalk stretch of the town's walks (each walk along a street).
std::vector<Vec2> polesAlongSidewalks(const citytest::ModelScene& s, Real t) {
    std::vector<Vec2> poles;
    const engine::LaneGraph& g = s.model->laneLevel;
    for (uint32_t w = 0; w < g.walks.size(); ++w)
        if (g.walks[w].street != engine::kNoIndex) poles.push_back(g.pointAt(engine::WalkId(w), g.walks[w].length * t));
    return poles;
}
}  // namespace

TEST_CASE(pedestrians_walk_around_static_poles) {
    // Signal poles are handed to the sim as static obstacles; a walker must steer
    // around them and never end up standing inside one. Place a pole at the mid-
    // point of every sidewalk so any walking ped meets one, then assert no ped
    // centre ever comes closer than the pole clearance — and that at least one got
    // pushed right up against a pole (so the avoidance was actually exercised).
    constexpr Real kPoleClearance = 0.7;   // mirror of the sim constant
    const citytest::ModelScene& s = town();
    CHECK(s.problems.empty());
    CitySim sim;
    s.build(sim, 30, 11);

    const std::vector<Vec2> poles = polesAlongSidewalks(s, 0.5);
    sim.setStaticObstacles(poles);

    Real minDist = 1e9;
    for (int i = 0; i < 4000; ++i) {
        citytest::keepEveryoneWalking(sim, i / 600);   // (no day walks them: ADR-0499)
        sim.step(0.1, 0.4);
        for (const Agent& a : sim.agents()) {
            if (!a.moving) continue;
            for (const Vec2& o : poles) {
                Real dx = a.pos.x - o.x, dy = a.pos.y - o.y;
                minDist = std::min(minDist, std::sqrt(dx * dx + dy * dy));
            }
        }
    }
    std::printf("    [poles] %zu poles; the nearest a walker came %.3f m\n", poles.size(), minDist);
    CHECK(minDist >= kPoleClearance - 1e-3);   // never standing inside a pole
    CHECK(minDist <= kPoleClearance + 0.6);    // a ped really did brush past one
}

TEST_CASE(static_obstacle_avoidance_is_deterministic) {
    const citytest::ModelScene& s = town();
    CitySim a, b;
    s.build(a, 24, 77);
    s.build(b, 24, 77);
    const std::vector<Vec2> poles = polesAlongSidewalks(s, 0.4);
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
