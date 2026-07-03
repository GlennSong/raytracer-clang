#include "test_framework.h"

#include "city_test_util.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cmath>
#include <vector>

using namespace engine;
using namespace citysim;

namespace {

// cross4 (city_test_util.h): routing between perpendicular arms forces a
// 90-degree turn through the centre junction, exercising the steering arc.
using citytest::cross4;

// Y junction: three arms at 0/120/240 deg around centre node 0. No two arms are
// collinear, so EVERY route between distinct arm tips bends ~60 deg at the
// centre — a reliable turn to exercise the steering arc, independent of which
// home/work pairs the seed happens to draw.
NavGraph yJunction(Real arm) {
    const Real c = -0.5 * arm, s = 0.8660254037844386 * arm;   // cos/sin 120 deg
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(arm, 0)}, {Vec2(c, s)}, {Vec2(c, -s)} };
    g.edges = {
        RoadEdge{0, 1, 8, RoadClass::Local, 0},
        RoadEdge{0, 2, 8, RoadClass::Local, 0},
        RoadEdge{0, 3, 8, RoadClass::Local, 0},
    };
    return buildNavGraph(g);
}

Real headingAngle(Vec2 a, Vec2 b) {
    Real al = std::sqrt(a.x * a.x + a.y * a.y);
    Real bl = std::sqrt(b.x * b.x + b.y * b.y);
    if (al < 1e-9 || bl < 1e-9) return 0.0;
    Real d = (a.x * b.x + a.y * b.y) / (al * bl);
    if (d > 1) d = 1; else if (d < -1) d = -1;
    return std::acos(d);
}

// Mirror of the sim's contract (city_sim.cpp): the tightest arc radius. Yaw rate
// is purely speed-proportional (v / radius) — a stopped car does not turn.
constexpr Real kMinTurnRadius = 6.0;

}  // namespace

TEST_CASE(car_heading_yaw_rate_is_bounded_by_turn_radius) {
    NavGraph nav = cross4(40.0);
    CitySim sim;
    sim.build(nav, 16, 0, 7);

    const Real dt = 0.1;
    std::vector<Vec2> prev(sim.agents().size());
    std::vector<bool> have(sim.agents().size(), false);

    bool sawMotion = false;
    for (int i = 0; i < 6000; ++i) {
        sim.step(dt, 0.5);
        const auto& ag = sim.agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            const Agent& a = ag[k];
            if (a.mode != Agent::Mode::Driver) continue;
            if (a.moving && have[k]) {
                sawMotion = true;
                Real ang = headingAngle(prev[k], a.heading);
                // The yaw applied this step used this step's (finalized) speed, so
                // that is what bounds it: v / radius * dt. A small epsilon covers
                // floating-point rounding in the rotation.
                Real bound = (a.speed / kMinTurnRadius * dt) + 1e-3;
                CHECK(ang <= bound);
            }
            prev[k] = a.heading;
            have[k] = a.moving;
        }
    }
    CHECK(sawMotion);   // cars actually drove (so the bound was exercised)
}

TEST_CASE(cars_still_complete_their_turns) {
    NavGraph nav = yJunction(40.0);
    CitySim sim;
    sim.build(nav, 6, 0, 2024);

    // Heading is always unit length, and at least one car negotiates a real
    // turn (its heading swings well past the straight-through case).
    Real maxSwing = 0.0;
    std::vector<Vec2> start(sim.agents().size());
    std::vector<bool> have(sim.agents().size(), false);
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.5);
        for (const Agent& a : sim.agents()) {
            Real len = std::sqrt(a.heading.x * a.heading.x + a.heading.y * a.heading.y);
            CHECK(std::fabs(len - 1.0) < 1e-6);   // heading stays normalized
        }
        const auto& ag = sim.agents();
        for (std::size_t k = 0; k < ag.size(); ++k) {
            if (ag[k].mode != Agent::Mode::Driver || !ag[k].moving) { have[k] = false; continue; }
            if (!have[k]) { start[k] = ag[k].heading; have[k] = true; }
            maxSwing = std::max(maxSwing, headingAngle(start[k], ag[k].heading));
        }
    }
    CHECK(maxSwing > 0.9);   // ~60 deg turn at the Y really completed
}

TEST_CASE(steering_preserves_determinism) {
    NavGraph nav = cross4(40.0);
    CitySim a, b;
    a.build(nav, 12, 8, 4242);
    b.build(nav, 12, 8, 4242);
    for (int i = 0; i < 3000; ++i) { a.step(0.1, 0.5); b.step(0.1, 0.5); }
    bool same = true;
    for (std::size_t i = 0; i < a.agents().size(); ++i)
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y ||
            a.agents()[i].heading.x != b.agents()[i].heading.x ||
            a.agents()[i].heading.y != b.agents()[i].heading.y)
            same = false;
    CHECK(same);
}
