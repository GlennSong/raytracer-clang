// Parking stalls of a lot (engine::surfaceStallsOf), and the trip lab's walker on a cross. (The old driving's kerb bays
// and garage stalls, and the tests of its cars parking in them, went with it: ADR-0462.)
#include "test_framework.h"

#include "model_scene.h"
#include "../src/apps/citysim/city_sim.h"
#include "../src/engine/procgen/city/city_lots.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace engine;
using namespace citysim;

// A BIG BOX'S OR A MALL'S PARKING LOT is stalls for the drivers (engine::surfaceStallsOf): every stall on the lot, none
// on another, nose-in off the aisles, and its way in on the street edge. A shallow lot runs its rows along the store
// (more stalls than rows running in toward it), a deep one in toward it.
TEST_CASE(a_parking_lot_is_stalls_for_the_drivers) {
    using engine::Vec2;
    const engine::Poly2 lot{{0, 0}, {80, 0}, {80, 44}, {0, 44}};   // the street along y = 0, the store beyond y = 44
    Vec2 portal, out;
    const std::vector<engine::ParkingStall> st = engine::surfaceStallsOf(lot, Vec2(0, -1), &portal, &out);
    std::printf("  %zu stalls; portal %.1f %.1f out %.1f %.1f\n", st.size(), portal.x, portal.y, out.x, out.y);
    CHECK(st.size() >= 60);
    CHECK(std::fabs(st.front().face.y) > 0.99);   // 44 m deep: rows along the store
    const std::vector<engine::ParkingStall> deep = engine::surfaceStallsOf(engine::Poly2{{0, 0}, {80, 0}, {80, 70}, {0, 70}}, Vec2(0, -1));
    std::printf("  70 m deep: %zu stalls, the first facing %.0f %.0f\n", deep.size(), deep.front().face.x, deep.front().face.y);
    CHECK(deep.size() > st.size());
    CHECK(std::fabs(portal.y) < 1e-6 && std::fabs(portal.x - 40) < 1e-6);
    CHECK(out.y < -0.99);
    for (std::size_t i = 0; i < st.size(); ++i) {
        CHECK(st[i].at.x > 1.5 && st[i].at.x < 78.5 && st[i].at.y > 8.0 && st[i].at.y < 36.0);   // clear of the lanes
        CHECK(std::fabs(std::fabs(st[i].face.x) + std::fabs(st[i].face.y) - 1.0) < 1e-6);   // along a row's axis
        for (std::size_t j = i + 1; j < st.size(); ++j) CHECK((st[i].at - st[j].at).length() > 2.6);
    }
}

// THE TRIP LAB'S WALKER (ADR-0415 as built): a person at rest beside a street sets off from where they stand -- not
// from the nearest junction, which put them there in one jump (19.5 m from a front door) to walk back past it -- and
// the line the lab draws for them (walkPoses: the stepper's own placing, sampled) starts where they are and runs on
// without a jump or a turn back, to where they stop.
TEST_CASE(trip_plan_walker_sets_off_where_it_stands_and_its_line_is_its_way) {
    // a cross of 100 m local roads, built as a level's roads are (model_scene.h); the places are on its east pavement
    // of the north arm, as the model places a sidewalk point (W2: was a hand-made street map with a parking band, the
    // points written at x = 9)
    const citytest::ModelScene& s = citytest::crossScene(100);
    CHECK(s.problems.empty());
    const Vec2 half = s.sidewalkBeside(Vec2(9, 50)), up = s.sidewalkBeside(Vec2(9, 85));
    CitySim sim;
    s.build(sim, 1, 5);
    CHECK(sim.agents().size() == 1);
    if (sim.agents().size() != 1) return;
    sim.setAgentDirected(0, true);
    // to the east pavement of the north arm, halfway up it; then wait for them to stop there
    CHECK(sim.sendAgentTo(0, half));
    for (int i = 0; i < 6000 && sim.agents()[0].moving; ++i) sim.step(0.05);
    const Agent& a = sim.agents()[0];
    CHECK(!a.moving);
    const Vec2 rest = a.pos;
    CHECK(std::fabs(rest.y - 50.0) < 3.0);
    // further up the same pavement: set off from right here
    CHECK(sim.sendAgentTo(0, up));
    CHECK((a.pos - rest).length() < 0.5);
    std::vector<Vec2> line;
    sim.walkPoses(0, 2.0, [&](const Agent& c) { line.push_back(c.pos); });
    CHECK(line.size() > 10);
    if (line.empty()) return;
    CHECK((line.front() - a.pos).length() < 1e-6);
    // NO JUMP: each 2 m of walk is at most kStepWithFile from the last pose (W3: 2.0 -> 2.25 m: a walker takes up its file
    // across the walk over its step's first 4 m, so a 2 m step along can carry it up to half a metre sideways too)
    constexpr Real kStepWithFile = 2.25;
    Real longest = 0;
    for (std::size_t i = 1; i < line.size(); ++i) {
        longest = std::max(longest, (line[i] - line[i - 1]).length());
        CHECK((line[i] - line[i - 1]).length() <= kStepWithFile);   // no jump
        CHECK(line[i].y >= line[i - 1].y - 1e-6);                 // never back the way it came
    }
    std::printf("[trip lab] the longest pose to pose %.2f m (2 m steps)\n", longest);
    CHECK(std::fabs(line.back().y - 85.0) < 1.0);   // ...to beside the point it is for
    // and walking it, the person keeps to that line
    Real worst = 0;
    for (int i = 0; i < 6000 && a.moving; ++i) {
        sim.step(0.05);
        Real nearest = 1e30;
        for (const Vec2& p : line) nearest = std::min(nearest, (p - a.pos).length());
        worst = std::max(worst, nearest);
    }
    std::printf("[trip lab] walker: %zu line points, furthest off the line %.2f m\n", line.size(), worst);
    CHECK(worst < 1.5);
}
