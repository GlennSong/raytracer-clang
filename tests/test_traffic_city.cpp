#include "test_framework.h"

#include "../src/engine/ai/agent_sim.h"
#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/ai/pathfind.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cmath>

using namespace engine;

// Exercises the navigation stack over the REAL procedural road generators
// (grid / radial / tensor), not just hand-built graphs — the integration the
// whole living-world feature stands on. Fully headless (ADR-0059).

namespace {

NavGraph gridCity(uint32_t seed) {
    GridRoadParams p;
    p.extent = 200;
    p.cellSize = 50;
    p.dropout = 0.0;
    p.seed = seed;
    return buildNavGraph(gridRoads(p));
}

}  // namespace

TEST_CASE(nav_from_grid_is_connected) {
    NavGraph nav = gridCity(11);
    CHECK(nav.nodeCount() > 10);
    CHECK(nav.linkCount() > 10);
    // Every link has at least one lane and positive length.
    bool sane = true;
    for (int i = 0; i < nav.linkCount(); ++i)
        if (nav.links[i].lanes < 1 || nav.links[i].length <= 0) sane = false;
    CHECK(sane);

    // A deformed-but-undropped grid is fully connected and two-way, so routes
    // exist between arbitrary node pairs. Sample a spread of them.
    int n = nav.nodeCount();
    int found = 0, tried = 0;
    for (int k = 0; k < 12; ++k) {
        int a = (k * 7) % n;
        int b = (k * 13 + 3) % n;
        if (a == b) continue;
        ++tried;
        if (findRoute(nav, a, b).valid()) ++found;
    }
    CHECK(tried > 0);
    CHECK(found == tried);   // grid is connected: all pairs route
}

TEST_CASE(nav_from_radial_is_nonempty) {
    RadialParams p;
    p.extent = 200;
    p.ringSpacing = 60;
    p.spokes = 8;
    p.seed = 3;
    NavGraph nav = buildNavGraph(radialRoads(p));
    CHECK(nav.nodeCount() > 0);
    CHECK(nav.linkCount() > 0);
}

TEST_CASE(nav_from_tensor_is_nonempty) {
    TensorRoadParams p;
    p.extent = 160;
    p.spacing = 60;
    p.step = 10;
    p.seed = 5;
    NavGraph nav = buildNavGraph(tensorRoads(p));
    // The tensor tracer always lays down some streets for these params.
    CHECK(nav.nodeCount() > 0);
    CHECK(nav.linkCount() > 0);
}

TEST_CASE(agents_commute_over_grid_city) {
    NavGraph nav = gridCity(11);
    AgentSim sim;
    sim.build(nav, 50, 20, 777);
    CHECK(sim.agents().size() == 70u);

    bool anyMoved = false, anyAtWork = false;
    for (int i = 0; i < 6000; ++i) {
        sim.step(0.1, 0.3);
        for (const Agent& a : sim.agents()) {
            if (a.moving) anyMoved = true;
            if (a.activity == AgentActivity::AtWork) anyAtWork = true;
        }
    }
    CHECK(anyMoved);
    CHECK(anyAtWork);

    // Nobody escapes the city footprint (extent 200 + lane/jitter margin).
    bool inBounds = true;
    for (const Agent& a : sim.agents())
        if (std::abs(a.pos.x) > 260 || std::abs(a.pos.y) > 260) inBounds = false;
    CHECK(inBounds);
}

TEST_CASE(sim_deterministic_over_generated_city) {
    NavGraph nav = gridCity(22);
    AgentSim a, b;
    a.build(nav, 30, 30, 9090);
    b.build(nav, 30, 30, 9090);
    for (int i = 0; i < 2500; ++i) {
        a.step(0.1, 0.3);
        b.step(0.1, 0.3);
    }
    bool identical = (a.agents().size() == b.agents().size());
    for (std::size_t i = 0; i < a.agents().size() && identical; ++i) {
        if (a.agents()[i].pos.x != b.agents()[i].pos.x ||
            a.agents()[i].pos.y != b.agents()[i].pos.y ||
            a.agents()[i].activity != b.agents()[i].activity)
            identical = false;
    }
    CHECK(identical);
}
