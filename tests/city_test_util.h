#ifndef RAYTRACER_TESTS_CITY_TEST_UTIL_H
#define RAYTRACER_TESTS_CITY_TEST_UTIL_H

// Shared road scenarios for the citysim tests — the graphs that were re-declared
// at the top of every test_city_*.cpp. Header-only (inline), so each test TU
// picks just what it needs; test logic and thresholds stay in the test files.

#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/procgen/city/road_network.h"

#include <cstdint>

namespace citytest {

// A 4-way cross: centre node 0 is a junction, arm tips 1..4. buildNavGraph makes
// each edge two-way, so routes between arms cross the signalled centre, and a
// perpendicular pair forces a 90-degree turn — one shared conflict point that
// surfaces Turning / Waiting (red) / Following, and is exactly where a naive
// "brake for everything you see" rule deadlocks.
inline engine::NavGraph cross4(engine::Real arm) {
    engine::RoadGraph g;
    g.nodes = { {engine::Vec2(0, 0)}, {engine::Vec2(0, arm)}, {engine::Vec2(0, -arm)},
                {engine::Vec2(arm, 0)}, {engine::Vec2(-arm, 0)} };
    g.edges = {
        engine::RoadEdge{0, 1, 8, engine::RoadClass::Local, 0},
        engine::RoadEdge{0, 2, 8, engine::RoadClass::Local, 0},
        engine::RoadEdge{0, 3, 8, engine::RoadClass::Local, 0},
        engine::RoadEdge{0, 4, 8, engine::RoadClass::Local, 0},
    };
    return engine::buildNavGraph(g);
}

// A single straight two-node road: one leg, no junction, no signal — a clear
// open stretch where car-following (or free cruising) is the only rule in play.
// `width` sets the carriageway: 8 m carries two lanes, 5 m forces a single lane
// (same-direction cars queue in one line).
inline engine::NavGraph straightRoad(engine::Real len, engine::Real width = 8) {
    engine::RoadGraph g;
    g.nodes = { {engine::Vec2(0, 0)}, {engine::Vec2(len, 0)} };
    g.edges = { engine::RoadEdge{0, 1, width, engine::RoadClass::Local, 0} };
    return engine::buildNavGraph(g);
}

// A dropout-free grid city (gridRoads): real junctions, alternative routes, and
// sidewalks — the general-purpose town for population/perception tests.
inline engine::NavGraph cityNav(engine::Real extent, engine::Real cellSize,
                                uint32_t seed) {
    engine::GridRoadParams p;
    p.extent = extent;
    p.cellSize = cellSize;
    p.dropout = 0.0;
    p.seed = seed;
    return engine::buildNavGraph(engine::gridRoads(p));
}

}  // namespace citytest

#endif
