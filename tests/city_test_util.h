#ifndef RAYTRACER_TESTS_CITY_TEST_UTIL_H
#define RAYTRACER_TESTS_CITY_TEST_UTIL_H

// Shared road scenarios for the citysim tests — the graphs that were re-declared
// at the top of every test_city_*.cpp. Header-only (inline), so each test TU
// picks just what it needs; test logic and thresholds stay in the test files.

#include "../src/engine/ai/nav_graph.h"
#include "../src/engine/procgen/city/road_network.h"
#include "../src/proc_rng.h"

#include <algorithm>
#include <cmath>
#include <vector>

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

// A T-junction: centre node 0 (degree 3 — below the 4-approach signal
// threshold, so it is UNCONTROLLED), arm tips 1..3. The kerb-discipline tests
// use it: gap acceptance is exactly the unsignalled-crossing rule.
inline engine::NavGraph tee(engine::Real arm) {
    engine::RoadGraph g;
    g.nodes = { {engine::Vec2(0, 0)}, {engine::Vec2(0, arm)},
                {engine::Vec2(arm, 0)}, {engine::Vec2(-arm, 0)} };
    g.edges = {
        engine::RoadEdge{0, 1, 8, engine::RoadClass::Local, 0},
        engine::RoadEdge{0, 2, 8, engine::RoadClass::Local, 0},
        engine::RoadEdge{0, 3, 8, engine::RoadClass::Local, 0},
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

// A JITTERED GRID TOWN, the test fixture the sim tests drive on: a square of `extent` half-size cut every
// `cellSize`, interior junctions jittered by `jitter` x the spacing, border lines arterials, every third line a
// collector, locals between (a local segment may drop out with `dropout`). It was the engine's `gridRoads`
// layout generator until the city planner became the one street generator (ADR-0267); it lives on here,
// draw for draw, so the tests' towns -- and the numbers measured on them -- are the same towns.
struct GridCityParams {
    engine::Vec2 center{0, 0};
    engine::Real extent = 400;     // half-size of the square region (m)
    engine::Real cellSize = 90;    // target block spacing (m)
    engine::Real jitter = 0.18;    // vertex jitter as a fraction of cellSize
    engine::Real dropout = 0.0;    // probability a local street segment is removed
    engine::Real arterialWidth = 16, collectorWidth = 11, localWidth = 8;
    uint32_t seed = 0;
};

inline engine::RoadGraph gridCity(const GridCityParams& p) {
    using engine::RoadClass;
    engine::RoadGraph g;
    engine::ProcRng rng(p.seed);
    const int n = std::max(1, static_cast<int>(std::lround(p.extent * 2 / p.cellSize)));
    const int dim = n + 1;
    const engine::Real step = p.extent * 2 / n;
    const engine::Vec2 o = p.center - engine::Vec2(p.extent, p.extent);
    std::vector<std::vector<int>> id(dim, std::vector<int>(dim, -1));
    for (int j = 0; j < dim; ++j)
        for (int i = 0; i < dim; ++i) {
            engine::Vec2 pos = o + engine::Vec2(i * step, j * step);
            const bool interior = (i > 0 && i < dim - 1 && j > 0 && j < dim - 1);
            if (interior) {
                const engine::Real jr = p.jitter * step;
                const engine::Real jx = rng.in(-jr, jr);   // (one draw a statement: GCC takes Vec2's arguments right to left)
                const engine::Real jy = rng.in(-jr, jr);
                pos += engine::Vec2(jx, jy);
            }
            id[j][i] = g.addNode(pos, 0.01);
        }
    auto classOf = [&](int line) {
        if (line == 0 || line == dim - 1) return RoadClass::Arterial;
        return (line % 3 == 0) ? RoadClass::Collector : RoadClass::Local;
    };
    auto widthOf = [&](RoadClass c) {
        return c == RoadClass::Arterial ? p.arterialWidth : c == RoadClass::Collector ? p.collectorWidth : p.localWidth;
    };
    // border arterials always stay, so the region stays enclosed
    for (int j = 0; j < dim; ++j)
        for (int i = 0; i < dim; ++i) {
            if (i + 1 < dim) {
                const RoadClass c = classOf(j);
                if (!(c == RoadClass::Local && rng.next() < p.dropout)) g.addEdge(id[j][i], id[j][i + 1], widthOf(c), c);
            }
            if (j + 1 < dim) {
                const RoadClass c = classOf(i);
                if (!(c == RoadClass::Local && rng.next() < p.dropout)) g.addEdge(id[j][i], id[j + 1][i], widthOf(c), c);
            }
        }
    return g;
}

// A dropout-free grid town: real junctions, alternative routes, and sidewalks — the general-purpose town for
// population/perception tests.
inline engine::NavGraph cityNav(engine::Real extent, engine::Real cellSize,
                                uint32_t seed) {
    GridCityParams p;
    p.extent = extent;
    p.cellSize = cellSize;
    p.dropout = 0.0;
    p.seed = seed;
    return engine::buildNavGraph(gridCity(p));
}

}  // namespace citytest

#endif
