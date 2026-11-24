#ifndef RAYTRACER_TESTS_CITY_TEST_UTIL_H
#define RAYTRACER_TESTS_CITY_TEST_UTIL_H

// The grid town the citysim tests' towns are drawn from: a road DRAWING, never a street map. The tests build it into a
// road model as a level's roads are (model_scene.h gridScene, #306 piece 6 W2); the hand-made street maps that were
// here (cross4, tee, straightRoad, cityNav: a street map over a drawing) are gone with that.

#include "../src/engine/procgen/city/road_network.h"
#include "../src/proc_rng.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <cstdint>

namespace citytest {

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

}  // namespace citytest

#endif
