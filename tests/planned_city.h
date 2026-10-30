#ifndef RAYTRACER_TESTS_PLANNED_CITY_H
#define RAYTRACER_TESTS_PLANNED_CITY_H

// The city the tests build on: planned by the city planner (ADR-0092) -- every city is, since the old metro
// grower retired (ADR-0267). A round brief small enough to plan in about a second: a grid core, midtown's
// rim, outskirts rings and spokes, a freeway ring and two expressways. Seeded, so a test judges several
// cities rather than one (every-seed rule).

#include "../src/engine/procgen/city/plan/city_plan.h"
#include "../src/engine/procgen/city/roads/road_entity.h"

#include <cmath>
#include <cstdint>
#include <vector>

namespace citytest {

inline engine::plan::Brief roundBrief(uint32_t seed = 11) {
    engine::plan::Brief b;
    b.name = "test_city";
    b.seed = seed;
    b.size = 1800;
    b.coreRadius = 280;
    b.midRadius = 480;
    b.freewayRadius = 620;
    b.spokes = 8;
    b.outerMargin = 100;
    return b;
}

// The streets as the lot pass is handed them in the engine. A plan's streets carry a node every 15-20 m along
// every street; a lane-built city reaches the lot pass through its road twin, which keeps nodes at corners
// only, at most 60 m apart (road_twin.cpp: a straight street with a node every few metres leaves faces whose
// miter inset collapses to nothing). So here too: a junction-free node on a straight run (within 0.5 m of the
// chord its neighbours make) goes, while the run it would leave stays under 60 m.
inline engine::RoadGraph chordStreets(const engine::RoadGraph& in) {
    engine::RoadGraph g = in;
    for (bool any = true; any;) {
        any = false;
        std::vector<std::vector<int>> inc(g.nodes.size());
        for (int e = 0; e < static_cast<int>(g.edges.size()); ++e) {
            inc[static_cast<std::size_t>(g.edges[e].a)].push_back(e);
            inc[static_cast<std::size_t>(g.edges[e].b)].push_back(e);
        }
        std::vector<char> dead(g.edges.size(), 0);
        std::vector<char> touched(g.nodes.size(), 0);
        for (std::size_t v = 0; v < g.nodes.size(); ++v) {
            if (inc[v].size() != 2) continue;
            const int e0 = inc[v][0], e1 = inc[v][1];
            if (dead[static_cast<std::size_t>(e0)] || dead[static_cast<std::size_t>(e1)]) continue;
            engine::RoadEdge& A = g.edges[static_cast<std::size_t>(e0)];
            const engine::RoadEdge& B = g.edges[static_cast<std::size_t>(e1)];
            if (A.klass != B.klass || A.width != B.width) continue;
            const int a = A.a == static_cast<int>(v) ? A.b : A.a, b = B.a == static_cast<int>(v) ? B.b : B.a;
            if (a == b || touched[static_cast<std::size_t>(a)] || touched[static_cast<std::size_t>(b)]) continue;
            const engine::Vec2 pa = g.nodes[static_cast<std::size_t>(a)].pos, pb = g.nodes[static_cast<std::size_t>(b)].pos,
                               pv = g.nodes[v].pos;
            const engine::Real len = (pb - pa).length();
            if (len > 60.0 || len < 1e-9) continue;
            if (std::fabs(engine::cross(pb - pa, pv - pa)) / len > 0.5) continue;   // a corner: keep it
            (A.a == static_cast<int>(v) ? A.a : A.b) = b;                           // A now runs a -> b
            dead[static_cast<std::size_t>(e1)] = 1;
            touched[v] = touched[static_cast<std::size_t>(a)] = touched[static_cast<std::size_t>(b)] = 1;
            any = true;
        }
        std::vector<engine::RoadEdge> kept;
        for (std::size_t i = 0; i < g.edges.size(); ++i) if (!dead[i]) kept.push_back(g.edges[i]);
        g.edges.swap(kept);
    }
    return g;   // (orphaned nodes stay: indices are stable, and nothing walks a node without an edge)
}

inline engine::plan::CityPlan plannedCity(uint32_t seed = 11) {
    engine::plan::CityPlan city = engine::plan::generatePlan(roundBrief(seed));
    city.streets = chordStreets(city.streets);
    return city;
}

// The same city as a road entity -- its streets, and with `freeway` its freeway ring and expressways too (edges of
// class Freeway, sharing no node with a street: grade-separated, as the plan draws them).
inline engine::RoadEntity plannedNet(uint32_t seed = 11, bool freeway = false) {
    const engine::plan::CityPlan city = plannedCity(seed);
    engine::RoadEntity net;
    net.look.autoRoundabout = false;
    net.graph = city.streets;
    if (freeway) {
        const int base = static_cast<int>(net.graph.nodes.size());
        for (const engine::RoadNode& n : city.freeway.nodes) net.graph.nodes.push_back(n);
        for (engine::RoadEdge e : city.freeway.edges) { e.a += base; e.b += base; net.graph.edges.push_back(e); }
    }
    return net;
}

}  // namespace citytest

#endif
