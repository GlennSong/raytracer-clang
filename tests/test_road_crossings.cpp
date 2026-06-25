#include "test_framework.h"

#include "../src/engine/procgen/city/road_crossings.h"
#include <cmath>

using namespace engine;

namespace {
// Does spine `s` contain a vertex within tol of p?
bool hasVertex(const GraphSpine& s, const Vec2& p, double tol = 1e-4) {
    for (const Vec2& v : s.points) if ((v - p).length() < tol) return true;
    return false;
}
}

// Two roads crossing at right angles resolve to ONE at-grade node, and the exact crossing point is
// spliced into BOTH centerlines so they share a vertex (the weld then fuses them with no notch).
TEST_CASE(crossing_splices_a_shared_node) {
    GraphSpine h; h.points = { Vec2(-10, 0), Vec2(10, 0) };
    GraphSpine v; v.points = { Vec2(0, -10), Vec2(0, 10) };
    ResolvedGraph g = resolveCrossings({ h, v });
    CHECK(g.nodes.size() == 1);
    CHECK((g.nodes[0].pos - Vec2(0, 0)).length() < 1e-6);
    CHECK(g.nodes[0].spines.size() == 2);
    CHECK(!g.nodes[0].gradeSeparated);
    CHECK(hasVertex(g.spines[0], Vec2(0, 0)));     // crossing inserted into the horizontal road
    CHECK(hasVertex(g.spines[1], Vec2(0, 0)));     // ... and the vertical road
}

// A T-junction (one road's endpoint landing on another) is a crossing too: the stem ends on the
// through road, and the touch point joins the graph.
TEST_CASE(crossing_handles_a_tee) {
    GraphSpine through; through.points = { Vec2(-10, 0), Vec2(10, 0) };
    GraphSpine stem;    stem.points    = { Vec2(0, 0), Vec2(0, 10) };   // ends on the through road
    ResolvedGraph g = resolveCrossings({ through, stem });
    CHECK(g.nodes.size() == 1);
    CHECK((g.nodes[0].pos - Vec2(0, 0)).length() < 1e-6);
    CHECK(g.nodes[0].spines.size() == 2);
    CHECK(hasVertex(g.spines[0], Vec2(0, 0)));     // the through road gains the tee vertex
}

// Roads that cross on DIFFERENT layers are a fly-over: the node is grade-separated (the upper road
// bridges; a ramp pass connects them later) rather than an at-grade weld.
TEST_CASE(crossing_flags_grade_separation) {
    GraphSpine ground; ground.points = { Vec2(-10, 0), Vec2(10, 0) }; ground.layer = 0;
    GraphSpine fly;    fly.points    = { Vec2(0, -10), Vec2(0, 10) };  fly.layer = 1;
    ResolvedGraph g = resolveCrossings({ ground, fly });
    CHECK(g.nodes.size() == 1);
    CHECK(g.nodes[0].gradeSeparated);
}

// Disjoint roads that never meet produce no nodes and pass through unchanged.
TEST_CASE(crossing_leaves_disjoint_roads_alone) {
    GraphSpine a; a.points = { Vec2(-10, 0), Vec2(10, 0) };
    GraphSpine b; b.points = { Vec2(-10, 50), Vec2(10, 50) };
    ResolvedGraph g = resolveCrossings({ a, b });
    CHECK(g.nodes.empty());
    CHECK(g.spines[0].points.size() == 2);
    CHECK(g.spines[1].points.size() == 2);
}

// A grid of two H + two V roads resolves to four at-grade nodes (the four intersections), each
// shared by the two roads that meet there — the generators' loose lines become one connected net.
TEST_CASE(crossing_resolves_a_grid_into_four_nodes) {
    std::vector<GraphSpine> grid = {
        { { Vec2(-10, -5), Vec2(10, -5) }, 2.0, 0, 0 },
        { { Vec2(-10,  5), Vec2(10,  5) }, 2.0, 0, 0 },
        { { Vec2(-5, -10), Vec2(-5, 10) }, 2.0, 0, 0 },
        { { Vec2( 5, -10), Vec2( 5, 10) }, 2.0, 0, 0 },
    };
    ResolvedGraph g = resolveCrossings(grid);
    CHECK(g.nodes.size() == 4);
    for (const CrossNode& n : g.nodes) {
        CHECK(n.spines.size() == 2);
        CHECK(!n.gradeSeparated);
    }
    // Each through road now carries its two crossing vertices (4 points: 2 ends + 2 crossings).
    for (const GraphSpine& s : g.spines) CHECK(s.points.size() == 4);
}
