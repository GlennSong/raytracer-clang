#include "test_framework.h"

#include "../src/engine/procgen/city/road_network.h"

using namespace engine;

namespace {

int degreeOf(const RoadGraph& g, int v) {
    int d = 0;
    for (const RoadEdge& e : g.edges) if (e.a == v || e.b == v) ++d;
    return d;
}

int maxDegree(const RoadGraph& g) {
    int mx = 0;
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v) mx = std::max(mx, degreeOf(g, v));
    return mx;
}

// A plain "+" crossing: a horizontal edge and a vertical edge that cross at the origin,
// each on `layerH` / `layerV`.
RoadGraph plusCrossing(int layerH, int layerV) {
    RoadGraph g;
    g.nodes = { {Vec2(-20, 0)}, {Vec2(20, 0)}, {Vec2(0, -20)}, {Vec2(0, 20)} };
    g.edges.push_back({0, 1, 8, RoadClass::Local, layerH});
    g.edges.push_back({2, 3, 8, RoadClass::Local, layerV});
    return g;
}

}  // namespace

// Same layer: the crossing becomes a real intersection — one shared node, degree 4.
TEST_CASE(layered_same_layer_makes_an_intersection) {
    RoadGraph g = plusCrossing(0, 0);
    CHECK(gradeSeparationCount(g) == 0);
    RoadGraph out = planarizeLayered(g);
    CHECK(maxDegree(out) == 4);                 // the four arms meet at the split node
    CHECK(out.edges.size() == 4);               // each edge cut in two
}

// Different layers: the crossing is an overpass — no shared node, both edges intact.
TEST_CASE(layered_different_layer_makes_an_overpass) {
    RoadGraph g = plusCrossing(0, 1);
    CHECK(gradeSeparationCount(g) == 1);
    RoadGraph out = planarizeLayered(g);
    CHECK(out.edges.size() == 2);               // neither edge was split
    CHECK(maxDegree(out) <= 2);                 // nothing meets: they pass over/under
    // The two layers are preserved on the output.
    int onGround = 0, above = 0;
    for (const RoadEdge& e : out.edges) (e.layer == 0 ? onGround : above)++;
    CHECK(onGround == 1);
    CHECK(above == 1);
}

// Default behaviour (everything on layer 0) matches the old planarize: a "+" intersects.
TEST_CASE(layered_defaults_to_at_grade) {
    RoadGraph g;
    g.nodes = { {Vec2(-20, 0)}, {Vec2(20, 0)}, {Vec2(0, -20)}, {Vec2(0, 20)} };
    g.addEdge(0, 1);            // layer defaults to 0
    g.addEdge(2, 3);
    CHECK(gradeSeparationCount(g) == 0);
    CHECK(planarizeLayered(g).edges.size() == planarize(g).edges.size());
}

// A freeway (layer 1) flying over a surface grid (layer 0): the grid still planarizes into
// its own intersections, while the freeway crosses every street as a separation.
TEST_CASE(layered_freeway_over_a_grid) {
    RoadGraph g;
    // Two parallel ground streets (vertical), crossed by one ground street (horizontal)...
    g.nodes = { {Vec2(-10, -30)}, {Vec2(-10, 30)},   // street A (x=-10)
                {Vec2(10, -30)},  {Vec2(10, 30)},    // street B (x=10)
                {Vec2(-30, 0)},   {Vec2(30, 0)},     // cross street (y=0)
                {Vec2(-30, 15)},  {Vec2(30, 15)} };  // the freeway (y=15), layer 1
    g.edges.push_back({0, 1, 8, RoadClass::Local, 0});
    g.edges.push_back({2, 3, 8, RoadClass::Local, 0});
    g.edges.push_back({4, 5, 8, RoadClass::Local, 0});
    g.edges.push_back({6, 7, 16, RoadClass::Arterial, 1});
    // The freeway crosses both vertical streets above grade -> 2 separations.
    CHECK(gradeSeparationCount(g) == 2);
    RoadGraph out = planarizeLayered(g);
    // The ground cross street splits both vertical streets (2 intersections, degree 4 each);
    // the freeway is untouched by them.
    int deg4 = 0;
    for (int v = 0; v < static_cast<int>(out.nodes.size()); ++v) if (degreeOf(out, v) == 4) ++deg4;
    CHECK(deg4 == 2);
}
