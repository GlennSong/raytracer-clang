#include "test_framework.h"

#include "../src/engine/procgen/city/road_constraints.h"
#include "../src/engine/procgen/city/road_net.h"
#include <cmath>

using namespace engine;

namespace {

// A hub: one centre node with `spokes` arms radiating out evenly to radius `len`.
RoadGraph radialHub(int spokes, double len = 60.0, double width = 12.0) {
    RoadGraph g;
    g.nodes.push_back({Vec2(0, 0)});                 // node 0 = the hub
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        int n = static_cast<int>(g.nodes.size());
        g.nodes.push_back({Vec2(std::cos(a) * len, std::sin(a) * len)});
        g.addEdge(0, n, width, RoadClass::Local);
    }
    return g;
}

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

bool everyNodeReferenced(const RoadGraph& g) {
    for (int v = 0; v < static_cast<int>(g.nodes.size()); ++v)
        if (degreeOf(g, v) == 0) return false;
    return true;
}

}  // namespace

// A clean 4-way crossing is a flat patch, not a roundabout: nothing should change.
TEST_CASE(constraints_leave_a_four_way_alone) {
    RoadGraph g = radialHub(4);
    CHECK(!nodeNeedsRoundabout(g, 0, {}));
    RoadGraph out = applyConstraints(g);
    CHECK(out.nodes.size() == g.nodes.size());
    CHECK(out.edges.size() == g.edges.size());
    CHECK(maxDegree(out) == 4);
}

// A 3-way T with healthy angles also stays flat.
TEST_CASE(constraints_leave_a_tee_alone) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(-30, 0)}, {Vec2(30, 0)}, {Vec2(0, 30)} };
    g.addEdge(0, 1); g.addEdge(0, 2); g.addEdge(0, 3);
    CHECK(!nodeNeedsRoundabout(g, 0, {}));
    CHECK(applyConstraints(g).edges.size() == g.edges.size());
}

// Many spokes -> the hub promotes to a roundabout: the degree-N super-node is gone and
// every surviving node is degree <= 3 (the case the analytic junction pad handles).
TEST_CASE(constraints_promote_a_busy_hub) {
    RoadGraph g = radialHub(8);
    CHECK(nodeNeedsRoundabout(g, 0, {}));
    RoadGraph out = applyConstraints(g);
    CHECK(maxDegree(out) <= 3);
    CHECK(out.edges.size() > g.edges.size());     // ring arcs were added
    CHECK(everyNodeReferenced(out));              // the super-node was compacted away
    // All 8 spokes survive: 8 nodes still sit at the original spoke radius (~60 m).
    int rim = 0;
    for (const RoadNode& n : out.nodes)
        if (std::fabs(n.pos.length() - 60.0) < 1e-3) ++rim;
    CHECK(rim == 8);
}

// Two arms too acute to share a flat junction promote even below the degree cap.
TEST_CASE(constraints_promote_an_acute_fork) {
    RoadGraph g;
    g.nodes = { {Vec2(0, 0)}, {Vec2(60, 0)}, {Vec2(60, 6)}, {Vec2(-40, 30)} };
    g.addEdge(0, 1); g.addEdge(0, 2); g.addEdge(0, 3);   // arms 1 and 2 ~6 deg apart
    CHECK(degreeOf(g, 0) == 3);
    CHECK(degreeOf(g, 0) <= RoadRules{}.maxDegree);       // not promoted for being busy
    CHECK(nodeNeedsRoundabout(g, 0, {}));                 // promoted for being acute
    CHECK(maxDegree(applyConstraints(g)) <= 3);
}

// The auto radius answers the spokes: more arms (tighter packing) -> a bigger ring.
TEST_CASE(constraints_radius_grows_with_spokes) {
    auto ringExtent = [](const RoadGraph& g) {
        RoadGraph out = applyConstraints(g);
        double mx = 0;                              // largest non-spoke radius ~ the ring radius
        for (const RoadNode& n : out.nodes) {
            double r = n.pos.length();
            if (r < 55.0) mx = std::max(mx, r);
        }
        return mx;
    };
    double r8 = ringExtent(radialHub(8));
    double r16 = ringExtent(radialHub(16));
    CHECK(r16 > r8);
}

// End to end: the editable road with a busy hub now meshes without blowing up.
TEST_CASE(constraints_editable_hub_meshes) {
    RoadNet net;
    net.nodes.push_back(Vec2(0, 0));
    const int spokes = 9;
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        net.nodes.push_back(Vec2(std::cos(a) * 70.0, std::sin(a) * 70.0));
        net.edges.push_back({0, static_cast<int>(net.nodes.size()) - 1});
    }
    net.width = 12.0;
    RenderMesh m = buildRoadNetMesh(net);
    CHECK(!m.vertices.empty());
    CHECK(m.indices.size() % 3 == 0);
    // The roundabout opens a real island: no road geometry sits at the hub centre.
    int nearOrigin = 0;
    for (const Vertex& v : m.vertices)
        if (std::sqrt(v.position.x * v.position.x + v.position.z * v.position.z) < 5.0) ++nearOrigin;
    CHECK(nearOrigin == 0);
}

// Coherence: terrain conform grades to the SAME roundabout the mesh shows, so the hub
// centre (the island) is left untouched — no conform footprint runs through the origin,
// where the raw spokes would otherwise all converge.
TEST_CASE(constraints_conform_clears_the_island) {
    RoadNet net;
    net.nodes.push_back(Vec2(0, 0));
    const int spokes = 8;
    for (int i = 0; i < spokes; ++i) {
        double a = (2.0 * M_PI * i) / spokes;
        net.nodes.push_back(Vec2(std::cos(a) * 70.0, std::sin(a) * 70.0));
        net.edges.push_back({0, static_cast<int>(net.nodes.size()) - 1});
    }
    net.width = 12.0;
    net.heightAt = [](double x, double z) { return 0.05 * x + 0.02 * z; };   // gentle slope
    std::vector<TerrainFlatten> regions = roadNetConformRegions(net);
    CHECK(!regions.empty());
    double nearest = 1e30;
    for (const TerrainFlatten& f : regions)
        for (const Vec3& p : f.polygon)
            nearest = std::min(nearest, std::sqrt(p.x * p.x + p.z * p.z));
    CHECK(nearest > 4.0);     // the island is clear; roads grade around it, not through it
}
