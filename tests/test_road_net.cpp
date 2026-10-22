#include "test_framework.h"
#include <cstdio>
#include <algorithm>

#include "../src/engine/procgen/city/roads/road_entity.h"
#include <nlohmann/json.hpp>
#include <cmath>
#include <limits>

using namespace engine;
using json = nlohmann::json;

namespace {

// A small T-junction net: a straight run 0-1-2 with a branch 1-3.
RoadEntity sampleNet() {
    RoadEntity n;
    n.look.defaultWidth = 10.0;
    n.look.sidewalk = 2.5;
    n.graph.nodes = { RoadNode{Vec2(-30, 0)}, RoadNode{Vec2(0, 0)},
                      RoadNode{Vec2(30, 0)}, RoadNode{Vec2(0, 30)} };
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    n.graph.addEdge(1, 2, n.look.defaultWidth);
    n.graph.addEdge(1, 3, n.look.defaultWidth);
    return n;
}

// The inspector "Width" edit: widths are RESOLVED on the graph now, so changing
// the default re-stamps every edge that sat at the old default (the
// properties.cpp rule) — an edge widened by hand keeps its own width.
void setDefaultWidth(RoadEntity& n, double w) {
    const double before = n.look.defaultWidth;
    n.look.defaultWidth = w;
    for (RoadEdge& e : n.graph.edges)
        if (static_cast<double>(e.width) == before) e.width = static_cast<Real>(w);
}

// Flat-projected carriageway area (sum of triangle areas on XZ).
double meshAreaXZ(const RenderMesh& m) {
    double a = 0;
    for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
        const Vec3& A = m.vertices[m.indices[i]].position;
        const Vec3& B = m.vertices[m.indices[i + 1]].position;
        const Vec3& C = m.vertices[m.indices[i + 2]].position;
        a += std::fabs((B.x - A.x) * (C.z - A.z) - (C.x - A.x) * (B.z - A.z)) * 0.5;
    }
    return a;
}

void bboxXZ(const RenderMesh& m, double& minX, double& maxX, double& minZ, double& maxZ) {
    minX = minZ = 1e30; maxX = maxZ = -1e30;
    for (const Vertex& v : m.vertices) {
        minX = std::min(minX, (double)v.position.x); maxX = std::max(maxX, (double)v.position.x);
        minZ = std::min(minZ, (double)v.position.z); maxZ = std::max(maxZ, (double)v.position.z);
    }
}

}  // namespace


TEST_CASE(road_net_width_has_a_floor) {
    // The old roadNetSetWidth clamp is gone; widths are resolved per edge and the
    // floor is now the resolve rule — a non-positive width edit reverts to the
    // look's default, so no edge ever collapses to zero/negative.
    RoadEntity n = sampleNet();
    CHECK(roadNetSetEdgeWidth(n, 0, -5.0));
    CHECK(n.graph.edges[0].width >= 0.5);           // never collapses to zero/negative
    for (const RoadEdge& e : n.graph.edges) CHECK(e.width > 0.0);
}


TEST_CASE(road_net_json_round_trips) {
    RoadEntity n = sampleNet();
    setDefaultWidth(n, 14.0);
    n.look.markings = false; n.look.cornerRadius = 2.0;
    n.look.color = Vec3(0.2, 0.2, 0.22);

    RoadEntity r = roadNetFromJson(roadNetToJson(n));
    CHECK(r.graph.nodes.size() == n.graph.nodes.size());
    CHECK(r.graph.edges.size() == n.graph.edges.size());
    CHECK_APPROX(r.look.defaultWidth, 14.0, 1e-9);
    CHECK_APPROX(r.look.cornerRadius, 2.0, 1e-9);
    CHECK(r.look.markings == false);
    CHECK_APPROX(r.graph.nodes[2].pos.x, 30.0, 1e-9);
    CHECK(r.graph.edges[2].a == 1 && r.graph.edges[2].b == 3);
    CHECK_APPROX(r.look.color.y, 0.2, 1e-9);
}


TEST_CASE(road_net_tangents_round_trip) {
    RoadEntity n = sampleNet();
    roadNetSetTangent(n, 1, Vec2(5.0, 9.0));
    RoadEntity r = roadNetFromJson(roadNetToJson(n));
    CHECK(r.graph.nodes.size() == n.graph.nodes.size());
    CHECK_APPROX(r.graph.nodes[1].tangent.x, 5.0, 1e-9);
    CHECK_APPROX(r.graph.nodes[1].tangent.y, 9.0, 1e-9);
}

TEST_CASE(road_net_node_elev_round_trips) {
    RoadEntity n = sampleNet();                          // 4 nodes
    n.graph.nodes[0].elev = 9.0;                      // node 0 elevated, others at-grade
    n.graph.nodes[0].elevAbsolute = true;
    n.graph.nodes[2].elev = 4.5;
    n.graph.nodes[2].elevAbsolute = true;
    RoadEntity r = roadNetFromJson(roadNetToJson(n));
    CHECK(r.graph.nodes.size() == n.graph.nodes.size());
    CHECK(r.graph.nodes[0].elevAbsolute);
    CHECK_APPROX((double)r.graph.nodes[0].elev, 9.0, 1e-9);
    CHECK(r.graph.nodes[2].elevAbsolute);
    CHECK_APPROX((double)r.graph.nodes[2].elev, 4.5, 1e-9);
    CHECK(!r.graph.nodes[1].elevAbsolute);            // null survives as at-grade
    CHECK(!r.graph.nodes[3].elevAbsolute);
}


TEST_CASE(road_net_per_edge_width_survives_topology_and_json) {
    RoadEntity n = sampleNet();
    roadNetSetEdgeWidth(n, 1, 20.0);                  // edge {1,2}
    // Split edge 0 ({0,1}); the override on edge 1 must still apply to the same road.
    roadNetSplitEdge(n, 0, Vec2(-15, 0));
    CHECK_APPROX((double)n.graph.edges[1].width, 20.0, 1e-9);
    // Delete a leaf (node 3, edge {1,3}); edge {1,2}'s width tracks the reindex.
    CHECK(roadNetDeleteNode(n, 3));
    int wide = -1;
    for (int i = 0; i < static_cast<int>(n.graph.edges.size()); ++i)
        if (n.graph.edges[i].width > 15.0) wide = i;
    CHECK(wide >= 0);                                 // the wide edge is still there

    RoadEntity r = roadNetFromJson(roadNetToJson(n));    // [a,b,width] round-trip
    int rwide = -1;
    for (int i = 0; i < static_cast<int>(r.graph.edges.size()); ++i)
        if (r.graph.edges[i].width > 15.0) rwide = i;
    CHECK(rwide >= 0);
    CHECK_APPROX((double)r.graph.edges[rwide].width, 20.0, 1e-9);
}


TEST_CASE(road_net_extend_grows_from_an_end) {
    RoadEntity n = sampleNet();
    int ni = roadNetExtend(n, 4 /*no such node yet*/, Vec2(0, 0));
    CHECK(ni == -1);                                  // from-node out of range
    ni = roadNetExtend(n, 2, Vec2(40, 60));           // grow off the junction
    CHECK(ni == 4);
    CHECK(n.graph.edges.size() == 4);
    CHECK(roadNetAddEdge(n, 0, 1) == false);          // already joined
    CHECK(roadNetAddEdge(n, 0, 3) == true);           // new connection
}

TEST_CASE(road_net_delete_removes_and_reindexes) {
    RoadEntity n = sampleNet();                          // nodes 0..3, edges {0,1}{1,2}{1,3}
    roadNetSetTangent(n, 3, Vec2(2, 7));              // give the last node a tangent
    CHECK(roadNetDeleteNode(n, 1));                   // the junction
    CHECK(n.graph.nodes.size() == 3);
    CHECK(n.graph.edges.empty());                     // all three edges touched node 1
    // Surviving nodes kept their positions (old 0,2,3 -> new 0,1,2).
    CHECK_APPROX(n.graph.nodes[0].pos.x, -30.0, 1e-9);
    CHECK_APPROX(n.graph.nodes[2].pos.y, 30.0, 1e-9);
    CHECK(!roadNetDeleteNode(n, 9));                  // out of range

    RoadEntity m = sampleNet();
    CHECK(roadNetDeleteNode(m, 3));                   // a leaf: only edge {1,3} drops
    CHECK(m.graph.nodes.size() == 3);
    CHECK(m.graph.edges.size() == 2);
    CHECK(m.graph.edges[1].a == 1 && m.graph.edges[1].b == 2);  // {1,2} survives, indices intact
}

