#include "test_framework.h"

#include "../src/engine/procgen/city/road_net.h"
#include <nlohmann/json.hpp>
#include <cmath>

using namespace engine;
using json = nlohmann::json;

namespace {

// A small T-junction net: a straight run 0-1-2 with a branch 1-3.
RoadNet sampleNet() {
    RoadNet n;
    n.nodes = { Vec2(-30, 0), Vec2(0, 0), Vec2(30, 0), Vec2(0, 30) };
    n.edges = { {0, 1}, {1, 2}, {1, 3} };
    n.width = 10.0;
    n.sidewalk = 2.5;
    return n;
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

TEST_CASE(road_net_builds_a_surface) {
    RenderMesh m = buildRoadNetMesh(sampleNet());
    CHECK(!m.vertices.empty());
    CHECK(!m.indices.empty());
    CHECK(m.indices.size() % 3 == 0);
}

TEST_CASE(road_net_widen_grows_the_carriageway) {
    RoadNet n = sampleNet();
    double narrow = meshAreaXZ(buildRoadNetMesh(n));
    roadNetSetWidth(n, 20.0);                       // the inspector "Width" control
    double wide = meshAreaXZ(buildRoadNetMesh(n));
    CHECK(n.width == 20.0);
    CHECK(wide > narrow * 1.4);                     // twice as wide ~> much more asphalt
}

TEST_CASE(road_net_width_has_a_floor) {
    RoadNet n = sampleNet();
    roadNetSetWidth(n, -5.0);
    CHECK(n.width >= 0.5);                          // never collapses to zero/negative
}

TEST_CASE(road_net_move_node_bends_the_road) {
    RoadNet n = sampleNet();
    double minX0, maxX0, minZ0, maxZ0;
    bboxXZ(buildRoadNetMesh(n), minX0, maxX0, minZ0, maxZ0);

    CHECK(roadNetMoveNode(n, 2, Vec2(48, 0)));      // drag the east end further east
    CHECK_APPROX(n.nodes[2].x, 48.0, 1e-9);
    double minX1, maxX1, minZ1, maxZ1;
    bboxXZ(buildRoadNetMesh(n), minX1, maxX1, minZ1, maxZ1);
    CHECK(maxX1 > maxX0 + 10.0);                     // the surface followed the moved node
    (void)minX0; (void)minZ0; (void)minX1; (void)minZ1; (void)maxZ1;

    CHECK(!roadNetMoveNode(n, 99, Vec2(0, 0)));     // out of range is rejected
}

TEST_CASE(road_net_json_round_trips) {
    RoadNet n = sampleNet();
    n.width = 14.0; n.markings = false; n.cornerRadius = 2.0;
    n.color = Vec3(0.2, 0.2, 0.22);

    RoadNet r = roadNetFromJson(roadNetToJson(n));
    CHECK(r.nodes.size() == n.nodes.size());
    CHECK(r.edges.size() == n.edges.size());
    CHECK_APPROX(r.width, 14.0, 1e-9);
    CHECK_APPROX(r.cornerRadius, 2.0, 1e-9);
    CHECK(r.markings == false);
    CHECK_APPROX(r.nodes[2].x, 30.0, 1e-9);
    CHECK(r.edges[2][0] == 1 && r.edges[2][1] == 3);
    CHECK_APPROX(r.color.y, 0.2, 1e-9);
}

TEST_CASE(road_net_curved_auto_smooths_a_chain) {
    // `curved` with no explicit tangents still builds (auto Catmull-Rom on the
    // degree-2 nodes, straight into the junction).
    RoadNet n = sampleNet();
    n.curved = true;
    CHECK(!buildRoadNetMesh(n).vertices.empty());
}

TEST_CASE(road_net_tangent_bends_off_the_chord) {
    // Three collinear nodes: straight stays tight in Z; a strong perpendicular
    // tangent on the middle knot bows the spline well off the chord.
    RoadNet n;
    n.nodes = { Vec2(-100, 0), Vec2(0, 0), Vec2(100, 0) };
    n.edges = { {0, 1}, {1, 2} };
    n.width = 12.0;
    double minX, maxX, minZ0, maxZ0;
    bboxXZ(buildRoadNetMesh(n), minX, maxX, minZ0, maxZ0);   // straight (curved == false)

    CHECK(roadNetSetTangent(n, 1, Vec2(0, 80)));            // drag the middle tangent off-axis
    CHECK(n.curved);                                        // a set tangent shows the spline
    double minZ1, maxZ1;
    bboxXZ(buildRoadNetMesh(n), minX, maxX, minZ1, maxZ1);
    // The straight road is just the carriageway band; the spline bows off the chord
    // (an S about the middle knot), so its Z extent is far wider.
    CHECK((maxZ1 - minZ1) > (maxZ0 - minZ0) + 15.0);
    CHECK(!roadNetSetTangent(n, 9, Vec2(1, 1)));           // out of range rejected
}

TEST_CASE(road_net_curved_and_tangents_round_trip) {
    RoadNet n = sampleNet();
    roadNetSetTangent(n, 1, Vec2(5.0, 9.0));                // also flips curved on
    RoadNet r = roadNetFromJson(roadNetToJson(n));
    CHECK(r.curved);
    CHECK(r.tangents.size() == n.tangents.size());
    CHECK_APPROX(r.tangents[1].x, 5.0, 1e-9);
    CHECK_APPROX(r.tangents[1].y, 9.0, 1e-9);
}

TEST_CASE(road_net_per_edge_width_overrides_default) {
    RoadNet n = sampleNet();                          // edges {0,1}{1,2}{1,3}, width 10
    CHECK_APPROX(roadNetEdgeWidth(n, 0), 10.0, 1e-9); // default before any override
    double base = meshAreaXZ(buildRoadNetMesh(n));

    CHECK(roadNetSetEdgeWidth(n, 1, 24.0));           // widen just the middle edge
    CHECK_APPROX(roadNetEdgeWidth(n, 1), 24.0, 1e-9);
    CHECK_APPROX(roadNetEdgeWidth(n, 0), 10.0, 1e-9); // others untouched
    CHECK(meshAreaXZ(buildRoadNetMesh(n)) > base + 50.0);   // that edge grew

    roadNetSetEdgeWidth(n, 1, 0.0);                   // <= 0 reverts to the default
    CHECK_APPROX(roadNetEdgeWidth(n, 1), 10.0, 1e-9);
    CHECK(!roadNetSetEdgeWidth(n, 9, 5.0));           // bad edge index
}

TEST_CASE(road_net_per_edge_width_survives_topology_and_json) {
    RoadNet n = sampleNet();
    roadNetSetEdgeWidth(n, 1, 20.0);                  // edge {1,2}
    // Split edge 0 ({0,1}); the override on edge 1 must still apply to the same road.
    roadNetSplitEdge(n, 0, Vec2(-15, 0));
    CHECK_APPROX(roadNetEdgeWidth(n, 1), 20.0, 1e-9);
    // Delete a leaf (node 3, edge {1,3}); edge {1,2}'s width tracks the reindex.
    CHECK(roadNetDeleteNode(n, 3));
    int wide = -1;
    for (int i = 0; i < static_cast<int>(n.edges.size()); ++i)
        if (roadNetEdgeWidth(n, i) > 15.0) wide = i;
    CHECK(wide >= 0);                                 // the wide edge is still there

    RoadNet r = roadNetFromJson(roadNetToJson(n));    // [a,b,width] round-trip
    int rwide = -1;
    for (int i = 0; i < static_cast<int>(r.edges.size()); ++i)
        if (roadNetEdgeWidth(r, i) > 15.0) rwide = i;
    CHECK(rwide >= 0);
    CHECK_APPROX(roadNetEdgeWidth(r, rwide), 20.0, 1e-9);
}

TEST_CASE(road_net_split_inserts_a_point) {
    RoadNet n = sampleNet();                          // 4 nodes, 3 edges
    int edge = roadNetNearestEdge(n, Vec2(-15, 0), 8.0);   // over the 0-1 run
    CHECK(edge == 0);
    int ni = roadNetSplitEdge(n, edge, Vec2(-15, 0));
    CHECK(ni == 4);                                   // appended
    CHECK(n.nodes.size() == 5);
    CHECK(n.edges.size() == 4);                       // one edge became two
    CHECK(!buildRoadNetMesh(n).vertices.empty());
    CHECK(roadNetSplitEdge(n, 99, Vec2(0, 0)) == -1); // bad edge
    CHECK(roadNetNearestEdge(n, Vec2(0, 500), 8.0) == -1);  // nothing near
}

TEST_CASE(road_net_extend_grows_from_an_end) {
    RoadNet n = sampleNet();
    int ni = roadNetExtend(n, 4 /*no such node yet*/, Vec2(0, 0));
    CHECK(ni == -1);                                  // from-node out of range
    ni = roadNetExtend(n, 2, Vec2(40, 60));           // grow off the junction
    CHECK(ni == 4);
    CHECK(n.edges.size() == 4);
    CHECK(roadNetAddEdge(n, 0, 1) == false);          // already joined
    CHECK(roadNetAddEdge(n, 0, 3) == true);           // new connection
}

TEST_CASE(road_net_delete_removes_and_reindexes) {
    RoadNet n = sampleNet();                          // nodes 0..3, edges {0,1}{1,2}{1,3}
    roadNetSetTangent(n, 3, Vec2(2, 7));              // give the last node a tangent
    CHECK(roadNetDeleteNode(n, 1));                   // the junction
    CHECK(n.nodes.size() == 3);
    CHECK(n.edges.empty());                           // all three edges touched node 1
    // Surviving nodes kept their positions (old 0,2,3 -> new 0,1,2).
    CHECK_APPROX(n.nodes[0].x, -30.0, 1e-9);
    CHECK_APPROX(n.nodes[2].y, 30.0, 1e-9);
    CHECK(!roadNetDeleteNode(n, 9));                  // out of range

    RoadNet m = sampleNet();
    CHECK(roadNetDeleteNode(m, 3));                   // a leaf: only edge {1,3} drops
    CHECK(m.nodes.size() == 3);
    CHECK(m.edges.size() == 2);
    CHECK(m.edges[1][0] == 1 && m.edges[1][1] == 2);  // {1,2} survives, indices intact
}

TEST_CASE(road_net_reads_authoring_json) {
    // The hand-authored `road` block a level/editor writes: edges as [a,b] pairs.
    json j = json::parse(R"({
        "nodes": [ {"x": -10, "z": 0}, {"x": 10, "z": 0} ],
        "edges": [ [0, 1] ],
        "width": 12, "sidewalk": 3
    })");
    RoadNet n = roadNetFromJson(j);
    CHECK(n.nodes.size() == 2);
    CHECK(n.edges.size() == 1);
    CHECK_APPROX(n.width, 12.0, 1e-9);
    CHECK_APPROX(n.sidewalk, 3.0, 1e-9);
    CHECK(!buildRoadNetMesh(n).vertices.empty());
}
