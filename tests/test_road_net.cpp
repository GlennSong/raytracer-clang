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
