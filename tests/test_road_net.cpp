#include "test_framework.h"
#include <cstdio>
#include <algorithm>

#include "../src/engine/procgen/city/road_net.h"
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

TEST_CASE(road_net_builds_a_surface) {
    RenderMesh m = buildRoadNetMesh(sampleNet(), nullptr);
    CHECK(!m.vertices.empty());
    CHECK(!m.indices.empty());
    CHECK(m.indices.size() % 3 == 0);
}

TEST_CASE(crosswalks_bake_a_setback_band_into_the_road_uv) {
    // Crosswalks are painted into the road TEXTURE (ADR-0062): the mesher bakes the
    // carriageway UV `v` (mv) as metres past the junction mouth, so the RoadMarkings
    // shader stripes a set-back band. Verify the baking is gated and lands near a
    // junction. (carriageway vertices carry u = mu >= ~1; sidewalks stay at u = 0.)
    RoadEntity n = sampleNet();     // node 1 is a degree-3 junction
    n.look.markings = true;

    // The shader paints where mv is in the set-back window (~0.5..3.6 m past the
    // mouth). Filter to the carriageway RIGHT rail (u == 3) — unambiguously road
    // (the raised sidewalk shares this surface but its 0..1 UV never reaches u = 3,
    // which is exactly why the shader also gates crosswalk paint on mu > 1.05).
    auto hasBand = [](const RenderMesh& m) {
        for (const Vertex& vtx : m.vertices)
            if (vtx.u >= 2.5 && vtx.v > 0.5 && vtx.v < 3.6) return true;
        return false;
    };
    // OFF: carriageway UV is the large sentinel, so nothing lands in the window.
    n.look.crosswalks = false;
    CHECK(!hasBand(buildRoadNetMesh(n, nullptr)));

    // ON: some carriageway vertex lands in the painted window, proving the band is
    // baked where the shader will stripe it.
    n.look.crosswalks = true;
    CHECK(hasBand(buildRoadNetMesh(n, nullptr)));
}

TEST_CASE(road_net_widen_grows_the_carriageway) {
    RoadEntity n = sampleNet();
    double narrow = meshAreaXZ(buildRoadNetMesh(n, nullptr));
    setDefaultWidth(n, 20.0);                       // the inspector "Width" control
    double wide = meshAreaXZ(buildRoadNetMesh(n, nullptr));
    CHECK(n.look.defaultWidth == 20.0);
    CHECK(wide > narrow * 1.4);                     // twice as wide ~> much more asphalt
}

TEST_CASE(road_net_width_has_a_floor) {
    // The old roadNetSetWidth clamp is gone; widths are resolved per edge and the
    // floor is now the resolve rule — a non-positive width edit reverts to the
    // look's default, so no edge ever collapses to zero/negative.
    RoadEntity n = sampleNet();
    CHECK(roadNetSetEdgeWidth(n, 0, -5.0));
    CHECK(n.graph.edges[0].width >= 0.5);           // never collapses to zero/negative
    for (const RoadEdge& e : n.graph.edges) CHECK(e.width > 0.0);
}

TEST_CASE(road_net_move_node_bends_the_road) {
    RoadEntity n = sampleNet();
    double minX0, maxX0, minZ0, maxZ0;
    bboxXZ(buildRoadNetMesh(n, nullptr), minX0, maxX0, minZ0, maxZ0);

    CHECK(roadNetMoveNode(n, 2, Vec2(48, 0)));      // drag the east end further east
    CHECK_APPROX(n.graph.nodes[2].pos.x, 48.0, 1e-9);
    double minX1, maxX1, minZ1, maxZ1;
    bboxXZ(buildRoadNetMesh(n, nullptr), minX1, maxX1, minZ1, maxZ1);
    CHECK(maxX1 > maxX0 + 10.0);                     // the surface followed the moved node
    (void)minX0; (void)minZ0; (void)minX1; (void)minZ1; (void)maxZ1;

    CHECK(!roadNetMoveNode(n, 99, Vec2(0, 0)));     // out of range is rejected
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

TEST_CASE(road_net_auto_smooths_a_chain) {
    // Every road is a spline: with no explicit tangents it still builds (auto Catmull-Rom on the
    // degree-2 nodes, straight into the junction).
    RoadEntity n = sampleNet();
    CHECK(!buildRoadNetMesh(n, nullptr).vertices.empty());
}

TEST_CASE(road_net_tangent_bends_off_the_chord) {
    // Three collinear nodes: straight stays tight in Z; a strong perpendicular
    // tangent on the middle knot bows the spline well off the chord.
    RoadEntity n;
    n.look.defaultWidth = 12.0;
    n.graph.nodes = { RoadNode{Vec2(-100, 0)}, RoadNode{Vec2(0, 0)}, RoadNode{Vec2(100, 0)} };
    n.graph.addEdge(0, 1, n.look.defaultWidth);
    n.graph.addEdge(1, 2, n.look.defaultWidth);
    double minX, maxX, minZ0, maxZ0;
    bboxXZ(buildRoadNetMesh(n, nullptr), minX, maxX, minZ0, maxZ0);   // collinear nodes -> a straight band

    CHECK(roadNetSetTangent(n, 1, Vec2(0, 80)));            // drag the middle tangent off-axis
    double minZ1, maxZ1;
    bboxXZ(buildRoadNetMesh(n, nullptr), minX, maxX, minZ1, maxZ1);
    // The straight road is just the carriageway band; the spline bows off the chord
    // (an S about the middle knot), so its Z extent is far wider.
    CHECK((maxZ1 - minZ1) > (maxZ0 - minZ0) + 15.0);
    CHECK(!roadNetSetTangent(n, 9, Vec2(1, 1)));           // out of range rejected
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

TEST_CASE(road_net_per_edge_width_overrides_default) {
    RoadEntity n = sampleNet();                          // edges {0,1}{1,2}{1,3}, width 10
    CHECK_APPROX((double)n.graph.edges[0].width, 10.0, 1e-9); // default before any override
    double base = meshAreaXZ(buildRoadNetMesh(n, nullptr));

    CHECK(roadNetSetEdgeWidth(n, 1, 24.0));           // widen just the middle edge
    CHECK_APPROX((double)n.graph.edges[1].width, 24.0, 1e-9);
    CHECK_APPROX((double)n.graph.edges[0].width, 10.0, 1e-9); // others untouched
    CHECK(meshAreaXZ(buildRoadNetMesh(n, nullptr)) > base + 50.0);   // that edge grew

    roadNetSetEdgeWidth(n, 1, 0.0);                   // <= 0 reverts to the default
    CHECK_APPROX((double)n.graph.edges[1].width, 10.0, 1e-9);
    CHECK(!roadNetSetEdgeWidth(n, 9, 5.0));           // bad edge index
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

TEST_CASE(road_net_split_inserts_a_point) {
    RoadEntity n = sampleNet();                          // 4 nodes, 3 edges
    int edge = roadNetNearestEdge(n, Vec2(-15, 0), 8.0);   // over the 0-1 run
    CHECK(edge == 0);
    int ni = roadNetSplitEdge(n, edge, Vec2(-15, 0));
    CHECK(ni == 4);                                   // appended
    CHECK(n.graph.nodes.size() == 5);
    CHECK(n.graph.edges.size() == 4);                 // one edge became two
    CHECK(!buildRoadNetMesh(n, nullptr).vertices.empty());
    CHECK(roadNetSplitEdge(n, 99, Vec2(0, 0)) == -1); // bad edge
    CHECK(roadNetNearestEdge(n, Vec2(0, 500), 8.0) == -1);  // nothing near
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

TEST_CASE(road_net_reads_authoring_json) {
    // The hand-authored `road` block a level/editor writes: edges as [a,b] pairs.
    json j = json::parse(R"({
        "nodes": [ {"x": -10, "z": 0}, {"x": 10, "z": 0} ],
        "edges": [ [0, 1] ],
        "width": 12, "sidewalk": 3
    })");
    RoadEntity n = roadNetFromJson(j);
    CHECK(n.graph.nodes.size() == 2);
    CHECK(n.graph.edges.size() == 1);
    CHECK_APPROX(n.look.defaultWidth, 12.0, 1e-9);
    CHECK_APPROX(n.look.sidewalk, 3.0, 1e-9);
    CHECK(!buildRoadNetMesh(n, nullptr).vertices.empty());
}

TEST_CASE(road_net_conform_regions_carve_a_sloped_road) {
    RoadEntity net;
    net.look.defaultWidth = 8.0;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(40, 0)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth);
    const RoadGroundFn ground = [](double x, double) { return 0.4 * x; };   // 40% slope along the road
    std::vector<TerrainFlatten> regs = roadNetConformRegions(net, ground, 1.5, 8.0, 0.10);
    CHECK(!regs.empty());
    const double base = 999.0;                                 // natural ground far above
    CHECK_APPROX(applyFlatten(regs, 20, 60, base), base, 1e-6);   // far off road -> natural
    // On the road the terrain is graded to a <=10% profile, well below the natural 8 m.
    CHECK(applyFlatten(regs, 20, 0, base) < 8.0);
}

TEST_CASE(road_net_conform_regions_empty_without_terrain) {
    RoadEntity net;
    net.graph.nodes = { RoadNode{Vec2(0, 0)}, RoadNode{Vec2(40, 0)} };
    net.graph.addEdge(0, 1, net.look.defaultWidth);
    CHECK(roadNetConformRegions(net, nullptr).empty());        // no ground -> flat -> none
}
