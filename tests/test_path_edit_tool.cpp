#include "test_framework.h"

#include "../src/engine/handle_source.h"
#include "../src/engine/path_edit_tool.h"
#include "../src/engine/procgen/city/road_net.h"

using namespace engine;

namespace {
// A ray dropping straight down through a world point — what the editor builds when the
// cursor is over a handle dot (looking down). viewDir matches: camera looking -Y, so the
// Screen plane is the ground and projection is clean to verify.
EditRay downThrough(const Vec3& p) { return {Vec3(p.x, p.y + 10, p.z), Vec3(0, -1, 0)}; }
const Vec3 LOOK_DOWN(0, -1, 0);

const EditHandle& find(const std::vector<EditHandle>& hs, HandleKind k, int index) {
    for (const EditHandle& h : hs) if (h.kind == k && h.index == index) return h;
    static EditHandle none{}; return none;
}
double dist(const Vec3& a, const Vec3& b) { return std::sqrt((a - b).lengthSquared()); }
}

TEST_CASE(path_tool_unbound_is_inert) {
    PathEditTool tool;
    CHECK(!tool.bound());
    CHECK(tool.handles().empty());
    CHECK(!tool.beginDrag(downThrough(Vec3(0, 0, 0)), 1.0));
    CHECK(!tool.dragging());
    CHECK(!tool.endDrag());
}

TEST_CASE(path_tool_grabs_handle_under_ray_misses_empty_space) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    CurveHandleSource src(c);
    PathEditTool tool;
    tool.bind(&src);

    Vec3 knot1 = find(tool.handles(), HandleKind::Knot, 1).position;
    CHECK(tool.beginDrag(downThrough(knot1), 1.0));      // cursor over the dot
    CHECK(tool.dragging());
    CHECK(tool.grabbedKind() == HandleKind::Knot);
    CHECK(tool.grabbedIndex() >= 0);
    CHECK(!tool.endDrag());                              // grabbed but never moved -> no commit
    CHECK(!tool.dragging());

    // A click in empty space grabs nothing, so a plain selection click still gets through.
    CHECK(!tool.beginDrag(downThrough(Vec3(100, 0, 100)), 1.0));
}

TEST_CASE(path_tool_drags_a_curve_knot_on_the_screen_plane) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    c.addKnot(Vec3(20, 0, 0));
    CurveHandleSource src(c);
    PathEditTool tool;
    int edits = 0;
    tool.onEdit([&] { ++edits; });
    tool.bind(&src);
    CHECK(tool.activePlane() == DragPlane::Screen);      // curve default

    Vec3 knot1 = find(tool.handles(), HandleKind::Knot, 1).position;
    CHECK(tool.beginDrag(downThrough(knot1), 1.0));
    tool.drag(downThrough(Vec3(14, 0, 0)), LOOK_DOWN);   // Screen plane == ground here
    CHECK(tool.endDrag());                               // moved -> commit
    CHECK(dist(c.knots[1].position, Vec3(14, 0, 0)) < 1e-6);
    CHECK(edits == 1);
}

TEST_CASE(path_tool_drags_a_road_node_on_the_ground) {
    RoadNet net;
    net.nodes = { Vec2(-20, 0), Vec2(0, 0), Vec2(20, 0) };
    net.edges = { {0, 1}, {1, 2} };
    RoadHandleSource src(net);
    PathEditTool tool;
    int regens = 0;
    tool.onEdit([&] { ++regens; });   // editor rebuilds the mesh here
    tool.bind(&src);
    CHECK(tool.activePlane() == DragPlane::Ground);      // road default

    Vec3 node1 = find(tool.handles(), HandleKind::Knot, 1).position;
    CHECK(tool.beginDrag(downThrough(node1), 1.0));
    // Drag node 1 to world (6, *, 9); the ground plane fixes Y, only X/Z move.
    tool.drag(downThrough(Vec3(6, node1.y, 9)), LOOK_DOWN);
    CHECK(tool.endDrag());
    CHECK_APPROX(net.nodes[1].x, 6.0, 1e-6);
    CHECK_APPROX(net.nodes[1].y, 9.0, 1e-6);             // Vec2.y is world Z
    CHECK(regens == 1);
}

TEST_CASE(path_tool_plane_override_and_clear) {
    RoadNet net;
    net.nodes = { Vec2(0, 0), Vec2(10, 0) };
    net.edges = { {0, 1} };
    RoadHandleSource src(net);
    PathEditTool tool;
    tool.bind(&src);
    CHECK(tool.activePlane() == DragPlane::Ground);      // source default
    tool.setPlaneOverride(DragPlane::YZ);
    CHECK(tool.activePlane() == DragPlane::YZ);           // user hotkey wins
    tool.clearPlaneOverride();
    CHECK(tool.activePlane() == DragPlane::Ground);      // back to the default
}
