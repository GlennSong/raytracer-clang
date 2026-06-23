#include "test_framework.h"

#include "../src/engine/handle_source.h"
#include "../src/engine/procgen/city/road_net.h"
#include <cmath>

using namespace engine;

namespace {
double dist(const Vec3& a, const Vec3& b) { return std::sqrt((a - b).lengthSquared()); }

int countKind(const std::vector<EditHandle>& hs, HandleKind k) {
    int n = 0;
    for (const EditHandle& h : hs) if (h.kind == k) ++n;
    return n;
}
const EditHandle& firstOf(const std::vector<EditHandle>& hs, HandleKind k, int index) {
    for (const EditHandle& h : hs) if (h.kind == k && h.index == index) return h;
    static EditHandle none{}; return none;
}
}

TEST_CASE(curve_handle_source_exposes_knot_and_tangents) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    CurveHandleSource src(c);
    std::vector<EditHandle> hs = src.handles();
    CHECK(countKind(hs, HandleKind::Knot) == 2);
    CHECK(countKind(hs, HandleKind::TangentIn) == 2);
    CHECK(countKind(hs, HandleKind::TangentOut) == 2);
    CHECK(!src.previewSegments().empty());
    CHECK(src.defaultPlane() == DragPlane::Screen);
}

TEST_CASE(curve_handle_source_drags_update_the_curve) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    c.addKnot(Vec3(20, 0, 0));
    CurveHandleSource src(c);
    src.moveHandle(firstOf(src.handles(), HandleKind::Knot, 1), Vec3(10, 5, 0));
    CHECK(dist(c.knots[1].position, Vec3(10, 5, 0)) < 1e-9);
    // Dragging the out-handle off-axis shapes the curve (mode leaves Auto).
    src.moveHandle(firstOf(src.handles(), HandleKind::TangentOut, 1), Vec3(13, 9, 0));
    CHECK(c.knots[1].mode != KnotMode::Auto);
    CHECK(dist(c.outHandleWorld(1), Vec3(13, 9, 0)) < 1e-6);
}

TEST_CASE(road_handle_source_knots_then_tangents_when_curved) {
    RoadNet net;
    net.nodes = { Vec2(-20, 0), Vec2(0, 0), Vec2(20, 0) };
    net.edges = { {0, 1}, {1, 2} };
    RoadHandleSource src(net);
    CHECK(countKind(src.handles(), HandleKind::Knot) == 3);
    CHECK(countKind(src.handles(), HandleKind::TangentOut) == 0);   // straight: no tangents
    CHECK(static_cast<int>(src.previewSegments().size()) == 2);     // one per edge
    CHECK(src.defaultPlane() == DragPlane::Ground);

    net.curved = true;
    CHECK(countKind(src.handles(), HandleKind::TangentOut) == 3);   // one per node now
}

TEST_CASE(road_handle_source_drags_update_the_road) {
    RoadNet net;
    net.nodes = { Vec2(-20, 0), Vec2(0, 0), Vec2(20, 0) };
    net.edges = { {0, 1}, {1, 2} };
    net.curved = true;
    RoadHandleSource src(net);
    // Move node 1 (a Knot) — only X/Z matter; the handle Y is the road surface.
    src.moveHandle(firstOf(src.handles(), HandleKind::Knot, 1), Vec3(2, 99, 8));
    CHECK_APPROX(net.nodes[1].x, 2.0, 1e-9);
    CHECK_APPROX(net.nodes[1].y, 8.0, 1e-9);     // Vec2.y is world Z
    // Drag a tangent handle -> sets that node's through-tangent.
    src.moveHandle(firstOf(src.handles(), HandleKind::TangentOut, 1), Vec3(2 + 6, 99, 8 + 4));
    CHECK(net.tangents.size() >= 2);
    CHECK_APPROX(net.tangents[1].x, 6.0, 1e-9);
    CHECK_APPROX(net.tangents[1].y, 4.0, 1e-9);
}
