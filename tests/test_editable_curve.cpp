#include "test_framework.h"

#include "../src/engine/editable_curve.h"
#include <cmath>

using namespace engine;

namespace {
double dist(const Vec3& a, const Vec3& b) { return std::sqrt((a - b).lengthSquared()); }
}

TEST_CASE(curve_straight_line_evaluates_linearly) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));                       // two collinear knots (auto handles)
    CHECK(c.segmentCount() == 1);
    CHECK(dist(c.position(0.5), Vec3(5, 0, 0)) < 1e-6);
    CHECK_APPROX(c.length(), 10.0, 1e-3);
    Vec3 t = c.tangent(0.5);
    CHECK(t.x > 0.0);
    CHECK(std::fabs(t.y) < 1e-6 && std::fabs(t.z) < 1e-6);
}

TEST_CASE(curve_auto_handles_round_a_corner) {
    EditableCurve c;
    c.addKnot(Vec3(-10, 0, 0));
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(0, 0, 10));                       // an L in the XZ plane
    // Auto (Catmull-Rom) handles bow the path off the straight legs near the corner;
    // a point a quarter along the first leg leaves the x-axis.
    CHECK(std::fabs(c.position(0.25).z) > 0.1);
}

TEST_CASE(curve_at_distance_is_constant_speed_on_a_line) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    CHECK(dist(c.atDistance(0.0), Vec3(0, 0, 0)) < 1e-6);
    CHECK(dist(c.atDistance(5.0), Vec3(5, 0, 0)) < 1e-3);
    CHECK(dist(c.atDistance(10.0), Vec3(10, 0, 0)) < 1e-3);   // clamps at the end
}

TEST_CASE(curve_aligned_handle_mirrors_the_other) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    c.addKnot(Vec3(20, 0, 0));
    c.setMode(1, KnotMode::Aligned);
    c.setOutHandle(1, Vec3(10, 0, 0) + Vec3(3, 4, 0));   // drag the out handle off-axis
    Vec3 inOff = c.knots[1].inHandle, outOff = c.knots[1].outHandle;
    CHECK(std::sqrt(cross(inOff, outOff).lengthSquared()) < 1e-6);   // collinear
    CHECK(dot(inOff, outOff) < 0.0);                                 // opposite directions
}

TEST_CASE(curve_setmode_bakes_auto_handles) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 5));
    c.addKnot(Vec3(20, 0, 0));
    CHECK(c.knots[1].outHandle.lengthSquared() < 1e-12);   // Auto: stored handle is zero
    c.setMode(1, KnotMode::Broken);
    CHECK(c.knots[1].outHandle.lengthSquared() > 1e-6);    // baked from the neighbours
}

TEST_CASE(curve_move_knot_carries_its_handles) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(10, 0, 0));
    c.addKnot(Vec3(20, 0, 0));
    c.setMode(1, KnotMode::Broken);
    Vec3 inOff = c.knots[1].inHandle;
    c.moveKnot(1, Vec3(10, 5, 0));
    CHECK(dist(c.knots[1].inHandle, inOff) < 1e-9);                       // offset unchanged
    CHECK(dist(c.inHandleWorld(1), Vec3(10, 5, 0) + inOff) < 1e-6);       // world rode along
}

TEST_CASE(curve_is_fully_3d) {
    EditableCurve c;
    c.addKnot(Vec3(0, 0, 0));
    c.addKnot(Vec3(5, 5, 5));                        // out of every axis plane
    Vec3 p = c.position(0.5), t = c.tangent(0.5);
    CHECK(p.x > 0 && p.y > 0 && p.z > 0);
    CHECK(t.x > 0 && t.y > 0 && t.z > 0);            // tangent carries all three axes
}

TEST_CASE(curve_add_remove_knots) {
    EditableCurve c;
    CHECK(c.addKnot(Vec3(0, 0, 0)) == 0);
    CHECK(c.addKnot(Vec3(10, 0, 0)) == 1);
    CHECK(c.segmentCount() == 1);
    CHECK(c.removeKnot(0));
    CHECK(c.knots.size() == 1);
    CHECK(c.segmentCount() == 0);
    CHECK(!c.removeKnot(5));
}
