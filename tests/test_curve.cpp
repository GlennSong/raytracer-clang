#include "test_framework.h"

#include "../src/curve.h"

using namespace engine;

namespace {
std::vector<Vec3> line(int n, double dx) {
    std::vector<Vec3> p;
    for (int i = 0; i < n; i++) p.push_back(Vec3(i * dx, 0, 0));
    return p;
}
}  // namespace

TEST_CASE(spline_interpolates_control_points) {
    std::vector<Vec3> pts = {Vec3(0, 0, 0), Vec3(1, 2, 0), Vec3(3, 1, -1),
                             Vec3(4, 0, 2)};
    Spline<Vec3> s = Spline<Vec3>::catmullRom(pts);
    CHECK(s.segments() == 3);
    // A Catmull-Rom curve passes through every control point (eval at integers).
    for (int i = 0; i < static_cast<int>(pts.size()); i++) {
        Vec3 v = s.eval(i);
        CHECK_APPROX(v.x, pts[i].x, 1e-9);
        CHECK_APPROX(v.y, pts[i].y, 1e-9);
        CHECK_APPROX(v.z, pts[i].z, 1e-9);
    }
}

TEST_CASE(spline_clamps_outside_range) {
    Spline<Vec3> s = Spline<Vec3>::catmullRom(line(3, 1.0));
    Vec3 lo = s.eval(-5.0), hi = s.eval(99.0);
    CHECK_APPROX(lo.x, 0.0, 1e-9);   // clamped to the first knot
    CHECK_APPROX(hi.x, 2.0, 1e-9);   // clamped to the last knot
}

TEST_CASE(spline_derivative_points_along_curve) {
    Spline<Vec3> s = Spline<Vec3>::catmullRom(line(4, 1.0));
    Vec3 d = s.derivative(1.5);   // mid-curve, straight line along +x
    CHECK(d.x > 0.0);
    CHECK_APPROX(d.y, 0.0, 1e-9);
    CHECK_APPROX(d.z, 0.0, 1e-9);
}

TEST_CASE(spline_length_of_straight_line) {
    Spline<Vec3> s = Spline<Vec3>::catmullRom(line(4, 2.0));   // 0..6 on x
    CHECK_APPROX(s.length(), 6.0, 1e-3);
}

TEST_CASE(spline_uniform_arc_length_is_evenly_spaced) {
    Spline<Vec3> s = Spline<Vec3>::catmullRom(line(4, 1.0));   // length 3
    std::vector<double> us = s.uniformArcLengthParams(7);
    CHECK(us.size() == 7u);
    CHECK_APPROX(us.front(), 0.0, 1e-9);
    CHECK_APPROX(us.back(), 3.0, 1e-9);
    // Evenly spaced in arc length => x advances by 0.5 each step on this line.
    for (int k = 0; k < 7; k++)
        CHECK_APPROX(s.eval(us[k]).x, 0.5 * k, 2e-2);
}

TEST_CASE(spline_scalar_value_type) {
    // An F-curve-style scalar channel: value over (here) parameter.
    std::vector<float> keys = {0.0f, 10.0f, 0.0f};
    Spline<float> s = Spline<float>::catmullRom(keys);
    CHECK_APPROX(s.eval(0.0), 0.0, 1e-6);
    CHECK_APPROX(s.eval(1.0), 10.0, 1e-6);
    CHECK_APPROX(s.eval(2.0), 0.0, 1e-6);
    CHECK(s.eval(0.5) > 0.0);   // rises toward the peak
}

TEST_CASE(rmf_frames_are_orthonormal_and_twist_free) {
    std::vector<Vec3> pts = line(5, 1.0);
    std::vector<Vec3> tan(pts.size(), Vec3(1, 0, 0));
    std::vector<CurveFrame> f = rotationMinimizingFrames(pts, tan);
    CHECK(f.size() == pts.size());

    Vec3 right0 = f[0].right;
    for (const CurveFrame& fr : f) {
        // Orthonormal: unit length, mutually perpendicular.
        CHECK_APPROX(fr.right.length(), 1.0, 1e-6);
        CHECK_APPROX(fr.up.length(), 1.0, 1e-6);
        CHECK_APPROX(dot(fr.right, fr.tangent), 0.0, 1e-6);
        CHECK_APPROX(dot(fr.up, fr.tangent), 0.0, 1e-6);
        CHECK_APPROX(dot(fr.right, fr.up), 0.0, 1e-6);
        // Straight line: no twist, so right stays put.
        CHECK_APPROX(dot(fr.right, right0), 1.0, 1e-6);
    }
}

TEST_CASE(rmf_follows_a_bend_without_flipping) {
    // An L-shaped path: tangent turns 90 degrees. Frames must stay orthonormal
    // and the right vector must vary smoothly (no sudden flip).
    std::vector<Vec3> pts = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(2, 0, 0),
                             Vec3(2, 1, 0), Vec3(2, 2, 0)};
    Spline<Vec3> s = Spline<Vec3>::catmullRom(pts);
    std::vector<Vec3> p, t;
    for (double u = 0; u <= s.segments(); u += 0.25) {
        p.push_back(s.eval(u));
        t.push_back(s.derivative(u));
    }
    std::vector<CurveFrame> f = rotationMinimizingFrames(p, t);
    for (size_t i = 0; i < f.size(); i++) {
        CHECK_APPROX(dot(f[i].right, f[i].tangent), 0.0, 1e-6);
        CHECK_APPROX(f[i].right.length(), 1.0, 1e-6);
        if (i > 0) CHECK(dot(f[i].right, f[i - 1].right) > 0.7);  // no flip
    }
}
