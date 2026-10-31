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

TEST_CASE(bezier_kernel_is_hermite_with_tripled_tangents) {
    // The two shared kernels describe the same cubic: m0 = 3 (p1 - p0), m1 = 3 (p3 - p2).
    const Vec3 p0(0, 0, 0), p1(1, 2, 0), p2(3, 2, 1), p3(4, 0, 1);
    for (int k = 0; k <= 8; ++k) {
        const double t = k / 8.0;
        const Vec3 b = bezier(p0, p1, p2, p3, t), h = hermite(p0, (p1 - p0) * 3.0, p3, (p3 - p2) * 3.0, t);
        CHECK_APPROX((b - h).length(), 0.0, 1e-12);
        const Vec3 db = bezierDerivative(p0, p1, p2, p3, t), dh = hermiteDerivative(p0, (p1 - p0) * 3.0, p3, (p3 - p2) * 3.0, t);
        CHECK_APPROX((db - dh).length(), 0.0, 1e-12);
    }
}

TEST_CASE(centripetal_spline_does_not_overshoot_uneven_points) {
    // Collinear points, one gap ten times shorter than its neighbours: uniform Catmull-Rom runs
    // past the close pair and doubles back; the centripetal curve keeps moving forward.
    const std::vector<Vec3> pts = {Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(1.1, 0, 0), Vec3(2.1, 0, 0)};
    auto monotonic = [](const Spline<Vec3>& s) {
        double last = s.eval(0.0).x;
        for (int k = 1; k <= 300; ++k) {
            const double x = s.eval(k / 100.0).x;
            if (x < last - 1e-12) return false;
            last = x;
        }
        return true;
    };
    CHECK(!monotonic(Spline<Vec3>::catmullRom(pts)));
    const Spline<Vec3> c = Spline<Vec3>::centripetal(pts);
    CHECK(monotonic(c));
    for (int i = 0; i < 4; ++i) CHECK_APPROX((c.eval(i) - pts[i]).length(), 0.0, 1e-12);
}

TEST_CASE(centripetal_spline_closed_wraps_smoothly) {
    const std::vector<Vec3> sq = {Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(10, 0, 10), Vec3(0, 0, 10)};
    const Spline<Vec3> c = Spline<Vec3>::centripetal(sq, /*closed=*/true);
    CHECK(c.segments() == 4);
    CHECK_APPROX((c.eval(4.0) - sq[0]).length(), 0.0, 1e-12);
    // The seam is as smooth as any other knot: arriving and leaving along the same direction.
    const Vec3 in = normalize(c.derivative(4.0)), out = normalize(c.derivative(0.0));
    CHECK_APPROX(dot(in, out), 1.0, 1e-9);
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
