#include "test_framework.h"

#include "../src/math.h"

namespace {
constexpr Real EPS = 1e-9;
}

TEST_CASE(vec3_arithmetic) {
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);

    Vec3 sum = a + b;
    CHECK_APPROX(sum.x, 5, EPS);
    CHECK_APPROX(sum.y, 7, EPS);
    CHECK_APPROX(sum.z, 9, EPS);

    Vec3 scaled = a * 2.0;
    CHECK_APPROX(scaled.x, 2, EPS);
    CHECK_APPROX(scaled.z, 6, EPS);

    CHECK_APPROX(dot(a, b), 32, EPS);
}

TEST_CASE(vec3_cross_and_normalize) {
    Vec3 x(1, 0, 0);
    Vec3 y(0, 1, 0);
    Vec3 z = cross(x, y);
    CHECK_APPROX(z.x, 0, EPS);
    CHECK_APPROX(z.y, 0, EPS);
    CHECK_APPROX(z.z, 1, EPS);

    Vec3 n = normalize(Vec3(3, 0, 4));
    CHECK_APPROX(n.length(), 1, EPS);
    CHECK_APPROX(n.x, 0.6, EPS);
    CHECK_APPROX(n.z, 0.8, EPS);

    // normalize of zero must not divide by zero.
    Vec3 zero = normalize(Vec3(0, 0, 0));
    CHECK_APPROX(zero.length(), 0, EPS);
}

TEST_CASE(mat4_identity_and_multiply) {
    Mat4 id = Mat4::identity();
    Mat4 t = Mat4::translate(5, 6, 7);
    Mat4 product = id * t;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            CHECK_APPROX(product.m[i][j], t.m[i][j], EPS);
}

TEST_CASE(mat4_transform_point) {
    Mat4 t = Mat4::translate(1, 2, 3);
    Vec3 p = t.transformPoint(Vec3(10, 20, 30));
    CHECK_APPROX(p.x, 11, EPS);
    CHECK_APPROX(p.y, 22, EPS);
    CHECK_APPROX(p.z, 33, EPS);

    Mat4 s = Mat4::scale(2, 3, 4);
    Vec3 q = s.transformPoint(Vec3(1, 1, 1));
    CHECK_APPROX(q.x, 2, EPS);
    CHECK_APPROX(q.y, 3, EPS);
    CHECK_APPROX(q.z, 4, EPS);
}

TEST_CASE(mat4_rotation_z) {
    Mat4 r = Mat4::rotateZ(degreesToRadians(90));
    Vec3 p = r.transformPoint(Vec3(1, 0, 0));
    CHECK_APPROX(p.x, 0, 1e-9);
    CHECK_APPROX(p.y, 1, 1e-9);
    CHECK_APPROX(p.z, 0, 1e-9);
}

// The point of moving projection engine-side (ADR-0009): depth must map to the
// Metal [0, 1] clip range, with the camera looking down -z. The old perspective
// matrix mapped to OpenGL's [-1, 1]; these tests would have caught that.
TEST_CASE(perspective_depth_maps_to_zero_one) {
    const Real nearPlane = 0.1;
    const Real farPlane = 100.0;
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.6, nearPlane, farPlane);

    Vec3 atNear = proj.transformPoint(Vec3(0, 0, -nearPlane));
    Vec3 atFar = proj.transformPoint(Vec3(0, 0, -farPlane));
    CHECK_APPROX(atNear.z, 0, 1e-6);
    CHECK_APPROX(atFar.z, 1, 1e-6);

    // A point between near and far stays inside (0, 1).
    Vec3 mid = proj.transformPoint(Vec3(0, 0, -10.0));
    CHECK(mid.z > 0.0 && mid.z < 1.0);
}

TEST_CASE(perspective_edge_maps_to_ndc_one) {
    const Real fov = degreesToRadians(90);
    const Real aspect = 1.0;
    Mat4 proj = Mat4::perspective(fov, aspect, 0.1, 100.0);

    // At distance d in front of the camera, a y of d*tan(fov/2) is the top edge,
    // which should land at NDC y = 1.
    const Real d = 5.0;
    Real edgeY = d * std::tan(fov * 0.5);
    Vec3 p = proj.transformPoint(Vec3(0, edgeY, -d));
    CHECK_APPROX(p.y, 1, 1e-6);
}

TEST_CASE(orthographic_depth_maps_to_zero_one) {
    const Real nearPlane = 0.5;
    const Real farPlane = 50.0;
    Mat4 proj = Mat4::orthographic(10.0, 1.6, nearPlane, farPlane);

    Vec3 atNear = proj.transformPoint(Vec3(0, 0, -nearPlane));
    Vec3 atFar = proj.transformPoint(Vec3(0, 0, -farPlane));
    CHECK_APPROX(atNear.z, 0, 1e-9);
    CHECK_APPROX(atFar.z, 1, 1e-9);

    // Orthographic depth is linear, so the midpoint lands at 0.5.
    Vec3 mid = proj.transformPoint(Vec3(0, 0, -(nearPlane + farPlane) * 0.5));
    CHECK_APPROX(mid.z, 0.5, 1e-9);
}

TEST_CASE(orthographic_xy_scale) {
    // Full height 10, aspect 2 -> full width 20; edges map to NDC +/-1.
    Mat4 proj = Mat4::orthographic(10.0, 2.0, 0.1, 100.0);
    Vec3 top = proj.transformPoint(Vec3(0, 5.0, -1.0));
    Vec3 right = proj.transformPoint(Vec3(10.0, 0, -1.0));
    CHECK_APPROX(top.y, 1, EPS);
    CHECK_APPROX(right.x, 1, EPS);
}
