#include "test_framework.h"

#include "../src/rt_math.h"

using namespace engine;  // namespace migration (ADR-0015)

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

TEST_CASE(vec3_lerp_distance_minmax) {
    Vec3 mid = lerp(Vec3(0, 0, 0), Vec3(10, 0, 0), 0.5);
    CHECK_APPROX(mid.x, 5, EPS);

    CHECK_APPROX(distance(Vec3(0, 0, 0), Vec3(3, 4, 0)), 5, EPS);
    CHECK_APPROX(distanceSquared(Vec3(0, 0, 0), Vec3(3, 4, 0)), 25, EPS);

    Vec3 lo = minVec(Vec3(1, 5, 3), Vec3(4, 2, 6));
    Vec3 hi = maxVec(Vec3(1, 5, 3), Vec3(4, 2, 6));
    CHECK(approxEqual(lo, Vec3(1, 2, 3)));
    CHECK(approxEqual(hi, Vec3(4, 5, 6)));

    CHECK(approxEqual(Vec3(1, 1, 1), Vec3(1, 1, 1.0000001), 1e-5));
    CHECK(!approxEqual(1.0, 1.1));
}

TEST_CASE(radians_degrees_round_trip) {
    CHECK_APPROX(radiansToDegrees(PI), 180, EPS);
    CHECK_APPROX(degreesToRadians(radiansToDegrees(1.234)), 1.234, EPS);
}

TEST_CASE(quat_identity_is_inert) {
    Vec3 v(1, 2, 3);
    Vec3 r = Quat::identity().rotate(v);
    CHECK(approxEqual(r, v));
}

TEST_CASE(quat_axis_angle_rotates_vector) {
    Quat qy = Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(90));
    Vec3 r = qy.rotate(Vec3(0, 0, -1));
    CHECK(approxEqual(r, Vec3(-1, 0, 0)));
    // Rotation preserves length.
    CHECK_APPROX(qy.rotate(Vec3(3, 0, 4)).length(), 5, EPS);
}

TEST_CASE(quat_compose_applies_right_first) {
    Quat qy90 = Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(90));
    Quat qy180 = qy90 * qy90;
    Vec3 r = qy180.rotate(Vec3(0, 0, -1));
    CHECK(approxEqual(r, Vec3(0, 0, 1)));
}

TEST_CASE(quat_inverse_undoes_rotation) {
    Quat q = Quat::fromAxisAngle(Vec3(1, 2, 3), 1.1);
    Quat back = q * q.inverse();
    CHECK(approxEqual(back.rotate(Vec3(5, -2, 1)), Vec3(5, -2, 1)));
}

TEST_CASE(quat_normalize) {
    Quat q(1, 2, 3, 4);
    CHECK_APPROX(q.normalized().length(), 1, EPS);
}

TEST_CASE(quat_matrix_matches_rotate) {
    Quat q = Quat::fromAxisAngle(normalize(Vec3(1, 1, 0)), degreesToRadians(50));
    Mat4 m = q.toMat4();
    Vec3 v(0.3, -1.2, 2.0);
    CHECK(approxEqual(m.transformDirection(v), q.rotate(v)));
}

TEST_CASE(quat_euler_round_trip) {
    Vec3 e(0.3, -0.4, 0.5);
    Vec3 back = Quat::fromEuler(e).toEuler();
    CHECK(approxEqual(back, e, 1e-6));
}

TEST_CASE(quat_slerp_endpoints_and_midpoint) {
    Quat a = Quat::identity();
    Quat b = Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(90));

    CHECK(approxEqual(Quat::slerp(a, b, 0.0).rotate(Vec3(0, 0, -1)), Vec3(0, 0, -1)));
    CHECK(approxEqual(Quat::slerp(a, b, 1.0).rotate(Vec3(0, 0, -1)), Vec3(-1, 0, 0)));

    // Halfway is a 45-degree rotation of -Z.
    Vec3 mid = Quat::slerp(a, b, 0.5).rotate(Vec3(0, 0, -1));
    Real s = std::sin(degreesToRadians(45));
    CHECK(approxEqual(mid, Vec3(-s, 0, -s)));
}

TEST_CASE(mat4_look_at) {
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    // World origin sits 5 units down the view -z; the eye maps to the origin.
    CHECK(approxEqual(view.transformPoint(Vec3(0, 0, 0)), Vec3(0, 0, -5)));
    CHECK(approxEqual(view.transformPoint(Vec3(0, 0, 5)), Vec3(0, 0, 0)));
}

TEST_CASE(mat4_trs) {
    // Translation only.
    Vec3 p = Mat4::trs(Vec3(1, 2, 3), Quat::identity(), Vec3(1, 1, 1))
                 .transformPoint(Vec3(0, 0, 0));
    CHECK(approxEqual(p, Vec3(1, 2, 3)));

    // Scale only.
    Vec3 s = Mat4::trs(Vec3(0, 0, 0), Quat::identity(), Vec3(2, 3, 4))
                 .transformPoint(Vec3(1, 1, 1));
    CHECK(approxEqual(s, Vec3(2, 3, 4)));

    // Scale then rotate then translate: rotate -Z by 90 about Y -> -X, +offset.
    Quat qy = Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(90));
    Vec3 r = Mat4::trs(Vec3(10, 0, 0), qy, Vec3(1, 1, 1))
                 .transformPoint(Vec3(0, 0, -1));
    CHECK(approxEqual(r, Vec3(9, 0, 0)));
}
