#include "test_framework.h"

#include "../src/rt_math.h"
#include "../src/engine/procgen/city/polygon.h"   // the plan-point approxEqual

#include <cmath>
#include <limits>

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

// Reverse-Z (ADR-0034 Phase 0): near->1, far->0, the opposite of the forward
// [0,1] mapping, for near-uniform depth precision across a wide far plane.
TEST_CASE(reverse_z_perspective_flips_near_and_far) {
    const Real nearPlane = 0.1;
    const Real farPlane = 8000.0;
    Mat4 proj = Mat4::reverseZ() * Mat4::perspective(degreesToRadians(60), 1.6,
                                                     nearPlane, farPlane);

    Vec3 atNear = proj.transformPoint(Vec3(0, 0, -nearPlane));
    Vec3 atFar = proj.transformPoint(Vec3(0, 0, -farPlane));
    CHECK_APPROX(atNear.z, 1, 1e-6);
    CHECK_APPROX(atFar.z, 0, 1e-6);

    // Closer geometry maps to a strictly greater depth than farther geometry
    // (so the GPU's Greater depth test keeps the nearer fragment), and distant
    // terrain stays strictly above the cleared far value of 0 — the property
    // that stops far terrain from being misclassified as sky.
    Vec3 near100 = proj.transformPoint(Vec3(0, 0, -100.0));
    Vec3 far4000 = proj.transformPoint(Vec3(0, 0, -4000.0));
    CHECK(near100.z > far4000.z);
    CHECK(far4000.z > 0.0);

    // The flip only touches z; x/y projection is unchanged.
    const Real fov = degreesToRadians(90);
    Mat4 rzEdge = Mat4::reverseZ() * Mat4::perspective(fov, 1.0, 0.1, 100.0);
    const Real d = 5.0;
    Vec3 p = rzEdge.transformPoint(Vec3(0, d * std::tan(fov * 0.5), -d));
    CHECK_APPROX(p.y, 1, 1e-6);
}

TEST_CASE(reverse_z_orthographic_flips_near_and_far) {
    const Real nearPlane = 0.5;
    const Real farPlane = 50.0;
    Mat4 proj = Mat4::reverseZ() * Mat4::orthographic(10.0, 1.6, nearPlane, farPlane);

    Vec3 atNear = proj.transformPoint(Vec3(0, 0, -nearPlane));
    Vec3 atFar = proj.transformPoint(Vec3(0, 0, -farPlane));
    CHECK_APPROX(atNear.z, 1, 1e-9);
    CHECK_APPROX(atFar.z, 0, 1e-9);
    // Linear depth, so the midpoint still lands at 0.5.
    Vec3 mid = proj.transformPoint(Vec3(0, 0, -(nearPlane + farPlane) * 0.5));
    CHECK_APPROX(mid.z, 0.5, 1e-9);
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

TEST_CASE(mat4_transpose) {
    Mat4 a;
    a.m[0][1] = 2; a.m[0][2] = 3;
    a.m[1][0] = 4; a.m[2][0] = 5;
    Mat4 t = a.transpose();
    CHECK_APPROX(t.m[1][0], 2, EPS);
    CHECK_APPROX(t.m[2][0], 3, EPS);
    CHECK_APPROX(t.m[0][1], 4, EPS);
    CHECK_APPROX(t.m[0][2], 5, EPS);
    // Diagonal unchanged
    CHECK_APPROX(t.m[0][0], 1, EPS);
    CHECK_APPROX(t.m[3][3], 1, EPS);
}

TEST_CASE(mat4_inverse_identity) {
    Mat4 inv = Mat4::identity().inverse();
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            CHECK_APPROX(inv.m[i][j], (i == j) ? 1.0 : 0.0, EPS);
}

TEST_CASE(mat4_inverse_translation) {
    Mat4 t = Mat4::translate(3, -5, 7);
    Mat4 inv = t.inverse();
    Vec3 p = (t * inv).transformPoint(Vec3(1, 2, 3));
    CHECK(approxEqual(p, Vec3(1, 2, 3)));
    // Inverse of a translation is the negative translation
    CHECK_APPROX(inv.m[0][3], -3, EPS);
    CHECK_APPROX(inv.m[1][3], 5, EPS);
    CHECK_APPROX(inv.m[2][3], -7, EPS);
}

TEST_CASE(mat4_inverse_perspective) {
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.6, 0.1, 100.0);
    Mat4 inv = proj.inverse();
    Mat4 product = proj * inv;
    // Product should be approximately identity
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            CHECK_APPROX(product.m[i][j], (i == j) ? 1.0 : 0.0, 1e-6);
}

TEST_CASE(mat4_inverse_view_projection) {
    Mat4 view = Mat4::lookAt(Vec3(5, 3, 10), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.6, 0.1, 100.0);
    Mat4 vp = proj * view;
    Mat4 inv = vp.inverse();
    Mat4 product = vp * inv;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            CHECK_APPROX(product.m[i][j], (i == j) ? 1.0 : 0.0, 1e-5);
}

// ---- Comparing computed numbers (rt_math.h: approxEqual and its ordered companions) ----------------------------------

namespace {

// How many of the three exclusive questions say yes about (a, b): must be exactly one for any two numbers.
int exclusiveAnswers(Real a, Real b, Real eps) {
    return (strictlyBefore(a, b, eps) ? 1 : 0) + (approxEqual(a, b, eps) ? 1 : 0) + (strictlyPast(a, b, eps) ? 1 : 0);
}

}  // namespace

TEST_CASE(approx_compare_exactly_one_of_before_equal_past) {
    // Hand-written values against b = 10 with a tolerance of 0.25 (both exact in binary, so "at the tolerance" is exact):
    // far before, just outside, AT the tolerance, just inside, the same, and the mirror of each.
    const Real b = 10, eps = 0.25;
    struct Row {
        Real a;
        bool before, equal, past;
    };
    // (One step of a double at 9.75 and at 10.25 is 1.8e-15, and each stepped value's difference from 10 is exact, so the
    // rows "one step outside" and "one step inside" are exactly that.)
    const Row rows[] = {
        {5.0, true, false, false},                          // far before
        {9.7499, true, false, false},                       // just outside the tolerance, before
        {std::nextafter(9.75, 0.0), true, false, false},    // ONE STEP of a double outside the edge: before
        {9.75, false, true, false},                         // exactly at the tolerance: the same
        {std::nextafter(9.75, 11.0), false, true, false},   // one step inside the edge: the same
        {9.7501, false, true, false},                       // just inside
        {10.0, false, true, false},                         // the same number
        {10.2499, false, true, false},                      // just inside, past
        {std::nextafter(10.25, 0.0), false, true, false},   // one step inside the edge: the same
        {10.25, false, true, false},                        // exactly at the tolerance: the same
        {std::nextafter(10.25, 11.0), false, false, true},  // ONE STEP outside the edge: past
        {10.2501, false, false, true},                      // just outside, past
        {15.0, false, false, true},                         // far past
    };
    for (const Row& r : rows) {
        CHECK(strictlyBefore(r.a, b, eps) == r.before);
        CHECK(approxEqual(r.a, b, eps) == r.equal);
        CHECK(strictlyPast(r.a, b, eps) == r.past);
        CHECK(exclusiveAnswers(r.a, b, eps) == 1);
        // The two inclusive questions are "before or the same" and "past or the same", from the same number.
        CHECK(atOrBefore(r.a, b, eps) == (r.before || r.equal));
        CHECK(atOrPast(r.a, b, eps) == (r.past || r.equal));
        CHECK(atOrBefore(r.a, b, eps) == !strictlyPast(r.a, b, eps));
        CHECK(atOrPast(r.a, b, eps) == !strictlyBefore(r.a, b, eps));
        // Asked the other way round, before and past swap and equal stays.
        CHECK(strictlyBefore(b, r.a, eps) == r.past);
        CHECK(strictlyPast(b, r.a, eps) == r.before);
        CHECK(approxEqual(b, r.a, eps) == r.equal);
    }
    // A tolerance of zero is the plain comparison.
    CHECK(strictlyBefore(1, 2, 0) && !approxEqual(1, 2, 0) && !strictlyPast(1, 2, 0));
    CHECK(!strictlyBefore(2, 2, 0) && approxEqual(2, 2, 0) && !strictlyPast(2, 2, 0));
    CHECK(atOrBefore(2, 2, 0) && atOrPast(2, 2, 0));
}

TEST_CASE(approx_compare_a_stop_at_a_start_is_at_or_before_it) {
    // THE COLLISION'S CASE (2026-11-14): a stop placed at a stretch's start came out of a sum 7.1e-15 m past it. The
    // bare comparison says it is not at or before the start; the question with the 1 mm tolerance says it is, and says
    // the two are the same place. (As measured in a driving merge scene: 7.105e-15 m is one step of a double between 32
    // and 64 m, so the stop was one step past the start; 60.6 m stands in for the measured place.)
    const Real start = 60.6;
    const Real stop = std::nextafter(start, 100.0);
    CHECK_APPROX(stop - start, 7.105e-15, 1e-18);
    CHECK(stop != start);       // the sum really is past, by the last bit
    CHECK(!(stop <= start));    // what the bare comparison answered
    CHECK(atOrBefore(stop, start, kSamePoint));
    CHECK(approxEqual(stop, start, kSamePoint));
    CHECK(!strictlyPast(stop, start, kSamePoint));
    // A stop really past the start (2 mm) is past it; one really short of it (2 mm) is before it.
    CHECK(strictlyPast(start + 0.002, start, kSamePoint) && !atOrBefore(start + 0.002, start, kSamePoint));
    CHECK(strictlyBefore(start - 0.002, start, kSamePoint) && atOrBefore(start - 0.002, start, kSamePoint));
}

TEST_CASE(approx_compare_named_tolerances_are_one_millimetre_and_one_centimetre) {
    CHECK(kSamePoint == 0.001);        // metres: two positions closer than this are one place
    CHECK(kWrittenStation == 0.01);    // metres: road files write stations to the centimetre
    // Two stations written to the centimetre from one number, 12.34 and 12.345 rounded either way, are one written
    // station and not one point.
    CHECK(approxEqual(12.34, 12.345, kWrittenStation));
    CHECK(!approxEqual(12.34, 12.345, kSamePoint));
}

TEST_CASE(approx_compare_nan_and_infinities) {
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    const Real inf = std::numeric_limits<Real>::infinity();
    const Real eps = kSamePoint;
    // A NaN on either side: every question answers false (so atOrBefore is not "not strictlyPast").
    for (const Real other : {0.0, 1.0, inf, nan}) {
        CHECK(!strictlyBefore(nan, other, eps) && !approxEqual(nan, other, eps) && !strictlyPast(nan, other, eps));
        CHECK(!atOrBefore(nan, other, eps) && !atOrPast(nan, other, eps));
        CHECK(!strictlyBefore(other, nan, eps) && !approxEqual(other, nan, eps) && !strictlyPast(other, nan, eps));
        CHECK(!atOrBefore(other, nan, eps) && !atOrPast(other, nan, eps));
    }
    // One side infinite: strictly before or past, never the same.
    CHECK(strictlyPast(inf, 5, eps) && atOrPast(inf, 5, eps) && !approxEqual(inf, 5, eps) && !atOrBefore(inf, 5, eps));
    CHECK(strictlyBefore(5, inf, eps) && atOrBefore(5, inf, eps) && !approxEqual(5, inf, eps) && !atOrPast(5, inf, eps));
    CHECK(strictlyBefore(-inf, 5, eps) && strictlyPast(5, -inf, eps));
    // Infinities of opposite signs: strictly before or past.
    CHECK(strictlyBefore(-inf, inf, eps) && !approxEqual(-inf, inf, eps) && !atOrPast(-inf, inf, eps));
    CHECK(strictlyPast(inf, -inf, eps) && !approxEqual(inf, -inf, eps) && !atOrBefore(inf, -inf, eps));
    // TWO INFINITIES OF ONE SIGN ARE THE SAME: the driving code's "never" against "never". A bare `never <= never` is
    // true, and so is the question that replaces it; neither is strictly before or past the other.
    for (const Real never : {inf, -inf}) {
        CHECK(never <= never);
        CHECK(approxEqual(never, never, eps) && atOrBefore(never, never, eps) && atOrPast(never, never, eps));
        CHECK(!strictlyBefore(never, never, eps) && !strictlyPast(never, never, eps));
        CHECK(approxEqual(never, never, 0) && atOrBefore(never, never, 0));
    }
    // So for every pair of numbers, the infinities included: exactly one of the three, and the inclusive questions are
    // the opposites of the strict ones.
    for (const Real a : {-inf, -1.0, 0.0, 0.0005, 1.0, inf})
        for (const Real b : {-inf, -1.0, 0.0, 0.0005, 1.0, inf}) {
            CHECK(exclusiveAnswers(a, b, eps) == 1);
            CHECK(atOrBefore(a, b, eps) == !strictlyPast(a, b, eps));
            CHECK(atOrPast(a, b, eps) == !strictlyBefore(a, b, eps));
        }
    // A stop that is "never" is not at or before a start that is somewhere; a stop somewhere is before "never".
    CHECK(!atOrBefore(inf, 60.6, eps) && atOrBefore(60.6, inf, eps));
    // A NaN tolerance: every question false.
    CHECK(exclusiveAnswers(1, 1, nan) == 0 && !atOrBefore(1, 1, nan) && !atOrPast(1, 1, nan));
    // The Vec3 one, coordinate by coordinate, takes the same answer for equal infinite coordinates.
    CHECK(approxEqual(Vec3(inf, 1, -inf), Vec3(inf, 1, -inf)));
    CHECK(!approxEqual(Vec3(inf, 1, 0), Vec3(-inf, 1, 0)));
}

TEST_CASE(approx_compare_plan_points_by_distance) {
    // Two plan points are the same when the DISTANCE between them is within the tolerance (the road code's same-point
    // checks are all distances), not when each coordinate is.
    const Vec2 p(100.0, -50.0);
    CHECK(approxEqual(p, p, kSamePoint));
    CHECK(approxEqual(p, Vec2(100.0005, -50.0005), kSamePoint));    // 0.5 mm off each way: 0.71 mm apart
    // 0.8 mm off each way: each coordinate is inside 1 mm, but the points are 1.13 mm apart.
    CHECK(!approxEqual(p, Vec2(100.0008, -50.0008), kSamePoint));
    CHECK(approxEqual(p.x, 100.0008, kSamePoint) && approxEqual(p.y, -50.0008, kSamePoint));
    CHECK(!approxEqual(p, Vec2(100.002, -50.0), kSamePoint));       // 2 mm off in x
    CHECK(!approxEqual(p, Vec2(100.0, -50.002), kSamePoint));       // 2 mm off in y
    CHECK(approxEqual(p, Vec2(100.002, -50.002), kWrittenStation)); // 2.8 mm apart: one written point
    // Exactly at the tolerance counts: 3 and 4 apart is 5 apart.
    CHECK(approxEqual(Vec2(0, 0), Vec2(3, 4), 5.0));
    CHECK(!approxEqual(Vec2(0, 0), Vec2(3, 4), std::nextafter(5.0, 0.0)));
    const Real nan = std::numeric_limits<Real>::quiet_NaN();
    const Real inf = std::numeric_limits<Real>::infinity();
    CHECK(!approxEqual(p, Vec2(nan, -50.0), kSamePoint));
    CHECK(approxEqual(Vec2(inf, 1), Vec2(inf, 1), kSamePoint));     // the same point is the same
    CHECK(!approxEqual(Vec2(inf, 1), Vec2(-inf, 1), kSamePoint));
}
