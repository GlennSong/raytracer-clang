#include "test_framework.h"

#include "../src/rt_math.h"
#include "../src/renderer/renderer.h"

using namespace engine;

namespace {
constexpr Real EPS = 1e-6;
}

TEST_CASE(bounding_sphere_single_vertex) {
    Vertex v(Vec3(3, 4, 5), Vec3(0, 1, 0));
    BoundingSphere bs = computeBoundingSphere(&v, 1);
    CHECK_APPROX(bs.center.x, 3, EPS);
    CHECK_APPROX(bs.center.y, 4, EPS);
    CHECK_APPROX(bs.center.z, 5, EPS);
    CHECK_APPROX(bs.radius, 0, EPS);
}

TEST_CASE(bounding_sphere_cube_vertices) {
    Vertex verts[8];
    int i = 0;
    for (int x = -1; x <= 1; x += 2)
        for (int y = -1; y <= 1; y += 2)
            for (int z = -1; z <= 1; z += 2)
                verts[i++] = Vertex(Vec3(x, y, z), Vec3(0, 1, 0));
    BoundingSphere bs = computeBoundingSphere(verts, 8);
    CHECK_APPROX(bs.center.x, 0, EPS);
    CHECK_APPROX(bs.center.y, 0, EPS);
    CHECK_APPROX(bs.center.z, 0, EPS);
    CHECK_APPROX(bs.radius, std::sqrt(3.0), EPS);
}

TEST_CASE(bounding_sphere_empty) {
    BoundingSphere bs = computeBoundingSphere(nullptr, 0);
    CHECK_APPROX(bs.radius, 0, EPS);
}

TEST_CASE(frustum_from_identity_vp) {
    Mat4 vp = Mat4::identity();
    Frustum f = Frustum::fromViewProjection(vp);
    // Origin should be inside
    CHECK(f.containsSphere(Vec3(0, 0, 0), 0.1));
}

TEST_CASE(frustum_perspective_basic) {
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 0.1, 100.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    // Object at origin, in front of camera — should be visible
    CHECK(f.containsSphere(Vec3(0, 0, 0), 0.5));

    // Object behind the camera — should be culled
    CHECK(!f.containsSphere(Vec3(0, 0, 10), 0.5));

    // Object far to the right — should be culled
    CHECK(!f.containsSphere(Vec3(100, 0, 0), 0.5));

    // Object far to the left — should be culled
    CHECK(!f.containsSphere(Vec3(-100, 0, 0), 0.5));

    // Object far above — should be culled
    CHECK(!f.containsSphere(Vec3(0, 100, 0), 0.5));

    // Object far below — should be culled
    CHECK(!f.containsSphere(Vec3(0, -100, 0), 0.5));
}

TEST_CASE(frustum_near_far_plane) {
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 1.0, 50.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    // Object between near and far — visible
    CHECK(f.containsSphere(Vec3(0, 0, 0), 0.5));

    // Object beyond far plane — culled
    CHECK(!f.containsSphere(Vec3(0, 0, -100), 0.5));
}

TEST_CASE(frustum_sphere_partial_intersection) {
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 0.1, 100.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    // Large sphere centered far to the side but with radius reaching into frustum
    CHECK(f.containsSphere(Vec3(0, 0, 0), 50.0));
}

TEST_CASE(handle_less_than) {
    Handle<MeshTag> a{1, 1};
    Handle<MeshTag> b{2, 1};
    Handle<MeshTag> c{1, 2};
    CHECK(a < b);
    CHECK(!(b < a));
    CHECK(a < c);
    CHECK(!(a < a));
}

TEST_CASE(unproject_depth_convention_near_is_zero) {
    // The pick ray (EditorSystem) unprojects NDC z=0 as the near plane and
    // z=1 as far. Regression: with these swapped the ray starts on the FAR
    // plane looking back at the camera, so overlapping objects pick
    // furthest-first.
    Vec3 eye(0, 0, 10);
    Mat4 view = Mat4::lookAt(eye, Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60.0), 1.0, 0.1, 100.0);
    Mat4 invVP = (proj * view).inverse();

    Vec3 nearP = invVP.transformPoint(Vec3(0, 0, 0.0));
    Vec3 farP = invVP.transformPoint(Vec3(0, 0, 1.0));

    CHECK((nearP - eye).length() < 1.0);    // next to the eye (near = 0.1)
    CHECK((farP - eye).length() > 50.0);    // out at the far plane

    // The center-pixel ray points the way the camera looks (-Z here): the
    // nearest of two objects along it has the smaller t.
    Vec3 dir = normalize(farP - nearP);
    CHECK(dir.z < -0.9);
}

TEST_CASE(bounding_volume_carries_model_space_box) {
    // The picker's slab test needs the tight box, not just the sphere — a
    // flat plane's sphere covers a disc of the whole scene, its box doesn't.
    Vertex verts[4] = {
        Vertex(Vec3(-2, 0, -3), Vec3(0, 1, 0), Vec3(1, 0, 0), 0, 0),
        Vertex(Vec3(2, 0, -3), Vec3(0, 1, 0), Vec3(1, 0, 0), 0, 0),
        Vertex(Vec3(-2, 0, 3), Vec3(0, 1, 0), Vec3(1, 0, 0), 0, 0),
        Vertex(Vec3(2, 0, 3), Vec3(0, 1, 0), Vec3(1, 0, 0), 0, 0),
    };
    BoundingSphere b = computeBoundingSphere(verts, 4);
    CHECK_APPROX(b.boxMin.x, -2.0, 1e-9);
    CHECK_APPROX(b.boxMax.x, 2.0, 1e-9);
    CHECK_APPROX(b.boxMin.y, 0.0, 1e-9);
    CHECK_APPROX(b.boxMax.y, 0.0, 1e-9);   // genuinely flat
    CHECK_APPROX(b.boxMin.z, -3.0, 1e-9);
    CHECK_APPROX(b.boxMax.z, 3.0, 1e-9);
    CHECK(b.radius > 3.0);                 // the sphere is much fatter
}

TEST_CASE(frustum_aabb_basic) {
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 0.1, 100.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    // Box at the origin, in front of the camera — visible.
    CHECK(f.containsAABB(Vec3(-0.5, -0.5, -0.5), Vec3(0.5, 0.5, 0.5)));
    // Box behind the camera — culled.
    CHECK(!f.containsAABB(Vec3(-0.5, -0.5, 9.5), Vec3(0.5, 0.5, 10.5)));
    // Box far to the side — culled.
    CHECK(!f.containsAABB(Vec3(99.5, -0.5, -0.5), Vec3(100.5, 0.5, 0.5)));
    // Box beyond the far plane — culled.
    CHECK(!f.containsAABB(Vec3(-0.5, -0.5, -200.0), Vec3(0.5, 0.5, -150.0)));
}

TEST_CASE(frustum_aabb_beats_sphere_for_flat_terrain) {
    // The motivating case (ADR-0034 Phase 1): a long, thin, flat slab sitting off
    // to the right of the view. It is entirely outside the right frustum plane, but
    // its bounding sphere is huge (radius ~ half its 400-unit length) and crosses
    // that plane, so the sphere test wrongly keeps it; the tight box culls it.
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(degreesToRadians(60), 1.0, 0.1, 1000.0);
    Frustum f = Frustum::fromViewProjection(proj * view);

    // x in [128,132] stays right of the frustum edge (≈118 at the deepest z here);
    // z spans ±200 so the bounding sphere radius is ≈200, far larger than the box.
    Vec3 bmin(128.0, -0.05, -200.0), bmax(132.0, 0.05, 200.0);
    Vec3 center((bmin.x + bmax.x) * 0.5, 0.0, 0.0);
    Real radius = (bmax - bmin).length() * 0.5;

    CHECK(f.containsSphere(center, radius));     // sphere is fooled
    CHECK(!f.containsAABB(bmin, bmax));          // box is not
}

TEST_CASE(transformed_aabb_translation_and_scale) {
    Vec3 outMin, outMax;
    // Unit box scaled 2x and translated by (10, 0, -5).
    Mat4 m = Mat4::trs(Vec3(10, 0, -5), Quat::identity(), Vec3(2, 2, 2));
    transformedAABB(m, Vec3(-1, -1, -1), Vec3(1, 1, 1), outMin, outMax);
    CHECK_APPROX(outMin.x, 8.0, EPS);
    CHECK_APPROX(outMax.x, 12.0, EPS);
    CHECK_APPROX(outMin.z, -7.0, EPS);
    CHECK_APPROX(outMax.z, -3.0, EPS);
    CHECK_APPROX(outMin.y, -2.0, EPS);
    CHECK_APPROX(outMax.y, 2.0, EPS);
}

TEST_CASE(transformed_aabb_rotation_grows_box) {
    Vec3 outMin, outMax;
    // 45° about Y turns a unit box into one spanning ±sqrt(2) in x and z.
    Mat4 m = Mat4::trs(Vec3(0, 0, 0),
                       Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(45)),
                       Vec3(1, 1, 1));
    transformedAABB(m, Vec3(-1, -1, -1), Vec3(1, 1, 1), outMin, outMax);
    CHECK_APPROX(outMax.x, std::sqrt(2.0), 1e-6);
    CHECK_APPROX(outMin.x, -std::sqrt(2.0), 1e-6);
    CHECK_APPROX(outMax.y, 1.0, 1e-6);   // unchanged about the rotation axis
}
