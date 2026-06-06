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
