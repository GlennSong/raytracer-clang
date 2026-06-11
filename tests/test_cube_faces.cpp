// Proves the cube-face bake cameras match Metal's cube-sampling convention
// (ADR-0017 Phase 3). Replays the shader's direction reconstruction — clip
// position through the inverse view-projection, exactly what vertexSkybox
// does per pixel — and checks the result against cubeFaceDirection for every
// face. If these agree, a face rendered with cubeFaceViewProjection and then
// sampled as a cube returns the radiance that was baked for that direction.

#include "test_framework.h"
#include "../src/renderer/cube_faces.h"

using namespace engine;

namespace {

// Reconstruct the world direction rendered at texel (u, v): texel → NDC
// (v = 0 is the top row, NDC y = +1), then invVP at the far plane, minus the
// eye — the same math as vertexSkybox.
Vec3 reconstructBakedDirection(const Mat4& invVP, const Vec3& eye,
                               Real u, Real v) {
    Real ndcX = u * 2.0 - 1.0;
    Real ndcY = 1.0 - v * 2.0;
    Real clip[4] = {ndcX, ndcY, 1.0, 1.0};
    Real world[4] = {0, 0, 0, 0};
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++)
            world[r] += invVP.m[r][c] * clip[c];
    Vec3 p(world[0] / world[3], world[1] / world[3], world[2] / world[3]);
    return normalize(p - eye);
}

}  // namespace

TEST_CASE(cube_face_cameras_reproduce_sampling_convention) {
    const Vec3 eye(0, 0, 0);
    const Real samples[] = {0.05, 0.3, 0.5, 0.75, 0.95};

    for (int face = 0; face < 6; face++) {
        Mat4 vp = cubeFaceViewProjection(face, eye, 0.1, 10.0);
        Mat4 invVP = vp.inverse();

        for (Real u : samples) {
            for (Real v : samples) {
                Vec3 baked = reconstructBakedDirection(invVP, eye, u, v);
                Vec3 expected = cubeFaceDirection(face, u, v);
                CHECK(dot(baked, expected) > 0.9999);
            }
        }
    }
}

TEST_CASE(cube_face_cameras_work_away_from_origin) {
    const Vec3 eye(3, 1, -2);   // probe-style bake from a scene position
    Mat4 vp = cubeFaceViewProjection(4, eye, 0.1, 200.0);  // +Z face
    Mat4 invVP = vp.inverse();

    Vec3 baked = reconstructBakedDirection(invVP, eye, 0.5, 0.5);
    Vec3 expected = cubeFaceDirection(4, 0.5, 0.5);
    CHECK(dot(baked, expected) > 0.9999);
}

TEST_CASE(cube_face_directions_cover_all_axes_at_centers) {
    CHECK_APPROX(cubeFaceDirection(0, 0.5, 0.5).x,  1.0, 1e-9);
    CHECK_APPROX(cubeFaceDirection(1, 0.5, 0.5).x, -1.0, 1e-9);
    CHECK_APPROX(cubeFaceDirection(2, 0.5, 0.5).y,  1.0, 1e-9);
    CHECK_APPROX(cubeFaceDirection(3, 0.5, 0.5).y, -1.0, 1e-9);
    CHECK_APPROX(cubeFaceDirection(4, 0.5, 0.5).z,  1.0, 1e-9);
    CHECK_APPROX(cubeFaceDirection(5, 0.5, 0.5).z, -1.0, 1e-9);
}
