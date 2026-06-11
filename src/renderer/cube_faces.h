#ifndef RAYTRACER_RENDERER_CUBE_FACES_H
#define RAYTRACER_RENDERER_CUBE_FACES_H

#include "../rt_math.h"

namespace engine {

// Cubemap face cameras matching Metal's cube-sampling convention (ADR-0017
// Phase 3). The convention (same as D3D/Vulkan) defines, for texel (u, v) in
// [0,1]² with v = 0 at the top row and u' = 2u−1, v' = 2v−1:
//
//   +X: ( 1, -v', -u')   -X: (-1, -v',  u')
//   +Y: ( u',  1,  v')   -Y: ( u', -1, -v')
//   +Z: ( u', -v',  1)   -Z: (-u', -v', -1)
//
// A face's texel basis is LEFT-handed when viewed from inside the cube, so a
// right-handed lookAt camera always renders a mirrored face. The fix is the
// standard one: per-face forward/up below PLUS an X-mirrored projection
// (cubeFaceViewProjection negates the projection's first row). The mirror
// flips winding, so passes that cull must use counterclockwise front faces.
// tests/test_cube_faces.cpp proves the reconstruction matches the table.

struct CubeFaceBasis {
    Vec3 forward;
    Vec3 up;
};

inline CubeFaceBasis cubeFaceBasis(int face) {
    static const CubeFaceBasis FACES[6] = {
        {{ 1, 0, 0}, {0, 1, 0}},   // +X
        {{-1, 0, 0}, {0, 1, 0}},   // -X
        {{0,  1, 0}, {0, 0, -1}},  // +Y
        {{0, -1, 0}, {0, 0, 1}},   // -Y
        {{0, 0,  1}, {0, 1, 0}},   // +Z
        {{0, 0, -1}, {0, 1, 0}},   // -Z
    };
    return FACES[face];
}

// Direction a cube sample fetches for texel (u, v) of `face` — the convention
// table above. u, v in [0,1], v = 0 at the top row.
inline Vec3 cubeFaceDirection(int face, Real u, Real v) {
    Real x = u * 2.0 - 1.0;
    Real y = v * 2.0 - 1.0;
    switch (face) {
        case 0:  return normalize(Vec3( 1, -y, -x));
        case 1:  return normalize(Vec3(-1, -y,  x));
        case 2:  return normalize(Vec3( x,  1,  y));
        case 3:  return normalize(Vec3( x, -1, -y));
        case 4:  return normalize(Vec3( x, -y,  1));
        default: return normalize(Vec3(-x, -y, -1));
    }
}

// View-projection for rendering one cube face from `eye`: 90° FOV, square
// aspect, X-mirrored projection (see header comment). Inverting this and
// reconstructing directions per pixel (as vertexSkybox does) yields exactly
// cubeFaceDirection for every texel.
inline Mat4 cubeFaceViewProjection(int face, const Vec3& eye,
                                   Real nearPlane, Real farPlane) {
    CubeFaceBasis basis = cubeFaceBasis(face);
    Mat4 proj = Mat4::perspective(degreesToRadians(90.0), 1.0,
                                  nearPlane, farPlane);
    proj.m[0][0] = -proj.m[0][0];   // mirror X to match the left-handed face basis
    Mat4 view = Mat4::lookAt(eye, eye + basis.forward, basis.up);
    return proj * view;
}

}  // namespace engine

#endif
