#ifndef RAYTRACER_APPS_CITYSIM_SCREEN_PROJECT_H
#define RAYTRACER_APPS_CITYSIM_SCREEN_PROJECT_H

#include "../../rt_math.h"                       // engine::Mat4, Vec3, Real
#include "../../engine/procgen/city/polygon.h"   // engine::Vec2 (2D pixel out)

namespace citysim {

using engine::Mat4;
using engine::Real;
using engine::Vec2;
using engine::Vec3;

// Project a world point to viewport pixel coordinates through `viewProj`
// (projection * view) for a `w`×`h` viewport. Returns false — leaving `outPx`
// untouched — when the point is at or behind the camera (clip w ≤ 0) or lands
// outside the frustum; only a true result yields a usable pixel position.
//
// Pure math (no ImGui, no camera object, no GPU) so the Living City debug labels
// that draw at each place's world position can be unit-tested headless. Pixel Y
// is flipped to screen convention (0 at the top).
inline bool worldToScreen(const Mat4& viewProj, Vec3 world, Real w, Real h,
                          Vec2& outPx) {
    const auto& m = viewProj.m;
    // Clip-space w = the perspective row (row 3 in this row-major Mat4, which
    // transformPoint divides by). ≤ 0 means at/behind the near plane — a point
    // there would project to a mirrored ghost on screen, so cull it.
    const Real cw = m[3][0] * world.x + m[3][1] * world.y + m[3][2] * world.z +
                    m[3][3];
    if (cw <= 1e-6) return false;
    const Vec3 ndc = viewProj.transformPoint(world);   // perspective divide
    if (ndc.x < -1 || ndc.x > 1 || ndc.y < -1 || ndc.y > 1) return false;
    outPx.x = (ndc.x * 0.5 + 0.5) * w;
    outPx.y = (1.0 - (ndc.y * 0.5 + 0.5)) * h;   // flip Y: pixel 0 is the top
    return true;
}

}  // namespace citysim

#endif
