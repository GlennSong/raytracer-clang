#ifndef RAYTRACER_RENDERER_CASCADE_FIT_H
#define RAYTRACER_RENDERER_CASCADE_FIT_H

// Cascaded-shadow-map fit: slice the camera frustum into depth bands and fit a
// texel-snapped, camera-centered orthographic light box to each. Pure math —
// no graphics-API types — shared by the render backends so the (previously
// triplicated) logic lives, and is unit-tested, exactly once. Header-only so
// Objective-C++ (metal_renderer.mm) can include it as-is.
//
// Conventions (ADR-0017 shadows; see also each backend's AGENTS.md):
// - CAMERA-CENTERED fit, not slice-centroid: the covered region depends only on
//   the camera position — never the look direction — so shadows don't pop or
//   shimmer as you turn. The cost is ~2x the minimal box extent; deliberate.
// - Radius is quantized and the light-space center is snapped to shadow-map
//   texels, so a moving camera slides the box in whole-texel steps.
// - The NDC depth range is a parameter: WebGPU/Metal use standard-Z (near 0,
//   far 1); the Vulkan backend uses reverse-Z (near 1, far 0).

#include "../rt_math.h"

#include <algorithm>
#include <cmath>

namespace engine {

// One fitted cascade — everything a backend needs to render and cull it.
struct CascadeFit {
    Mat4 viewProj;     // lightProj * lightView: upload for the shadow pass
    Mat4 lightView;    // light view alone: in its space the box XY is ±radius
    Vec3 center;       // snapped world-space box center (probe/cull anchor)
    Real radius = 0;   // orthographic half-extent
    Real splitFar = 0; // view-space far depth of the slice (cascade selection)
};

struct CascadeFitInput {
    Mat4 camViewProj;          // camera view-projection to slice
    Vec3 cameraEye;            // fit anchor (see camera-centered note above)
    Vec3 sunDir;               // normalized direction from the scene TOWARD the sun
    Real camNear = 0.1;
    Real camFar = 1000.0;
    Real shadowDistance = 150; // furthest shadowed view depth
    Real splitLambda = 0.5;    // 0 = uniform splits, 1 = logarithmic
    int cascadeCount = 4;      // clamped to [1, 4]
    int shadowMapSize = 2048;  // texel-snap granularity
    Real ndcNearZ = 0.0;       // NDC depth of the near plane (Vulkan reverse-Z: 1)
    Real ndcFarZ = 1.0;        // NDC depth of the far plane  (Vulkan reverse-Z: 0)
};

// Fills out[0..count) and returns count (the clamped cascade total).
inline int computeCascadeFits(const CascadeFitInput& in, CascadeFit out[4]) {
    const int cc = std::max(1, std::min(in.cascadeCount, 4));
    const Real shadowDist =
        std::max(std::min(in.shadowDistance, in.camFar), in.camNear * 2.0);

    const Vec3 up = std::abs(in.sunDir.y) > 0.99 ? Vec3(0, 0, 1) : Vec3(0, 1, 0);
    const Mat4 invVP = in.camViewProj.inverse();
    auto corner = [&](Real x, Real y, Real z) { return invVP.transformPoint(Vec3(x, y, z)); };
    const Vec3 nearC[4] = {corner(-1, -1, in.ndcNearZ), corner(1, -1, in.ndcNearZ),
                           corner(1, 1, in.ndcNearZ), corner(-1, 1, in.ndcNearZ)};
    const Vec3 farC[4] = {corner(-1, -1, in.ndcFarZ), corner(1, -1, in.ndcFarZ),
                          corner(1, 1, in.ndcFarZ), corner(-1, 1, in.ndcFarZ)};

    Real prevFar = in.camNear;
    for (int c = 0; c < cc; ++c) {
        // Practical split scheme: blend uniform and logarithmic by splitLambda.
        Real f = Real(c + 1) / Real(cc);
        Real uni = in.camNear + (shadowDist - in.camNear) * f;
        Real lg = in.camNear * std::pow(shadowDist / in.camNear, f);
        Real zFar = in.splitLambda * lg + (1.0 - in.splitLambda) * uni;
        Real zNear = prevFar;
        prevFar = zFar;

        // Slice corners by lerping the near->far frustum edges.
        Real fN = (zNear - in.camNear) / (in.camFar - in.camNear);
        Real fF = (zFar - in.camNear) / (in.camFar - in.camNear);
        Vec3 corners[8];
        for (int k = 0; k < 4; ++k) {
            corners[k] = nearC[k] + (farC[k] - nearC[k]) * fN;
            corners[k + 4] = nearC[k] + (farC[k] - nearC[k]) * fF;
        }

        // Camera-centered sphere over the slice; quantized so the radius (and
        // with it texelWorld) doesn't jitter frame to frame.
        Vec3 center = in.cameraEye;
        Real radius = 0.01;
        for (const Vec3& p : corners) radius = std::max(radius, (p - center).length());
        radius = std::ceil(radius * 16.0) / 16.0;

        // Pull the light back past the sphere so tall casters above the box
        // still register; snap the center to whole shadow-map texels.
        Real pullback = radius + 50.0;
        Real texelWorld = (radius * 2.0) / static_cast<Real>(in.shadowMapSize);
        Mat4 lightView = Mat4::lookAt(center + in.sunDir * pullback, center, up);
        Vec3 centerLS = lightView.transformPoint(center);
        centerLS.x = std::round(centerLS.x / texelWorld) * texelWorld;
        centerLS.y = std::round(centerLS.y / texelWorld) * texelWorld;
        Vec3 snapped = lightView.inverse().transformPoint(centerLS);

        lightView = Mat4::lookAt(snapped + in.sunDir * pullback, snapped, up);
        Mat4 lightProj = Mat4::orthographic(radius * 2.0, 1.0, 0.1, pullback + radius);

        out[c].viewProj = lightProj * lightView;
        out[c].lightView = lightView;
        out[c].center = snapped;
        out[c].radius = radius;
        out[c].splitFar = zFar;
    }
    return cc;
}

}  // namespace engine

#endif
