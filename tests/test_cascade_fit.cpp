#include "test_framework.h"

#include "../src/renderer/cascade_fit.h"

using namespace engine;

// The shared cascaded-shadow fit (cascade_fit.h) — the math all render
// backends use. Invariants, not golden values: split monotonicity, slice
// coverage by the fitted box, texel snapping, and camera-centered stability.

namespace {
CascadeFitInput makeInput() {
    CascadeFitInput in;
    Real aspect = 16.0 / 9.0;
    Mat4 proj = Mat4::perspective(60.0 * M_PI / 180.0, aspect, 0.1, 500.0);
    Mat4 view = Mat4::lookAt(Vec3(10, 5, 20), Vec3(0, 0, 0), Vec3(0, 1, 0));
    in.camViewProj = proj * view;
    in.cameraEye = Vec3(10, 5, 20);
    in.sunDir = normalize(Vec3(0.4, 0.8, 0.3));
    in.camNear = 0.1;
    in.camFar = 500.0;
    in.shadowDistance = 150.0;
    in.splitLambda = 0.5;
    in.cascadeCount = 4;
    in.shadowMapSize = 2048;
    return in;
}
}  // namespace

TEST_CASE(cascade_fit_splits_monotonic_and_bounded) {
    CascadeFit fits[4];
    CascadeFitInput in = makeInput();
    int n = computeCascadeFits(in, fits);
    CHECK(n == 4);
    Real prev = in.camNear;
    for (int c = 0; c < n; ++c) {
        CHECK(fits[c].splitFar > prev);         // strictly increasing
        CHECK(fits[c].radius > 0);
        prev = fits[c].splitFar;
    }
    CHECK_APPROX(fits[n - 1].splitFar, in.shadowDistance, 1e-6);
    // Count clamping.
    in.cascadeCount = 9;
    CHECK(computeCascadeFits(in, fits) == 4);
    in.cascadeCount = 0;
    CHECK(computeCascadeFits(in, fits) == 1);
}

TEST_CASE(cascade_fit_box_covers_its_slice) {
    // Every slice corner must land inside its cascade's ortho box in light
    // space (that is what guarantees the receiver is shadow-mapped). Recompute
    // the slice corners independently from the public camera matrix.
    CascadeFit fits[4];
    CascadeFitInput in = makeInput();
    int n = computeCascadeFits(in, fits);
    Mat4 invVP = in.camViewProj.inverse();
    auto corner = [&](Real x, Real y, Real z) { return invVP.transformPoint(Vec3(x, y, z)); };
    Real prevFar = in.camNear;
    for (int c = 0; c < n; ++c) {
        Real fN = (prevFar - in.camNear) / (in.camFar - in.camNear);
        Real fF = (fits[c].splitFar - in.camNear) / (in.camFar - in.camNear);
        prevFar = fits[c].splitFar;
        for (int k = 0; k < 4; ++k) {
            Real xs[4] = {-1, 1, 1, -1}, ys[4] = {-1, -1, 1, 1};
            Vec3 nc = corner(xs[k], ys[k], 0), fc = corner(xs[k], ys[k], 1);
            for (Real t : {fN, fF}) {
                Vec3 p = nc + (fc - nc) * t;
                Vec3 ls = fits[c].lightView.transformPoint(p);
                // Inside the XY extents (the fitted sphere guarantees it; snap
                // shifts the box by at most one texel).
                Real slack = fits[c].radius * 2.0 / in.shadowMapSize + 1e-6;
                CHECK(std::abs(ls.x) <= fits[c].radius + slack);
                CHECK(std::abs(ls.y) <= fits[c].radius + slack);
            }
        }
    }
}

TEST_CASE(cascade_fit_center_is_texel_snapped) {
    CascadeFit fits[4];
    CascadeFitInput in = makeInput();
    int n = computeCascadeFits(in, fits);
    for (int c = 0; c < n; ++c) {
        Real texel = fits[c].radius * 2.0 / in.shadowMapSize;
        Vec3 ls = fits[c].lightView.transformPoint(fits[c].center);
        // The snapped center sits at the light-space origin of its own view
        // (lookAt targets it), so snapping shows up as: the ORIGINAL camera eye
        // maps within one texel of a whole-texel grid point.
        Vec3 eyeLS = fits[c].lightView.transformPoint(in.cameraEye);
        Real fx = std::abs(eyeLS.x - std::round(eyeLS.x / texel) * texel);
        Real fy = std::abs(eyeLS.y - std::round(eyeLS.y / texel) * texel);
        CHECK(fx <= texel * 0.5 + 1e-9);
        CHECK(fy <= texel * 0.5 + 1e-9);
        CHECK(std::abs(ls.x) < 1e-6);   // center is the view target
        CHECK(std::abs(ls.y) < 1e-6);
    }
}

TEST_CASE(cascade_fit_is_look_direction_invariant) {
    // The camera-centered fit's whole point (see the header): turning the
    // camera in place must not change any cascade's radius, and the box center
    // may shift only by texel snapping (sub-texel world distance).
    CascadeFit a[4], b[4];
    CascadeFitInput in = makeInput();
    int n = computeCascadeFits(in, a);
    Mat4 proj = Mat4::perspective(60.0 * M_PI / 180.0, 16.0 / 9.0, 0.1, 500.0);
    Mat4 turned = proj * Mat4::lookAt(in.cameraEye, in.cameraEye + Vec3(-1, 0, -1), Vec3(0, 1, 0));
    in.camViewProj = turned;
    CHECK(computeCascadeFits(in, b) == n);
    for (int c = 0; c < n; ++c) {
        CHECK_APPROX(a[c].radius, b[c].radius, 1e-9);
        Real texel = a[c].radius * 2.0 / 2048.0;
        CHECK((a[c].center - b[c].center).length() <= texel * 1.5);
    }
}
