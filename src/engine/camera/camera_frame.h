#ifndef RAYTRACER_ENGINE_CAMERA_FRAME_H
#define RAYTRACER_ENGINE_CAMERA_FRAME_H

// FRAMING A THING (Glenn, 2026-11-07: "find the bounding box of the gas station and then make sure it fits in the
// camera's frustum ... some function to assist placing a camera for screenshots or even cinematic purposes"). Given
// a world box and the way to look at it (pitch and yaw in the fly camera's convention: yaw 0 looks down -Z, positive
// pitch up), the eye that puts every corner of the box inside the view -- the vertical field of view and the aspect
// the frame is drawn at, less a margin -- looking at the box's middle. Pure, so the tests can hold it.

#include "../../rt_math.h"
#include <algorithm>
#include <cmath>

namespace engine {

// The fly camera's look direction for a pitch and yaw (degrees).
inline Vec3 flyForward(Real pitchDeg, Real yawDeg) {
    const Real p = pitchDeg * 3.14159265358979323846 / 180.0, y = yawDeg * 3.14159265358979323846 / 180.0;
    return Vec3(std::sin(y) * std::cos(p), std::sin(p), -std::cos(y) * std::cos(p));
}

// The eye framing [lo, hi]: `fill` is the share of the frame the box may take (0.85: a little room round it).
inline Vec3 frameBox(const Vec3& lo, const Vec3& hi, Real pitchDeg, Real yawDeg, Real vfovDeg, Real aspect,
                     Real fill = 0.85) {
    const Vec3 c = (lo + hi) * 0.5;
    const Vec3 f = flyForward(pitchDeg, yawDeg);
    Vec3 r = cross(f, Vec3(0, 1, 0));
    r = r.length() > 1e-6 ? r * (1.0 / r.length()) : Vec3(1, 0, 0);   // (straight down: any right will do)
    const Vec3 u = cross(r, f);
    const Real ty = std::tan(vfovDeg * 3.14159265358979323846 / 360.0) * fill;
    const Real tx = ty * std::max(Real(0.1), aspect);
    Real d = 0;
    for (int k = 0; k < 8; ++k) {
        const Vec3 p((k & 1) ? hi.x : lo.x, (k & 2) ? hi.y : lo.y, (k & 4) ? hi.z : lo.z);
        const Vec3 q = p - c;
        const Real x = std::fabs(dot(q, r)), y = std::fabs(dot(q, u)), z = dot(q, f);
        d = std::max(d, std::max(x / tx, y / ty) - z);   // its corner at depth d + z must fit the half-angle
    }
    return c - f * std::max(d, Real(0.5));
}

}  // namespace engine

#endif
