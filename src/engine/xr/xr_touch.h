// Nearest-point queries against the sandbox's grabbable shapes — the pure
// math behind precise picking and fingertip depth cues. Grab detection used
// pinch-to-CENTRE distance, which shaped every pick region like a sphere: a
// lying-down pillar could only be grabbed at its middle, and the highlight
// box didn't follow the object's orientation (device feedback, session six).
// Surface distance against the ORIENTED shape makes the pick region exactly
// the object, whatever way it rests.
//
// Pure math over rt_math types — no physics or renderer includes — so the
// clamp/rotation logic is host-tested (tests/test_xr_touch.cpp), including
// the exact lying-pillar case that motivated it.

#ifndef ENGINE_XR_TOUCH_H
#define ENGINE_XR_TOUCH_H

#include "../../rt_math.h"

namespace engine {

// Nearest point ON OR IN an oriented box to `p` (a point inside comes back
// unchanged — distance 0, which is the correct grab semantics: fingers
// inside the object are certainly touching it).
Vec3 xrNearestPointOnBox(const Vec3& halfExtent, const Vec3& center,
                         const Quat& orientation, const Vec3& p);

// Same for a sphere (orientation-free).
Vec3 xrNearestPointOnSphere(Real radius, const Vec3& center, const Vec3& p);

// Distance from `p` to the shape's surface, 0 inside. The one number both
// grab detection and the approach affordance run on.
inline Real xrBoxSurfaceDistance(const Vec3& halfExtent, const Vec3& center,
                                 const Quat& orientation, const Vec3& p) {
    return (p - xrNearestPointOnBox(halfExtent, center, orientation, p))
        .length();
}
inline Real xrSphereSurfaceDistance(Real radius, const Vec3& center,
                                    const Vec3& p) {
    const Real d = (p - center).length() - radius;
    return d > 0 ? d : 0;
}

}  // namespace engine

#endif  // ENGINE_XR_TOUCH_H
