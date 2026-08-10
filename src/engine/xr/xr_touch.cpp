#include "xr_touch.h"

#include <algorithm>

namespace engine {

Vec3 xrNearestPointOnBox(const Vec3& halfExtent, const Vec3& center,
                         const Quat& orientation, const Vec3& p) {
    // Into the box's local frame (conjugate rotation — unit quaternion),
    // clamp per axis, back out. A point already inside clamps to itself.
    const Vec3 local = orientation.conjugate().rotate(p - center);
    const Vec3 clamped(
        std::clamp(local.x, -halfExtent.x, halfExtent.x),
        std::clamp(local.y, -halfExtent.y, halfExtent.y),
        std::clamp(local.z, -halfExtent.z, halfExtent.z));
    return center + orientation.rotate(clamped);
}

Vec3 xrNearestPointOnSphere(Real radius, const Vec3& center, const Vec3& p) {
    const Vec3 d = p - center;
    const Real len = d.length();
    if (len <= radius || len < 1e-12) return p;   // inside (or dead centre)
    return center + d * (radius / len);
}

}  // namespace engine
