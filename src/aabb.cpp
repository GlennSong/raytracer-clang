#include "aabb.h"
#include <algorithm>

namespace engine {

void AABB::expand(const Vec3& point) {
    min.x = std::min(min.x, point.x);
    min.y = std::min(min.y, point.y);
    min.z = std::min(min.z, point.z);
    max.x = std::max(max.x, point.x);
    max.y = std::max(max.y, point.y);
    max.z = std::max(max.z, point.z);
}

void AABB::expand(const AABB& other) {
    expand(other.min);
    expand(other.max);
}

bool AABB::intersect(const Ray& ray, double tMin, double tMax) const {
    for (int axis = 0; axis < 3; axis++) {
        double origin = (&ray.origin.x)[axis];
        double dir = (&ray.direction.x)[axis];
        double lo = (&min.x)[axis];
        double hi = (&max.x)[axis];

        double invD = 1.0 / dir;
        double t0 = (lo - origin) * invD;
        double t1 = (hi - origin) * invD;
        if (invD < 0) std::swap(t0, t1);

        tMin = std::max(tMin, t0);
        tMax = std::min(tMax, t1);
        if (tMax < tMin) return false;
    }
    return true;
}

int AABB::longestAxis() const {
    Vec3 extent = max - min;
    if (extent.x > extent.y && extent.x > extent.z) return 0;
    if (extent.y > extent.z) return 1;
    return 2;
}

double AABB::surfaceArea() const {
    Vec3 d = max - min;
    return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
}

AABB triangleBounds(const Triangle& tri) {
    AABB box;
    box.expand(tri.v0);
    box.expand(tri.v1);
    box.expand(tri.v2);
    return box;
}

}  // namespace engine
