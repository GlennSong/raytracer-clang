#ifndef RAYTRACER_AABB_H
#define RAYTRACER_AABB_H

#include "rt_math.h"
#include "geometry.h"

namespace engine {

// The path tracer's axis-aligned box: the triangle BVH's and the instance BVH's node bounds.
struct AABB {
    Vec3 min;
    Vec3 max;

    AABB() : min(1e20, 1e20, 1e20), max(-1e20, -1e20, -1e20) {}
    AABB(const Vec3& min, const Vec3& max) : min(min), max(max) {}

    void expand(const Vec3& point);
    void expand(const AABB& other);
    bool intersect(const Ray& ray, double tMin, double tMax) const;
    int longestAxis() const;
    double surfaceArea() const;
};

AABB triangleBounds(const Triangle& tri);

}  // namespace engine

#endif
