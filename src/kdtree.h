#ifndef RAYTRACER_KDTREE_H
#define RAYTRACER_KDTREE_H

#include "rt_math.h"
#include "geometry.h"
#include <vector>
#include <memory>

namespace engine {

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

struct KdNode {
    AABB bounds;
    std::unique_ptr<KdNode> left;
    std::unique_ptr<KdNode> right;
    std::vector<int> triangleIndices;

    bool isLeaf() const { return !left && !right; }
};

class KdTree {
public:
    void build(const std::vector<Triangle>& triangles);
    // The live triangle array is supplied at query time rather than cached as a
    // pointer: the tree stores only indices, so it stays valid when the owning
    // Scene is copied or moved (a cached pointer would dangle to the source's
    // vector). The caller passes the SAME vector it built from.
    bool intersect(const std::vector<Triangle>& triangles, const Ray& ray,
                   double tMin, double tMax, HitRecord& rec) const;
    bool isEmpty() const { return !root; }

private:
    static const int MAX_DEPTH = 25;
    static const int MIN_TRIANGLES = 4;

    std::unique_ptr<KdNode> root;

    std::unique_ptr<KdNode> buildRecursive(const std::vector<Triangle>& triangles,
                                           const std::vector<int>& indices, int depth);
    bool intersectNode(const std::vector<Triangle>& triangles, const KdNode* node,
                       const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
};


}  // namespace engine

#endif
