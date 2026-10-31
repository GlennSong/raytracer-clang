#ifndef RAYTRACER_TRIANGLE_BVH_H
#define RAYTRACER_TRIANGLE_BVH_H

#include "aabb.h"
#include "geometry.h"
#include <vector>
#include <memory>

namespace engine {

struct TriangleBvhNode {
    AABB bounds;
    std::unique_ptr<TriangleBvhNode> left;
    std::unique_ptr<TriangleBvhNode> right;
    std::vector<int> triangleIndices;

    bool isLeaf() const { return !left && !right; }
};

// A BVH over a scene's loose triangles: each node splits its triangles at the MEDIAN centroid along the longest
// axis of its bounds (so it is a bounding-volume hierarchy, not a k-d tree: children overlap, space is not split).
// Named KdTree until #117 wave 0.
class TriangleBVH {
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

    std::unique_ptr<TriangleBvhNode> root;

    std::unique_ptr<TriangleBvhNode> buildRecursive(const std::vector<Triangle>& triangles,
                                           const std::vector<int>& indices, int depth);
    bool intersectNode(const std::vector<Triangle>& triangles, const TriangleBvhNode* node,
                       const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
};


}  // namespace engine

#endif
