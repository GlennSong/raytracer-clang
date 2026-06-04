#ifndef RAYTRACER_KDTREE_H
#define RAYTRACER_KDTREE_H

#include "rt_math.h"
#include "geometry.h"
#include <vector>
#include <memory>

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
    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
    bool isEmpty() const { return !root; }

private:
    static const int MAX_DEPTH = 25;
    static const int MIN_TRIANGLES = 4;

    std::unique_ptr<KdNode> root;
    const std::vector<Triangle>* triangleData = nullptr;

    std::unique_ptr<KdNode> buildRecursive(const std::vector<int>& indices, int depth);
    bool intersectNode(const KdNode* node, const Ray& ray,
                       double tMin, double tMax, HitRecord& rec) const;
};

#endif
