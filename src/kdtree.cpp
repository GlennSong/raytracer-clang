#include "kdtree.h"
#include <algorithm>
#include <numeric>

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

void KdTree::build(const std::vector<Triangle>& triangles) {
    triangleData = &triangles;
    if (triangles.empty()) {
        root = nullptr;
        return;
    }

    std::vector<int> indices(triangles.size());
    std::iota(indices.begin(), indices.end(), 0);
    root = buildRecursive(indices, 0);
}

std::unique_ptr<KdNode> KdTree::buildRecursive(const std::vector<int>& indices, int depth) {
    auto node = std::make_unique<KdNode>();

    for (int idx : indices) {
        node->bounds.expand(triangleBounds((*triangleData)[idx]));
    }

    if (static_cast<int>(indices.size()) <= MIN_TRIANGLES || depth >= MAX_DEPTH) {
        node->triangleIndices = indices;
        return node;
    }

    int axis = node->bounds.longestAxis();

    std::vector<int> sorted = indices;
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        const Triangle& triA = (*triangleData)[a];
        const Triangle& triB = (*triangleData)[b];
        double centroidA = ((&triA.v0.x)[axis] + (&triA.v1.x)[axis] + (&triA.v2.x)[axis]) / 3.0;
        double centroidB = ((&triB.v0.x)[axis] + (&triB.v1.x)[axis] + (&triB.v2.x)[axis]) / 3.0;
        return centroidA < centroidB;
    });

    int mid = static_cast<int>(sorted.size()) / 2;
    std::vector<int> leftIndices(sorted.begin(), sorted.begin() + mid);
    std::vector<int> rightIndices(sorted.begin() + mid, sorted.end());

    node->left = buildRecursive(leftIndices, depth + 1);
    node->right = buildRecursive(rightIndices, depth + 1);

    return node;
}

bool KdTree::intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    if (!root) return false;
    return intersectNode(root.get(), ray, tMin, tMax, rec);
}

bool KdTree::intersectNode(const KdNode* node, const Ray& ray,
                           double tMin, double tMax, HitRecord& rec) const {
    if (!node->bounds.intersect(ray, tMin, tMax)) return false;

    if (node->isLeaf()) {
        bool hitAnything = false;
        for (int idx : node->triangleIndices) {
            HitRecord tempRec;
            if ((*triangleData)[idx].intersect(ray, tMin, tMax, tempRec)) {
                hitAnything = true;
                tMax = tempRec.t;
                rec = tempRec;
            }
        }
        return hitAnything;
    }

    bool hitLeft = intersectNode(node->left.get(), ray, tMin, tMax, rec);
    if (hitLeft) tMax = rec.t;
    bool hitRight = intersectNode(node->right.get(), ray, tMin, tMax, rec);

    return hitLeft || hitRight;
}

}  // namespace engine

