#include "triangle_bvh.h"
#include "profile.h"
#include <algorithm>
#include <numeric>

namespace engine {

void TriangleBVH::build(const std::vector<Triangle>& triangles) {
    RT_PROFILE_ZONE_NAMED("triangleBvhBuild");
    if (triangles.empty()) {
        root = nullptr;
        return;
    }

    std::vector<int> indices(triangles.size());
    std::iota(indices.begin(), indices.end(), 0);
    root = buildRecursive(triangles, indices, 0);
}

std::unique_ptr<TriangleBvhNode> TriangleBVH::buildRecursive(const std::vector<Triangle>& triangles,
                                               const std::vector<int>& indices, int depth) {
    auto node = std::make_unique<TriangleBvhNode>();

    for (int idx : indices) {
        node->bounds.expand(triangleBounds(triangles[idx]));
    }

    if (static_cast<int>(indices.size()) <= MIN_TRIANGLES || depth >= MAX_DEPTH) {
        node->triangleIndices = indices;
        return node;
    }

    int axis = node->bounds.longestAxis();

    std::vector<int> sorted = indices;
    std::sort(sorted.begin(), sorted.end(), [&](int a, int b) {
        const Triangle& triA = triangles[a];
        const Triangle& triB = triangles[b];
        double centroidA = ((&triA.v0.x)[axis] + (&triA.v1.x)[axis] + (&triA.v2.x)[axis]) / 3.0;
        double centroidB = ((&triB.v0.x)[axis] + (&triB.v1.x)[axis] + (&triB.v2.x)[axis]) / 3.0;
        return centroidA < centroidB;
    });

    int mid = static_cast<int>(sorted.size()) / 2;
    std::vector<int> leftIndices(sorted.begin(), sorted.begin() + mid);
    std::vector<int> rightIndices(sorted.begin() + mid, sorted.end());

    node->left = buildRecursive(triangles, leftIndices, depth + 1);
    node->right = buildRecursive(triangles, rightIndices, depth + 1);

    return node;
}

bool TriangleBVH::intersect(const std::vector<Triangle>& triangles, const Ray& ray,
                       double tMin, double tMax, HitRecord& rec) const {
    if (!root) return false;
    return intersectNode(triangles, root.get(), ray, tMin, tMax, rec);
}

bool TriangleBVH::intersectNode(const std::vector<Triangle>& triangles, const TriangleBvhNode* node,
                           const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    if (!node->bounds.intersect(ray, tMin, tMax)) return false;

    if (node->isLeaf()) {
        bool hitAnything = false;
        for (int idx : node->triangleIndices) {
            HitRecord tempRec;
            if (triangles[idx].intersect(ray, tMin, tMax, tempRec)) {
                hitAnything = true;
                tMax = tempRec.t;
                rec = tempRec;
            }
        }
        return hitAnything;
    }

    bool hitLeft = intersectNode(triangles, node->left.get(), ray, tMin, tMax, rec);
    if (hitLeft) tMax = rec.t;
    bool hitRight = intersectNode(triangles, node->right.get(), ray, tMin, tMax, rec);

    return hitLeft || hitRight;
}

}  // namespace engine

