#include "instance.h"

#include <algorithm>

namespace engine {

void MeshProto::build() {
    localBounds = AABB();
    for (const Triangle& t : triangles) localBounds.expand(triangleBounds(t));
    if (!triangles.empty()) blas.build(triangles);
}

bool MeshProto::intersectLocal(const Ray& localRay, double tMin, double tMax,
                               HitRecord& rec) const {
    if (!blas.isEmpty())
        return blas.intersect(triangles, localRay, tMin, tMax, rec);
    bool hit = false;
    double closest = tMax;
    HitRecord tmp;
    for (const Triangle& t : triangles) {
        if (t.intersect(localRay, tMin, closest, tmp)) {
            hit = true;
            closest = tmp.t;
            rec = tmp;
        }
    }
    return hit;
}

SceneInstance makeInstance(int proto, const Mat4& toWorld, const MeshProto& p) {
    SceneInstance inst;
    inst.proto = proto;
    inst.toWorld = toWorld;
    inst.toLocal = toWorld.inverse();
    inst.normalToWorld = inst.toLocal.transpose();
    // World AABB: transform the eight local-bound corners and bound them.
    const Vec3& lo = p.localBounds.min;
    const Vec3& hi = p.localBounds.max;
    for (int c = 0; c < 8; ++c) {
        Vec3 corner((c & 1) ? hi.x : lo.x, (c & 2) ? hi.y : lo.y,
                    (c & 4) ? hi.z : lo.z);
        inst.worldBounds.expand(toWorld.transformPoint(corner));
    }
    return inst;
}

namespace {
Vec3 aabbCentroid(const AABB& b) { return (b.min + b.max) * 0.5; }
}  // namespace

int InstanceBVH::buildRecursive(const std::vector<SceneInstance>& instances,
                                std::vector<int>& idx, int begin, int end,
                                int depth) {
    int nodeIndex = static_cast<int>(nodes_.size());
    nodes_.push_back(Node{});
    AABB bounds;
    for (int i = begin; i < end; ++i) bounds.expand(instances[idx[i]].worldBounds);

    int count = end - begin;
    // Leaf: few instances or too deep.
    if (count <= 2 || depth > 32) {
        Node leaf;
        leaf.bounds = bounds;
        leaf.leaf = true;
        leaf.start = begin;
        leaf.count = count;
        nodes_[nodeIndex] = leaf;
        return nodeIndex;
    }

    // Split on the longest axis of the centroid bounds at the median.
    AABB cbounds;
    for (int i = begin; i < end; ++i)
        cbounds.expand(aabbCentroid(instances[idx[i]].worldBounds));
    int axis = cbounds.longestAxis();
    auto comp = [&](int a, int b) {
        Vec3 ca = aabbCentroid(instances[a].worldBounds);
        Vec3 cb = aabbCentroid(instances[b].worldBounds);
        return (axis == 0 ? ca.x : axis == 1 ? ca.y : ca.z) <
               (axis == 0 ? cb.x : axis == 1 ? cb.y : cb.z);
    };
    int mid = begin + count / 2;
    std::nth_element(idx.begin() + begin, idx.begin() + mid, idx.begin() + end, comp);

    int left = buildRecursive(instances, idx, begin, mid, depth + 1);
    int right = buildRecursive(instances, idx, mid, end, depth + 1);
    Node node;
    node.bounds = bounds;
    node.left = left;
    node.right = right;
    nodes_[nodeIndex] = node;
    return nodeIndex;
}

void InstanceBVH::build(const std::vector<SceneInstance>& instances) {
    nodes_.clear();
    order_.clear();
    if (instances.empty()) return;
    order_.resize(instances.size());
    for (std::size_t i = 0; i < instances.size(); ++i)
        order_[i] = static_cast<int>(i);
    nodes_.reserve(instances.size() * 2);
    buildRecursive(instances, order_, 0, static_cast<int>(order_.size()), 0);
}

bool InstanceBVH::intersect(const std::vector<SceneInstance>& instances,
                            const std::vector<MeshProto>& protos, const Ray& ray,
                            double tMin, double tMax, HitRecord& rec) const {
    if (nodes_.empty()) return false;
    bool hit = false;
    double closest = tMax;

    // Iterative traversal with an explicit stack (avoids deep recursion per ray).
    int stack[64];
    int sp = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node& node = nodes_[stack[--sp]];
        if (!node.bounds.intersect(ray, tMin, closest)) continue;
        if (node.leaf) {
            for (int i = node.start; i < node.start + node.count; ++i) {
                const SceneInstance& inst = instances[order_[i]];
                if (inst.proto < 0) continue;
                const MeshProto& proto = protos[inst.proto];
                // Map the world ray into the proto frame. Direction is NOT
                // renormalised, so `t` is identical in both frames.
                Ray localRay(inst.toLocal.transformPoint(ray.origin),
                             inst.toLocal.transformDirection(ray.direction));
                HitRecord tmp;
                if (proto.intersectLocal(localRay, tMin, closest, tmp)) {
                    hit = true;
                    closest = tmp.t;
                    rec = tmp;
                    rec.point = inst.toWorld.transformPoint(tmp.point);
                    rec.normal = normalize(inst.normalToWorld.transformDirection(tmp.normal));
                    if (tmp.tangent.lengthSquared() > 1e-12)
                        rec.tangent =
                            normalize(inst.toWorld.transformDirection(tmp.tangent));
                    rec.t = tmp.t;
                }
            }
        } else {
            if (sp + 2 <= 64) {
                stack[sp++] = node.left;
                stack[sp++] = node.right;
            }
        }
    }
    return hit;
}

}  // namespace engine
