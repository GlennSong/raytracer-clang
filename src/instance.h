#ifndef RAYTRACER_INSTANCE_H
#define RAYTRACER_INSTANCE_H

#include "rt_math.h"
#include "geometry.h"
#include "kdtree.h"
#include <vector>

namespace engine {

// Two-level acceleration for the path tracer (ADR-0041): a prototype mesh is
// built once into its own kd-tree (a BLAS) and placed by many instances; a
// top-level BVH (TLAS) over instance world-AABBs gates which protos a ray even
// touches. The ray is transformed into a proto's local frame rather than the
// geometry into the world, so N placements cost one BLAS, not N triangle copies.

// A prototype mesh (BLAS): triangles in LOCAL space (materialIndex into
// Scene::materials, like the flat soup), plus a kd-tree built over them once.
struct MeshProto {
    std::vector<Triangle> triangles;
    KdTree blas;
    AABB localBounds;

    void build();   // build the BLAS + local bounds from `triangles`
    // Nearest hit in the proto's own frame. `localRay` is the world ray mapped
    // by the instance's inverse — NOT renormalised, so the returned `t` is the
    // same parameter in both frames.
    bool intersectLocal(const Ray& localRay, double tMin, double tMax,
                        HitRecord& rec) const;
};

// One placement of a proto. Caches the inverse + normal matrix + world AABB so
// the per-ray hot path does no matrix inversion.
struct SceneInstance {
    int  proto = -1;
    Mat4 toWorld;        // local -> world
    Mat4 toLocal;        // world -> local (inverse of toWorld)
    Mat4 normalToWorld;  // transpose(toLocal): maps local normals to world
    AABB worldBounds;
};

// Build a placement: inverts `toWorld` once and transforms the proto's local
// bounds into a world AABB for the TLAS.
SceneInstance makeInstance(int proto, const Mat4& toWorld, const MeshProto& p);

// A top-level BVH over instance world-AABBs (median split on the longest axis).
// Stores instance indices; leaf ranges index into a reordered index array.
class InstanceBVH {
public:
    void build(const std::vector<SceneInstance>& instances);
    bool isEmpty() const { return nodes_.empty(); }

    // Nearest hit across all instances, returned in WORLD space (point, normal,
    // tangent mapped out of the proto frame). Vertex colour, UVs and the proto's
    // materialIndex are carried through unchanged.
    bool intersect(const std::vector<SceneInstance>& instances,
                   const std::vector<MeshProto>& protos,
                   const Ray& ray, double tMin, double tMax, HitRecord& rec) const;

private:
    struct Node {
        AABB bounds;
        int  left = -1, right = -1;   // child node indices (-1 at a leaf)
        int  start = 0, count = 0;    // leaf range into order_
        bool leaf = false;
    };
    std::vector<Node> nodes_;
    std::vector<int>  order_;         // instance indices

    int buildRecursive(const std::vector<SceneInstance>& instances,
                       std::vector<int>& idx, int begin, int end, int depth);
};

}  // namespace engine

#endif
