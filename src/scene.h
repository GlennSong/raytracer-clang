#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "rt_math.h"
#include "geometry.h"
#include "material.h"
#include "kdtree.h"
#include <vector>
#include <limits>

namespace engine {

class Scene {
public:
    std::vector<Sphere> spheres;
    std::vector<Triangle> triangles;
    std::vector<Quad> quads;
    std::vector<Material> materials;
    KdTree kdTree;

    int addMaterial(const Material& mat);
    void addSphere(const Vec3& center, double radius, int matIdx);
    void addTriangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, int matIdx);
    void addQuad(const Vec3& corner, const Vec3& edge1, const Vec3& edge2, int matIdx);
    void addMeshSphere(const Vec3& center, double radius, int matIdx,
                       int stacks = 16, int slices = 32);

    void buildAccelerator();
    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
    Vec3 tracePath(const Ray& ray, int maxBounces) const;
};


}  // namespace engine

#endif
