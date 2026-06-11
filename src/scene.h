#ifndef RAYTRACER_SCENE_H
#define RAYTRACER_SCENE_H

#include "rt_math.h"
#include "geometry.h"
#include "material.h"
#include "kdtree.h"
#include <vector>
#include <limits>

namespace engine {

// Sky + sun for rays that leave the scene. Disabled by default, which returns
// black — exactly the tracer's historical miss behavior, so existing scenes
// (the Cornell box) render unchanged. Imported levels enable it: outdoor
// scenes have no emissive ceiling panel to light them.
struct EnvironmentLight {
    bool enabled = false;
    Vec3 skyHorizon{0.5, 0.6, 0.7};
    Vec3 skyZenith{0.15, 0.3, 0.55};
    double skyIntensity = 1.0;
    Vec3 sunDirection{0.35, -0.8, 0.25};   // direction the light travels
    Vec3 sunColor{1.0, 0.95, 0.85};
    double sunIntensity = 0.0;             // illuminance-style scale
    double sunAngularRadius = 0.06;        // radians; padded to cut variance

    Vec3 radiance(const Vec3& unitDir) const;
};

class Scene {
public:
    std::vector<Sphere> spheres;
    std::vector<Triangle> triangles;
    std::vector<Quad> quads;
    std::vector<Material> materials;
    KdTree kdTree;
    EnvironmentLight environment;

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
