#ifndef RAYTRACER_GEOMETRY_H
#define RAYTRACER_GEOMETRY_H

#include "rt_math.h"

namespace engine {

struct HitRecord {
    double t;
    Vec3 point;
    Vec3 normal;
    bool frontFace;
    int materialIndex;
    double u = 0.0, v = 0.0;     // interpolated texture coords (meshes)
    Vec3 color{1, 1, 1};         // interpolated vertex color (modulates albedo)
    Vec3 tangent;                // interpolated surface tangent (zero = none)

    void setFaceNormal(const Ray& ray, const Vec3& outwardNormal) {
        frontFace = dot(ray.direction, outwardNormal) < 0;
        normal = frontFace ? outwardNormal : -outwardNormal;
    }
};

struct Sphere {
    Vec3 center;
    double radius;
    int materialIndex;

    Sphere() : radius(0), materialIndex(0) {}
    Sphere(const Vec3& center, double radius, int matIdx = 0)
        : center(center), radius(radius), materialIndex(matIdx) {}

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
};

struct Triangle {
    Vec3 v0, v1, v2;
    int materialIndex;

    // Optional per-vertex attributes, barycentric-interpolated at the hit
    // (mesh geometry sets them; primitives leave the defaults). Zero normals
    // mean "use the flat face normal"; color defaults to white; uv to zero.
    Vec3 n0, n1, n2;
    Vec3 t0, t1, t2;             // per-vertex tangents (zero = no normal mapping)
    Vec3 c0{1, 1, 1}, c1{1, 1, 1}, c2{1, 1, 1};
    double uv0[2] = {0, 0}, uv1[2] = {0, 0}, uv2[2] = {0, 0};

    Triangle() : materialIndex(0) {}
    Triangle(const Vec3& v0, const Vec3& v1, const Vec3& v2, int matIdx = 0)
        : v0(v0), v1(v1), v2(v2), materialIndex(matIdx) {}

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
    Vec3 faceNormal() const { return normalize(cross(v1 - v0, v2 - v0)); }
};

struct Quad {
    Vec3 corner;
    Vec3 edge1;
    Vec3 edge2;
    Vec3 normal;
    int materialIndex;

    Quad() : materialIndex(0) {}
    Quad(const Vec3& corner, const Vec3& edge1, const Vec3& edge2, int matIdx = 0);

    bool intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const;
};


}  // namespace engine

#endif
