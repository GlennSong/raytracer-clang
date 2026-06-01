#include "geometry.h"

bool Sphere::intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    Vec3 oc = ray.origin - center;
    double a = dot(ray.direction, ray.direction);
    double halfB = dot(oc, ray.direction);
    double c = dot(oc, oc) - radius * radius;
    double discriminant = halfB * halfB - a * c;

    if (discriminant < 0) return false;

    double sqrtD = std::sqrt(discriminant);
    double root = (-halfB - sqrtD) / a;
    if (root < tMin || root > tMax) {
        root = (-halfB + sqrtD) / a;
        if (root < tMin || root > tMax) return false;
    }

    rec.t = root;
    rec.point = ray.at(root);
    Vec3 outwardNormal = (rec.point - center) / radius;
    rec.setFaceNormal(ray, outwardNormal);
    rec.materialIndex = materialIndex;
    return true;
}

bool Triangle::intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    Vec3 edge1 = v1 - v0;
    Vec3 edge2 = v2 - v0;
    Vec3 h = cross(ray.direction, edge2);
    double a = dot(edge1, h);

    if (a > -1e-8 && a < 1e-8) return false;

    double f = 1.0 / a;
    Vec3 s = ray.origin - v0;
    double u = f * dot(s, h);
    if (u < 0.0 || u > 1.0) return false;

    Vec3 q = cross(s, edge1);
    double v = f * dot(ray.direction, q);
    if (v < 0.0 || u + v > 1.0) return false;

    double t = f * dot(edge2, q);
    if (t < tMin || t > tMax) return false;

    rec.t = t;
    rec.point = ray.at(t);
    Vec3 outwardNormal = normalize(cross(edge1, edge2));
    rec.setFaceNormal(ray, outwardNormal);
    rec.materialIndex = materialIndex;
    return true;
}

Quad::Quad(const Vec3& corner, const Vec3& edge1, const Vec3& edge2, int matIdx)
    : corner(corner), edge1(edge1), edge2(edge2),
      normal(normalize(cross(edge1, edge2))), materialIndex(matIdx) {}

bool Quad::intersect(const Ray& ray, double tMin, double tMax, HitRecord& rec) const {
    double denom = dot(normal, ray.direction);
    if (std::abs(denom) < 1e-8) return false;

    double t = dot(corner - ray.origin, normal) / denom;
    if (t < tMin || t > tMax) return false;

    Vec3 hitPoint = ray.at(t);
    Vec3 localHit = hitPoint - corner;

    double e1Len2 = dot(edge1, edge1);
    double e2Len2 = dot(edge2, edge2);
    double u = dot(localHit, edge1) / e1Len2;
    double v = dot(localHit, edge2) / e2Len2;

    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0) return false;

    rec.t = t;
    rec.point = hitPoint;
    rec.setFaceNormal(ray, normal);
    rec.materialIndex = materialIndex;
    return true;
}
