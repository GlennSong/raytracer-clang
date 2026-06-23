#include "curve_edit.h"

#include <cmath>

namespace engine {

Vec3 dragPlaneNormal(DragPlane plane, const Vec3& view) {
    switch (plane) {
        case DragPlane::Ground:
        case DragPlane::XZ:     return Vec3(0, 1, 0);     // the floor: move in X and Z
        case DragPlane::XY:     return Vec3(0, 0, 1);
        case DragPlane::YZ:     return Vec3(1, 0, 0);
        case DragPlane::Screen: {
            double l = std::sqrt(view.lengthSquared());
            return l > 1e-9 ? view * (1.0 / l) : Vec3(0, 0, 1);
        }
    }
    return Vec3(0, 1, 0);
}

bool rayPlaneHit(const EditRay& ray, const Vec3& planePoint, const Vec3& normal, Vec3& hit) {
    double denom = dot(normal, ray.direction);
    if (std::fabs(denom) < 1e-9) return false;           // parallel: no stable intersection
    double t = dot(normal, planePoint - ray.origin) / denom;
    hit = ray.origin + ray.direction * t;
    return true;
}

Vec3 projectDrag(const EditRay& ray, const Vec3& anchor, DragPlane plane, const Vec3& view) {
    Vec3 hit;
    if (rayPlaneHit(ray, anchor, dragPlaneNormal(plane, view), hit)) return hit;
    return anchor;                                        // edge-on: leave it where it was
}

int nearestHandle(const std::vector<Vec3>& handles, const EditRay& ray, double maxDist) {
    double dl = std::sqrt(ray.direction.lengthSquared());
    if (dl < 1e-9) return -1;
    Vec3 d = ray.direction * (1.0 / dl);
    int best = -1;
    double bestD2 = maxDist * maxDist;
    for (int i = 0; i < static_cast<int>(handles.size()); ++i) {
        Vec3 oh = handles[i] - ray.origin;
        double tproj = dot(oh, d);
        if (tproj < 0) continue;                         // behind the camera
        double d2 = (handles[i] - (ray.origin + d * tproj)).lengthSquared();
        if (d2 < bestD2) { bestD2 = d2; best = i; }
    }
    return best;
}

}  // namespace engine
