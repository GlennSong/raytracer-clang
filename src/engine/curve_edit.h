#ifndef RAYTRACER_ENGINE_CURVE_EDIT_H
#define RAYTRACER_ENGINE_CURVE_EDIT_H

#include "../rt_math.h"
#include <vector>

namespace engine {

// The manipulator math behind the curve/path edit tool (ADR-0050): pick a handle
// under the cursor, and drag it on a constraint plane. Pure + headless — the editor
// supplies the pick ray and camera, draws the handles, and applies the result; this
// owns only the geometry, so it's reusable by any handle-based gizmo (curves, roads,
// future bone/IK pickers) and unit-testable without a viewport.

struct EditRay {
    Vec3 origin;
    Vec3 direction;            // need not be normalized
};

// Which plane a handle drag is constrained to. The plane passes through the handle's
// current position; the constraint fixes its NORMAL. Ground == XZ (move on the floor);
// XY/YZ/XZ lock to an axis plane; Screen faces the camera (free move in view).
enum class DragPlane { Ground, XY, YZ, XZ, Screen };

// The plane normal for a constraint. `view` is the camera forward (used by Screen).
Vec3 dragPlaneNormal(DragPlane plane, const Vec3& view);

// Intersect `ray` with the plane through `planePoint` with `normal`. False if the ray
// is parallel to the plane (no stable hit); `hit` holds the intersection otherwise.
bool rayPlaneHit(const EditRay& ray, const Vec3& planePoint, const Vec3& normal, Vec3& hit);

// Where a drag lands: the ray's hit on the constraint plane through `anchor` (the
// handle's current position). Falls back to `anchor` when the plane is edge-on.
Vec3 projectDrag(const EditRay& ray, const Vec3& anchor, DragPlane plane, const Vec3& view);

// Index of the handle nearest `ray` within `maxDist` (perpendicular world distance) and
// in front of the ray origin, or -1. For picking which dot the cursor is over.
int nearestHandle(const std::vector<Vec3>& handles, const EditRay& ray, double maxDist);

}  // namespace engine

#endif
