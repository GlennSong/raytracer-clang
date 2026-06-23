#include "test_framework.h"

#include "../src/engine/curve_edit.h"
#include <cmath>

using namespace engine;

namespace {
double dist(const Vec3& a, const Vec3& b) { return std::sqrt((a - b).lengthSquared()); }
}

TEST_CASE(curve_edit_plane_normals) {
    CHECK(dist(dragPlaneNormal(DragPlane::Ground, Vec3()), Vec3(0, 1, 0)) < 1e-9);
    CHECK(dist(dragPlaneNormal(DragPlane::XZ, Vec3()), Vec3(0, 1, 0)) < 1e-9);
    CHECK(dist(dragPlaneNormal(DragPlane::XY, Vec3()), Vec3(0, 0, 1)) < 1e-9);
    CHECK(dist(dragPlaneNormal(DragPlane::YZ, Vec3()), Vec3(1, 0, 0)) < 1e-9);
    // Screen faces the camera (normalized view direction).
    CHECK(dist(dragPlaneNormal(DragPlane::Screen, Vec3(0, 0, -4)), Vec3(0, 0, -1)) < 1e-9);
}

TEST_CASE(curve_edit_ray_plane_hits_the_floor) {
    EditRay ray{Vec3(3, 10, -2), Vec3(0, -1, 0)};        // straight down
    Vec3 hit;
    CHECK(rayPlaneHit(ray, Vec3(0, 0, 0), Vec3(0, 1, 0), hit));
    CHECK(dist(hit, Vec3(3, 0, -2)) < 1e-9);
}

TEST_CASE(curve_edit_ray_parallel_to_plane_misses) {
    EditRay ray{Vec3(0, 5, 0), Vec3(1, 0, 0)};           // travels in the plane
    Vec3 hit;
    CHECK(!rayPlaneHit(ray, Vec3(0, 0, 0), Vec3(0, 1, 0), hit));
}

TEST_CASE(curve_edit_drag_on_ground_vs_vertical) {
    // A handle at (0,2,0); a ray angled down-forward from above.
    Vec3 anchor(0, 2, 0);
    EditRay ray{Vec3(0, 6, 6), Vec3(0, -1, -1)};
    // Ground constraint: drops the handle to the anchor's plane y=2 — the XZ plane
    // through it (normal Y), so the hit keeps y == 2.
    Vec3 g = projectDrag(ray, anchor, DragPlane::Ground, Vec3(0, 0, -1));
    CHECK(std::fabs(g.y - 2.0) < 1e-9);
    // XY constraint (normal Z): the hit lies on z == anchor.z == 0.
    Vec3 v = projectDrag(ray, anchor, DragPlane::XY, Vec3(0, 0, -1));
    CHECK(std::fabs(v.z - 0.0) < 1e-9);
    // A ray parallel to the YZ plane it's constrained to (edge-on) falls back to anchor.
    EditRay along{Vec3(3, 2, 5), Vec3(0, 0, 1)};
    Vec3 f = projectDrag(along, anchor, DragPlane::YZ, Vec3(0, 0, -1));
    CHECK(dist(f, anchor) < 1e-9);
}

TEST_CASE(curve_edit_nearest_handle_picks_under_the_ray) {
    std::vector<Vec3> handles = { Vec3(0, 0, 0), Vec3(10, 0, 0), Vec3(0, 0, 10) };
    // A ray from above aimed just over the second handle.
    EditRay ray{Vec3(10.3, 8, 0), Vec3(0, -1, 0)};
    CHECK(nearestHandle(handles, ray, 1.0) == 1);        // within 1 m of handle 1
    CHECK(nearestHandle(handles, ray, 0.1) == -1);       // none within 0.1 m
    // A handle behind the ray origin is not picked.
    EditRay up{Vec3(0, -8, 0), Vec3(0, -1, 0)};           // pointing away, down, below all
    CHECK(nearestHandle(handles, up, 1.0) == -1);
}
