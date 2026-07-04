#include "test_framework.h"

#include "../src/apps/citysim/screen_project.h"

using namespace engine;
using namespace citysim;

// Living City debug labels (ADR-0066) project each place's world position to the
// screen through the camera's view-projection. worldToScreen is pure math, so
// its edge cases — behind the camera, off-frustum, centre, corners — are pinned
// here headless (the ImGui text draw itself is viewer-gated).

namespace {
// A simple camera looking down -Z from +Z at the origin, 90° vertical FOV,
// square viewport — so a point at the origin lands dead centre and the geometry
// is easy to reason about.
Mat4 lookDownZ() {
    Mat4 view = Mat4::lookAt(Vec3(0, 0, 5), Vec3(0, 0, 0), Vec3(0, 1, 0));
    Mat4 proj = Mat4::perspective(1.5707963, 1.0, 0.1, 100.0);   // 90°, aspect 1
    return proj * view;
}
}  // namespace

TEST_CASE(world_to_screen_origin_is_centre) {
    Mat4 vp = lookDownZ();
    Vec2 px;
    CHECK(worldToScreen(vp, Vec3(0, 0, 0), 800, 600, px));
    CHECK_APPROX(px.x, 400.0, 1e-6);   // horizontal centre
    CHECK_APPROX(px.y, 300.0, 1e-6);   // vertical centre
}

TEST_CASE(world_to_screen_behind_camera_culled) {
    Mat4 vp = lookDownZ();
    Vec2 px;
    // The camera sits at z=+5 looking toward -Z; a point at z=+10 is behind it.
    CHECK(!worldToScreen(vp, Vec3(0, 0, 10), 800, 600, px));
}

TEST_CASE(world_to_screen_offscreen_culled) {
    Mat4 vp = lookDownZ();
    Vec2 px;
    // Far to the side at the focal plane: outside the 90° frustum → culled.
    CHECK(!worldToScreen(vp, Vec3(100, 0, 0), 800, 600, px));
}

TEST_CASE(world_to_screen_y_flips_for_screen_space) {
    Mat4 vp = lookDownZ();
    Vec2 up, down;
    // A point ABOVE the centre (in world +Y) must map ABOVE centre on screen
    // (smaller pixel Y), because pixel Y grows downward.
    CHECK(worldToScreen(vp, Vec3(0, 1, 0), 800, 600, up));
    CHECK(worldToScreen(vp, Vec3(0, -1, 0), 800, 600, down));
    CHECK(up.y < 300.0);
    CHECK(down.y > 300.0);
    CHECK_APPROX(up.x, 400.0, 1e-6);   // no horizontal shift
    CHECK_APPROX(down.x, 400.0, 1e-6);
}
