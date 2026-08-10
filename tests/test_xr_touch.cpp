#include "test_framework.h"

#include "../src/engine/xr/xr_touch.h"

#include <cmath>

using namespace engine;

// Oriented nearest-point queries (engine/xr/xr_touch.h) — the math behind
// precise grab regions and fingertip depth cues. The lying-pillar case is
// the device scenario that motivated the module: centre-distance picking
// made a fallen pillar grabbable only at its middle.

namespace {
bool nearVec(const Vec3& a, const Vec3& b, Real tol) {
    return (a - b).length() <= tol;
}
}  // namespace

TEST_CASE(touch_box_face_edge_corner_and_inside) {
    const Vec3 he(1, 2, 3);
    const Vec3 c(0, 0, 0);
    const Quat q = Quat::identity();

    // Straight off the +X face.
    CHECK(nearVec(xrNearestPointOnBox(he, c, q, Vec3(5, 0, 0)),
                  Vec3(1, 0, 0), 1e-9));
    // Off an edge (x and y both past their extents, z within).
    CHECK(nearVec(xrNearestPointOnBox(he, c, q, Vec3(4, 5, 1)),
                  Vec3(1, 2, 1), 1e-9));
    // Off a corner.
    CHECK(nearVec(xrNearestPointOnBox(he, c, q, Vec3(9, 9, 9)),
                  Vec3(1, 2, 3), 1e-9));
    // Inside: unchanged, distance zero.
    const Vec3 inside(0.5, -1.0, 2.0);
    CHECK(nearVec(xrNearestPointOnBox(he, c, q, inside), inside, 1e-12));
    CHECK(xrBoxSurfaceDistance(he, c, q, inside) == 0.0);
}

TEST_CASE(touch_box_respects_translation) {
    const Vec3 he(0.5, 0.5, 0.5);
    const Vec3 c(10, 5, -2);
    CHECK(nearVec(
        xrNearestPointOnBox(he, c, Quat::identity(), Vec3(10, 8, -2)),
        Vec3(10, 5.5, -2), 1e-9));
}

TEST_CASE(touch_lying_pillar_pick_region_follows_orientation) {
    // The device case: the tall skinny pillar (0.04 x 0.18 x 0.04 half
    // extents) tipped over to LIE along X (rotated 90 deg about Z). A pinch
    // 3cm above one END of the lying pillar is touching-close by surface
    // distance, while its centre distance is nearly the pillar's half
    // length — the sphere-shaped pick region the old code used would have
    // called it unreachable at tight thresholds and, worse, shaped the
    // region as if the pillar were still standing.
    const Vec3 he(0.04, 0.18, 0.04);
    const Vec3 c(0, 0.04, 0);   // resting on the floor on its side
    const Quat lying = Quat::fromAxisAngle(Vec3(0, 0, 1), PI / 2);

    const Vec3 pinchNearEnd(0.17, 0.10, 0.0);   // above the +X end
    const Real surface = xrBoxSurfaceDistance(he, c, lying, pinchNearEnd);
    const Real centre = (pinchNearEnd - c).length();
    CHECK(surface < 0.035);          // touching-close to the actual shape
    CHECK(centre > 0.17);            // centre metric says "far"

    // And the same offset applied where the STANDING pillar would have had
    // its end is now far from the lying shape — the region really rotated.
    const Vec3 whereTopWas(0.0, 0.04 + 0.18 + 0.03, 0.0);
    CHECK(xrBoxSurfaceDistance(he, c, lying, whereTopWas) > 0.12);
}

TEST_CASE(touch_sphere_nearest_and_distance) {
    const Vec3 c(1, 1, 1);
    CHECK(nearVec(xrNearestPointOnSphere(0.5, c, Vec3(3, 1, 1)),
                  Vec3(1.5, 1, 1), 1e-9));
    const Vec3 inside(1.1, 1, 1);
    CHECK(nearVec(xrNearestPointOnSphere(0.5, c, inside), inside, 1e-12));
    CHECK(xrSphereSurfaceDistance(0.5, c, inside) == 0.0);
    CHECK(std::abs(xrSphereSurfaceDistance(0.5, c, Vec3(3, 1, 1)) - 1.5) <
          1e-9);
}
