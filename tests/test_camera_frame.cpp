// FRAMING A BOX (ADR-0359): every corner inside the view, at any look.

#include "test_framework.h"
#include "../src/engine/camera/camera_frame.h"

using namespace engine;

TEST_CASE(framing_puts_every_corner_in_view) {
    const Vec3 lo(-528, 21, 888), hi(-498, 27, 921);   // a gas station's lot
    const Real fov = 60, aspect = 16.0 / 9.0;
    for (const Real pitch : {-89.0, -60.0, -30.0, -10.0})
        for (const Real yaw : {0.0, -45.0, 90.0, 200.0}) {
            const Vec3 eye = frameBox(lo, hi, pitch, yaw, fov, aspect);
            const Vec3 f = flyForward(pitch, yaw);
            Vec3 r = cross(f, Vec3(0, 1, 0));
            r = r.length() > 1e-6 ? r * (1.0 / r.length()) : Vec3(1, 0, 0);
            const Vec3 u = cross(r, f);
            const Real ty = std::tan(fov * 3.14159265358979323846 / 360.0), tx = ty * aspect;
            Real worst = 0;
            for (int k = 0; k < 8; ++k) {
                const Vec3 p((k & 1) ? hi.x : lo.x, (k & 2) ? hi.y : lo.y, (k & 4) ? hi.z : lo.z);
                const Vec3 q = p - eye;
                const Real z = dot(q, f);
                CHECK(z > 0);   // in front
                worst = std::max(worst, std::max(std::fabs(dot(q, r)) / (z * tx), std::fabs(dot(q, u)) / (z * ty)));
            }
            CHECK(worst <= 1.0);    // inside the frame...
            CHECK(worst > 0.6);     // ...and filling it, not a dot in the distance
        }
}

TEST_CASE(fly_forward_is_the_fly_cameras_convention) {
    const Vec3 f0 = flyForward(0, 0), f90 = flyForward(0, 90), down = flyForward(-90, 0);
    CHECK(f0.z < -0.99);   // yaw 0 looks down -Z
    CHECK(f90.x > 0.99);   // yaw 90 looks down +X
    CHECK(down.y < -0.99);
}
