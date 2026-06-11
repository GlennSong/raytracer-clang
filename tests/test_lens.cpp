#include "test_framework.h"

#include "../src/camera.h"

using namespace engine;  // namespace migration (ADR-0015)

namespace {
constexpr double EPS = 1e-9;

const Vec3 FROM(278, 278, -800);
const Vec3 AT(278, 278, 0);
const Vec3 UP(0, 1, 0);

LensParams wideOpen() {
    LensParams lens;
    lens.fStop = 1.0;           // 50mm/1000 * scale 100 -> lens radius 2.5
    lens.focusDistance = 800.0; // the focal plane is z = 0
    return lens;
}
}  // namespace

TEST_CASE(lens_pinhole_matches_legacy_camera) {
    Camera legacy(FROM, AT, UP, 40.0, 1.0);
    LensParams pinhole;
    pinhole.fStop = 0.0;
    Camera thin(FROM, AT, UP, 40.0, 1.0, pinhole, 100.0);

    const double uvs[][2] = {{0.5, 0.5}, {0.0, 0.0}, {1.0, 1.0}, {0.25, 0.8}};
    for (const auto& uv : uvs) {
        Ray a = legacy.generateRay(uv[0], uv[1]);
        Ray b = thin.generateRay(uv[0], uv[1], 0.7, 0.3);  // lens sample ignored
        CHECK_APPROX(a.origin.x, b.origin.x, EPS);
        CHECK_APPROX(a.origin.y, b.origin.y, EPS);
        CHECK_APPROX(a.origin.z, b.origin.z, EPS);
        CHECK_APPROX(a.direction.x, b.direction.x, EPS);
        CHECK_APPROX(a.direction.y, b.direction.y, EPS);
        CHECK_APPROX(a.direction.z, b.direction.z, EPS);
    }
}

TEST_CASE(lens_aperture_rays_converge_on_focal_plane) {
    Camera cam(FROM, AT, UP, 40.0, 1.0, wideOpen(), 100.0);

    // Different aperture samples for the same pixel must hit the same point on
    // the focal plane (z = 0) — that is what "in focus" means.
    Ray a = cam.generateRay(0.3, 0.7, 0.2, 0.3);
    Ray b = cam.generateRay(0.3, 0.7, 0.9, 0.8);
    CHECK(std::fabs(a.origin.x - b.origin.x) > 1e-6);  // distinct lens points

    double ta = -a.origin.z / a.direction.z;
    double tb = -b.origin.z / b.direction.z;
    Vec3 pa = a.origin + ta * a.direction;
    Vec3 pb = b.origin + tb * b.direction;
    CHECK_APPROX(pa.x, pb.x, 1e-6);
    CHECK_APPROX(pa.y, pb.y, 1e-6);

    // And out-of-focus geometry diverges: at z = -400 the two rays differ.
    double sa = (-400.0 - a.origin.z) / a.direction.z;
    double sb = (-400.0 - b.origin.z) / b.direction.z;
    Vec3 qa = a.origin + sa * a.direction;
    Vec3 qb = b.origin + sb * b.direction;
    CHECK(std::fabs(qa.x - qb.x) > 1e-3);
}

TEST_CASE(lens_distortion_warps_corners_not_center) {
    LensParams lens = wideOpen();
    lens.fStop = 0.0;  // isolate distortion
    lens.distortionK1 = 0.2;
    Camera warped(FROM, AT, UP, 40.0, 1.0, lens, 100.0);
    Camera straight(FROM, AT, UP, 40.0, 1.0);

    Ray center = warped.generateRay(0.5, 0.5);
    Ray centerRef = straight.generateRay(0.5, 0.5);
    CHECK_APPROX(center.direction.x, centerRef.direction.x, EPS);
    CHECK_APPROX(center.direction.y, centerRef.direction.y, EPS);

    Ray corner = warped.generateRay(0.0, 0.0);
    Ray cornerRef = straight.generateRay(0.0, 0.0);
    // k1 > 0 pushes the sampled image point outward: the corner ray bends
    // further from the optical axis (smaller dot with the view direction).
    Vec3 axis = normalize(AT - FROM);
    CHECK(dot(corner.direction, axis) < dot(cornerRef.direction, axis) - 1e-6);
}

TEST_CASE(lens_chromatic_scale_shifts_off_center_rays) {
    LensParams lens;
    lens.fStop = 0.0;
    Camera cam(FROM, AT, UP, 40.0, 1.0, lens, 100.0);

    Ray g = cam.generateRay(0.2, 0.6, 0.0, 0.0, 1.0);
    Ray r = cam.generateRay(0.2, 0.6, 0.0, 0.0, 1.01);
    CHECK(std::fabs(g.direction.x - r.direction.x) > 1e-6);

    Ray gc = cam.generateRay(0.5, 0.5, 0.0, 0.0, 1.0);
    Ray rc = cam.generateRay(0.5, 0.5, 0.0, 0.0, 1.01);
    CHECK_APPROX(gc.direction.x, rc.direction.x, EPS);
    CHECK_APPROX(gc.direction.y, rc.direction.y, EPS);
}
