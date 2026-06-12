#include "test_framework.h"

#include "../src/engine/camera/scene_camera.h"
#include "../src/engine/camera/fly_camera_controller.h"

using namespace engine;  // namespace migration (ADR-0015)

namespace {
constexpr Real EPS = 1e-6;

void checkVec(const Vec3& v, Real x, Real y, Real z, Real eps = EPS) {
    CHECK_APPROX(v.x, x, eps);
    CHECK_APPROX(v.y, y, eps);
    CHECK_APPROX(v.z, z, eps);
}
}  // namespace

TEST_CASE(lens_fov_derives_from_focal_length_and_sensor) {
    LensParams lens;  // 50mm on a 24mm-tall sensor
    CHECK_APPROX(lens.verticalFovDegrees(), 26.9905, 0.001);

    lens.focalLength = 24.0;  // wide angle: 2*atan(0.5)
    CHECK_APPROX(lens.verticalFovDegrees(), 53.1301, 0.001);

    lens.focalLength = 0.0;   // degenerate lens falls back to a nominal FOV
    CHECK_APPROX(lens.verticalFovDegrees(), 60.0, EPS);
}

TEST_CASE(lens_aperture_diameter_from_f_stop) {
    LensParams lens;  // 50mm at f/8 -> 6.25mm
    CHECK_APPROX(lens.apertureDiameter(), 0.00625, EPS);

    lens.fStop = 0.0;  // pinhole
    CHECK_APPROX(lens.apertureDiameter(), 0.0, EPS);
}

TEST_CASE(scene_camera_state_from_identity_transform) {
    Transform t;
    t.position = Vec3(1, 2, 3);
    SceneCamera cam;

    CameraState state = sceneCameraState(t, cam, 1.5f);
    checkVec(state.position, 1, 2, 3);
    checkVec(state.target, 1, 2, 2);    // identity orientation looks down -Z
    checkVec(state.up, 0, 1, 0);
    CHECK(state.projection == CameraProjection::Perspective);
    CHECK_APPROX(state.fovDegrees, 26.9905, 0.001);
    CHECK_APPROX(state.aspectRatio, 1.5, EPS);
    CHECK_APPROX(state.nearPlane, 0.1, EPS);
    CHECK_APPROX(state.farPlane, 1000.0, EPS);
}

TEST_CASE(scene_camera_state_follows_orientation) {
    Transform t;
    t.position = Vec3(0, 5, 0);
    t.orientation = orientationFromYawPitch(90.0, 0.0);  // facing +X
    SceneCamera cam;

    CameraState state = sceneCameraState(t, cam, 1.0f);
    checkVec(state.target, 1, 5, 0);
    checkVec(state.up, 0, 1, 0);

    t.orientation = orientationFromYawPitch(0.0, 90.0);  // straight up
    state = sceneCameraState(t, cam, 1.0f);
    checkVec(state.target, 0, 6, 0);
    checkVec(state.up, 0, 0, 1);   // up tilts back as the camera pitches
}

TEST_CASE(yaw_pitch_orientation_matches_fly_controller) {
    const Real angles[][2] = {{0, 0}, {90, 0}, {45, -30}, {180, 60}, {-135, 15}};
    for (const auto& a : angles) {
        FlyCameraController fly;
        fly.yaw = a[0];
        fly.pitch = a[1];
        Vec3 expected = fly.forward();
        Vec3 actual = orientationFromYawPitch(a[0], a[1]).rotate(Vec3(0, 0, -1));
        checkVec(actual, expected.x, expected.y, expected.z);
    }
}

TEST_CASE(orientation_from_forward_roundtrips) {
    const Vec3 forwards[] = {
        Vec3(0, 0, -1), Vec3(1, 0, 0), Vec3(0.3, -0.8, 0.5), Vec3(-2, 1, 3),
        Vec3(0, 1, 0),  // straight up (pitch limit case)
    };
    for (const Vec3& f : forwards) {
        Vec3 expected = normalize(f);
        Vec3 actual = orientationFromForward(f).rotate(Vec3(0, 0, -1));
        checkVec(actual, expected.x, expected.y, expected.z);
    }
}

TEST_CASE(collect_cameras_finds_complete_camera_entities) {
    World world;

    Entity a = world.create();
    world.add<Transform>(a);
    world.add<SceneCamera>(a);

    Entity transformOnly = world.create();
    world.add<Transform>(transformOnly);

    Entity b = world.create();
    world.add<Transform>(b);
    world.add<SceneCamera>(b);

    std::vector<Entity> cameras = collectCameras(world);
    CHECK(cameras.size() == 2);
    CHECK(cameras[0] == a);
    CHECK(cameras[1] == b);

    world.destroy(a);
    cameras = collectCameras(world);
    CHECK(cameras.size() == 1);
    CHECK(cameras[0] == b);
}

TEST_CASE(cycle_camera_wraps_through_editor) {
    World world;
    std::vector<Entity> cams;
    for (int i = 0; i < 3; i++) {
        Entity e = world.create();
        world.add<Transform>(e);
        world.add<SceneCamera>(e);
        cams.push_back(e);
    }
    const std::vector<Entity> list = collectCameras(world);
    const Entity editor;  // invalid handle = editor view

    CHECK(cycleCamera(list, editor, +1) == list[0]);
    CHECK(cycleCamera(list, list[0], +1) == list[1]);
    CHECK(cycleCamera(list, list[2], +1) == editor);   // wraps home after last

    CHECK(cycleCamera(list, editor, -1) == list[2]);
    CHECK(cycleCamera(list, list[1], -1) == list[0]);
    CHECK(cycleCamera(list, list[0], -1) == editor);
}

TEST_CASE(cycle_camera_handles_empty_and_stale) {
    const std::vector<Entity> empty;
    CHECK(cycleCamera(empty, Entity{}, +1) == Entity{});

    World world;
    Entity a = world.create();
    world.add<Transform>(a);
    world.add<SceneCamera>(a);
    std::vector<Entity> list = collectCameras(world);

    Entity stale{99, 7};  // not in the list (e.g. just destroyed)
    CHECK(cycleCamera(list, stale, +1) == Entity{});   // returns to the editor
}
