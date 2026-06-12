#include "test_framework.h"

#include "../src/engine/camera_store.h"
#include "../src/engine/components.h"
#include <cstdio>

using namespace engine;  // namespace migration (ADR-0015)

namespace {
const char* TMP_PATH = "test_cameras_tmp.json";

struct TmpFile {  // removes the scratch file even when checks fail
    ~TmpFile() { std::remove(TMP_PATH); }
};
}  // namespace

TEST_CASE(camera_store_round_trips) {
    TmpFile cleanup;
    World world;

    Entity a = world.create();
    Transform ta;
    ta.position = Vec3(4, 12, -3);
    ta.orientation = orientationFromYawPitch(135.0, -35.0);
    world.add<Transform>(a, ta);
    SceneCamera camA;
    camA.name = "Crane";
    camA.lens.focalLength = 35.0;
    camA.lens.fStop = 2.8;
    camA.lens.focusDistance = 8.0;
    camA.lens.distortionK1 = -0.1;
    camA.lens.chromaticAberration = 0.01;
    world.add<SceneCamera>(a, camA);

    Entity b = world.create();
    Transform tb;
    tb.position = Vec3(0, 1.5, 8);
    world.add<Transform>(b, tb);
    SceneCamera camB;
    camB.name = "Wide";
    camB.lens.focalLength = 24.0;
    world.add<SceneCamera>(b, camB);

    CHECK(CameraStore::save(TMP_PATH, world));

    World loaded;
    CHECK(CameraStore::load(TMP_PATH, loaded));
    std::vector<Entity> cams = collectCameras(loaded);
    CHECK(cams.size() == 2);
    if (cams.size() != 2) return;

    // Match by name (entity order is save order here, but don't rely on it).
    for (Entity e : cams) {
        SceneCamera* cam = loaded.get<SceneCamera>(e);
        Transform* t = loaded.get<Transform>(e);
        CHECK(cam && t);
        if (!cam || !t) continue;
        CHECK(loaded.has<PrevTransform>(e));

        if (cam->name == "Crane") {
            CHECK_APPROX(t->position.y, 12.0, 1e-9);
            CHECK_APPROX(cam->lens.focalLength, 35.0, 1e-9);
            CHECK_APPROX(cam->lens.fStop, 2.8, 1e-9);
            CHECK_APPROX(cam->lens.focusDistance, 8.0, 1e-9);
            CHECK_APPROX(cam->lens.distortionK1, -0.1, 1e-9);
            CHECK_APPROX(cam->lens.chromaticAberration, 0.01, 1e-9);

            // Orientation round-trips through the stored forward vector.
            Vec3 fOrig = ta.orientation.rotate(Vec3(0, 0, -1));
            Vec3 fLoad = t->orientation.rotate(Vec3(0, 0, -1));
            CHECK_APPROX(fLoad.x, fOrig.x, 1e-9);
            CHECK_APPROX(fLoad.y, fOrig.y, 1e-9);
            CHECK_APPROX(fLoad.z, fOrig.z, 1e-9);
        } else {
            CHECK(cam->name == "Wide");
            CHECK_APPROX(cam->lens.focalLength, 24.0, 1e-9);
            // Default-constructed Transform looks down -Z.
            Vec3 f = t->orientation.rotate(Vec3(0, 0, -1));
            CHECK_APPROX(f.z, -1.0, 1e-9);
        }
    }
}

TEST_CASE(camera_store_missing_file_is_not_an_error) {
    World world;
    CHECK(!CameraStore::load("does_not_exist.cameras.json", world));
    CHECK(collectCameras(world).empty());
}

TEST_CASE(camera_store_saves_empty_list) {
    TmpFile cleanup;
    World world;
    CHECK(CameraStore::save(TMP_PATH, world));

    // Pre-populate the target world to prove load() doesn't resurrect or
    // disturb anything when the store is empty.
    World loaded;
    Entity existing = loaded.create();
    loaded.add<Transform>(existing);
    CHECK(CameraStore::load(TMP_PATH, loaded));
    CHECK(collectCameras(loaded).empty());
    CHECK(loaded.alive(existing));
}
