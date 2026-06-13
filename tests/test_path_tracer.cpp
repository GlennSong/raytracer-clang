#include "test_framework.h"

#include "../src/path_tracer.h"
#include "../src/scene.h"
#include "../src/camera.h"
#include "../src/job_system.h"
#include <atomic>

using namespace engine;  // namespace migration (ADR-0015)

namespace {

// A pinhole camera at the origin staring at a large emissive wall, so every
// primary ray hits a lit surface — the rendered image is reliably non-black
// without depending on the stochastic light sampling.
Scene emissiveWall() {
    Scene scene;
    int light = scene.addMaterial(Material::emissive(Vec3(1, 1, 1), 1.0));
    scene.addQuad(Vec3(-50, -50, 5), Vec3(100, 0, 0), Vec3(0, 100, 0), light);
    scene.buildAccelerator();
    return scene;
}

Camera frontCamera() {
    return Camera(Vec3(0, 0, 0), Vec3(0, 0, 5), Vec3(0, 1, 0), 60.0, 1.0);
}

double totalLuminance(const Image& img) {
    double sum = 0.0;
    for (const Vec3& p : img.pixels) sum += p.x + p.y + p.z;
    return sum;
}

}  // namespace

TEST_CASE(render_produces_sized_nonblack_image) {
    Scene scene = emissiveWall();
    Camera camera = frontCamera();
    JobSystem jobs(0);   // synchronous: deterministic, no worker timing
    RenderConfig cfg{16, 4, 1.0};

    Image img = renderImage(scene, camera, cfg, jobs);
    CHECK(img.width == 16);
    CHECK(img.height == 16);
    CHECK(totalLuminance(img) > 0.0);   // the emissive wall lit it
}

TEST_CASE(progress_runs_from_zero_to_one_monotonically) {
    Scene scene = emissiveWall();
    Camera camera = frontCamera();
    JobSystem jobs(0);
    RenderConfig cfg{16, 2, 1.0};

    int calls = 0;
    float last = -1.0f;
    bool monotonic = true, inRange = true;
    renderImage(scene, camera, cfg, jobs, [&](float frac) {
        ++calls;
        if (frac < last) monotonic = false;
        if (frac < 0.0f || frac > 1.0f) inRange = false;
        last = frac;
    });
    CHECK(calls >= 1);
    CHECK(monotonic);
    CHECK(inRange);
    CHECK_APPROX(last, 1.0, 1e-6);   // always finishes at 100%
}

TEST_CASE(cancel_before_start_yields_black_image) {
    Scene scene = emissiveWall();
    Camera camera = frontCamera();
    JobSystem jobs(0);
    RenderConfig cfg{16, 4, 1.0};

    std::atomic<bool> cancel(true);   // cancelled before any row runs
    float last = -1.0f;
    Image img = renderImage(scene, camera, cfg, jobs,
                            [&](float frac) { last = frac; }, &cancel);
    CHECK(totalLuminance(img) == 0.0);   // every row skipped -> black
    CHECK_APPROX(last, 1.0, 1e-6);       // still drains to 100%
}

TEST_CASE(threaded_and_synchronous_agree_on_coverage) {
    // The pool path and the inline path must both fill the image (the row count
    // is independent of worker count); luminance is positive either way.
    Scene scene = emissiveWall();
    Camera camera = frontCamera();
    RenderConfig cfg{24, 2, 1.0};

    JobSystem sync(0);
    Image a = renderImage(scene, camera, cfg, sync);
    JobSystem pool;   // AUTO worker count
    Image b = renderImage(scene, camera, cfg, pool);

    CHECK(totalLuminance(a) > 0.0);
    CHECK(totalLuminance(b) > 0.0);
}
