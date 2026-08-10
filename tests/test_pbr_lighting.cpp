#include "test_framework.h"

#include "../src/scene.h"
#include "../src/level_scene.h"
#include <cstdio>
#include <fstream>

using namespace engine;  // namespace migration (ADR-0015)

// Path-traced parity with the realtime renderer's lights and shadows
// (ADR-0017 / lighting_brdf.metal): explicit light sampling, GGX surfaces,
// windowed falloff, HDR environment with the dominant sun extracted.

namespace {

// A 10x10 ground plane at y = 0 with a PBR material.
Scene groundScene(double metallic = 0.0, double roughness = 1.0) {
    Scene scene;
    int mat = scene.addMaterial(Material::pbr(Vec3(1, 1, 1), metallic, roughness));
    scene.addQuad(Vec3(-5, 0, -5), Vec3(10, 0, 0), Vec3(0, 0, 10), mat);
    return scene;
}

Vec3 directAtOrigin(const Scene& scene) {
    // Surface point at the origin, facing up, viewed from above.
    return scene.sampleDirectLight(Vec3(0, 0, 0), Vec3(0, 1, 0), Vec3(0, 1, 0),
                                   Vec3(1, 1, 1), 0.0, 1.0);
}

}  // namespace

TEST_CASE(direct_light_zero_without_lights) {
    // The Cornell box path: no SceneLights means no NEE contribution, so
    // legacy renders are untouched.
    Scene scene = groundScene();
    Vec3 c = directAtOrigin(scene);
    CHECK_APPROX(c.x, 0.0, 1e-12);
}

TEST_CASE(point_light_illuminates_and_is_shadowed) {
    Scene scene = groundScene();
    SceneLight lamp;
    lamp.type = SceneLight::Type::Point;
    lamp.position = Vec3(0, 2, 0);
    lamp.intensity = 4.0;
    scene.lights.push_back(lamp);

    Vec3 lit = directAtOrigin(scene);
    // Head-on Lambert: ~(1-F0)*albedo/pi * I/d^2 = 0.96/pi * 1 (plus a small
    // rough-specular term) — generous bounds around 0.31.
    CHECK(lit.x > 0.2);
    CHECK(lit.x < 0.45);

    // An occluding quad between the light and the point kills it.
    int mat = scene.addMaterial(Material::pbr(Vec3(1, 1, 1), 0, 1));
    scene.addQuad(Vec3(-1, 1, -1), Vec3(2, 0, 0), Vec3(0, 0, 2), mat);
    Vec3 shadowed = directAtOrigin(scene);
    CHECK_APPROX(shadowed.x, 0.0, 1e-12);
}

TEST_CASE(point_light_falloff_is_windowed_to_range) {
    // Mirrors lighting_brdf.metal's distanceAttenuation: inverse-square, smoothly
    // windowed to exactly zero at range.
    Scene near = groundScene();
    Scene far = groundScene();
    SceneLight lamp;
    lamp.type = SceneLight::Type::Point;
    lamp.intensity = 4.0;
    lamp.range = 10.0;

    lamp.position = Vec3(0, 2, 0);
    near.lights.push_back(lamp);
    lamp.position = Vec3(0, 9.99, 0);   // just inside range
    far.lights.push_back(lamp);

    Vec3 nearC = directAtOrigin(near);
    Vec3 farC = directAtOrigin(far);
    CHECK(nearC.x > 100.0 * farC.x);    // window crushes the tail

    Scene out = groundScene();
    lamp.position = Vec3(0, 12, 0);     // beyond range: exactly dark
    out.lights.push_back(lamp);
    CHECK_APPROX(directAtOrigin(out).x, 0.0, 1e-12);
}

TEST_CASE(spot_light_cone_matches_viewer_smoothstep) {
    Scene scene = groundScene();
    SceneLight spot;
    spot.type = SceneLight::Type::Spot;
    spot.position = Vec3(0, 3, 0);
    spot.direction = Vec3(0, -1, 0);   // aims straight down (toward target)
    spot.intensity = 6.0;
    spot.innerConeAngle = 0.3;
    spot.outerConeAngle = 0.5;
    scene.lights.push_back(spot);

    // Directly under the spot: inside the inner cone, fully lit.
    Vec3 under = directAtOrigin(scene);
    CHECK(under.x > 0.1);

    // 3m to the side: ~45 degrees off-axis, outside the outer cone (0.5 rad).
    Vec3 outside = scene.sampleDirectLight(Vec3(3, 0, 0), Vec3(0, 1, 0),
                                           Vec3(0, 1, 0), Vec3(1, 1, 1), 0, 1);
    CHECK_APPROX(outside.x, 0.0, 1e-12);
}

TEST_CASE(directional_sun_lights_and_shadows) {
    Scene scene = groundScene();
    SceneLight sun;   // defaults: directional, toward (0.5, 0.7, -0.3)
    sun.intensity = 4.0;
    sun.angularRadius = 0.0;   // exact direction for a deterministic check
    scene.lights.push_back(sun);

    Vec3 lit = directAtOrigin(scene);
    CHECK(lit.x > 0.1);

    // A wall toward the sun shadows the origin (sun azimuth is +x, -z).
    int mat = scene.addMaterial(Material::pbr(Vec3(1, 1, 1), 0, 1));
    scene.addQuad(Vec3(1, 0, -1.2), Vec3(0, 0, 2.4), Vec3(0, 5, 0), mat);
    CHECK_APPROX(directAtOrigin(scene).x, 0.0, 1e-12);
}

TEST_CASE(pbr_paths_stay_finite) {
    // Smoke test over the full PBR scatter (lobe pick, GGX sampling, NEE):
    // radiance must stay finite and non-negative for diffuse, glossy, and
    // metallic surfaces under sun + sky.
    const double params[][2] = {{0.0, 1.0}, {0.0, 0.2}, {1.0, 0.1}, {0.5, 0.5}};
    for (const auto& p : params) {
        Scene scene = groundScene(p[0], p[1]);
        scene.environment.enabled = true;
        SceneLight sun;
        sun.intensity = 4.0;
        scene.lights.push_back(sun);

        Vec3 sum;
        for (int i = 0; i < 500; i++) {
            Vec3 c = scene.tracePath(
                Ray(Vec3(0.3, 2, 0.3), normalize(Vec3(0.05, -1, 0.05))), 6);
            CHECK(std::isfinite(c.x) && std::isfinite(c.y) && std::isfinite(c.z));
            CHECK(c.x >= 0.0 && c.y >= 0.0 && c.z >= 0.0);
            sum += c;
        }
        CHECK(sum.lengthSquared() > 0.0);   // actually lit, not silently black
    }
}

TEST_CASE(checkerboard_albedo_matches_viewer_pattern) {
    // common.metal: 1m world squares, dark squares at albedo * 0.3. Compare
    // direct light on adjacent squares under the same lamp.
    Scene scene;
    Material mat = Material::pbr(Vec3(1, 1, 1), 0.0, 1.0);
    mat.checkerboard = true;
    int floor = scene.addMaterial(mat);
    scene.addQuad(Vec3(-5, 0, -5), Vec3(10, 0, 0), Vec3(0, 0, 10), floor);
    scene.environment.enabled = false;
    SceneLight sun;
    sun.direction = Vec3(0, 1, 0);   // straight down, no geometry bias
    sun.intensity = 4.0;
    sun.angularRadius = 0.0;
    scene.lights.push_back(sun);

    // Pixels resolve through tracePath (checker is applied at the hit).
    Vec3 light = scene.tracePath(Ray(Vec3(0.5, 3, 0.5), Vec3(0, -1, 0)), 1);
    Vec3 dark = scene.tracePath(Ray(Vec3(1.5, 3, 0.5), Vec3(0, -1, 0)), 1);
    CHECK(light.x > 0.0);
    CHECK_APPROX(dark.x / light.x, 0.3, 0.02);
}

TEST_CASE(environment_map_equirect_sampling) {
    // 4x2 synthetic map: top row red, bottom row blue.
    EnvironmentMap map;
    map.width = 4;
    map.height = 2;
    map.pixels.assign(4 * 2 * 3, 0.0f);
    for (int x = 0; x < 4; x++) {
        map.pixels[(0 * 4 + x) * 3 + 0] = 1.0f;   // top row: red
        map.pixels[(1 * 4 + x) * 3 + 2] = 1.0f;   // bottom row: blue
    }
    CHECK(map.valid());

    Vec3 up = map.sample(Vec3(0, 1, 0));
    CHECK(up.x > 0.9);
    CHECK(up.z < 0.1);
    Vec3 down = map.sample(Vec3(0, -1, 0));
    CHECK(down.z > 0.9);
    Vec3 side = map.sample(Vec3(1, 0, 0));   // equator: bilinear mix of rows
    CHECK_APPROX(side.x, 0.5, 0.01);
    CHECK_APPROX(side.z, 0.5, 0.01);
}

TEST_CASE(environment_map_sun_disc_suppression) {
    // A bright "sun" texel over a dim sky, with a circumsolar ring: after
    // suppression the disc reads like the ring, the sky is untouched.
    EnvironmentMap map;
    map.width = 8;
    map.height = 4;
    map.pixels.assign(8 * 4 * 3, 0.2f);                 // dim grey sky
    auto setTexel = [&](int x, int y, float v) {
        float* p = &map.pixels[(static_cast<size_t>(y) * 8 + x) * 3];
        p[0] = p[1] = p[2] = v;
    };
    setTexel(2, 1, 100.0f);   // the sun
    setTexel(3, 1, 30.0f);    // circumsolar ring (>= 10% of disc threshold)

    map.suppressSunDisc();
    const float* sun = &map.pixels[(1 * 8 + 2) * 3];
    CHECK_APPROX(sun[0], 30.0, 1e-3);                   // disc -> ring average
    const float* sky = &map.pixels[0];
    CHECK_APPROX(sky[0], 0.2, 1e-6);                    // sky untouched
}

TEST_CASE(level_imports_lights_and_hdr_path) {
    const char* path = "test_lights_tmp.json";
    {
        std::ofstream f(path);
        f << R"({
          "environment": { "skyColor": [0.1, 0.1, 0.1], "hdr": "../env/sky.hdr" },
          "lighting": {
            "sun": { "direction": [0.2, 0.9, 0.1], "intensity": 5.0 },
            "pointLights": [ { "position": [1, 2, 3], "intensity": 2.0, "range": 8 } ],
            "spotLights": [ { "position": [0, 4, 0], "direction": [0, -1, 0],
                              "innerConeAngle": 0.2, "outerConeAngle": 0.4 } ]
          },
          "entities": [ { "shape": "box", "size": [1, 1, 1], "position": [0, 0, 0],
            "material": { "albedo": [0.5, 0.5, 0.5], "flags": ["checkerboard"] } } ]
        })";
    }

    Scene scene;
    std::string hdrPath;
    bool ok = LevelScene::load(path, scene, &hdrPath);
    std::remove(path);
    CHECK(ok);

    CHECK(hdrPath == "../env/sky.hdr");   // level dir is "", relative kept
    CHECK(scene.lights.size() == 3);      // no default sun: level has one
    CHECK(scene.lights[0].type == SceneLight::Type::Directional);
    CHECK_APPROX(scene.lights[0].intensity, 5.0, 1e-9);
    CHECK(scene.lights[1].type == SceneLight::Type::Point);
    CHECK_APPROX(scene.lights[1].range, 8.0, 1e-9);
    CHECK(scene.lights[2].type == SceneLight::Type::Spot);
    CHECK_APPROX(scene.lights[2].innerConeAngle, 0.2, 1e-9);
    CHECK(scene.materials[0].checkerboard);
}
