#include "test_framework.h"

#include "../src/level_scene.h"
#include "../src/engine/mesh_builder.h"
#include <cstdio>
#include <fstream>

using namespace engine;  // namespace migration (ADR-0015)

TEST_CASE(bake_proc_model_emits_parts_and_instanced_geometry) {
    // ADR-0042 Phase 4: a composable ProcModel bakes into the path-tracer Scene —
    // parts to world triangles, instance groups to a shared BLAS placed by the
    // TLAS (ADR-0041). Renderer-agnostic: the same model a Lua recipe returns.
    ProcModel m;
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1, 1, 1)); m.parts.push_back(p); }  // a part at the origin
    ProcInstanceGroup g;
    g.proto = MeshBuilder::box(Vec3(0.5, 0.5, 0.5));      // a prototype...
    g.transforms = { Mat4::translate(5, 0, 0), Mat4::translate(10, 0, 0),
                     Mat4::translate(15, 0, 0) };          // ...placed three times
    m.instances.push_back(g);

    Scene scene;
    bakeProcModel(m, scene);
    scene.buildAccelerator();
    CHECK(!scene.triangles.empty());        // the part baked to triangles
    CHECK(scene.protos.size() == 1);        // one shared BLAS for the proto
    CHECK(scene.instances.size() == 3);     // three placements

    // A ray hits the instanced box at x = 10 (only reachable via the TLAS).
    HitRecord rec;
    Ray ray(Vec3(10, 0, 20), Vec3(0, 0, -1));
    CHECK(scene.intersect(ray, 0.001, 1e9, rec));
    CHECK_APPROX(rec.point.x, 10.0, 1e-6);
}

namespace {
const char* LEVEL_PATH = "test_level_tmp.json";
const char* SIDECAR_PATH = "test_level_tmp.json.cameras.json";

struct TmpFiles {  // removes scratch files even when checks fail
    ~TmpFiles() {
        std::remove(LEVEL_PATH);
        std::remove(SIDECAR_PATH);
    }
};

void writeFile(const char* path, const char* contents) {
    std::ofstream f(path);
    f << contents;
}

const char* SMALL_LEVEL = R"({
  "version": 1,
  "environment": { "skyColor": [0.2, 0.3, 0.4] },
  "entities": [
    { "shape": "box", "size": [10, 1, 10], "position": [0, -0.5, 0],
      "material": { "albedo": [0.7, 0.7, 0.7], "roughness": 0.8 } },
    { "shape": "sphere", "size": [2, 0, 0], "position": [0, 2, 0],
      "material": { "albedo": [0.9, 0.9, 0.9], "metallic": 1.0, "roughness": 0.1 } },
    { "shape": "sphere", "size": [1, 0, 0], "position": [3, 5, 0],
      "material": { "emission": [4, 4, 3] } },
    { "shape": "model", "file": "ignored.gltf", "position": [0, 0, 0] }
  ]
})";
}  // namespace

TEST_CASE(level_scene_imports_shapes_and_materials) {
    TmpFiles cleanup;
    writeFile(LEVEL_PATH, SMALL_LEVEL);

    Scene scene;
    CHECK(LevelScene::load(LEVEL_PATH, scene));

    // Box -> 12 triangles; the two unrotated/unscaled spheres stay analytic;
    // the glTF model is skipped.
    CHECK(scene.triangles.size() == 12);
    CHECK(scene.spheres.size() == 2);
    CHECK(scene.materials.size() == 3);

    // Materials keep full PBR parameters (viewer GGX parity).
    CHECK(scene.materials[0].type == MaterialType::PBR);
    CHECK_APPROX(scene.materials[0].metallic, 0.0, 1e-9);
    CHECK_APPROX(scene.materials[0].roughness, 0.8, 1e-9);
    CHECK(scene.materials[1].type == MaterialType::PBR);
    CHECK_APPROX(scene.materials[1].metallic, 1.0, 1e-9);
    CHECK_APPROX(scene.materials[1].roughness, 0.1, 1e-9);
    CHECK(scene.materials[2].type == MaterialType::EMISSIVE);
    CHECK_APPROX(scene.materials[2].emission.x, 4.0, 1e-9);

    // Outdoor lighting is on, tinted by the level's sky, with the default sun
    // as an explicitly sampled light (the level declares none, no HDR).
    CHECK(scene.environment.enabled);
    CHECK_APPROX(scene.environment.skyHorizon.y, 0.3, 1e-9);
    CHECK(scene.lights.size() == 1);
    CHECK(scene.lights[0].type == SceneLight::Type::Directional);
    CHECK(scene.lights[0].intensity > 0.0);

    // A ray straight down from above the floor (clear of the spheres) hits its
    // top face (geometry is in world space and the KD-tree was built).
    HitRecord rec;
    CHECK(scene.intersect(Ray(Vec3(4, 5, 4), Vec3(0, -1, 0)), 0.001, 1e9, rec));
    CHECK_APPROX(rec.point.y, 0.0, 1e-6);

    // Environment light reaches a miss ray.
    Vec3 sky = scene.tracePath(Ray(Vec3(0, 50, 0), Vec3(0, 1, 0)), 2);
    CHECK(sky.lengthSquared() > 0.0);
}

TEST_CASE(planet_demo_level_loads_and_bakes_a_planet) {
    // The shipped planet scene (procedural-planet-plan Tier 1): a shape:"script"
    // entity runs ../scripts/planet.lua -> planet.rocky, baked to world triangles by
    // the offline loader (the realtime twin renders it in the viewer). The recipe
    // runs only when scripting is compiled in (the Lua-free `make test` skips it and
    // just checks the level parses + the sun loads).
#ifdef RT_SOURCE_DIR
    const std::string path = std::string(RT_SOURCE_DIR) + "/assets/levels/planet_demo.json";
#else
    const std::string path = "assets/levels/planet_demo.json";
#endif
    Scene scene;
    CHECK(LevelScene::load(path, scene));
    CHECK(scene.lights.size() >= 1);
    CHECK(scene.lights[0].type == SceneLight::Type::Directional);
#ifdef RT_ENABLE_SCRIPTING
    // The displaced cube-sphere (face_res 96) bakes to > 100k triangles.
    CHECK(scene.triangles.size() > 5000);
#endif
}

TEST_CASE(level_scene_rotated_box_is_tessellated_in_world_space) {
    TmpFiles cleanup;
    writeFile(LEVEL_PATH, R"({
      "entities": [
        { "shape": "box", "size": [2, 2, 2], "position": [10, 0, 0],
          "orientation": { "axis": [0, 1, 0], "angleDeg": 45 } }
      ]
    })");

    Scene scene;
    CHECK(LevelScene::load(LEVEL_PATH, scene));
    CHECK(scene.triangles.size() == 12);

    // A 45-degree yawed unit cube reaches sqrt(2) along X: a ray down at
    // x = 11.2 (outside an unrotated box, inside the rotated one) must hit.
    HitRecord rec;
    CHECK(scene.intersect(Ray(Vec3(11.2, 5, 0), Vec3(0, -1, 0)), 0.001, 1e9, rec));
    // And one outside the diagonal must miss.
    CHECK(!scene.intersect(Ray(Vec3(11.2, 5, 1.2), Vec3(0, -1, 0)), 0.001, 1e9, rec));
}

TEST_CASE(level_scene_sidecar_camera_selection) {
    TmpFiles cleanup;
    writeFile(LEVEL_PATH, R"({ "entities": [] })");
    writeFile(SIDECAR_PATH, R"({
      "version": 1,
      "cameras": [
        { "name": "Crane", "position": [4, 12, -3], "forward": [0, -0.6, 0.8],
          "lens": { "focalLength": 35, "fStop": 2.8, "focusDistance": 8 } },
        { "name": "Wide", "position": [0, 1, 5], "forward": [0, 0, -1] }
      ]
    })");

    SidecarCamera cam;
    CHECK(loadSidecarCamera(LEVEL_PATH, "Crane", cam));
    CHECK_APPROX(cam.position.y, 12.0, 1e-9);
    CHECK_APPROX(cam.forward.z, 0.8, 1e-9);
    CHECK_APPROX(cam.lens.focalLength, 35.0, 1e-9);
    CHECK_APPROX(cam.lens.fStop, 2.8, 1e-9);

    // Empty name selects the first camera; unknown names fail.
    CHECK(loadSidecarCamera(LEVEL_PATH, "", cam));
    CHECK(cam.name == "Crane");
    CHECK(!loadSidecarCamera(LEVEL_PATH, "Nope", cam));

    // Missing sidecar fails cleanly.
    std::remove(SIDECAR_PATH);
    CHECK(!loadSidecarCamera(LEVEL_PATH, "", cam));
}

TEST_CASE(level_scene_grows_tree_and_terrain) {
    TmpFiles cleanup;
    writeFile(LEVEL_PATH, R"({
      "version": 1,
      "terrain": { "size": 16, "resolution": 16, "heightScale": 1.0 },
      "entities": [
        { "shape": "tree", "position": [0, 0, 0],
          "tree": { "seed": 3, "iterations": 4 } }
      ]
    })");

    Scene scene;
    CHECK(LevelScene::load(LEVEL_PATH, scene));
    CHECK(!scene.triangles.empty());   // terrain + bark + leaf cards
    CHECK(!scene.textures.empty());    // leaf alpha texture
    bool sawAlphaMat = false;
    for (const auto& m : scene.materials) sawAlphaMat |= (m.alphaTex >= 0);
    CHECK(sawAlphaMat);                 // the leaf material is alpha-cut
}

TEST_CASE(scene_alpha_cutout_and_vertex_color) {
    Scene scene;
    Texture tex;
    tex.width = 1; tex.height = 1; tex.channels = 4;
    tex.pixels = {255, 255, 255, 0};   // fully transparent texel
    int alphaTex = scene.addTexture(tex);

    Material clear = Material::pbr(Vec3(1, 1, 1), 0.0, 1.0);
    clear.alphaTex = alphaTex;
    int clearMat = scene.addMaterial(clear);
    int solidMat = scene.addMaterial(Material::pbr(Vec3(1, 0, 0), 0.0, 1.0));

    auto card = [](double z, int mat, Vec3 col) {
        Triangle t(Vec3(-1, -1, z), Vec3(1, -1, z), Vec3(0, 2, z), mat);
        t.c0 = t.c1 = t.c2 = col;
        return t;
    };
    scene.addTriangle(card(-1, clearMat, Vec3(1, 1, 1)));        // near, transparent
    scene.addTriangle(card(-2, solidMat, Vec3(0.2, 0.4, 0.6))); // far, opaque
    scene.buildAccelerator();

    Ray ray(Vec3(0, 0, 0), Vec3(0, 0, -1));
    HitRecord rec;
    CHECK(scene.intersectVisible(ray, 1e-3, 1e9, rec));
    CHECK(rec.materialIndex == solidMat);     // passed through the clear card
    CHECK_APPROX(rec.t, 2.0, 1e-6);
    CHECK_APPROX(rec.color.x, 0.2, 1e-6);     // interpolated vertex color
    CHECK_APPROX(rec.color.z, 0.6, 1e-6);

    // Plain intersect is alpha-ignorant: it stops at the nearer card.
    HitRecord near;
    CHECK(scene.intersect(ray, 1e-3, 1e9, near));
    CHECK(near.materialIndex == clearMat);
}
