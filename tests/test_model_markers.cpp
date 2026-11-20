// MARKERS AND NAMED MATERIALS IN A MODEL (#191, model step 2).
//
// A marker is a named point on a node where something can hook on later: a seat, a lamp, a hook. It has a position,
// a turn, a size and a role, and does nothing by itself. A model's materials are one table: a part says which entry
// it is made of, so forty parts in one paint hold one material, and a recipe can give an entry a name ("paint").
//
// What is held here:
//   - the record: a marker's name is unique across the whole tree, and where it is in the model follows every node
//     above it; the material table always has the plain material first, holds one entry per material, and keeps one
//     material per name;
//   - merging two models carries markers and materials across and refuses when the two disagree;
//   - Lua: `m:marker`, `n:marker`, `m:marker_at`, `m:material`, a part made of a material by name;
//   - the readers: the offline bake makes one scene material (and one copy of its maps) per material of the model;
//     the game's loader gives each part its material's numbers;
//   - a recipe that names nothing is untouched (the fingerprints of every existing recipe, in the commit message).
#include "test_framework.h"

#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/level_loader.h"
#include "../src/engine/mesh_builder.h"
#include "../src/engine/mesh_uploader.h"
#include "../src/engine/procgen/proc_model.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/scripting/script_vm.h"
#include "../src/level_scene.h"
#include "../src/renderer/renderer.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

using namespace engine;

namespace {

bool near(const Vec3& a, const Vec3& b, Real eps = 1e-9) { return (a - b).length() < eps; }

bool runModel(const std::string& code, ProcModel& out, std::string* error = nullptr) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    return runProcgenModelValue(vm, code, out, error);
}

std::shared_ptr<const ProcMaterial> material(float roughness, float metallic = 0.0f) {
    std::shared_ptr<ProcMaterial> m = std::make_shared<ProcMaterial>();
    m->roughness = roughness;
    m->metallic = metallic;
    return m;
}

ProcMarker marker(const std::string& name, const Vec3& pos = Vec3(0, 0, 0)) {
    ProcMarker mk;
    mk.name = name;
    mk.frame.position = pos;
    return mk;
}

struct TmpFile {   // a scratch file, removed even when a check fails
    std::string path;
    TmpFile(const std::string& p, const std::string& contents) : path(p) { std::ofstream(path) << contents; }
    ~TmpFile() { std::remove(path.c_str()); }
};

}  // namespace

TEST_CASE(a_models_materials_are_one_table_with_the_plain_one_first) {
    ProcModel m;
    CHECK(m.materials.size() == 1);
    CHECK(m.materials[0].name.empty());
    CHECK(ProcPart{}.material == 0);                          // a part that says nothing is made of the plain one
    CHECK(m.materialOf(ProcPart{}).roughness == ProcMaterial{}.roughness);
    CHECK(!m.materialOf(ProcPart{}).textured);

    const std::shared_ptr<const ProcMaterial> paint = material(0.3f, 0.5f), glass = material(0.05f);
    const int a = m.addMaterial("", paint);
    CHECK(a == 1);
    CHECK(m.addMaterial("", paint) == a);                     // the same material twice is one entry
    CHECK(m.addMaterial("", glass) == 2);
    CHECK(m.materials.size() == 3);

    // A name finds the entry from then on, and a material used before it was named keeps its entry.
    CHECK(m.findMaterial("paint") == -1);
    CHECK(m.addMaterial("paint", paint) == a);
    CHECK(m.findMaterial("paint") == a);
    CHECK(m.addMaterial("paint", paint) == a);
    CHECK(m.addMaterial("", paint) == a);
    CHECK(m.addMaterial("paint", glass) == -1);               // one name, one material
    CHECK(m.addMaterial("trim", nullptr) == -1);
    CHECK(m.findMaterial("") == -1);                          // no name is not a name
    CHECK(m.materials.size() == 3);                           // the refused ones added nothing

    ProcPart part;
    part.material = m.findMaterial("paint");
    CHECK(&m.materialOf(part) == paint.get());                // shared, not copied
}

TEST_CASE(a_marker_is_found_by_name_through_the_whole_tree) {
    ProcModel m;
    const Quat quarter = Quat::fromAxisAngle(Vec3(0, 1, 0), 90 * DEG_TO_RAD);
    const int cabin = m.addNode("cabin", 0, ProcFrame{Vec3(0, 4, 0), quarter});
    const int boom = m.addNode("boom", cabin, ProcFrame{Vec3(0, 1, 0), Quat()});

    CHECK(m.addMarker(0, marker("foot")));
    ProcMarker hook = marker("hook", Vec3(0, 0, 5));           // 5 m along the boom
    hook.role = "hook";
    hook.size = 0.4;
    CHECK(m.addMarker(boom, hook));
    CHECK(!m.addMarker(cabin, marker("hook")));               // the name is taken, on whatever node
    CHECK(!m.addMarker(0, marker("")));
    CHECK(!m.addMarker(9, marker("lost")));
    CHECK(m.markerCount() == 2);

    int node = -1;
    const ProcMarker* found = m.findMarker("hook", &node);
    CHECK(found != nullptr && node == boom);
    if (found != nullptr) {
        CHECK(found->role == "hook");
        CHECK(found->size == 0.4);
    }
    CHECK(m.findMarker("foot") != nullptr && m.findMarker("foot")->role == "point");   // the role when none is said
    CHECK(m.findMarker("nothing") == nullptr);

    // In the model: the cabin's quarter turn carries the boom's z onto the model's x.
    ProcFrame at;
    CHECK(m.markerFrame("hook", at));
    CHECK(near(at.position, Vec3(5, 5, 0)));
    CHECK(near(at.rotation.rotate(Vec3(0, 0, 1)), Vec3(1, 0, 0)));
    CHECK(!m.markerFrame("nothing", at));
}

TEST_CASE(merging_carries_markers_and_materials_and_refuses_a_disagreement) {
    const std::shared_ptr<const ProcMaterial> paint = material(0.3f), rubber = material(0.9f), otherPaint = material(0.6f);

    ProcModel car;
    car.addMaterial("paint", paint);
    car.addMarker(0, marker("driver"));
    { ProcPart p; p.material = car.findMaterial("paint"); car.parts.push_back(p); }

    ProcModel wheel;
    const int tyre = wheel.addMaterial("rubber", rubber);
    const int rim = wheel.addMaterial("paint", paint);          // the same paint under the same name: no disagreement
    const int hub = wheel.addNode("hub", 0, ProcFrame{Vec3(1, 0, 0), Quat()});
    wheel.addMarker(hub, marker("valve", Vec3(0, 0.2, 0)));
    wheel.addMarker(0, marker("axle"));
    { ProcPart p; p.material = tyre; p.node = hub; wheel.parts.push_back(p); }
    { ProcPart p; p.material = rim; wheel.parts.push_back(p); }
    wheel.parts.push_back(ProcPart{});                         // and one in the plain material

    CHECK(car.merge(wheel));
    CHECK(car.materials.size() == 3);                          // plain, paint, rubber: the paint is held once
    CHECK(car.parts.size() == 4);
    CHECK(&car.materialOf(car.parts[1]) == rubber.get());
    CHECK(car.parts[2].material == car.findMaterial("paint"));
    CHECK(car.parts[3].material == 0);
    CHECK(car.markerCount() == 3);
    int node = -1;
    CHECK(car.findMarker("axle", &node) != nullptr && node == 0);            // the wheel's root is the car's root
    CHECK(car.findMarker("valve", &node) != nullptr && node == car.findNode("hub"));
    ProcFrame at;
    CHECK(car.markerFrame("valve", at) && near(at.position, Vec3(1, 0.2, 0)));

    // A second marker of one name, or one name for two different materials: refused, nothing changed.
    std::string clash;
    ProcModel twin;
    twin.addMarker(0, marker("driver"));
    CHECK(!car.merge(twin, &clash));
    CHECK(clash == "marker 'driver'");
    ProcModel repaint;
    repaint.addMaterial("paint", otherPaint);
    repaint.addNode("spoiler", 0, ProcFrame{});
    CHECK(!car.merge(repaint, &clash));
    CHECK(clash == "material 'paint'");
    CHECK(car.materials.size() == 3 && car.markerCount() == 3 && car.parts.size() == 4);
    CHECK(car.findNode("spoiler") == -1);
}

TEST_CASE(a_recipe_names_materials_and_makes_parts_of_them) {
    ProcModel m;
    std::string err;
    CHECK(runModel(R"LUA(
        local m = model.new()
        local paint = material.new{ roughness = 0.3, metallic = 0.55 }
        m:material("paint", paint)
        m:material("glass", material.new{ roughness = 0.05, opacity = 0.3 })
        m:add(mesh.box{ 1, 1, 1 })                              -- the plain material
        m:add(mesh.box{ 1, 1, 1 }, "paint")                     -- by name
        m:add(mesh.box{ 1, 1, 1 }, paint)                       -- by the material itself: the same entry
        local door = m:node("door")
        for _ = 1, 40 do door:add(mesh.box{ 0.1, 0.1, 0.1 }, "paint") end
        door:add(mesh.box{ 1, 1, 0.01 }, "glass")
        m:add_solid(mesh.box{ 2, 0.1, 2 }, "paint")
        local loose = material.new{ roughness = 0.7 }           -- used without a name
        m:add(mesh.box{ 1, 1, 1 }, loose)
        m:add(mesh.box{ 1, 1, 1 }, loose)
        assert(m:has_material("paint") and not m:has_material("chrome"))
        assert(m:material_count() == 4)                         -- plain, paint, glass, the loose one
        return m
    )LUA", m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    CHECK(m.materials.size() == 4);
    const int paint = m.findMaterial("paint"), glass = m.findMaterial("glass");
    CHECK(paint == 1 && glass == 2);
    CHECK(m.parts.size() == 47);
    CHECK(m.parts[0].material == 0);
    CHECK(m.parts[1].material == paint && m.parts[2].material == paint);
    int painted = 0;
    for (const ProcPart& p : m.parts) painted += p.material == paint ? 1 : 0;
    CHECK(painted == 43);                                      // forty-three parts, one paint
    CHECK(m.parts[43].material == glass);
    CHECK_APPROX(m.materialOf(m.parts[43]).opacity, 0.3, 1e-6);
    CHECK_APPROX(m.materialOf(m.parts[1]).metallic, 0.55, 1e-6);
    CHECK(m.parts[45].material == 3 && m.parts[46].material == 3);
    CHECK(m.materials[3].name.empty());

    // What stops a recipe.
    CHECK(!runModel(R"LUA(local m = model.new() m:add(mesh.box{1,1,1}, "chrome") return m)LUA", m, &err));
    CHECK(err.find("no material named 'chrome'") != std::string::npos);
    CHECK(!runModel(R"LUA(local m = model.new()
        m:material("paint", material.new{}) m:material("paint", material.new{}) return m)LUA", m, &err));
    CHECK(err.find("already has a different material named 'paint'") != std::string::npos);
    CHECK(!runModel(R"LUA(local m = model.new() m:material("", material.new{}) return m)LUA", m, &err));
    CHECK(err.find("needs a name") != std::string::npos);
    CHECK(!runModel(R"LUA(
        local a, b = model.new(), model.new()
        a:material("paint", material.new{}) b:material("paint", material.new{})
        return a:merge(b))LUA", m, &err));
    CHECK(err.find("disagree on material 'paint'") != std::string::npos);
}

TEST_CASE(a_recipe_puts_markers_on_nodes_and_asks_where_they_are) {
    ProcModel m;
    std::string err;
    CHECK(runModel(R"LUA(
        local m = model.new()
        m:marker("foot")
        local cabin = m:node("cabin", { pos = { 0, 4, 0 }, turn = { 0, 90, 0 } })
        local boom = cabin:node("boom", { pos = { 0, 1, 0 } })
        assert(boom:marker("hook", { pos = { 0, 0, 5 }, turn = { 90, 0, 0 }, size = 0.4, role = "hook" }) == boom)
        assert(m:marker_count() == 2)
        local at, facing = m:marker_at("hook")
        assert(math.abs(at[1] - 5) < 1e-9 and math.abs(at[2] - 5) < 1e-9 and math.abs(at[3]) < 1e-9)
        -- turned 90 degrees about its own x, the marker's z points down
        assert(math.abs(facing[1]) < 1e-9 and math.abs(facing[2] + 1) < 1e-9 and math.abs(facing[3]) < 1e-9)
        assert(m:marker_at("nothing") == nil)
        return m
    )LUA", m, &err));
    if (!err.empty()) std::printf("    script error: %s\n", err.c_str());
    int node = -1;
    const ProcMarker* hook = m.findMarker("hook", &node);
    CHECK(hook != nullptr && node == m.findNode("boom"));
    if (hook != nullptr) {
        CHECK(hook->role == "hook");
        CHECK_APPROX(hook->size, 0.4, 1e-12);
        CHECK(near(hook->frame.position, Vec3(0, 0, 5)));
    }
    const ProcMarker* foot = m.findMarker("foot", &node);
    CHECK(foot != nullptr && node == 0);
    if (foot != nullptr) CHECK(foot->role == "point" && foot->size == 0);

    CHECK(!runModel(R"LUA(local m = model.new() m:marker("hook") m:node("boom"):marker("hook") return m)LUA", m, &err));
    CHECK(err.find("already has a marker named 'hook'") != std::string::npos);
    CHECK(!runModel(R"LUA(local m = model.new() m:marker("") return m)LUA", m, &err));
    CHECK(err.find("needs a name") != std::string::npos);
    CHECK(!runModel(R"LUA(
        local a, b = model.new(), model.new()
        a:marker("hook") b:marker("hook")
        return a:merge(b))LUA", m, &err));
    CHECK(err.find("disagree on marker 'hook'") != std::string::npos);
}

TEST_CASE(the_bake_makes_one_scene_material_per_material_of_the_model) {
    // Three parts in one textured material and one plain part: two scene materials and ONE copy of the map, where
    // every part used to bring its own.
    std::shared_ptr<ProcMaterial> brick = std::make_shared<ProcMaterial>();
    brick->textured = true;
    brick->albedo.width = 2; brick->albedo.height = 2; brick->albedo.channels = 3;
    brick->albedo.pixels.assign(12, 200);
    brick->roughness = 0.8f;
    brick->tile = 2.0;

    ProcModel m;
    const int made = m.addMaterial("brick", brick);
    m.addMaterial("spare", material(0.5f));                    // named but used by no part: not baked
    for (int i = 0; i < 3; ++i) {
        ProcPart p;
        p.mesh = MeshBuilder::box(Vec3(1, 1, 1));
        p.material = made;
        m.parts.push_back(p);
    }
    { ProcPart p; p.mesh = MeshBuilder::box(Vec3(1, 1, 1)); m.parts.push_back(p); }

    Scene scene;
    bakeProcModel(m, scene);
    CHECK(scene.triangles.size() == 48);
    CHECK(scene.materials.size() == 2);
    CHECK(scene.textures.size() == 1);
    const Material& first = scene.materials[static_cast<std::size_t>(scene.triangles[0].materialIndex)];
    CHECK(first.albedoTex == 0);
    CHECK_APPROX(first.roughness, 0.8, 1e-6);
    CHECK_APPROX(first.texScale, 0.5, 1e-12);
    CHECK(scene.triangles[0].materialIndex == scene.triangles[35].materialIndex);    // the three brick parts
    const Material& plain = scene.materials[static_cast<std::size_t>(scene.triangles[36].materialIndex)];
    CHECK(plain.albedoTex == -1);
    CHECK(scene.triangles[36].materialIndex != scene.triangles[0].materialIndex);
}

TEST_CASE(the_game_gives_each_part_its_materials_numbers) {
    const TmpFile recipe("test_model_markers_tmp.lua", R"LUA(
        local m = model.new()
        m:material("paint", material.new{ roughness = 0.31, metallic = 0.55 })
        m:material("glass", material.new{ roughness = 0.05, opacity = 0.3, two_sided = true })
        m:add(mesh.box{ 1, 1, 1 }, "paint")
        m:node("door", { pos = { 2, 0, 0 } }):add(mesh.box{ 1, 1, 0.1 }, "glass")
        m:add(mesh.box{ 1, 1, 1 })
        return m
    )LUA");
    const TmpFile level("test_model_markers_tmp.json", R"({
      "version": 1,
      "entities": [ { "name": "car", "shape": "script", "file": "test_model_markers_tmp.lua" } ]
    })");
    const TmpFile sidecar("test_model_markers_tmp.json.cameras.json", "");

    std::unique_ptr<Renderer> renderer = Renderer::create();
    RendererMeshUploader uploader(*renderer);
    AssetManager assets(uploader);
    World world;
    RenderView view;
    CHECK(LevelLoader::load(level.path, world, *renderer, view, assets, false));
    int paint = 0, glass = 0, plain = 0;
    world.each<Renderable, Transform>([&](Entity, Renderable& r, Transform& t) {
        if (std::abs(r.material.roughness - 0.31f) < 1e-6f && std::abs(r.material.metallic - 0.55f) < 1e-6f) ++paint;
        else if (std::abs(r.material.opacity - 0.3f) < 1e-6f && (r.material.flags & RenderMaterial::FLAG_TWO_SIDED) != 0 &&
                 near(t.position, Vec3(2, 0, 0))) ++glass;
        else if (std::abs(r.material.roughness - ProcMaterial{}.roughness) < 1e-6f && r.material.opacity == 1.0f) ++plain;
    });
    CHECK(paint == 1);
    CHECK(glass == 1);
    CHECK(plain == 1);
    world.destroyAll();
    assets.clear();
}
