#include "test_framework.h"

#include "../src/engine/level_writer.h"
#include "../src/engine/components.h"
#include "../src/engine/editor_bridge.h"
#include "../src/engine/camera/scene_camera.h"
#include "../src/engine/model_importer.h"
#include "../src/engine/property_json.h"
#include "../src/log.h"
#include <nlohmann/json.hpp>
#include <cstdio>
#include <fstream>

using namespace engine;  // namespace migration (ADR-0015)
using json = nlohmann::json;

namespace {
const char* TMP_PATH = "test_level_writer_tmp.json";

struct TmpFile {
    ~TmpFile() {
        std::remove(TMP_PATH);
        // saveDocument also writes the camera sidecar next to the level.
        std::remove((std::string(TMP_PATH) + ".cameras.json").c_str());
    }
};

Entity addBox(World& world, const Vec3& pos) {
    Entity e = world.create();
    Transform t;
    t.position = pos;
    world.add<Transform>(e, t);
    SourceSpec spec;
    spec.shape = "box";
    spec.size = Vec3(2, 1, 3);
    world.add<SourceSpec>(e, spec);
    world.add<Renderable>(e);
    return e;
}
}  // namespace

TEST_CASE(quat_axis_angle_round_trips) {
    Vec3 axis;
    Real angle;
    Quat q = Quat::fromAxisAngle(Vec3(1, 2, -0.5), 1.2345);
    q.toAxisAngle(axis, angle);
    Quat back = Quat::fromAxisAngle(axis, angle);
    Vec3 v = Vec3(0.3, -0.7, 0.9);
    Vec3 a = q.rotate(v), b = back.rotate(v);
    CHECK_APPROX(a.x, b.x, 1e-9);
    CHECK_APPROX(a.y, b.y, 1e-9);
    CHECK_APPROX(a.z, b.z, 1e-9);

    Quat::identity().toAxisAngle(axis, angle);   // degenerate case is defined
    CHECK_APPROX(angle, 0.0, 1e-9);
}

TEST_CASE(quat_from_rotation_matrix_round_trips) {
    const Real angles[] = {0.3, 1.7, 2.9, -0.8};
    for (Real a : angles) {
        Quat q = Quat::fromAxisAngle(normalize(Vec3(0.4, 1.0, -0.2)), a);
        // Build the rotation matrix from the quat's basis vectors (columns).
        Mat4 m;
        Vec3 cx = q.rotate(Vec3(1, 0, 0));
        Vec3 cy = q.rotate(Vec3(0, 1, 0));
        Vec3 cz = q.rotate(Vec3(0, 0, 1));
        m.m[0][0] = cx.x; m.m[1][0] = cx.y; m.m[2][0] = cx.z;
        m.m[0][1] = cy.x; m.m[1][1] = cy.y; m.m[2][1] = cy.z;
        m.m[0][2] = cz.x; m.m[1][2] = cz.y; m.m[2][2] = cz.z;

        Quat back = Quat::fromRotationMatrix(m);
        Vec3 v(0.5, -0.2, 0.8);
        Vec3 expect = q.rotate(v), got = back.rotate(v);
        CHECK_APPROX(got.x, expect.x, 1e-9);
        CHECK_APPROX(got.y, expect.y, 1e-9);
        CHECK_APPROX(got.z, expect.z, 1e-9);
    }
}

TEST_CASE(world_destroy_all_bumps_generations) {
    World world;
    Entity a = world.create();
    world.add<Transform>(a);
    Entity b = world.create();
    world.add<Renderable>(b);

    world.destroyAll();
    CHECK(world.entityCount() == 0);
    CHECK(!world.alive(a));
    CHECK(!world.alive(b));
    CHECK(world.get<Transform>(a) == nullptr);

    Entity c = world.create();   // slots recycle with fresh generations
    CHECK(world.alive(c));
    CHECK(!(c == a));
}

TEST_CASE(level_writer_serializes_document_entities_only) {
    TmpFile cleanup;
    World world;

    Entity box = addBox(world, Vec3(1, 2, 3));
    Transform* t = world.get<Transform>(box);
    t->scale = Vec3(2, 2, 2);
    t->orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), degreesToRadians(30));
    Renderable* r = world.get<Renderable>(box);
    r->material.albedo = Vec3(0.9, 0.1, 0.2);
    r->material.metallic = 0.5f;
    r->material.flags = RenderMaterial::FLAG_CHECKERBOARD;
    SourceSpec* spec = world.get<SourceSpec>(box);
    spec->hasPhysics = true;
    spec->motion = "dynamic";
    spec->friction = 0.7;

    // A runtime spawn (no SourceSpec) must never reach the document.
    Entity bullet = world.create();
    world.add<Transform>(bullet);
    world.add<Renderable>(bullet);

    CHECK(LevelWriter::save(TMP_PATH, world));

    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    CHECK(root["entities"].size() == 1);

    const json& ent = root["entities"][0];
    CHECK(ent["shape"] == "box");
    CHECK_APPROX(ent["size"][2].get<double>(), 3.0, 1e-9);
    CHECK_APPROX(ent["position"][1].get<double>(), 2.0, 1e-9);
    CHECK_APPROX(ent["scale"][0].get<double>(), 2.0, 1e-9);
    CHECK_APPROX(ent["orientation"]["angleDeg"].get<double>(), 30.0, 1e-6);
    CHECK_APPROX(ent["orientation"]["axis"][1].get<double>(), 1.0, 1e-9);
    CHECK_APPROX(ent["material"]["albedo"][0].get<double>(), 0.9, 1e-6);
    CHECK_APPROX(ent["material"]["metallic"].get<double>(), 0.5, 1e-6);
    // Property-layer JSON spells flags as bools (the loaders also still read
    // the pre-migration "flags" array form).
    CHECK(ent["material"]["checkerboard"].get<bool>());
    CHECK(ent["physics"]["motion"] == "dynamic");
    CHECK_APPROX(ent["physics"]["friction"].get<double>(), 0.7, 1e-9);
}

TEST_CASE(level_writer_emits_named_materials_table) {
    TmpFile cleanup;
    World world;

    // Two entities sharing one named material asset (SourceSpec.materialName),
    // plus one with an inline material.
    for (int i = 0; i < 2; ++i) {
        Entity e = addBox(world, Vec3(i, 0, 0));
        world.get<SourceSpec>(e)->materialName = "brickWall";
        Renderable* r = world.get<Renderable>(e);
        r->material.albedo = Vec3(0.6, 0.3, 0.2);
        r->material.roughness = 0.85f;
        r->material.setSurface(RenderMaterial::Surface::Brick);
    }
    Entity inlineBox = addBox(world, Vec3(5, 0, 0));
    world.get<Renderable>(inlineBox)->material.albedo = Vec3(0.1, 0.8, 0.2);

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);

    // The shared material is referenced by name and defined once in the table.
    CHECK(root.contains("materials"));
    CHECK(root["materials"].contains("brickWall"));
    CHECK_APPROX(root["materials"]["brickWall"]["albedo"][0].get<double>(), 0.6, 1e-6);

    int refs = 0, inlineObjs = 0;
    for (const json& ent : root["entities"]) {
        if (!ent.contains("material")) continue;
        if (ent["material"].is_string()) {
            CHECK(ent["material"] == "brickWall");
            refs++;
        } else if (ent["material"].is_object()) {
            inlineObjs++;
        }
    }
    CHECK(refs == 2);          // both shared entities reference the name
    CHECK(inlineObjs == 1);    // the inline one stays inline
}

TEST_CASE(level_writer_preserves_surface_through_play_save) {
    // The Play loop saves then reloads. A textured material's "surface" lives
    // only in the named table (binding the baked set clears the runtime flag),
    // so the writer must keep the existing definition or Play loses the textures.
    TmpFile cleanup;
    {
        std::ofstream f(TMP_PATH);
        f << R"({"version":1,"materials":{"brick":{"albedo":[1,1,1],"surface":"brick"}}})";
    }

    World world;
    Entity e = addBox(world, Vec3(0, 0, 0));
    world.get<SourceSpec>(e)->materialName = "brick";
    // Runtime material as it looks after the loader bound textures: no surface.
    world.get<Renderable>(e)->material.albedo = Vec3(1, 1, 1);

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    CHECK(root["materials"]["brick"].contains("surface"));
    CHECK(root["materials"]["brick"]["surface"] == "brick");   // survived the round-trip
    CHECK(root["entities"][0]["material"] == "brick");
}

TEST_CASE(level_writer_serializes_inline_surface) {
    // An inline (unnamed) textured material round-trips its surface: the writer
    // emits "surface" so a Play save→reload re-bakes the textures.
    TmpFile cleanup;
    World world;
    Entity e = addBox(world, Vec3(0, 0, 0));
    Renderable* r = world.get<Renderable>(e);
    r->material.albedo = Vec3(1, 1, 1);
    r->material.setSurface(RenderMaterial::Surface::Cobblestone);   // textured, no name

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    const json& mat = root["entities"][0]["material"];
    CHECK(mat.is_object());                       // inline, not a named reference
    CHECK(mat["surface"] == "cobblestone");       // surface survived the round-trip
}

TEST_CASE(level_writer_preserves_unowned_sections) {
    TmpFile cleanup;
    {
        std::ofstream f(TMP_PATH);
        f << R"({
          "version": 1,
          "environment": { "skyColor": [0.1, 0.2, 0.3], "hdr": "../env/x.hdr" },
          "lighting": { "sun": { "intensity": 5.0 } },
          "player": { "position": [0, 1, 0] },
          "entities": [ { "shape": "torus", "position": [9, 9, 9] } ]
        })";
    }

    World world;
    addBox(world, Vec3(4, 5, 6));
    CHECK(LevelWriter::save(TMP_PATH, world));

    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    // Entities are replaced by the world's document entities...
    CHECK(root["entities"].size() == 1);
    CHECK(root["entities"][0]["shape"] == "box");
    // ...while sections the editor doesn't own survive verbatim.
    CHECK(root["environment"]["hdr"] == "../env/x.hdr");
    CHECK_APPROX(root["lighting"]["sun"]["intensity"].get<double>(), 5.0, 1e-9);
    CHECK_APPROX(root["player"]["position"][1].get<double>(), 1.0, 1e-9);
    CHECK(root["version"] == 1);
}

TEST_CASE(level_writer_serializes_entity_names) {
    TmpFile cleanup;
    World world;
    Entity box = addBox(world, Vec3(0, 0, 0));
    world.get<SourceSpec>(box)->name = "Crate A";
    addBox(world, Vec3(1, 0, 0));   // unnamed: no "name" key written

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    bool sawNamed = false, sawAnonymous = false;
    for (const auto& ent : root["entities"]) {
        if (ent.contains("name"))
            sawNamed = (ent["name"] == "Crate A");
        else
            sawAnonymous = true;
    }
    CHECK(sawNamed);
    CHECK(sawAnonymous);

    // Named entities stand alone in the hierarchy; unnamed keep shape #id.
    EditorBridge bridge;
    bridge.attach(&world, nullptr, TMP_PATH);
    bool labelled = false, fallback = false;
    for (const auto& info : bridge.listEntities()) {
        labelled |= info.label == "Crate A";
        fallback |= info.label.find("box #") == 0;
    }
    CHECK(labelled);
    CHECK(fallback);
}

TEST_CASE(editor_bridge_swaps_environment_hdr) {
    TmpFile cleanup;
    {
        std::ofstream f(TMP_PATH);
        f << R"({ "version": 1,
                  "environment": { "skyColor": [0.1, 0.2, 0.3] },
                  "entities": [] })";
    }
    World world;
    addBox(world, Vec3(0, 0, 0));

    EditorBridge bridge;
    bridge.attach(&world, nullptr, TMP_PATH);
    CHECK(bridge.environmentHdr().empty());

    CHECK(bridge.setEnvironmentHdr("../env/studio.hdr"));
    CHECK(bridge.environmentHdr() == "../env/studio.hdr");
    {
        std::ifstream f(TMP_PATH);
        json root = json::parse(f);
        CHECK(root["environment"]["hdr"] == "../env/studio.hdr");
        // The rest of the environment block survives the edit.
        CHECK_APPROX(root["environment"]["skyColor"][2].get<double>(), 0.3,
                     1e-9);
        CHECK(root["entities"].size() == 1);   // entities were saved first
    }

    // Removing the HDR hands lighting back to the procedural sky.
    CHECK(bridge.setEnvironmentHdr(""));
    CHECK(bridge.environmentHdr().empty());
    {
        std::ifstream f(TMP_PATH);
        json root = json::parse(f);
        CHECK(!root["environment"].contains("hdr"));
        CHECK_APPROX(root["environment"]["skyColor"][0].get<double>(), 0.1,
                     1e-9);
    }

    // Observer mode may not touch the document's environment either.
    bridge.attachObserver(&world, TMP_PATH);
    CHECK(!bridge.setEnvironmentHdr("../env/other.hdr"));
}

TEST_CASE(level_writer_serializes_hierarchy_and_groups) {
    TmpFile cleanup;
    World world;

    // A group (null object) with a box parented under it.
    Entity group = world.create();
    world.add<Transform>(group);
    SourceSpec gs;
    gs.id = 10;
    gs.shape = "";          // group
    gs.name = "Rig";
    world.add<SourceSpec>(group, gs);

    Entity child = addBox(world, Vec3(1, 0, 0));
    SourceSpec* cs = world.get<SourceSpec>(child);
    cs->id = 11;
    cs->parentId = 10;

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);

    bool sawGroup = false, sawChild = false;
    for (const auto& ent : root["entities"]) {
        if (ent.value("id", 0u) == 10) {
            sawGroup = true;
            CHECK(ent.value("group", false));
            CHECK(ent["name"] == "Rig");
            CHECK(!ent.contains("shape"));
        }
        if (ent.value("id", 0u) == 11) {
            sawChild = true;
            CHECK(ent.value("parent", 0u) == 10);
        }
    }
    CHECK(sawGroup);
    CHECK(sawChild);
}

TEST_CASE(level_writer_assigns_ids_to_unidentified_entities) {
    TmpFile cleanup;
    World world;
    addBox(world, Vec3(0, 0, 0));   // no id set
    addBox(world, Vec3(1, 0, 0));

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    // Every saved entity carries a unique, nonzero id.
    uint32_t a = root["entities"][0].value("id", 0u);
    uint32_t b = root["entities"][1].value("id", 0u);
    CHECK(a != 0);
    CHECK(b != 0);
    CHECK(a != b);
}

TEST_CASE(editor_bridge_reparent_preserves_world_position) {
    World world;
    Entity group = addBox(world, Vec3(10, 0, 0));   // stand-in for a group
    Entity ball = addBox(world, Vec3(5, 0, 0));
    world.get<SourceSpec>(group)->id = 1;
    world.get<SourceSpec>(ball)->id = 2;

    EditorBridge bridge;
    bridge.attach(&world, nullptr, "level.json");

    // Parenting must NOT move the ball in the world: its local transform is
    // rewritten relative to the parent (5 - 10 = -5), world stays (5,0,0).
    bridge.reparent(ball, group);
    CHECK(world.get<SourceSpec>(ball)->parentId == 1);
    CHECK_APPROX(world.get<Transform>(ball)->position.x, -5.0, 1e-9);
    Vec3 worldPos = worldMatrix(world, ball).transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(worldPos.x, 5.0, 1e-9);

    // Moving the group carries the child by its preserved offset.
    world.get<Transform>(group)->position = Vec3(20, 0, 0);
    Vec3 moved = worldMatrix(world, ball).transformPoint(Vec3(0, 0, 0));
    CHECK_APPROX(moved.x, 15.0, 1e-9);   // 20 + (-5)

    // Un-parenting back to root also keeps world position (now 15).
    bridge.reparent(ball, Entity{});
    CHECK(world.get<SourceSpec>(ball)->parentId == 0);
    CHECK_APPROX(world.get<Transform>(ball)->position.x, 15.0, 1e-9);
}

TEST_CASE(editor_bridge_reparents_with_cycle_guard) {
    World world;
    Entity a = addBox(world, Vec3(0, 0, 0));
    Entity b = addBox(world, Vec3(1, 0, 0));
    world.get<SourceSpec>(a)->id = 1;
    world.get<SourceSpec>(b)->id = 2;

    EditorBridge bridge;
    bridge.attach(&world, nullptr, "level.json");

    // Parent b under a (a is at origin, so b's local stays (1,0,0)).
    bridge.reparent(b, a);
    CHECK(world.get<SourceSpec>(b)->parentId == 1);

    // listEntities exposes the graph for the tree.
    bool childLinked = false;
    for (const auto& info : bridge.listEntities())
        if (info.entity == b) childLinked = (info.parentId == 1);
    CHECK(childLinked);

    // Parenting a under b would cycle — refused, link unchanged.
    bridge.reparent(a, b);
    CHECK(world.get<SourceSpec>(a)->parentId == 0);

    // Reparent to root (invalid parent).
    bridge.reparent(b, Entity{});
    CHECK(world.get<SourceSpec>(b)->parentId == 0);

    // Observer mode refuses reparenting.
    bridge.attachObserver(&world, "level.json");
    bridge.reparent(b, a);
    CHECK(world.get<SourceSpec>(b)->parentId == 0);
}

TEST_CASE(level_writer_round_trips_mesh_entities) {
    TmpFile cleanup;
    World world;
    Entity e = world.create();
    world.add<Transform>(e);
    SourceSpec spec;
    spec.shape = "";
    spec.meshFile = "models/crate.gltf";
    world.add<SourceSpec>(e, spec);

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    CHECK(root["entities"][0]["mesh"] == "models/crate.gltf");
    CHECK(!root["entities"][0].contains("shape"));
}

TEST_CASE(level_writer_round_trips_recipe_entities) {
    // Regression: a procedural recipe entity (shape:"tree") used to carry no
    // SourceSpec, so the writer dropped it on save — the tree vanished on the
    // edit->play "compile" (silent data loss). It must now survive, recipe and
    // all, with no auto size/material block leaking in.
    TmpFile cleanup;
    World world;
    Entity tree = world.create();
    Transform t;
    t.position = Vec3(0, 0, -8);
    world.add<Transform>(tree, t);
    SourceSpec spec;
    spec.shape = "tree";
    spec.recipe = R"({"seed":7,"iterations":6,"leafColor":[0.2,0.46,0.14]})";
    spec.hasPhysics = true;
    spec.motion = "static";
    world.add<SourceSpec>(tree, spec);
    Renderable r;                       // the bark render entity also has a material
    r.material.albedo = Vec3(1, 1, 1);
    world.add<Renderable>(tree, r);

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    CHECK(root["entities"].size() == 1);

    const json& ent = root["entities"][0];
    CHECK(ent["shape"] == "tree");
    CHECK(ent.contains("tree"));
    CHECK(ent["tree"]["seed"].get<int>() == 7);
    CHECK(ent["tree"]["iterations"].get<int>() == 6);
    CHECK_APPROX(ent["tree"]["leafColor"][1].get<double>(), 0.46, 1e-9);
    CHECK_APPROX(ent["position"][2].get<double>(), -8.0, 1e-9);
    CHECK(ent["physics"]["motion"] == "static");
    // The generator owns size and color — neither leaks into the document.
    CHECK(!ent.contains("size"));
    CHECK(!ent.contains("material"));
}

TEST_CASE(level_writer_round_trips_script_recipe_flat) {
    // Regression: a Lua recipe entity (shape:"script", ADR-0042) reads its params
    // (file/seed) from the entity's top level, not a nested block. The writer must
    // emit them flat so a Play save->reload reproduces the original
    // `{shape:"script", file, seed}` — otherwise the recipe is dropped and the
    // world comes back empty ("the world disappears the instant Play reloads").
    TmpFile cleanup;
    World world;
    Entity script = world.create();
    Transform t;
    world.add<Transform>(script, t);
    SourceSpec spec;
    spec.shape = "script";
    spec.recipe = R"({"file":"city.lua","seed":7})";
    world.add<SourceSpec>(script, spec);

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    CHECK(root["entities"].size() == 1);

    const json& ent = root["entities"][0];
    CHECK(ent["shape"] == "script");
    CHECK(ent["file"] == "city.lua");          // flat, not nested under "script"
    CHECK(ent["seed"].get<int>() == 7);
    CHECK(!ent.contains("script"));            // no nested recipe block
    CHECK(!ent.contains("size"));              // the recipe owns its geometry
    CHECK(!ent.contains("material"));
}

TEST_CASE(editor_bridge_lists_and_selects) {
    World world;
    Entity box = addBox(world, Vec3(0, 0, 0));
    Entity cam = world.create();
    world.add<Transform>(cam);
    SceneCamera sc;
    sc.name = "Crane";
    world.add<SceneCamera>(cam, sc);

    EditorBridge bridge;
    CHECK(!bridge.attached());
    CHECK(bridge.listEntities().empty());   // detached: inert, not crashing

    bridge.attach(&world, nullptr, "level.json");
    auto list = bridge.listEntities();
    CHECK(list.size() == 2);
    bool sawBox = false, sawCam = false;
    for (const auto& info : list) {
        if (info.entity == box) {
            sawBox = true;
            CHECK(info.label.find("box") != std::string::npos);
            CHECK(!info.isCamera);
        }
        if (info.entity == cam) {
            sawCam = true;
            CHECK(info.label == "Crane");
            CHECK(info.isCamera);
        }
    }
    CHECK(sawBox && sawCam);

    bridge.deleteEntity(box);
    CHECK(!world.alive(box));
    CHECK(bridge.listEntities().size() == 1);

    bridge.detach();
    CHECK(!bridge.attached());
}

TEST_CASE(editor_bridge_observer_mode_is_read_only) {
    TmpFile cleanup;
    World world;
    Entity box = addBox(world, Vec3(1, 2, 3));
    // A play-style entity: the live player (no SourceSpec).
    Entity player = world.create();
    world.add<Transform>(player);
    world.add<ControlledBy>(player);

    EditorBridge bridge;
    bridge.attachObserver(&world, TMP_PATH);
    CHECK(bridge.attached());
    CHECK(!bridge.editable());

    // Panels can look: the hierarchy lists document entities AND the player.
    auto list = bridge.listEntities();
    bool sawPlayer = false;
    for (const auto& info : list)
        if (info.entity == player) sawPlayer = true;
    CHECK(sawPlayer);
    CHECK(list.size() == 2);

    // Selection is bridge-held (no editor system owns one during play).
    bridge.select(box);
    CHECK(bridge.selected() == box);

    // ...but nothing touches: document writes refused, mutations no-ops,
    // and there is no command log to record onto.
    CHECK(!bridge.saveDocument());
    bridge.deleteEntity(box);
    CHECK(world.alive(box));
    bridge.addComponent(box, "Camera");
    CHECK(!world.has<SceneCamera>(box));
    CHECK(bridge.undoStack() == nullptr);
    CHECK(!bridge.documentDirty());

    // The one deliberate observer write: bake the live world over the
    // document — what physics did becomes the level.
    world.get<Transform>(box)->position = Vec3(9, 0.5, -2);   // "settled"
    CHECK(bridge.bakePlaytestToDocument());
    {
        std::ifstream f(TMP_PATH);
        json root = json::parse(f);
        CHECK(root["entities"].size() == 1);   // the player never leaks in
        CHECK_APPROX(root["entities"][0]["position"][0].get<double>(), 9.0,
                     1e-9);
    }

    bridge.detach();
    CHECK(!bridge.attached());
    CHECK(!bridge.bakePlaytestToDocument());   // detached: refused
}

TEST_CASE(registry_runtime_rows_are_display_only) {
    World world;
    Entity e = world.create();
    world.add<Transform>(e);
    world.add<Velocity>(e, {Vec3(1, 2, 3), Vec3()});
    RigidBody rb;
    rb.motion = BodyMotion::Dynamic;
    world.add<RigidBody>(e, rb);
    world.add<ControlledBy>(e, ControlledBy{0});

    ComponentRegistry registry;
    registerEngineComponents(registry);

    // The playtest rows enumerate and walk...
    int present = 0;
    for (const auto& entry : registry.entries()) {
        if (!entry.has(world, e)) continue;
        present++;
        nlohmann::json j;
        JsonWriteVisitor writer(j);
        entry.visit(world, e, writer);
        if (entry.name == "Velocity")
            CHECK_APPROX(j["linear"][0].get<double>(), 1.0, 1e-9);
        if (entry.name == "Rigid Body") CHECK(j["motion"] == "dynamic");
    }
    CHECK(present == 4);   // Transform, Velocity, Rigid Body, Controlled By

    // ...and none of them are authorable from the menu.
    CHECK(!registry.find("Velocity")->addTo);
    CHECK(!registry.find("Rigid Body")->addTo);
    CHECK(!registry.find("Controlled By")->removeFrom);
}

TEST_CASE(editor_bridge_notifies_shell_of_state_changes) {
    TmpFile cleanup;
    World world;
    Entity box = addBox(world, Vec3(0, 0, 0));

    EditorBridge bridge;
    bridge.attach(&world, nullptr, TMP_PATH);

    // Attach queued a mode change; draining empties the queue.
    auto notices = bridge.drainNotices();
    bool sawMode = false;
    for (EditorNotice n : notices) sawMode |= (n == EditorNotice::ModeChanged);
    CHECK(sawMode);
    CHECK(bridge.drainNotices().empty());

    bridge.select(box);
    notices = bridge.drainNotices();
    CHECK(notices.size() == 1);
    CHECK(notices[0] == EditorNotice::SelectionChanged);
    bridge.select(box);   // no change, no notice
    CHECK(bridge.drainNotices().empty());

    CHECK(bridge.saveDocument());
    notices = bridge.drainNotices();
    CHECK(notices.size() == 1);
    CHECK(notices[0] == EditorNotice::DocumentSaved);
}

TEST_CASE(log_sink_receives_each_line) {
    std::vector<std::string> lines;
    engine::logging::setSink(
        [&](engine::logging::Level level, const std::string& text) {
            if (level == engine::logging::Level::Warn) lines.push_back(text);
        });
    LOG_WARN << "sink" << 42;
    LOG_INFO << "filtered by the test sink";
    engine::logging::setSink(nullptr);
    LOG_WARN << "after clear: not captured";

    CHECK(lines.size() == 1);
    CHECK(lines[0] == "sink42");
}

TEST_CASE(model_importer_validate_checks_gltf) {
    std::string error;
    CHECK(!ModelImporter::validate("does_not_exist.gltf", error));
    CHECK(!error.empty());

    // A minimal but valid glTF 2.0 document parses clean.
    const char* path = "test_minimal_tmp.gltf";
    {
        std::ofstream f(path);
        f << R"({ "asset": { "version": "2.0" } })";
    }
    error.clear();
    bool ok = ModelImporter::validate(path, error);
    std::remove(path);
    CHECK(ok);
}

TEST_CASE(level_writer_syncs_player_spawn_into_player_block) {
    TmpFile cleanup;
    {
        std::ofstream f(TMP_PATH);
        f << R"({ "version": 1,
                  "player": { "position": [0, 1, 0], "collider": { "radius": 0.3 } },
                  "entities": [] })";
    }

    World world;
    Entity spawn = world.create();
    Transform t;
    t.position = Vec3(7, 2, -3);
    world.add<Transform>(spawn, t);
    world.add<PlayerSpawn>(spawn);

    CHECK(LevelWriter::save(TMP_PATH, world));
    std::ifstream f(TMP_PATH);
    json root = json::parse(f);
    // Position follows the moved spawn entity; the rest of the block survives.
    CHECK_APPROX(root["player"]["position"][0].get<double>(), 7.0, 1e-9);
    CHECK_APPROX(root["player"]["position"][2].get<double>(), -3.0, 1e-9);
    CHECK_APPROX(root["player"]["collider"]["radius"].get<double>(), 0.3, 1e-9);
}
