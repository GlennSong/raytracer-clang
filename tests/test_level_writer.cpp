#include "test_framework.h"

#include "../src/engine/level_writer.h"
#include "../src/engine/components.h"
#include "../src/engine/editor_bridge.h"
#include "../src/engine/camera/scene_camera.h"
#include "../src/engine/model_importer.h"
#include "../src/log.h"
#include <nlohmann/json.hpp>
#include <cstdio>
#include <fstream>

using namespace engine;  // namespace migration (ADR-0015)
using json = nlohmann::json;

namespace {
const char* TMP_PATH = "test_level_writer_tmp.json";

struct TmpFile {
    ~TmpFile() { std::remove(TMP_PATH); }
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

    bridge.detach();
    CHECK(!bridge.attached());
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
