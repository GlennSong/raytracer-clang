// City Planner bridge + panel coverage (P7.3 phase 1), run inside the
// editor_qt_tests offscreen QApplication (called from
// test_property_inspector_qt.cpp's main). Exercises the real pipeline on a
// minimal in-memory world with a generated metro road: recipe round-trip,
// graph-only regenerate with sane stats, SourceSpec preservation, overlay
// layer entities + toggles, camera presets, and the Qt panel driving it all.

#include "../src/editor_app/city_planner_panel.h"
#include "../src/engine/asset_manager.h"
#include "../src/engine/components.h"
#include "../src/engine/editor_bridge.h"
#include "../src/engine/procgen/city/road_net.h"
#include "../src/engine/systems/editor_system.h"

#include <QCheckBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <cmath>
#include <cstdio>

#include <nlohmann/json.hpp>

using namespace engine;
using nlohmann::json;

namespace {

int failures = 0;
#define REQUIRE(cond)                                                         \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
            failures++;                                                       \
        }                                                                     \
    } while (0)

// Counts uploads/removes with fake handles (the test_asset_manager pattern)
// so overlay/bake lifetime is verifiable with no GPU backend.
struct StubUploader : MeshUploader {
    int uploads = 0;
    int removes = 0;
    uint32_t next = 1;

    MeshHandle uploadMesh(const RenderMesh&) override {
        uploads++;
        return MeshHandle{next++, 1};
    }
    void removeMesh(MeshHandle) override { removes++; }
    BoundingSphere getMeshBounds(MeshHandle) const override { return {}; }
};

int visibleGroups(World& world) {
    int n = 0;
    world.each<InstanceGroup>([&](Entity, InstanceGroup& g) {
        if (!g.transforms.empty() && g.mesh.valid()) ++n;
    });
    return n;
}

}  // namespace

int runCityPlannerQtTests() {
    failures = 0;

    // A generated road entity, built the way the loader builds one: the
    // recipe rides SourceSpec, the RoadNet starts empty (the planner's
    // regenerate fills it — the graph-only path under test).
    World world;
    json gen = {
        {"kind", "metro"}, {"radius", 300.0},  {"hotspots", 3},
        {"seed", 11},      {"freeways", false}, {"block_size", 80.0},
    };
    Entity road = world.create();
    world.add<Transform>(road);
    SourceSpec spec;
    spec.shape = "road";
    spec.id = 1;
    json roadBlock;
    roadBlock["width"] = 10.0;
    roadBlock["generate"] = gen;
    spec.recipe = roadBlock.dump();
    world.add<SourceSpec>(road, spec);
    RoadNet net;
    net.width = 10.0;
    net.autoRoundabout = false;
    world.add<RoadNet>(road, net);
    world.add<Renderable>(road);

    EditorBridge bridge;
    CameraSystem cameras;
    EditorSystem editor(cameras, "planner_test.json", nullptr, &bridge);
    bridge.attach(&world, &editor, "planner_test.json");
    StubUploader uploader;
    AssetManager assets(uploader);
    bridge.attachPlanner(&assets, nullptr, &cameras);

    // --- recipe round-trip --------------------------------------------------
    std::string rec = bridge.plannerRecipe();
    REQUIRE(!rec.empty());
    json parsed = json::parse(rec, nullptr, false);
    REQUIRE(parsed.is_object());
    REQUIRE(parsed.value("seed", 0) == 11);

    // --- graph-only regenerate: sane stats, RoadNet + SourceSpec updated ----
    parsed["seed"] = 12;
    CityPlannerStats stats = bridge.plannerApplyRecipe(parsed.dump());
    REQUIRE(stats.ok);
    REQUIRE(stats.nodes > 10);
    REQUIRE(stats.edges > 10);
    REQUIRE(stats.spans > 0);
    REQUIRE(stats.spans <= stats.edges);
    REQUIRE(stats.hubs >= 1);   // metro recipes grow cityHubs
    REQUIRE(stats.regenMs >= 0.0);

    RoadNet* live = world.get<RoadNet>(road);
    REQUIRE(live && static_cast<int>(live->nodes.size()) == stats.nodes);
    REQUIRE(live && static_cast<int>(live->edges.size()) == stats.edges);

    json saved = json::parse(world.get<SourceSpec>(road)->recipe, nullptr, false);
    REQUIRE(saved.is_object() && saved.contains("generate"));
    REQUIRE(saved["generate"].value("seed", 0) == 12);
    // Keys the caller didn't touch survive the round-trip.
    REQUIRE(saved["generate"].value("block_size", 0.0) == 80.0);
    REQUIRE(saved.value("width", 0.0) == 10.0);   // the look block survives too

    // --- determinism: the same recipe grows the same graph ------------------
    CityPlannerStats again = bridge.plannerApplyRecipe(parsed.dump());
    REQUIRE(again.nodes == stats.nodes);
    REQUIRE(again.edges == stats.edges);
    REQUIRE(again.spans == stats.spans);
    REQUIRE(again.hubs == stats.hubs);

    // --- overlays: layer groups exist, toggles hide/show, no accumulation ---
    REQUIRE(visibleGroups(world) == 3);   // hubs + arterials + nodes
    bridge.plannerSetLayer("arterials", false);
    REQUIRE(visibleGroups(world) == 2);
    bridge.plannerSetLayer("arterials", true);
    REQUIRE(visibleGroups(world) == 3);
    // Two rebuilds happened; the first rebuild's meshes were released (the
    // manager frees at refcount zero), so exactly one mesh set is live.
    REQUIRE(uploader.removes >= 3);

    // --- bake: routed through the AssetManager, swap not leak ---------------
    const int uploadsBeforeBake = uploader.uploads;
    REQUIRE(bridge.plannerBakeMesh());
    REQUIRE(uploader.uploads == uploadsBeforeBake + 1);
    MeshHandle firstBake = world.get<Renderable>(road)->mesh;
    REQUIRE(firstBake.valid());
    const int removesBefore = uploader.removes;
    REQUIRE(bridge.plannerBakeMesh());   // second bake releases the first
    REQUIRE(uploader.removes == removesBefore + 1);
    REQUIRE(world.get<Renderable>(road)->mesh.valid());
    REQUIRE(!(world.get<Renderable>(road)->mesh == firstBake));

    // --- camera presets: controller + projection ----------------------------
    bridge.plannerCameraPreset(0);   // Top
    REQUIRE(!cameras.flyControllerActive());
    REQUIRE(cameras.orbitController().isOrthographic());
    REQUIRE(cameras.orbitController().pitch > 80.0);
    REQUIRE(cameras.orbitController().yaw == 0.0);
    REQUIRE(cameras.orbitController().distance >= 2.0 * 300.0 - 1e-6);
    bridge.plannerCameraPreset(1);   // Iso
    REQUIRE(cameras.orbitController().isOrthographic());
    REQUIRE(std::fabs(cameras.orbitController().yaw - 45.0) < 1e-9);
    REQUIRE(std::fabs(cameras.orbitController().pitch - 35.264) < 1e-6);
    bridge.plannerCameraPreset(2);   // Free
    REQUIRE(cameras.flyControllerActive());
    REQUIRE(!cameras.flyController().isOrthographic());

    // --- the Qt panel drives the same path ----------------------------------
    CityPlannerPanel panel(bridge);
    panel.refresh();   // editable -> loads the recipe into the widgets
    auto* seedSpin = panel.findChild<QSpinBox*>("plannerSeed");
    REQUIRE(seedSpin != nullptr);
    if (seedSpin) {
        REQUIRE(seedSpin->value() == 12);   // read from the saved recipe
        seedSpin->setValue(13);
    }
    auto* regen = panel.findChild<QPushButton*>("plannerRegenerate");
    REQUIRE(regen != nullptr);
    if (regen) {
        regen->click();
        json after = json::parse(bridge.plannerRecipe(), nullptr, false);
        REQUIRE(after.is_object() && after.value("seed", 0) == 13);
        REQUIRE(after.value("block_size", 0.0) == 80.0);   // undisplayed key kept
        auto* statsLabel = panel.findChild<QLabel*>("plannerStats");
        REQUIRE(statsLabel && statsLabel->text().contains("nodes"));
    }
    auto* hubsBox = panel.findChild<QCheckBox*>("plannerLayer_hubs");
    REQUIRE(hubsBox != nullptr);
    if (hubsBox) {
        hubsBox->setChecked(false);
        REQUIRE(visibleGroups(world) == 2);
        hubsBox->setChecked(true);
        REQUIRE(visibleGroups(world) == 3);
    }

    // --- reopen: a NEW session must invalidate the panel's cached recipe ----
    // Opening a level from the asset browser swaps editor states within ONE
    // engine frame, so a polling panel never observes an editable() dip —
    // only the bridge's attach generation moves. (A boolean latch here kept
    // the previous level's recipe: "No recipe to regenerate" on the freshly
    // opened level.)
    const uint64_t genBefore = bridge.attachGeneration();
    World world2;
    Entity road2 = world2.create();
    world2.add<Transform>(road2);
    SourceSpec spec2;
    spec2.shape = "road";
    spec2.id = 1;
    json roadBlock2;
    roadBlock2["width"] = 10.0;
    json gen2 = gen;
    gen2["seed"] = 21;
    roadBlock2["generate"] = gen2;
    spec2.recipe = roadBlock2.dump();
    world2.add<SourceSpec>(road2, spec2);
    RoadNet net2;
    net2.width = 10.0;
    net2.autoRoundabout = false;
    world2.add<RoadNet>(road2, net2);
    world2.add<Renderable>(road2);
    EditorSystem editor2(cameras, "planner_test2.json", nullptr, &bridge);
    bridge.attach(&world2, &editor2, "planner_test2.json");
    bridge.attachPlanner(&assets, nullptr, &cameras);
    REQUIRE(bridge.attachGeneration() != genBefore);
    panel.refresh();   // editable() never dipped — the generation must trigger
    if (seedSpin) REQUIRE(seedSpin->value() == 21);   // the NEW level's recipe
    if (regen) {
        regen->click();
        json after2 = json::parse(bridge.plannerRecipe(), nullptr, false);
        REQUIRE(after2.is_object() && after2.value("seed", 0) == 21);
        REQUIRE(world2.get<RoadNet>(road2)->nodes.size() > 10);
    }

    // Observer/detached: planner API goes inert, panel grays out.
    bridge.detach();
    REQUIRE(bridge.plannerRecipe().empty());
    REQUIRE(!bridge.plannerApplyRecipe(parsed.dump()).ok);
    panel.refresh();
    REQUIRE(!panel.isEnabled());

    std::printf(failures == 0 ? "city planner qt test: OK\n"
                              : "city planner qt test: %d FAILURES\n",
                failures);
    return failures;
}
