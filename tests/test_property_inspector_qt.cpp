// Headless (offscreen) exercise of the generated Qt inspector: build it for a
// real entity, then EDIT widgets programmatically — the exact path that
// crashed when widget signals captured the stack-local build visitor. Runs
// under QT_QPA_PLATFORM=offscreen; no display needed.

#include "../src/editor_app/property_inspector.h"
#include "../src/engine/components.h"
#include "../src/engine/camera/scene_camera.h"
#include "../src/engine/systems/editor_system.h"
#include "../src/engine/undo_stack.h"

#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QMenu>
#include <QPushButton>
#include <cstdio>

using namespace engine;

namespace {
int failures = 0;
#define REQUIRE(cond)                                                         \
    do {                                                                      \
        if (!(cond)) {                                                        \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);       \
            failures++;                                                       \
        }                                                                     \
    } while (0)
}  // namespace

int main(int argc, char** argv) {
    qputenv("QT_QPA_PLATFORM", "offscreen");
    QApplication app(argc, argv);

    World world;
    Entity e = world.create();
    Transform t;
    t.position = Vec3(1, 2, 3);
    world.add<Transform>(e, t);
    world.add<PrevTransform>(e, {t});
    world.add<Renderable>(e);

    EditorBridge bridge;
    CameraSystem cameras;
    EditorSystem editor(cameras, "test.json", nullptr, &bridge);
    bridge.attach(&world, &editor, "test.json");
    bridge.select(e);

    PropertyInspector inspector(bridge);
    inspector.refresh();   // selection changed -> builds sections

    auto spins = inspector.findChildren<QDoubleSpinBox*>();
    REQUIRE(spins.size() >= 6);   // Transform pos/scale + material scalars

    // Edit the first spin box (Position.x). This fires valueChanged ->
    // writeField — the path that used to call through a dead visitor.
    if (!spins.isEmpty()) {
        spins[0]->setValue(42.5);
        REQUIRE(world.get<Transform>(e)->position.x == 42.5);
        // PrevTransform follows so editor edits don't smear.
        REQUIRE(world.get<PrevTransform>(e)->value.position.x == 42.5);
    }

    // Toggle the checkerboard flag through its checkbox.
    auto checks = inspector.findChildren<QCheckBox*>();
    REQUIRE(!checks.isEmpty());
    if (!checks.isEmpty()) {
        bool before =
            (world.get<Renderable>(e)->material.flags &
             RenderMaterial::FLAG_CHECKERBOARD) != 0;
        checks.last()->setChecked(!checks.last()->isChecked());
        bool after =
            (world.get<Renderable>(e)->material.flags &
             RenderMaterial::FLAG_CHECKERBOARD) != 0;
        REQUIRE(before != after);
    }

    // Sync pass with nothing focused must not write back or crash.
    inspector.refresh();
    REQUIRE(world.get<Transform>(e)->position.x == 42.5);

    // Every widget edit above landed on the command log; undo through the
    // bridge walks them back (the checkbox toggle, then the spin edit).
    REQUIRE(bridge.canUndo());
    bridge.undo();
    bridge.undo();
    REQUIRE(world.get<Transform>(e)->position.x == 1.0);
    REQUIRE(bridge.canRedo());
    bridge.redo();
    REQUIRE(world.get<Transform>(e)->position.x == 42.5);
    bridge.undo();   // back again so later checks see the original pose
    REQUIRE(world.get<Transform>(e)->position.x == 1.0);

    // Add Component: the menu lists registry types the entity lacks; adding
    // a camera through the bridge rebuilds the inspector with a new section.
    inspector.refresh();
    auto buttons = inspector.findChildren<QPushButton*>();
    QPushButton* addButton = nullptr;
    for (auto* b : buttons)
        if (b->text() == "Add Component") addButton = b;
    REQUIRE(addButton != nullptr);
    if (addButton) {
        QMenu* menu = addButton->menu();
        REQUIRE(menu && !menu->actions().isEmpty());
        QAction* cameraAction = nullptr;
        for (QAction* a : menu->actions())
            if (a->text() == "Camera") cameraAction = a;
        REQUIRE(cameraAction != nullptr);
        if (cameraAction) {
            cameraAction->trigger();
            REQUIRE(world.has<SceneCamera>(e));
            inspector.refresh();   // structureDirty -> rebuild with Camera

            // The section's Remove Component button detaches it again...
            QPushButton* removeButton = nullptr;
            for (auto* b : inspector.findChildren<QPushButton*>())
                if (b->text() == "Remove Component") removeButton = b;
            REQUIRE(removeButton != nullptr);
            if (removeButton) {
                removeButton->click();
                REQUIRE(!world.has<SceneCamera>(e));
                inspector.refresh();
            }

            // ...and both menu actions are one undo step each.
            bridge.undo();
            REQUIRE(world.has<SceneCamera>(e));
            bridge.undo();
            REQUIRE(!world.has<SceneCamera>(e));
            inspector.refresh();
        }
    }

    // Selection cleared -> sections torn down; deleting the entity while
    // shown must degrade gracefully on the next refresh.
    bridge.select(Entity{});
    inspector.refresh();
    bridge.select(e);
    inspector.refresh();
    world.destroy(e);
    inspector.refresh();

    std::printf(failures == 0 ? "inspector qt test: OK\n"
                              : "inspector qt test: %d FAILURES\n",
                failures);
    return failures == 0 ? 0 : 1;
}
