// Headless (offscreen) exercise of the generated Qt inspector: build it for a
// real entity, then EDIT widgets programmatically — the exact path that
// crashed when widget signals captured the stack-local build visitor. Runs
// under QT_QPA_PLATFORM=offscreen; no display needed.

#include "../src/editor_app/property_inspector.h"
#include "../src/engine/components.h"
#include "../src/engine/systems/editor_system.h"

#include <QApplication>
#include <QCheckBox>
#include <QDoubleSpinBox>
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
