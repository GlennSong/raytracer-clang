#include "arena_state.h"
#include "../engine/components.h"
#include "../engine/editor_bridge.h"
#include "../engine/level_loader.h"
#include "../engine/asset_manager.h"
#include "../engine/systems/dev_control_system.h"
#include "../engine/systems/camera_system.h"
#include "../engine/systems/motion_system.h"
#include "../engine/systems/day_night_system.h"
#include "../engine/systems/render_system.h"
#include "../engine/systems/camera_panel_system.h"
#ifdef RT_ENABLE_PHYSICS
#include "../engine/systems/physics_system.h"
#include "../engine/systems/player_system.h"
#include "../engine/systems/shooting_system.h"
#endif
#include "../log.h"

using namespace engine;

ArenaState::ArenaState(Window& window, Renderer& renderer,
                       const std::string& levelFile,
                       EditorFactory makeEditorState, EditorBridge* bridge)
    : PlayingState(window), arenaRenderer(renderer), levelFile(levelFile),
      makeEditorState(std::move(makeEditorState)), bridge(bridge)
{
    // With an editor to return to, Esc stops the playtest instead of quitting.
    addSystem<DevControlSystem>(this->makeEditorState == nullptr);
    auto& camSys = addSystem<CameraSystem>();
#ifdef RT_ENABLE_PHYSICS
    auto& physSys = addSystem<PhysicsSystem>();
    addSystem<PlayerSystem>(camSys.flyController(), physSys);
    addSystem<ShootingSystem>(camSys.flyController(), physSys, renderer);
#endif
    addSystem<MotionSystem>();
    addSystem<DayNightSystem>();
    addSystem<RenderSystem>();
    addSystem<CameraPanelSystem>(camSys);
}

void ArenaState::update(FrameContext& ctx) {
    PlayingState::update(ctx);
    if (makeEditorState && ctx.actions.pressed("quit"))
        ctx.transition.replaceWith(makeEditorState());
}

void ArenaState::onEnter(FrameContext& ctx) {
    // Always start from the document (edit mode may have just rewritten it).
    ctx.world.destroyAll();
    ctx.assets.clear();   // free the previous level's deduped GPU meshes
    ctx.settings.setBool("cameraFreeLook", false);
    ctx.settings.setBool("cameraDetachEnabled", true);
    if (!LevelLoader::load(levelFile, ctx.world, arenaRenderer, ctx.view, ctx.assets)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }

    // Play From Here (one-shot flag): start the player at the editor's view
    // instead of the document's spawn. The position rides the same settings
    // the editor camera already persists on state exit (flyEye*), and the
    // facing carries over for free — the play camera restores flyYaw/Pitch.
    if (ctx.settings.getBool("playFromHere", false)) {
        ctx.settings.setBool("playFromHere", false);
        Vec3 here(ctx.settings.getDouble("flyEyeX", 0.0),
                  ctx.settings.getDouble("flyEyeY", 1.5),
                  ctx.settings.getDouble("flyEyeZ", 0.0));
        ctx.world.each<Transform, ControlledBy>(
            [&](Entity e, Transform& t, ControlledBy&) {
                t.position = here;
                if (auto* prev = ctx.world.get<PrevTransform>(e))
                    prev->value = t;
            });
        // Bodies spawn from these transforms when PhysicsSystem starts below;
        // gravity settles the capsule onto whatever is underneath.
    }

    // Placed cameras persist in a sidecar next to the level; CameraSystem
    // loads/saves it (must be set before PlayingState::onEnter starts systems).
    // The level path itself feeds the panel's offline-render button.
    ctx.settings.setString("cameraStorePath", levelFile + ".cameras.json");
    ctx.settings.setString("levelPath", levelFile);
    PlayingState::onEnter(ctx);

    // The shell watches the playtest read-only: live hierarchy/inspector,
    // editing disabled (EditorBridge::editable() is false without an editor).
    if (bridge) bridge->attachObserver(&ctx.world, levelFile);
}

void ArenaState::onExit(FrameContext& ctx) {
    if (bridge) bridge->detach();
    PlayingState::onExit(ctx);
}
