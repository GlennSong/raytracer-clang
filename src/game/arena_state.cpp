#include "arena_state.h"
#include "../engine/level_loader.h"
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
                       EditorFactory makeEditorState)
    : PlayingState(window), arenaRenderer(renderer), levelFile(levelFile),
      makeEditorState(std::move(makeEditorState))
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
    ctx.settings.setBool("cameraFreeLook", false);
    ctx.settings.setBool("cameraDetachEnabled", true);
    if (!LevelLoader::load(levelFile, ctx.world, arenaRenderer, ctx.view)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }
    // Placed cameras persist in a sidecar next to the level; CameraSystem
    // loads/saves it (must be set before PlayingState::onEnter starts systems).
    // The level path itself feeds the panel's offline-render button.
    ctx.settings.setString("cameraStorePath", levelFile + ".cameras.json");
    ctx.settings.setString("levelPath", levelFile);
    PlayingState::onEnter(ctx);
}
