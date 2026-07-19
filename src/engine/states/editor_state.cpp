#include "editor_state.h"

#include "../../renderer/window.h"
#include "../level_loader.h"
#include "../asset_manager.h"
#include "../systems/dev_control_system.h"
#include "../systems/camera_system.h"
#include "../systems/camera_panel_system.h"
#include "../systems/terrain_lod_system.h"
#include "../systems/render_system.h"
#ifdef __EMSCRIPTEN__
#include "../systems/day_night_system.h"
#endif
#include "../../log.h"

namespace engine {

EditorState::EditorState(Window& window, Renderer& renderer,
                         std::string levelFile,
                         EditorSystem::PlayFactory makePlayState,
                         EditorBridge* bridge,
                         EditorSystem::OpenLevelFactory openLevel)
    : PlayingState(window), editorRenderer(renderer),
      levelFile(std::move(levelFile)) {
    addSystem<DevControlSystem>();   // Esc quits the app from the editor
    auto& camSys = addSystem<CameraSystem>();
#ifdef __EMSCRIPTEN__
    // The web editor has no ImGui day/night panel, so run the cycle here and
    // let the page's debug panel drive it through settings (rt_web_env). Native
    // editors keep a static sky and reach the cycle via play mode's ImGui.
    addSystem<DayNightSystem>();
#endif
    addSystem<TerrainLodSystem>();   // CDLOD terrain draws (ADR-0036)
    addSystem<RenderSystem>();
    addSystem<CameraPanelSystem>(camSys);
    addSystem<EditorSystem>(camSys, this->levelFile, std::move(makePlayState),
                            bridge, std::move(openLevel));
}

void EditorState::onEnter(FrameContext& ctx) {
    // The document is the only truth: drop whatever the previous mode left
    // (play-session spawns, physics aftermath) and load fresh.
    ctx.world.destroyAll();
    ctx.assets.clear();   // free the previous level's deduped GPU meshes

    // Editor camera: scene-view style — right-drag looks, WASD/QE fly,
    // scroll dollies, left click is selection. (Buttonless free-look is the
    // in-game freecam's behavior, not the editor's: it fights the mouse.)
    ctx.settings.setString("cameraMode", "fly");
    ctx.settings.setBool("cameraFreeLook", false);
    ctx.settings.setBool("cameraDetachEnabled", false);   // F = frame selected
    ctx.settings.setString("levelPath", levelFile);
    ctx.settings.setString("cameraStorePath", levelFile + ".cameras.json");

    if (!LevelLoader::load(levelFile, ctx.world, editorRenderer, ctx.view,
                           ctx.assets, /*editorMode=*/true)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }
    PlayingState::onEnter(ctx);
    // PlayingState captures the pointer for first-person play; the editor is
    // a pointing UI — picking, gizmos, panels — so the cursor stays visible.
    window.setCursorMode(CursorMode::Normal);
    LOG_INFO << "Edit mode: click selects, 1/2/3 move/rotate/scale, "
                "Play runs the level";
}

void EditorState::onResume(FrameContext& ctx) {
    PlayingState::onResume(ctx);
    window.setCursorMode(CursorMode::Normal);   // see onEnter
}

}  // namespace engine
