#include "editor_state.h"

#include "../level_loader.h"
#include "../systems/dev_control_system.h"
#include "../systems/camera_system.h"
#include "../systems/camera_panel_system.h"
#include "../systems/render_system.h"
#include "../../log.h"

namespace engine {

EditorState::EditorState(Window& window, Renderer& renderer,
                         std::string levelFile,
                         EditorSystem::PlayFactory makePlayState)
    : PlayingState(window), editorRenderer(renderer),
      levelFile(std::move(levelFile)) {
    addSystem<DevControlSystem>();   // Esc quits the app from the editor
    auto& camSys = addSystem<CameraSystem>();
    addSystem<RenderSystem>();
    addSystem<CameraPanelSystem>(camSys);
    addSystem<EditorSystem>(camSys, this->levelFile, std::move(makePlayState));
}

void EditorState::onEnter(FrameContext& ctx) {
    // The document is the only truth: drop whatever the previous mode left
    // (play-session spawns, physics aftermath) and load fresh.
    ctx.world.destroyAll();

    // Editor camera: free-fly with buttonless mouse look, never player-pinned.
    ctx.settings.setString("cameraMode", "fly");
    ctx.settings.setBool("cameraFreeLook", true);
    ctx.settings.setString("cameraStorePath", levelFile + ".cameras.json");

    if (!LevelLoader::load(levelFile, ctx.world, editorRenderer, ctx.view)) {
        LOG_ERROR << "Failed to load level: " << levelFile;
    }
    PlayingState::onEnter(ctx);
    LOG_INFO << "Edit mode: click selects, 1/2/3 move/rotate/scale, "
                "Play runs the level";
}

}  // namespace engine
