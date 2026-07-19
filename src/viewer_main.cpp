#include "engine/application.h"
#include "engine/states/editor_state.h"
#include "engine/recent_scenes.h"
#include "game/arena_state.h"
#include "log.h"
#include <cstring>
#include <fstream>
#include <memory>

using namespace engine;

int main(int argc, char** argv) {
    Application app;
    if (!app.initialize({1280, 720, "FPS Arena", "settings.json"},
                        createPlatformWindow())) {
        LOG_ERROR << "Failed to initialize application";
        return 1;
    }

    // Optional level path as the first arg. The viewer IS the game: it boots
    // straight into play (no editor or debug chrome — backtick opens the
    // debug overlay, Esc returns to edit mode). --edit starts in the editor;
    // --play is accepted for compatibility (it is the default now).
    std::string levelPath = "assets/levels/arena.json";
    bool startInPlay = true;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--edit") == 0) startInPlay = false;
        else if (std::strcmp(argv[i], "--play") == 0) startInPlay = true;
        else levelPath = argv[i];
    }

    // Resolve the level the way the docs write it: a BARE NAME ("metropolis")
    // means assets/levels/<name>.json; an existing path is taken as-is. A
    // level that resolves to nothing is a HARD ERROR — booting the empty
    // editor on a bad path used to SAVE an empty level to that literal path
    // on the play-toggle, silently creating a junk file that made every later
    // run "successfully" load an empty world ("there's nothing in that
    // scene"). Fail loud, up front, before any state exists to save.
    {
        auto exists = [](const std::string& p) { return std::ifstream(p).good(); };
        if (!exists(levelPath)) {
            const std::string byName = "assets/levels/" + levelPath;
            const std::string byJson = byName + ".json";
            if (exists(byJson))      levelPath = byJson;
            else if (exists(byName)) levelPath = byName;
            else {
                LOG_ERROR << "Level not found: '" << levelPath << "' (also tried "
                          << byJson << " and " << byName << ")";
                return 1;
            }
        }
    }

    // Remember the launched scene so the editor's "Recent Scenes" list can
    // offer it back, and keep the current path in one shared spot so switching
    // scenes from that list also changes what Play loads.
    recordRecentScene(app.settings(), levelPath);
    auto currentLevel = std::make_shared<std::string>(levelPath);

    // The two modes construct each other through factories, so neither layer
    // hard-codes the other (the engine's editor never names game states).
    // makeEditorFor(path) additionally lets the editor swap to a different
    // scene at runtime — the hook behind the recents list.
    std::function<std::unique_ptr<AppState>()> makePlay;
    std::function<std::unique_ptr<AppState>()> makeEditor;
    EditorSystem::OpenLevelFactory makeEditorFor;
    makeEditor = [&app, currentLevel, &makePlay,
                  &makeEditorFor]() -> std::unique_ptr<AppState> {
        return std::make_unique<EditorState>(app.windowRef(), app.renderer(),
                                             *currentLevel, makePlay, nullptr,
                                             makeEditorFor);
    };
    makePlay = [&app, currentLevel, &makeEditor]() -> std::unique_ptr<AppState> {
        return std::make_unique<ArenaState>(app.windowRef(), app.renderer(),
                                            *currentLevel, makeEditor);
    };
    makeEditorFor = [&app, currentLevel, &makeEditor](
                        const std::string& path) -> std::unique_ptr<AppState> {
        *currentLevel = path;                      // Play now loads this scene too
        recordRecentScene(app.settings(), path);   // float it to the top
        return makeEditor();
    };

    app.settings().setString("cameraMode", "fly");
    app.pushState(startInPlay ? makePlay() : makeEditor());

    app.run();
    return 0;
}
