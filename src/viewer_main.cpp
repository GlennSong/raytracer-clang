#include "engine/application.h"
#include "engine/states/editor_state.h"
#include "game/arena_state.h"
#include "log.h"
#include <cstring>

using namespace engine;

int main(int argc, char** argv) {
    Application app;
    if (!app.initialize({1280, 720, "FPS Arena", "settings.json"})) {
        LOG_ERROR << "Failed to initialize application";
        return 1;
    }

    // Optional level path as the first arg; --play starts straight in the
    // game (the editor's Play button and the game's Esc swap between them).
    std::string levelPath = "assets/levels/arena.json";
    bool startInPlay = false;
    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--play") == 0) startInPlay = true;
        else levelPath = argv[i];
    }

    // The two modes construct each other through factories, so neither layer
    // hard-codes the other (the engine's editor never names game states).
    std::function<std::unique_ptr<AppState>()> makePlay;
    std::function<std::unique_ptr<AppState>()> makeEditor;
    makeEditor = [&app, levelPath, &makePlay]() -> std::unique_ptr<AppState> {
        return std::make_unique<EditorState>(app.windowRef(), app.renderer(),
                                             levelPath, makePlay);
    };
    makePlay = [&app, levelPath, &makeEditor]() -> std::unique_ptr<AppState> {
        return std::make_unique<ArenaState>(app.windowRef(), app.renderer(),
                                            levelPath, makeEditor);
    };

    app.settings().setString("cameraMode", "fly");
    app.pushState(startInPlay ? makePlay() : makeEditor());

    app.run();
    return 0;
}
