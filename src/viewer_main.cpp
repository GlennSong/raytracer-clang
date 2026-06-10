#include "engine/application.h"
#include "game/arena_state.h"
#include "log.h"

using namespace engine;

int main(int argc, char** argv) {
    Application app;
    if (!app.initialize({1280, 720, "FPS Arena", "settings.json"})) {
        LOG_ERROR << "Failed to initialize application";
        return 1;
    }

    // Optional level path as the first arg, e.g. ./viewer assets/levels/shadow_test.json
    const char* levelPath = (argc > 1) ? argv[1] : "assets/levels/arena.json";

    app.settings().setString("cameraMode", "fly");
    app.pushState(std::make_unique<ArenaState>(
        app.windowRef(), app.renderer(), levelPath));

    app.run();
    return 0;
}
