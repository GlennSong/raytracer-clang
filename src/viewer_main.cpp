#include "engine/application.h"
#include "game/arena_state.h"
#include "log.h"

using namespace engine;

int main() {
    Application app;
    if (!app.initialize({1280, 720, "FPS Arena", "settings.json"})) {
        LOG_ERROR << "Failed to initialize application";
        return 1;
    }

    app.settings().setString("cameraMode", "fly");
    app.pushState(std::make_unique<ArenaState>(
        app.windowRef(), app.renderer(), "assets/levels/arena.json"));

    app.run();
    return 0;
}
