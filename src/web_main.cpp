// Web entry point (ADR-0058). The browser build can't use Application::run()'s
// blocking `while (running()) runFrame()` loop — the page would hang and never
// yield to the event loop. Instead we run begin() once and hand runFrame() to
// emscripten_set_main_loop, which drives it from requestAnimationFrame, exactly
// the per-frame decomposition the editor host already uses (application.h).
//
// The WebGPU device is created on the JS side before this module runs and passed
// in via Module.preinitializedWebGPUDevice (web/index.html); the WebGPU backend
// picks it up through emscripten_webgpu_get_device(), so initialize() stays
// synchronous. Single-threaded: JobSystem runs in synchronous mode, so the build
// needs no -pthread / SharedArrayBuffer / cross-origin isolation.

#include "engine/application.h"
#include "engine/states/editor_state.h"
#include "game/arena_state.h"
#include "log.h"

#include <emscripten/emscripten.h>
#include <functional>
#include <memory>

using namespace engine;

namespace {

// File-scope so it outlives main() — the browser keeps calling frame() long
// after main() has returned control to the event loop.
Application g_app;

void frame() {
    g_app.runFrame();
}

}  // namespace

int main() {
    // The canvas is sized by the page; GLFW (the Emscripten shim) reads it back.
    if (!g_app.initialize({1280, 720, "FPS Arena (Web)", "settings.json"},
                          createPlatformWindow())) {
        LOG_ERROR << "Failed to initialize web application";
        return 1;
    }

    const std::string levelPath = "assets/levels/arena.json";

    std::function<std::unique_ptr<AppState>()> makePlay;
    std::function<std::unique_ptr<AppState>()> makeEditor;
    makeEditor = [levelPath, &makePlay]() -> std::unique_ptr<AppState> {
        return std::make_unique<EditorState>(g_app.windowRef(), g_app.renderer(),
                                             levelPath, makePlay);
    };
    makePlay = [levelPath, &makeEditor]() -> std::unique_ptr<AppState> {
        return std::make_unique<ArenaState>(g_app.windowRef(), g_app.renderer(),
                                            levelPath, makeEditor);
    };

    g_app.settings().setString("cameraMode", "fly");
    g_app.pushState(makePlay());

    g_app.begin();

    // fps=0 → drive from requestAnimationFrame; simulate_infinite_loop=true so
    // main() unwinds cleanly while the loop keeps running (the standard
    // Emscripten pattern). Control returns to the browser here.
    emscripten_set_main_loop(frame, 0, true);
    return 0;
}
