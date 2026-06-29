// Web entry point (ADR-0058). The browser build can't use Application::run()'s
// blocking `while (running()) runFrame()` loop — the page would hang and never
// yield to the event loop. Instead we run begin() once and hand runFrame() to
// emscripten_set_main_loop, which drives it from requestAnimationFrame, exactly
// the per-frame decomposition the editor host already uses (application.h).
//
// The WebGPU device is acquired by the backend itself (RequestAdapter/Device,
// awaited via ASYNCIFY) so Renderer::initialize() stays synchronous. Single-
// threaded: JobSystem runs in synchronous mode, so the build needs no -pthread /
// SharedArrayBuffer / cross-origin isolation.
//
// Starts in editor mode (views any level, no gameplay/pointer-lock) and reads
// the level from the URL: ?level=<name> loads assets/levels/<name>.json. Default
// is a self-contained procedural scene (no external glTF/HDR to 404).

#include "engine/application.h"
#include "engine/states/editor_state.h"
#include "game/arena_state.h"
#include "log.h"

#include <emscripten/emscripten.h>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>

using namespace engine;

namespace {

// File-scope so it outlives main() — the browser keeps calling frame() long
// after main() has returned control to the event loop.
Application g_app;

void frame() {
    g_app.runFrame();
}

// Resolve the level path from ?level=<name> (default: a self-contained scene).
std::string levelFromUrl() {
    char* c = static_cast<char*>(EM_ASM_PTR({
        var p = new URLSearchParams(location.search).get('level') || 'showcase';
        if (!p.endsWith('.json')) p += '.json';
        if (p.indexOf('/') < 0) p = 'assets/levels/' + p;
        return stringToNewUTF8(p);
    }));
    std::string path = c ? c : "assets/levels/showcase.json";
    std::free(c);
    return path;
}

}  // namespace

int main() {
    // Size the surface to the canvas's device-pixel size so the aspect ratio is
    // correct and the image isn't stretched (the WebGPU surface config drives the
    // canvas backing size). Fall back to the window if the canvas isn't laid out.
    int cw = EM_ASM_INT({
        return Math.max(1, Math.floor((document.getElementById('canvas').clientWidth
            || window.innerWidth) * (window.devicePixelRatio || 1)));
    });
    int ch = EM_ASM_INT({
        return Math.max(1, Math.floor((document.getElementById('canvas').clientHeight
            || window.innerHeight) * (window.devicePixelRatio || 1)));
    });
    if (!g_app.initialize({cw, ch, "Raytracer Engine (Web)", "settings.json"},
                          createPlatformWindow())) {
        LOG_ERROR << "Failed to initialize web application";
        return 1;
    }

    const std::string levelPath = levelFromUrl();
    LOG_INFO << "Web level: " << levelPath;

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

    // Editor mode: views the level (no FPS gameplay, no pointer lock). Orbit
    // camera so a single touch-drag rotates the view on a phone.
    g_app.settings().setString("cameraMode", "orbit");
    g_app.pushState(makeEditor());

    g_app.begin();

    // fps=0 → drive from requestAnimationFrame; simulate_infinite_loop=true so
    // main() unwinds cleanly while the loop keeps running (the standard
    // Emscripten pattern). Control returns to the browser here.
    emscripten_set_main_loop(frame, 0, true);
    return 0;
}
