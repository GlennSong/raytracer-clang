#ifndef RAYTRACER_ENGINE_APPLICATION_H
#define RAYTRACER_ENGINE_APPLICATION_H

#include "system.h"
#include "state_stack.h"
#include "world.h"
#include "clock.h"
#include "../renderer/renderer.h"
#include "../renderer/window.h"
#include "../renderer/settings.h"
#include <memory>
#include <string>
#include <vector>
#include <utility>

namespace engine {

// Owns the window, renderer, world, clock, and the set of Systems, and drives
// the fixed-timestep frame loop that ticks them. Pure mechanism: all
// game-specific behaviour lives in Systems and the scene built into world().
class Application {
public:
    struct Config {
        int width = 1024;
        int height = 1024;
        std::string title = "Application";
        std::string settingsFile = "settings.json";
    };

    Application();
    ~Application();

    bool initialize(const Config& config);
    void run();

    void pushState(std::unique_ptr<AppState> state);
    void popState();

    World& world() { return worldState; }
    Renderer& renderer() { return *rendererPtr; }
    RenderView& renderView() { return view; }
    Window& windowRef() { return window; }
    Settings& settings() { return settingsStore; }

private:
    void reconcileFramebuffer();
    void renderFrame();
    FrameContext makeContext();

    Window window;
    std::unique_ptr<Renderer> rendererPtr;
    World worldState;
    SimClock clock;
    Settings settingsStore;
    // The one shared thread pool (ADR-0014). Declared before `systems` so it
    // outlives them — a system (e.g. physics) may hold work referencing it.
    JobSystem jobs;
    InputMap inputMap;
    PlayerInputs playerInputs;
    RenderView view;
    StateStack stateStack;
    bool debugOverlayActive = false;

    std::string settingsFile;
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    double frameDelta = 0.0;
    double interpolation = 0.0;
    bool quit = false;
};


}  // namespace engine

#endif
