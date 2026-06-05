#ifndef RAYTRACER_ENGINE_APPLICATION_H
#define RAYTRACER_ENGINE_APPLICATION_H

#include "system.h"
#include "world.h"
#include "clock.h"
#include "../renderer/renderer.h"
#include "../renderer/window.h"
#include "../renderer/settings.h"
#include <memory>
#include <string>
#include <vector>
#include <utility>

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

    // Systems run in registration order across every phase.
    template <typename T, typename... Args>
    T& addSystem(Args&&... args) {
        auto system = std::make_unique<T>(std::forward<Args>(args)...);
        T& ref = *system;
        systems.push_back(std::move(system));
        return ref;
    }

    World& world() { return worldState; }
    Renderer& renderer() { return *rendererPtr; }
    RenderView& renderView() { return view; }
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
    // The one shared thread pool (ADR-0013). Declared before `systems` so it
    // outlives them — a system (e.g. physics) may hold work referencing it.
    JobSystem jobs;
    InputMap inputMap;
    PlayerInputs playerInputs;
    RenderView view;
    std::vector<std::unique_ptr<System>> systems;

    std::string settingsFile;
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    double frameDelta = 0.0;
    double interpolation = 0.0;
    bool quit = false;
};

#endif
