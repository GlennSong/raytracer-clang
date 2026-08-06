#ifndef RAYTRACER_ENGINE_APPLICATION_H
#define RAYTRACER_ENGINE_APPLICATION_H

#include "system.h"
#include "state_stack.h"
#include "world.h"
#include "xr/xr_backend.h"
#include "clock.h"
#include "frame_stats.h"
#include "asset_manager.h"
#include "audio/audio_engine.h"
#include "mesh_uploader.h"
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

        // How the audio engine opens (ADR-0069). Auto probes for a real output
        // device and falls back to the null backend if opening one FAILS.
        //
        // That fallback cannot save a host where opening one HANGS: on visionOS
        // AURemoteIO deadlocks unless the app has configured and activated an
        // AVAudioSession first, and CoreAudio aborts the process after its RPC
        // timeout — there is no failure to catch. Such a host passes Null until
        // it does that setup, which is why this is the host's choice and not a
        // constant inside Application.
        AudioBackendMode audio = AudioBackendMode::Auto;
    };

    Application();
    ~Application();

    // The host chooses the window implementation: createPlatformWindow()
    // (GLFW) for the standalone runtime, a HostedWindow for embedding inside
    // an application shell (the Qt editor). Keeping the choice host-side
    // keeps engine_core free of any windowing-library linkage.
    bool initialize(const Config& config, std::unique_ptr<Window> appWindow);

    // The standalone loop: begin(); while (running()) runFrame(); end().
    void run();

    // Host-driven mode (editor): same lifecycle, one frame at a time.
    void begin();        // states' onStart + draw-callback hookup
    void runFrame();     // one full frame: events, update, fixed steps, render
    bool running() const;
    void end();          // states' onStop, settings persistence, shutdowns

    void pushState(std::unique_ptr<AppState> state);
    void popState();

    // Shell-initiated state swap (the editor's toolbar Play/Stop, opening a
    // different level): same deferred pop-then-push as in-engine requests.
    void requestState(std::unique_ptr<AppState> state) {
        transitionRequest.replaceWith(std::move(state));
    }

    World& world() { return worldState; }
    Renderer& renderer() { return *rendererPtr; }
    AssetManager& assets() { return *assetManager; }
    RenderView& renderView() { return view; }
    Window& windowRef() { return *window; }
    Settings& settings() { return settingsStore; }
    // Absolute path settings were loaded from (Config::settingsFile) — hosts
    // whose working directory is not the settings directory (the visionOS app
    // saves to Documents) must save back to THIS path, not a relative name.
    const std::string& settingsFilePath() const { return settingsFile; }
    EventBus& events() { return eventBus; }
    DebugDraw& debugDraw() { return debugLines; }
    AudioEngine& audio() { return audioEngine; }
    // The simulation clock, exposed so an editor shell can drive pause /
    // single-step transport controls during play (same switch Space toggles).
    SimClock& simClock() { return clock; }
    // The frame ledger (ADR-0077). Hosts without ImGui — the visionOS panel —
    // read summarize()/lastFrame() here; any host may start a CSV capture.
    FrameStats& stats() { return frameStats; }

private:
    void reconcileFramebuffer();
    void renderFrame();
    FrameContext makeContext();

    std::unique_ptr<Window> window;
    std::unique_ptr<Renderer> rendererPtr;
    // The asset manager drives the renderer through a seam adapter; both are
    // created in initialize() once the renderer exists, and destroyed (reverse
    // order) before it. ~AssetManager is trivial — GPU teardown is the
    // renderer's job at shutdown — so the order is safe regardless.
    std::unique_ptr<RendererMeshUploader> meshUploader;
    std::unique_ptr<AssetManager> assetManager;
    World worldState;
    SimClock clock;
    FrameStats frameStats;
    // Interval (seconds) between LOG_INFO frame summaries, 0 = off. Set from
    // RT_FRAME_STATS_LOG for hosts where only a console is visible (device
    // logs on visionOS, headless runs).
    double statsLogInterval = 0.0;
    double statsLogTimer = 0.0;
    // Previous frame's monotonic upload totals, so the ledger can record
    // per-frame resource creation as a delta (see runFrame).
    uint64_t prevMeshUploads = 0;
    uint64_t prevTextureUploads = 0;
    Settings settingsStore;
    // The one shared thread pool (ADR-0014). Declared before `systems` so it
    // outlives them — a system (e.g. physics) may hold work referencing it.
    // The web build (ADR-0058) is single-threaded: it links without -pthread, so
    // it forces synchronous mode (0 workers) — spawning a std::thread there would
    // abort at runtime. parallelFor/run still execute inline, just on the caller.
#ifdef __EMSCRIPTEN__
    JobSystem jobs{0};
#else
    JobSystem jobs;
#endif
    EventBus eventBus;
    DebugDraw debugLines;
    AudioEngine audioEngine;
    InputMap inputMap;
    PlayerInputs playerInputs;
    // Headset state (engine/xr/). `xr` is the renderer's backend or null;
    // `xrState` is refreshed at the top of every runFrame and handed to all
    // systems through FrameContext. Inert (active=false) without a headset.
    XrBackend* xr = nullptr;
    XrState xrState;
    std::vector<XrInputEvent> xrInputScratch;  // per-frame drain buffer
    double xrPinchSeconds = 0.0;               // current pinch hold duration
    RenderView view;
    StateStack stateStack;
    bool debugOverlayActive = false;

    std::string settingsFile;
    int framebufferWidth = 0;
    int framebufferHeight = 0;
    double frameDelta = 0.0;
    double interpolation = 0.0;
    bool quit = false;
    StateTransition transitionRequest;
};


}  // namespace engine

#endif
