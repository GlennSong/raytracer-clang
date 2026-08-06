#include "application.h"
#include "states/debug_overlay_state.h"
#include "../log.h"
#include "../profile.h"
#include <thread>
#include <chrono>

namespace engine {

StateTransition::StateTransition() = default;
StateTransition::~StateTransition() = default;
void StateTransition::replaceWith(std::unique_ptr<AppState> state) {
    next = std::move(state);
}

Application::Application() = default;
Application::~Application() = default;

bool Application::initialize(const Config& config,
                             std::unique_ptr<Window> appWindow) {
    settingsFile = config.settingsFile;
    settingsStore.load(settingsFile);

    int winWidth = static_cast<int>(settingsStore.getDouble("windowWidth", config.width));
    int winHeight = static_cast<int>(settingsStore.getDouble("windowHeight", config.height));

    window = std::move(appWindow);
    if (!window || !window->initialize(winWidth, winHeight, config.title))
        return false;

    window->getFramebufferSize(framebufferWidth, framebufferHeight);

    rendererPtr = Renderer::create();
    // Hand over the windowing seam before initialize() so a backend that builds
    // its own surface from it (Vulkan; the native handle is null on Linux) can.
    // The Metal/Null backends ignore this (ADR-0057).
    rendererPtr->setWindow(window.get());
    if (!rendererPtr->initialize(window->nativeWindowHandle(),
                                 framebufferWidth, framebufferHeight)) {
        return false;
    }

    // Headset backend, if this renderer owns one (visionOS Metal). Null
    // everywhere else, which keeps xrState.active false and XR inert.
    xr = rendererPtr->xrBackend();

    // Debug UI (ADR-0011): renderer creates the ImGui context, then the window
    // attaches its platform backend. Both no-ops without RT_ENABLE_IMGUI.
    // The asset manager owns GPU mesh lifetime, driving the renderer through
    // the seam adapter (ROADMAP 3.1). Created here, once the renderer exists.
    meshUploader = std::make_unique<RendererMeshUploader>(*rendererPtr);
    assetManager = std::make_unique<AssetManager>(*meshUploader);

    rendererPtr->initDebugUi(window->nativeWindowHandle());
    window->initDebugUi();

    // Audio is best-effort: a build without RT_ENABLE_AUDIO, or a machine
    // with no device, degrades to silence (ADR-0069) — never a failed boot.
    // The host picks the mode (see Config::audio); Auto is the desktop default.
    audioEngine.initialize(config.audio);

    clock.setFixedStep(settingsStore.getDouble("fixedTimestep", 1.0 / 60.0));
    return true;
}

void Application::pushState(std::unique_ptr<AppState> state) {
    stateStack.pushState(std::move(state));
}

void Application::popState() {
    stateStack.popState();
}

FrameContext Application::makeContext() {
    int winW = 0, winH = 0;
    window->getSize(winW, winH);
    return FrameContext{
        worldState, *rendererPtr, *assetManager, view, clock, settingsStore, jobs,
        eventBus, debugLines, audioEngine,
        window->getInput(), inputMap, playerInputs, xrState,
        framebufferWidth, framebufferHeight, winW, winH,
        frameDelta, interpolation, quit, transitionRequest,
        debugOverlayActive
    };
}

void Application::reconcileFramebuffer() {
    int w, h;
    window->getFramebufferSize(w, h);
    if (w != framebufferWidth || h != framebufferHeight) {
        framebufferWidth = w;
        framebufferHeight = h;
        rendererPtr->resize(w, h);
    }
}

void Application::renderFrame() {
    RT_PROFILE_ZONE_NAMED("render");
    reconcileFramebuffer();
    FrameContext ctx = makeContext();
    rendererPtr->beginFrame();
    stateStack.forEachRenderable([&](AppState& state) { state.render(ctx); });
    rendererPtr->endFrame();
    // RT_DUMP_STATS=1: periodic frame-cost report on stderr, so a headless run
    // answers "what is eating the frame" without the ImGui HUD (perf triage,
    // 8km-city plan P6).
    static const bool dumpStats = std::getenv("RT_DUMP_STATS") != nullptr;
    if (dumpStats) {
        static int frames = 0;
        static double accum = 0.0;
        accum += frameDelta;
        if (++frames % 120 == 0) {
            const RenderStats rs = rendererPtr->getRenderStats();
            LOG_INFO << "[stats] " << (frames / accum) << " fps ("
                     << (accum / frames * 1000.0) << " ms) draws "
                     << rs.drawCalls << " (inst " << rs.instancedDrawCalls
                     << ") instances " << rs.totalInstances << " tris "
                     << rs.trianglesDrawn / 1000000.0 << "M overflow i"
                     << rs.instanceOverflow << "/s" << rs.shadowOverflow
                     << "/f" << rs.foliageOverflow;
            frames = 0;
            accum = 0.0;
        }
    }
}

void Application::begin() {
    {
        FrameContext ctx = makeContext();
        stateStack.onStart(ctx);
    }
    // Render through the window so it keeps painting during a modal resize,
    // when pollEvents blocks and the loop is suspended. (Hosted windows
    // ignore this — the host paints by calling runFrame.)
    window->setDrawCallback([this]() { renderFrame(); });
}

bool Application::running() const {
    return !window->shouldClose() && !quit;
}

void Application::runFrame() {
    window->pollEvents();
    frameDelta = window->getDeltaTime();
    reconcileFramebuffer();
    debugLines.update(frameDelta);   // age timed debug shapes (ADR-0067)

    // Headset pose for this frame, BEFORE any system updates: camera writers
    // (XrCameraSystem) must see the pose the user's head will have when the
    // frame is displayed, not last frame's. Inert when no backend exists.
    if (xr) {
        if (!xr->beginFrame(xrState)) xrState.active = false;
    } else {
        xrState.active = false;
    }

    {
        RT_PROFILE_ZONE_NAMED("update");
        FrameContext ctx = makeContext();
        inputMap.beginFrame();
        playerInputs.beginFrame();
        for (const Event& event : window->getEvents()) {
            inputMap.processEvent(event);
            playerInputs.routeEvent(event);
            if (event.type == EventType::WindowCloseRequested) quit = true;

            // Backtick toggles debug overlay
            if (event.type == EventType::KeyPressed
                && event.key == KeyCode::GraveAccent
                && !event.repeat) {
                if (debugOverlayActive) {
                    stateStack.popState();
                    debugOverlayActive = false;
                } else {
                    stateStack.pushState(
                        std::make_unique<DebugOverlayState>(*window));
                    debugOverlayActive = true;
                }
            } else {
                stateStack.onEvent(event, ctx);
            }
        }
        // XR spatial input: drain the backend's thread-safe queue, fold the
        // gaze ray + pinch edges into xrState, and re-issue pinch as XrButton
        // events through the exact same routing as window events — game code
        // binds "player_teleport" to XrButton::Pinch like any key.
        if (xr) {
            xrState.pinchBegan = false;
            xrState.pinchEnded = false;
            xrInputScratch.clear();
            xr->pollInput(xrInputScratch);
            for (const XrInputEvent& e : xrInputScratch) {
                xrState.gazeValid = true;
                xrState.gazeOrigin = e.rayOrigin;
                xrState.gazeDir = e.rayDir;
                Event ev(EventType::XrButtonPressed);
                ev.xrButton = XrButton::Pinch;
                switch (e.kind) {
                    case XrInputEvent::Kind::PinchBegan:
                        xrState.pinchHeld = true;
                        xrState.pinchBegan = true;
                        xrPinchSeconds = 0.0;
                        break;
                    case XrInputEvent::Kind::PinchMoved:
                        continue;   // ray already captured above; no edge
                    case XrInputEvent::Kind::PinchEnded:
                        xrState.pinchHeld = false;
                        xrState.pinchEnded = true;   // a REAL release edge
                        ev.type = EventType::XrButtonReleased;
                        break;
                    case XrInputEvent::Kind::PinchCancelled:
                        xrState.pinchHeld = false;   // clears held state, but
                        ev.type = EventType::XrButtonReleased;  // no pinchEnded:
                        break;                        // gameplay ignores cancels
                }
                inputMap.processEvent(ev);
                playerInputs.routeEvent(ev);
                stateStack.onEvent(ev, ctx);
            }
            if (xrState.pinchHeld) xrPinchSeconds += frameDelta;
            xrState.pinchHeldSeconds = xrPinchSeconds;
        }
        playerInputs.updateGamepads(window->getGamepads());
        inputMap.updateGamepad(window->getGamepads()[0]);
        stateStack.forEachActive([&](AppState& state) { state.update(ctx); });
    }

    int steps = clock.advance(frameDelta);
    interpolation = clock.interpolationAlpha();
    if (clock.droppedBacklog())
        LOG_WARN << "SimClock dropped backlog (stall #" << clock.droppedBacklogCount()
                 << "): fixed steps capped at " << steps
                 << " this frame — motion will visibly jump";
    {
        RT_PROFILE_ZONE_NAMED("fixedUpdate");
        FrameContext ctx = makeContext();
        // RT_DUMP_STATS phase timing: how much of the frame is the fixed
        // step, and how many steps ran (perf triage without Tracy).
        static const bool dumpStats = std::getenv("RT_DUMP_STATS") != nullptr;
        const auto t0 = std::chrono::steady_clock::now();
        ctx.fixedStepCount = steps;
        for (int i = 0; i < steps; i++) {
            ctx.fixedStepIndex = i;
            stateStack.forEachActive([&](AppState& state) { state.fixedUpdate(ctx); });
        }
        if (dumpStats && steps > 0) {
            static int frames = 0;
            static double ms = 0.0;
            static int stepSum = 0;
            ms += std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0).count();
            stepSum += steps;
            if (++frames % 120 == 0) {
                LOG_INFO << "[stats] fixedUpdate " << (ms / frames)
                         << " ms/frame over " << (static_cast<double>(stepSum) / frames)
                         << " steps/frame (" << (ms / stepSum) << " ms/step)";
                frames = 0; ms = 0.0; stepSum = 0;
            }
        }
    }

    // Deliver everything enqueued during update/fixedUpdate before the frame
    // renders, so reactions land in the same frame as their cause (ADR-0066).
    eventBus.dispatchQueued();

    auto frameStart = std::chrono::steady_clock::now();
    renderFrame();
    // Drop expired debug shapes now that they've been drawn; one-frame shapes
    // (the immediate-mode default) live exactly this long. The modal-resize
    // draw callback renders without expiring, so paused frames keep their
    // overlay.
    debugLines.endFrame();

    if (rendererPtr->targetFps > 0) {
        auto targetDuration = std::chrono::duration<double>(1.0 / rendererPtr->targetFps);
        auto elapsed = std::chrono::steady_clock::now() - frameStart;
        if (elapsed < targetDuration)
            std::this_thread::sleep_for(targetDuration - elapsed);
    }

    {
        FrameContext ctx = makeContext();
        stateStack.applyPending(ctx);

        // A requested state swap (editor Play / game Stop) runs as
        // pop-then-push so onExit/onEnter bracket the switch cleanly.
        if (transitionRequest.pending()) {
            stateStack.popState();
            stateStack.pushState(std::move(transitionRequest.next));
            stateStack.applyPending(ctx);
        }
    }
    RT_PROFILE_FRAME();
}

void Application::end() {
    {
        FrameContext ctx = makeContext();
        stateStack.onStop(ctx);
    }

    int winWidth, winHeight;
    window->getSize(winWidth, winHeight);
    settingsStore.setDouble("windowWidth", winWidth);
    settingsStore.setDouble("windowHeight", winHeight);
    settingsStore.setDouble("fixedTimestep", clock.fixedStep());
    settingsStore.save(settingsFile);

    audioEngine.shutdown();
    window->shutdownDebugUi();
    rendererPtr->shutdownDebugUi();
    rendererPtr->shutdown();
}

void Application::run() {
    begin();
    while (running()) runFrame();
    end();
}

}  // namespace engine
