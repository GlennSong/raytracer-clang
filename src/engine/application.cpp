#include "application.h"
#include "states/debug_overlay_state.h"
#include "../log.h"
#include "../profile.h"
#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>

namespace engine {

std::string describeCaptureContext(const FrameContext& ctx) {
    // Resolution first: it is the single biggest lever on a GPU-bound frame
    // (a Retina framebuffer is 4x its logical window), and it was the first
    // thing we could not reconstruct when two captures disagreed by 2x.
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "framebuffer=%dx%d window=%dx%d megapixels=%.2f "
                  "ssao=%d ssr=%d bloom=%d probes=%d envmap=%d prepass=%d "
                  "shadow_cascades=%d shadow_distance=%.0f tonemap=%d "
                  "target_fps=%d fixed_step=%.4f",
                  ctx.framebufferWidth, ctx.framebufferHeight,
                  ctx.windowWidth, ctx.windowHeight,
                  ctx.framebufferWidth * ctx.framebufferHeight / 1.0e6,
                  ctx.renderer.ssaoEnabled ? 1 : 0,
                  ctx.renderer.ssrEnabled ? 1 : 0,
                  ctx.renderer.bloomEnabled ? 1 : 0,
                  ctx.renderer.reflectionProbesEnabled ? 1 : 0,
                  ctx.renderer.environmentMapEnabled ? 1 : 0,
                  ctx.renderer.depthPrepassEnabled ? 1 : 0,
                  ctx.renderer.shadowParams.cascadeCount,
                  static_cast<double>(ctx.renderer.shadowParams.distance),
                  ctx.renderer.tonemapOperator,
                  ctx.renderer.targetFps,
                  ctx.clock.fixedStep());
    return std::string(buf);
}

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

    // Frame-ledger capture without a UI (ADR-0077): RT_FRAME_STATS=<path.csv>
    // records every frame from boot; RT_FRAME_STATS_LOG=<seconds> prints a
    // periodic summary line — the capture path on hosts where only a console
    // is reachable (visionOS device logs, headless soaks). Same env-var
    // convention as RT_DEBUG_VIEW / RT_FRAME_DUMP.
    // Deferred to the first frame so the capture header records the REAL
    // framebuffer size and pass config, not the pre-resize guesses.
    if (const char* capturePath = std::getenv("RT_FRAME_STATS"))
        pendingCapturePath = capturePath;
    if (const char* logEvery = std::getenv("RT_FRAME_STATS_LOG")) {
        statsLogInterval = std::atof(logEvery);
        if (statsLogInterval <= 0.0) statsLogInterval = 5.0;
    }
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
        eventBus, debugLines, audioEngine, frameStats,
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
    // Three brackets, because "render is slow" was never actionable (ADR-0077):
    // acquire BLOCKS while the GPU is behind, encode is the world walk, submit
    // builds the pass graph's command buffers. See FramePhase.
    {
        RT_PROFILE_ZONE_NAMED("acquire");
        frameStats.beginPhase(FramePhase::RenderAcquire);
        rendererPtr->beginFrame();
        frameStats.endPhase(FramePhase::RenderAcquire);
    }
    {
        RT_PROFILE_ZONE_NAMED("encode");
        frameStats.beginPhase(FramePhase::RenderEncode);
        stateStack.forEachRenderable([&](AppState& state) { state.render(ctx); });
        frameStats.endPhase(FramePhase::RenderEncode);
    }
    {
        RT_PROFILE_ZONE_NAMED("submit");
        frameStats.beginPhase(FramePhase::RenderSubmit);
        rendererPtr->endFrame();
        frameStats.endPhase(FramePhase::RenderSubmit);
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
    frameStats.beginFrame();
    // Bracketed because it is OS/driver code we don't control and a real
    // capture caught a 307 ms frame whose named phases summed to 30 ms —
    // whatever stalled it lived outside every bracket (ADR-0077).
    frameStats.beginPhase(FramePhase::Poll);
    window->pollEvents();
    frameDelta = window->getDeltaTime();
    reconcileFramebuffer();
    frameStats.endPhase(FramePhase::Poll);

    if (!pendingCapturePath.empty()) {
        FrameContext ctx = makeContext();
        const std::string context = describeCaptureContext(ctx);
        if (frameStats.startCapture(pendingCapturePath, context))
            LOG_INFO("frame stats capture -> %s (%s)",
                     pendingCapturePath.c_str(), context.c_str());
        else
            LOG_WARN("frame stats capture failed to open %s",
                     pendingCapturePath.c_str());
        pendingCapturePath.clear();
    }
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
        frameStats.beginPhase(FramePhase::Update);
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
        frameStats.endPhase(FramePhase::Update);
    }

    int steps = clock.advance(frameDelta);
    interpolation = clock.interpolationAlpha();
    {
        RT_PROFILE_ZONE_NAMED("fixedUpdate");
        frameStats.beginPhase(FramePhase::FixedUpdate);
        FrameContext ctx = makeContext();
        for (int i = 0; i < steps; i++)
            stateStack.forEachActive([&](AppState& state) { state.fixedUpdate(ctx); });
        frameStats.endPhase(FramePhase::FixedUpdate);
    }

    // Deliver everything enqueued during update/fixedUpdate before the frame
    // renders, so reactions land in the same frame as their cause (ADR-0066).
    frameStats.beginPhase(FramePhase::Dispatch);
    eventBus.dispatchQueued();
    frameStats.endPhase(FramePhase::Dispatch);

    auto frameStart = std::chrono::steady_clock::now();
    frameStats.beginPhase(FramePhase::Render);
    renderFrame();
    frameStats.endPhase(FramePhase::Render);
    // Drop expired debug shapes now that they've been drawn; one-frame shapes
    // (the immediate-mode default) live exactly this long. The modal-resize
    // draw callback renders without expiring, so paused frames keep their
    // overlay.
    debugLines.endFrame();

    if (rendererPtr->targetFps > 0) {
        auto targetDuration = std::chrono::duration<double>(1.0 / rendererPtr->targetFps);
        auto elapsed = std::chrono::steady_clock::now() - frameStart;
        if (elapsed < targetDuration) {
            frameStats.beginPhase(FramePhase::Wait);
            std::this_thread::sleep_for(targetDuration - elapsed);
            frameStats.endPhase(FramePhase::Wait);
        }
    }

    {
        // Its own bracket, separate from the event drain: a push here runs the
        // new state's onEnter (a level load, an overlay's first-time setup),
        // which is a different cause with a different fix.
        frameStats.beginPhase(FramePhase::StateSwap);
        FrameContext ctx = makeContext();
        stateStack.applyPending(ctx);

        // A requested state swap (editor Play / game Stop) runs as
        // pop-then-push so onExit/onEnter bracket the switch cleanly.
        if (transitionRequest.pending()) {
            stateStack.popState();
            stateStack.pushState(std::move(transitionRequest.next));
            stateStack.applyPending(ctx);
        }
        frameStats.endPhase(FramePhase::StateSwap);
    }

    // Close this frame's ledger row with the renderer's submission counters,
    // so a capture correlates time spikes with what was drawn. The same
    // counters feed Tracy plots in profiler builds (ADR-0068).
    RenderStats rs = rendererPtr->getRenderStats();
    // Resources created during THIS frame: the backend counts monotonically,
    // we diff. A nonzero count on a slow frame names the hitch's cause
    // outright — something was built mid-play instead of at load.
    const uint32_t meshUploads =
        static_cast<uint32_t>(rs.meshUploadsTotal - prevMeshUploads);
    const uint32_t textureUploads =
        static_cast<uint32_t>(rs.textureUploadsTotal - prevTextureUploads);
    prevMeshUploads = rs.meshUploadsTotal;
    prevTextureUploads = rs.textureUploadsTotal;
    frameStats.endFrame(frameDelta, steps, rs.drawCalls, rs.totalInstances,
                        rs.trianglesDrawn, rendererPtr->lastGpuFrameMs(),
                        meshUploads, textureUploads);
    RT_PROFILE_PLOT("draw calls", static_cast<int64_t>(rs.drawCalls));
    RT_PROFILE_PLOT("instances", static_cast<int64_t>(rs.totalInstances));
    RT_PROFILE_PLOT("triangles", static_cast<int64_t>(rs.trianglesDrawn));

    if (statsLogInterval > 0.0) {
        statsLogTimer += frameDelta;
        if (statsLogTimer >= statsLogInterval) {
            statsLogTimer = 0.0;
            FrameStats::Summary s = frameStats.summarize();
            LOG_INFO("frame %.2fms avg / %.2f p95 / %.2f max (%.0f fps) | "
                     "update %.2f fixed %.2f render %.2f wait %.2f",
                     s.avgTotalMs, s.p95TotalMs, s.maxTotalMs, s.avgFps,
                     s.avgUpdateMs, s.avgFixedMs, s.avgRenderMs, s.avgWaitMs);
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

    if (frameStats.capturing()) {
        LOG_INFO("frame stats capture closed: %ld frames -> %s",
                 frameStats.capturedFrames(), frameStats.capturePath().c_str());
        frameStats.stopCapture();
    }
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
