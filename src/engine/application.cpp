#include "application.h"
#include "states/debug_overlay_state.h"
#include "../log.h"
#include "../profile.h"
#include <algorithm>
#include <cmath>
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
    // Unattended pass-cost sweep: measure every post pass at several window
    // sizes and quit, so a ranking needs one command and no interaction (and
    // none of the mistakes a manual procedure invites).
    if (const char* sweepPath = std::getenv("RT_PASS_SWEEP"))
        passSweep.begin(sweepPath);
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
    // RT_DUMP_STATS=1: periodic frame COMPOSITION report on stderr — draw calls,
    // instances, triangles, shadow casters, capacity overflow. Kept alongside
    // the phase ledger above because the two answer different questions: the
    // ledger says WHERE the time went, this says WHAT was submitted. (It is
    // what found CityWalkerSystem eating 10 of 13 ms in the P8.2 round.)
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
                     << rs.trianglesDrawn / 1000000.0 << "M shadowcasters "
                     << rs.shadowCasters << " (terrain " << rs.shadowTerrainNodes
                     << ") overflow i"
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
    // Control channel (ADR-0078): apply externally staged commands HERE, at
    // the top of the frame on the main thread — the one place Settings,
    // Renderer, SimClock, the overlay stack, and transitionRequest are all
    // legally touchable (the socket thread only buffers; ADR-0072 staging).
    pumpControlChannel();
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
                setDebugOverlay(!debugOverlayActive);
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
    if (clock.droppedBacklog())
        LOG_WARN << "SimClock dropped backlog (stall #" << clock.droppedBacklogCount()
                 << "): fixed steps capped at " << steps
                 << " this frame — motion will visibly jump";
    {
        RT_PROFILE_ZONE_NAMED("fixedUpdate");
        frameStats.beginPhase(FramePhase::FixedUpdate);
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
        frameStats.endPhase(FramePhase::FixedUpdate);
        // The ledger times the whole phase; this adds the STEP COUNT, which is
        // what separates "the step got dearer" from "we ran more steps" — the
        // difference between a slow simulation and a backlog.
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

    if (passSweep.active()) {
        FrameContext ctx = makeContext();
        passSweep.update(ctx, *window, quit);
    }

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
    controlChannel.shutdown();   // stop the socket thread before states die
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

// --- control channel (ADR-0078) ---------------------------------------------

bool Application::enableControlChannel(
    std::function<std::unique_ptr<AppState>()> reloadFactory) {
    // RT_CONTROL=0 is the kill-switch; anything else (including unset) is on —
    // the channel is a dev tool's front door, and "Glenn's already-open viewer
    // is attachable" only works if attachability is the default.
    if (const char* env = std::getenv("RT_CONTROL"))
        if (env[0] == '0') return false;
    controlReloadFactory = std::move(reloadFactory);
    return controlChannel.initialize(ControlBackendMode::Socket);
}

void Application::pumpControlChannel() {
    ++frameCounter;
    controlChannel.drain(
        [this](const std::string& line) { return handleControlCommand(line); });
}

void Application::setDebugOverlay(bool on) {
    if (on == debugOverlayActive) return;
    if (on) {
        stateStack.pushState(std::make_unique<DebugOverlayState>(*window));
        debugOverlayActive = true;
    } else {
        stateStack.popState();
        debugOverlayActive = false;
    }
}

std::string Application::handleControlCommand(const std::string& line) {
    const ControlCommand cmd = parseControlCommand(line);
    auto num = [](const std::string& s, double& out) {
        try { out = std::stod(s); return true; } catch (...) { return false; }
    };

    if (cmd.name == "ping") return "ok pong";

    if (cmd.name == "info") {
        char buf[512];
        std::snprintf(buf, sizeof(buf),
                      "ok level=%s frame=%llu paused=%d overlay=%d hud=%d ui=%d",
                      settingsStore.getString("levelPath", "?").c_str(),
                      static_cast<unsigned long long>(frameCounter),
                      clock.paused() ? 1 : 0, debugOverlayActive ? 1 : 0,
                      rendererPtr->showHud ? 1 : 0,
                      rendererPtr->uiHidden ? 1 : 0);
        return buf;
    }

    if (cmd.name == "camera") {
        // Staged as Settings + a one-shot apply flag; CameraSystem consumes it
        // next update (the citysim.* idiom), seeds the fly pose, and detaches
        // so the pose wins even in play mode.
        double v[5];
        if (cmd.args.size() < 5 || !num(cmd.args[0], v[0]) ||
            !num(cmd.args[1], v[1]) || !num(cmd.args[2], v[2]) ||
            !num(cmd.args[3], v[3]) || !num(cmd.args[4], v[4]))
            return "err usage: camera <x> <y> <z> <pitchDeg> <yawDeg>";
        settingsStore.setDouble("flyEyeX", v[0]);
        settingsStore.setDouble("flyEyeY", v[1]);
        settingsStore.setDouble("flyEyeZ", v[2]);
        settingsStore.setDouble("flyPitch", v[3]);
        settingsStore.setDouble("flyYaw", v[4]);
        settingsStore.setDouble("cameraApply", 1.0);
        return "ok camera staged";
    }

    if (cmd.name == "camera?") {
        const CameraState& c = view.camera;
        Vec3 fwd = c.target - c.position;
        const Real len = fwd.length();
        if (len > 1e-9) fwd = fwd * (1.0 / len);
        constexpr Real kRadToDeg = 57.29577951308232;
        char buf[256];
        std::snprintf(buf, sizeof(buf),
                      "ok eye=%.2f,%.2f,%.2f pitch=%.2f yaw=%.2f",
                      c.position.x, c.position.y, c.position.z,
                      std::asin(std::clamp(fwd.y, Real(-1), Real(1))) * kRadToDeg,
                      std::atan2(fwd.x, -fwd.z) * kRadToDeg);
        return buf;
    }

    if (cmd.name == "shot") {
        if (cmd.args.empty()) return "err usage: shot <path.png>";
        // Fires on the next composited frame; the caller polls for the file
        // (the MCP shim does). Replying "written" would stall the frame.
        return rendererPtr->requestFrameDump(cmd.args[0])
                   ? "ok armed " + cmd.args[0]
                   : "err this backend cannot capture frames";
    }

    if (cmd.name == "overlay") {
        if (cmd.args.size() < 2) return "err usage: overlay <name> <on|off>";
        const std::string& what = cmd.args[0];
        const bool on = cmd.args[1] == "on" || cmd.args[1] == "1";
        if (what == "hud") { rendererPtr->showHud = on; return "ok hud"; }
        if (what == "ui") { rendererPtr->uiHidden = !on; return "ok ui"; }
        if (what == "debug") { setDebugOverlay(on); return "ok debug"; }
        // The citysim one-shot keys (city_render consumes and resets them) —
        // exactly what the web build's rt_web_city writes.
        if (what == "master" || what == "agents" || what == "cones" ||
            what == "nav" || what == "plan") {
            settingsStore.setDouble("citysim." + what, on ? 1.0 : 0.0);
            return "ok " + what;
        }
        return "err unknown overlay (hud|ui|debug|master|agents|cones|nav|plan)";
    }

    if (cmd.name == "sim") {
        if (cmd.args.empty()) return "err usage: sim pause|resume|step|speed <x>";
        const std::string& what = cmd.args[0];
        if (what == "pause") { clock.setPaused(true); return "ok paused"; }
        if (what == "resume") { clock.setPaused(false); return "ok resumed"; }
        if (what == "step") { clock.requestStep(); return "ok stepped"; }
        if (what == "speed") {
            double s;
            if (cmd.args.size() < 2 || !num(cmd.args[1], s) || s <= 0.0)
                return "err usage: sim speed <multiplier>";
            clock.setTimeScale(s);
            return "ok speed";
        }
        return "err unknown sim action";
    }

    if (cmd.name == "reload") {
        if (!controlReloadFactory) return "err no reload factory registered";
        requestState(controlReloadFactory());
        return "ok reloading";
    }

    return "err unknown command: " + cmd.name +
           " (ping|info|camera|camera?|shot|overlay|sim|reload)";
}

}  // namespace engine
