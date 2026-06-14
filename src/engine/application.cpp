#include "application.h"
#include "states/debug_overlay_state.h"
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
    if (!rendererPtr->initialize(window->nativeWindowHandle(),
                                 framebufferWidth, framebufferHeight)) {
        return false;
    }

    // Debug UI (ADR-0011): renderer creates the ImGui context, then the window
    // attaches its platform backend. Both no-ops without RT_ENABLE_IMGUI.
    // The asset manager owns GPU mesh lifetime, driving the renderer through
    // the seam adapter (ROADMAP 3.1). Created here, once the renderer exists.
    meshUploader = std::make_unique<RendererMeshUploader>(*rendererPtr);
    assetManager = std::make_unique<AssetManager>(*meshUploader);

    rendererPtr->initDebugUi(window->nativeWindowHandle());
    window->initDebugUi();

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
        window->getInput(), inputMap, playerInputs,
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
    reconcileFramebuffer();
    FrameContext ctx = makeContext();
    rendererPtr->beginFrame();
    stateStack.forEachRenderable([&](AppState& state) { state.render(ctx); });
    rendererPtr->endFrame();
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

    {
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
        playerInputs.updateGamepads(window->getGamepads());
        inputMap.updateGamepad(window->getGamepads()[0]);
        stateStack.forEachActive([&](AppState& state) { state.update(ctx); });
    }

    int steps = clock.advance(frameDelta);
    interpolation = clock.interpolationAlpha();
    {
        FrameContext ctx = makeContext();
        for (int i = 0; i < steps; i++)
            stateStack.forEachActive([&](AppState& state) { state.fixedUpdate(ctx); });
    }

    auto frameStart = std::chrono::steady_clock::now();
    renderFrame();

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
