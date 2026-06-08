#include "application.h"
#include "states/debug_overlay_state.h"

namespace engine {

Application::Application() = default;
Application::~Application() = default;

bool Application::initialize(const Config& config) {
    settingsFile = config.settingsFile;
    settingsStore.load(settingsFile);

    int winWidth = static_cast<int>(settingsStore.getDouble("windowWidth", config.width));
    int winHeight = static_cast<int>(settingsStore.getDouble("windowHeight", config.height));

    if (!window.initialize(winWidth, winHeight, config.title)) return false;

    window.getFramebufferSize(framebufferWidth, framebufferHeight);

    rendererPtr = Renderer::create();
    if (!rendererPtr->initialize(window.nativeWindowHandle(),
                                 framebufferWidth, framebufferHeight)) {
        return false;
    }

    // Debug UI (ADR-0011): renderer creates the ImGui context, then the window
    // attaches its GLFW backend. Both no-ops without RT_ENABLE_IMGUI.
    rendererPtr->initDebugUi(window.nativeWindowHandle());
    window.initDebugUi();

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
    return FrameContext{
        worldState, *rendererPtr, view, clock, settingsStore, jobs,
        window.getInput(), inputMap, playerInputs,
        framebufferWidth, framebufferHeight,
        frameDelta, interpolation, quit
    };
}

void Application::reconcileFramebuffer() {
    int w, h;
    window.getFramebufferSize(w, h);
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

void Application::run() {
    {
        FrameContext ctx = makeContext();
        stateStack.onStart(ctx);
    }

    // Render through the window so it keeps painting during a modal resize,
    // when pollEvents blocks and the loop below is suspended.
    window.setDrawCallback([this]() { renderFrame(); });

    while (!window.shouldClose() && !quit) {
        window.pollEvents();
        frameDelta = window.getDeltaTime();
        reconcileFramebuffer();

        {
            FrameContext ctx = makeContext();
            inputMap.beginFrame();
            playerInputs.beginFrame();
            for (const Event& event : window.getEvents()) {
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
                            std::make_unique<DebugOverlayState>(window));
                        debugOverlayActive = true;
                    }
                } else {
                    stateStack.onEvent(event, ctx);
                }
            }
            playerInputs.updateGamepads(window.getGamepads());
            inputMap.updateGamepad(window.getGamepads()[0]);
            stateStack.forEachActive([&](AppState& state) { state.update(ctx); });
        }

        int steps = clock.advance(frameDelta);
        interpolation = clock.interpolationAlpha();
        {
            FrameContext ctx = makeContext();
            for (int i = 0; i < steps; i++)
                stateStack.forEachActive([&](AppState& state) { state.fixedUpdate(ctx); });
        }

        renderFrame();

        {
            FrameContext ctx = makeContext();
            stateStack.applyPending(ctx);
        }
    }

    {
        FrameContext ctx = makeContext();
        stateStack.onStop(ctx);
    }

    int winWidth, winHeight;
    window.getSize(winWidth, winHeight);
    settingsStore.setDouble("windowWidth", winWidth);
    settingsStore.setDouble("windowHeight", winHeight);
    settingsStore.setDouble("fixedTimestep", clock.fixedStep());
    settingsStore.save(settingsFile);

    window.shutdownDebugUi();
    rendererPtr->shutdownDebugUi();
    rendererPtr->shutdown();
}

}  // namespace engine
