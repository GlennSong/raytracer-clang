#include "application.h"

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

FrameContext Application::makeContext() {
    return FrameContext{
        worldState, *rendererPtr, view, clock, settingsStore,
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
    for (auto& system : systems) system->render(ctx);
    rendererPtr->endFrame();
}

void Application::run() {
    {
        FrameContext ctx = makeContext();
        for (auto& system : systems) system->onStart(ctx);
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
                for (auto& system : systems) system->onEvent(event, ctx);
            }
            playerInputs.updateGamepads(window.getGamepads());
            // The viewer's global actions (camera, dev controls) also follow the
            // first gamepad, so a single controller drives the viewer without a
            // player slot. Per-player gameplay input still flows via players.
            inputMap.updateGamepad(window.getGamepads()[0]);
            for (auto& system : systems) system->update(ctx);
        }

        int steps = clock.advance(frameDelta);
        interpolation = clock.interpolationAlpha();
        {
            FrameContext ctx = makeContext();
            for (int i = 0; i < steps; i++)
                for (auto& system : systems) system->fixedUpdate(ctx);
        }

        renderFrame();
    }

    {
        FrameContext ctx = makeContext();
        for (auto& system : systems) system->onStop(ctx);
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
