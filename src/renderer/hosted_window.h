#ifndef RAYTRACER_HOSTED_WINDOW_H
#define RAYTRACER_HOSTED_WINDOW_H

#include "window.h"
#include <chrono>

namespace engine {

// The Window implementation for embedding the engine inside a host
// application (the Qt editor — editor-app plan, Phase A1/A2). The host owns
// the real OS window: it provides the native view handle for the renderer,
// forwards its UI-toolkit events through the inject* API, and drives frames
// itself (setDrawCallback is ignored). pollEvents() publishes everything
// injected since the previous frame and derives per-frame state (mouse
// deltas, scroll). Platform-free by construction, so the whole translation
// layer is unit-testable headlessly.
class HostedWindow final : public Window {
public:
    // --- Window interface -------------------------------------------------
    bool initialize(int width, int height, const std::string& title) override;
    void shutdown() override {}
    bool shouldClose() const override { return closeRequested; }
    void pollEvents() override;
    void getSize(int& width, int& height) const override;
    void getFramebufferSize(int& width, int& height) const override;
    void* nativeWindowHandle() const override { return nativeHandle; }
    const InputState& getInput() const override { return input; }
    const std::vector<Event>& getEvents() const override { return events; }
    const GamepadSet& getGamepads() const override { return gamepads; }
    double getDeltaTime() const override { return deltaTime; }
    void setDrawCallback(std::function<void()>) override {}   // host paints
    void setCursorMode(CursorMode) override {}                // host's cursor
    void resetMouseDelta() override;
    void initDebugUi() override {}
    void newDebugUiFrame() override;
    void shutdownDebugUi() override {}

    // --- Host integration --------------------------------------------------
    // The native view the renderer binds to (e.g. an NSView*/CAMetalLayer*
    // from the Qt viewport widget). Set before Application::initialize.
    void setNativeHandle(void* handle) { nativeHandle = handle; }

    // Geometry, in the host's coordinate spaces (logical points vs pixels).
    void setSizes(int winW, int winH, int fbW, int fbH);

    // Input, translated by the host from its toolkit's events. Effective at
    // the next pollEvents(), mirroring the platform window's batching.
    void injectMouseMove(double x, double y);
    void injectMouseButton(MouseButton button, bool down, double x, double y);
    void injectScroll(double delta);
    void injectKey(KeyCode key, bool down, bool repeat = false);
    void injectEvent(const Event& event);   // anything else (focus, resize...)
    void requestClose() { closeRequested = true; }

private:
    void* nativeHandle = nullptr;
    int windowWidth = 0, windowHeight = 0;
    int framebufferWidth = 0, framebufferHeight = 0;
    bool closeRequested = false;

    InputState input;                 // published snapshot
    std::vector<Event> events;        // published queue (this frame's)
    std::vector<Event> pendingEvents; // injected since last poll
    GamepadSet gamepads;              // none in hosted mode (editor)

    double pendingScroll = 0.0;
    double lastMouseX = 0.0, lastMouseY = 0.0;
    bool firstMouse = true;

    double deltaTime = 0.0;
    bool firstPoll = true;
    std::chrono::steady_clock::time_point lastPoll;
};

}  // namespace engine

#endif
