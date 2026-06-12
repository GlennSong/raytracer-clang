#ifndef RAYTRACER_WINDOW_H
#define RAYTRACER_WINDOW_H

#include "event.h"
#include "gamepad.h"
#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace engine {

enum class CursorMode { Normal, Disabled };

struct InputState {
    double mouseX, mouseY;
    double mouseDeltaX, mouseDeltaY;
    double scrollDelta;
    bool mouseLeftDown;
    bool mouseRightDown;
    bool keyW, keyA, keyS, keyD, keyQ, keyE;
    bool keyShift;
    bool keyUp, keyDown;
    // True while the debug UI (ImGui) owns the pointer — hovering or dragging
    // a panel. Mouse-driven camera/gameplay input should yield (always false
    // without RT_ENABLE_IMGUI).
    bool uiWantsMouse;

    InputState()
        : mouseX(0), mouseY(0), mouseDeltaX(0), mouseDeltaY(0),
          scrollDelta(0), mouseLeftDown(false), mouseRightDown(false),
          keyW(false), keyA(false), keyS(false), keyD(false),
          keyQ(false), keyE(false), keyShift(false),
          keyUp(false), keyDown(false), uiWantsMouse(false) {}
};

// The windowing/input seam — now an interface (editor-app plan, Phase A1).
// Two implementations exist: the GLFW-backed platform window (window.cpp,
// behind createPlatformWindow) used by the standalone runtime, and
// HostedWindow (hosted_window.h), where a host application (the Qt editor)
// owns the real window and injects events. Engine and app code depend only on
// this interface and the backend-neutral InputState / Event types.
class Window {
public:
    virtual ~Window() = default;

    virtual bool initialize(int width, int height, const std::string& title) = 0;
    virtual void shutdown() = 0;
    virtual bool shouldClose() const = 0;
    virtual void pollEvents() = 0;
    virtual void getSize(int& width, int& height) const = 0;
    virtual void getFramebufferSize(int& width, int& height) const = 0;

    // Opaque native OS handle (NSWindow*/NSView* on macOS, HWND on Windows...)
    // for the renderer to bind its surface to. The only seam through which a
    // platform handle crosses into the renderer.
    virtual void* nativeWindowHandle() const = 0;

    virtual const InputState& getInput() const = 0;
    virtual const std::vector<Event>& getEvents() const = 0;
    // Polled gamepad snapshots, indexed by device id. Refreshed by pollEvents;
    // connect/disconnect also surface as events in getEvents().
    virtual const GamepadSet& getGamepads() const = 0;
    virtual double getDeltaTime() const = 0;

    // Invoked to redraw a frame. The platform calls this both from the normal
    // loop and during a modal resize (when pollEvents blocks), so the window
    // keeps painting instead of freezing while the user drags its edge. Hosted
    // windows ignore it — the host drives painting.
    virtual void setDrawCallback(std::function<void()> callback) = 0;

    virtual void setCursorMode(CursorMode mode) = 0;
    virtual void resetMouseDelta() = 0;

    // Debug-UI (Dear ImGui) platform hooks — see ADR-0011. No-ops unless the
    // build defines RT_ENABLE_IMGUI. newDebugUiFrame() is called from
    // pollEvents; init/shutdown bracket the app lifetime.
    virtual void initDebugUi() = 0;
    virtual void newDebugUiFrame() = 0;
    virtual void shutdownDebugUi() = 0;
};

// The GLFW-backed platform window (the standalone runtime's default).
// Defined in window.cpp; the one place GLFW types exist.
std::unique_ptr<Window> createPlatformWindow();


}  // namespace engine

#endif
