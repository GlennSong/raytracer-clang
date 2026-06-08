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

    InputState()
        : mouseX(0), mouseY(0), mouseDeltaX(0), mouseDeltaY(0),
          scrollDelta(0), mouseLeftDown(false), mouseRightDown(false),
          keyW(false), keyA(false), keyS(false), keyD(false),
          keyQ(false), keyE(false), keyShift(false),
          keyUp(false), keyDown(false) {}
};

// The windowing/input seam. All GLFW (and any other backend) state is hidden in
// the .cpp-only Impl, so this header carries no windowing-library types — engine
// and app code depend only on backend-neutral InputState / Event.
class Window {
public:
    Window();
    ~Window();

    bool initialize(int width, int height, const std::string& title);
    void shutdown();
    bool shouldClose() const;
    void pollEvents();
    void getSize(int& width, int& height) const;
    void getFramebufferSize(int& width, int& height) const;

    // Opaque native OS window pointer (NSWindow* on macOS, HWND on Windows...)
    // for the renderer to bind its surface to. The only seam through which a
    // platform handle crosses into the renderer; keeps GLFW out of the backend.
    void* nativeWindowHandle() const;

    const InputState& getInput() const;
    const std::vector<Event>& getEvents() const;
    // Polled gamepad snapshots, indexed by device id. Refreshed by pollEvents;
    // connect/disconnect also surface as events in getEvents().
    const GamepadSet& getGamepads() const;
    double getDeltaTime() const;

    // Invoked to redraw a frame. The platform calls this both from the normal
    // loop and during a modal resize (when pollEvents blocks), so the window
    // keeps painting instead of freezing while the user drags its edge.
    void setDrawCallback(std::function<void()> callback);

    // Debug-UI (Dear ImGui) GLFW-backend hooks — see ADR-0011. No-ops unless
    // the build defines RT_ENABLE_IMGUI. newDebugUiFrame() is called from
    // pollEvents; init/shutdown bracket the app lifetime.
    void setCursorMode(CursorMode mode);
    void resetMouseDelta();

    void initDebugUi();
    void newDebugUiFrame();
    void shutdownDebugUi();

    // Opaque implementation, defined in window.cpp. Public only so the
    // file-local platform callbacks there can name the type; its members and
    // the impl pointer below remain implementation details.
    struct Impl;

private:
    std::unique_ptr<Impl> impl;
};


}  // namespace engine

#endif
