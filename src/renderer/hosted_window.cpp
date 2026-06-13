#include "hosted_window.h"

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

namespace engine {

bool HostedWindow::initialize(int width, int height, const std::string&) {
    // The host already owns a real window; just adopt its initial geometry
    // (framebuffer == logical until the host reports a DPI scale).
    if (windowWidth == 0) setSizes(width, height, width, height);
    return true;
}

void HostedWindow::setSizes(int winW, int winH, int fbW, int fbH) {
    // The first geometry report is initialization, not a resize.
    bool changed = (framebufferWidth != 0 || framebufferHeight != 0) &&
                   (fbW != framebufferWidth || fbH != framebufferHeight);
    windowWidth = winW;
    windowHeight = winH;
    framebufferWidth = fbW;
    framebufferHeight = fbH;
    if (changed) {
        Event resized(EventType::FramebufferResized);
        resized.width = fbW;
        resized.height = fbH;
        pendingEvents.push_back(resized);
    }
}

void HostedWindow::getSize(int& width, int& height) const {
    width = windowWidth;
    height = windowHeight;
}

void HostedWindow::getFramebufferSize(int& width, int& height) const {
    width = framebufferWidth;
    height = framebufferHeight;
}

void HostedWindow::setCursorMode(CursorMode mode) {
    bool wantRelative = (mode == CursorMode::Disabled);
    if (wantRelative != relativeMode) {
        relativeMode = wantRelative;
        resetMouseDelta();
    }
    if (cursorModeCallback) cursorModeCallback(mode);
}

void HostedWindow::injectMouseMove(double x, double y) {
    input.mouseX = x;
    input.mouseY = y;
    Event moved(EventType::MouseMoved);
    moved.x = x;
    moved.y = y;
    pendingEvents.push_back(moved);
}

void HostedWindow::injectMouseDelta(double dx, double dy) {
    pendingDeltaX += dx;
    pendingDeltaY += dy;
}

void HostedWindow::injectMouseButton(MouseButton button, bool down,
                                     double x, double y) {
    if (button == MouseButton::Left) input.mouseLeftDown = down;
    if (button == MouseButton::Right) input.mouseRightDown = down;
    Event event(down ? EventType::MouseButtonPressed
                     : EventType::MouseButtonReleased);
    event.button = button;
    event.x = x;
    event.y = y;
    pendingEvents.push_back(event);
}

void HostedWindow::injectScroll(double delta) {
    pendingScroll += delta;
    Event event(EventType::MouseScrolled);
    event.y = delta;
    pendingEvents.push_back(event);
}

void HostedWindow::injectKey(KeyCode key, bool down, bool repeat) {
    // Mirror the platform window's raw-key snapshot (a few systems read these
    // directly: exposure ramp, legacy paths).
    switch (key) {
        case KeyCode::W: input.keyW = down; break;
        case KeyCode::A: input.keyA = down; break;
        case KeyCode::S: input.keyS = down; break;
        case KeyCode::D: input.keyD = down; break;
        case KeyCode::Q: input.keyQ = down; break;
        case KeyCode::E: input.keyE = down; break;
        case KeyCode::Up: input.keyUp = down; break;
        case KeyCode::Down: input.keyDown = down; break;
        case KeyCode::LeftShift:
        case KeyCode::RightShift: input.keyShift = down; break;
        default: break;
    }
    Event event(down ? EventType::KeyPressed : EventType::KeyReleased);
    event.key = key;
    event.repeat = repeat;
    pendingEvents.push_back(event);
}

void HostedWindow::injectEvent(const Event& event) {
    pendingEvents.push_back(event);
}

void HostedWindow::resetMouseDelta() {
    input.mouseDeltaX = 0.0;
    input.mouseDeltaY = 0.0;
    pendingDeltaX = 0.0;
    pendingDeltaY = 0.0;
    firstMouse = true;
}

void HostedWindow::pollEvents() {
    auto now = std::chrono::steady_clock::now();
    deltaTime = firstPoll ? 0.0
                          : std::chrono::duration<double>(now - lastPoll).count();
    lastPoll = now;
    firstPoll = false;

    // Publish what the host injected since the previous frame.
    events.clear();
    events.swap(pendingEvents);

    // Per-frame derived state, matching the platform window's semantics: the
    // delta is position movement since last poll (suppressed on the first
    // sample so focus changes don't spike the camera). While captured, the
    // host reports pure deltas instead (its cursor is pinned to the viewport
    // center, so absolute positions carry no motion).
    if (relativeMode) {
        input.mouseDeltaX = pendingDeltaX;
        input.mouseDeltaY = pendingDeltaY;
        pendingDeltaX = 0.0;
        pendingDeltaY = 0.0;
        firstMouse = true;   // no absolute-position spike on release
    } else if (firstMouse) {
        input.mouseDeltaX = 0.0;
        input.mouseDeltaY = 0.0;
        firstMouse = false;
    } else {
        input.mouseDeltaX = input.mouseX - lastMouseX;
        input.mouseDeltaY = input.mouseY - lastMouseY;
    }
    lastMouseX = input.mouseX;
    lastMouseY = input.mouseY;

    input.scrollDelta = pendingScroll;
    pendingScroll = 0.0;

    newDebugUiFrame();
}

void HostedWindow::newDebugUiFrame() {
#ifdef RT_ENABLE_IMGUI
    // Minimal ImGui platform layer for hosted mode (the GLFW backend can't
    // run here): geometry, time, and pointer state — enough for the
    // in-viewport panels and gizmos. Keyboard/text input for ImGui widgets is
    // deliberately not bridged yet (the native shell owns text editing).
    if (ImGui::GetCurrentContext() == nullptr) return;
    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(windowWidth),
                            static_cast<float>(windowHeight));
    if (windowWidth > 0 && windowHeight > 0)
        io.DisplayFramebufferScale =
            ImVec2(static_cast<float>(framebufferWidth) / windowWidth,
                   static_cast<float>(framebufferHeight) / windowHeight);
    io.DeltaTime = deltaTime > 0.0 ? static_cast<float>(deltaTime)
                                   : 1.0f / 60.0f;
    io.AddMousePosEvent(static_cast<float>(input.mouseX),
                        static_cast<float>(input.mouseY));
    io.AddMouseButtonEvent(0, input.mouseLeftDown);
    io.AddMouseButtonEvent(1, input.mouseRightDown);
    if (input.scrollDelta != 0.0)
        io.AddMouseWheelEvent(0.0f, static_cast<float>(input.scrollDelta));

    input.uiWantsMouse = io.WantCaptureMouse;
#endif
}

}  // namespace engine
