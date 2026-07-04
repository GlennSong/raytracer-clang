#ifndef RAYTRACER_EVENT_H
#define RAYTRACER_EVENT_H

namespace engine {

// Window/input events, produced by the platform window layer and consumed by
// the application. These types are deliberately independent of any windowing
// library: KeyCode values do NOT mirror GLFW (or any backend) constants, so
// nothing outside the window implementation can couple to a backend value.
// Swapping GLFW for another backend means rewriting only window.cpp.

enum class EventType {
    KeyPressed,
    KeyReleased,
    MouseButtonPressed,
    MouseButtonReleased,
    MouseMoved,
    MouseScrolled,
    WindowResized,        // logical window size changed
    FramebufferResized,   // pixel size changed — what the renderer resizes to
    WindowFocused,
    WindowUnfocused,
    WindowMinimized,
    WindowRestored,
    WindowCloseRequested,
    GamepadConnected,
    GamepadDisconnected,
};

enum class KeyCode {
    Unknown,
    A, B, C, D, E, F, G, H, I, J, K, L, M,
    N, O, P, Q, R, S, T, U, V, W, X, Y, Z,
    Num0, Num1, Num2, Num3, Num4, Num5, Num6, Num7, Num8, Num9,
    Up, Down, Left, Right,
    Escape, Space, Enter, Tab, Backspace,
    LeftShift, RightShift, LeftControl, RightControl, LeftAlt, RightAlt,
    Comma, Period, Slash, Semicolon, Minus, Equal,
    LeftBracket, RightBracket,
    GraveAccent,
};

enum class MouseButton {
    Left,
    Right,
    Middle,
};

// Flat tagged event. Only the fields relevant to `type` are meaningful:
//   Key{Pressed,Released}        -> key, repeat
//   MouseButton{Pressed,Released}-> button, x, y (cursor position)
//   MouseMoved                   -> x, y (cursor position)
//   MouseScrolled                -> x, y (scroll offset)
//   WindowResized/FramebufferResized -> width, height
//   Gamepad{Connected,Disconnected}  -> gamepad (device id)
struct Event {
    EventType type;
    KeyCode key = KeyCode::Unknown;
    bool repeat = false;
    MouseButton button = MouseButton::Left;
    double x = 0.0, y = 0.0;
    int width = 0, height = 0;
    int gamepad = -1;

    explicit Event(EventType type) : type(type) {}
};


}  // namespace engine

#endif
