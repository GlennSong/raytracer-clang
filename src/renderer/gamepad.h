#ifndef RAYTRACER_GAMEPAD_H
#define RAYTRACER_GAMEPAD_H

#include <array>
#include <cstddef>

// Backend-neutral gamepad input. Like KeyCode/Event, these enums are the
// engine's own — they do NOT mirror GLFW (or any backend) constants, so the
// window seam is the only place that maps hardware to them (ADR-0001, ADR-0010).
// The layout is the standard "Xbox-style" pad that backends normalize varied
// controllers onto (GLFW does this via its built-in gamepad mapping database).

enum class GamepadButton {
    A, B, X, Y,
    LeftBumper, RightBumper,
    Back, Start, Guide,
    LeftThumb, RightThumb,
    DpadUp, DpadRight, DpadDown, DpadLeft,
    Count
};

enum class GamepadAxis {
    LeftX, LeftY,
    RightX, RightY,
    LeftTrigger, RightTrigger,
    Count
};

constexpr std::size_t GAMEPAD_BUTTON_COUNT =
    static_cast<std::size_t>(GamepadButton::Count);
constexpr std::size_t GAMEPAD_AXIS_COUNT =
    static_cast<std::size_t>(GamepadAxis::Count);

// Maximum simultaneously-tracked gamepads (GLFW supports joystick ids 0..15).
constexpr int MAX_GAMEPADS = 16;

// A polled snapshot of one gamepad. Sticks are in [-1, 1]; triggers normalized
// to [0, 1] (rest 0, fully pressed 1). Filled by the window layer each frame.
struct GamepadState {
    bool connected = false;
    std::array<bool, GAMEPAD_BUTTON_COUNT> buttons{};
    std::array<float, GAMEPAD_AXIS_COUNT> axes{};

    bool button(GamepadButton b) const {
        return buttons[static_cast<std::size_t>(b)];
    }
    float axis(GamepadAxis a) const {
        return axes[static_cast<std::size_t>(a)];
    }
};

// All tracked gamepads, indexed by device id [0, MAX_GAMEPADS).
using GamepadSet = std::array<GamepadState, MAX_GAMEPADS>;

#endif
