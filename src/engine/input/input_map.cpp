#include "input_map.h"

#include <algorithm>

namespace engine {

namespace {

// Mouse buttons and gamepad buttons share the encoded-source space with keys;
// offset each past the others' range so encodings never collide.
constexpr int MOUSE_SOURCE_BASE = 100000;
constexpr int GAMEPAD_BUTTON_BASE = 200000;
constexpr int XR_BUTTON_BASE = 300000;

// Canonical KeyCode <-> name table. Order mirrors the enum for readability.
const std::vector<std::pair<KeyCode, const char*>>& keyNameTable() {
    static const std::vector<std::pair<KeyCode, const char*>> table = {
        {KeyCode::A, "A"}, {KeyCode::B, "B"}, {KeyCode::C, "C"}, {KeyCode::D, "D"},
        {KeyCode::E, "E"}, {KeyCode::F, "F"}, {KeyCode::G, "G"}, {KeyCode::H, "H"},
        {KeyCode::I, "I"}, {KeyCode::J, "J"}, {KeyCode::K, "K"}, {KeyCode::L, "L"},
        {KeyCode::M, "M"}, {KeyCode::N, "N"}, {KeyCode::O, "O"}, {KeyCode::P, "P"},
        {KeyCode::Q, "Q"}, {KeyCode::R, "R"}, {KeyCode::S, "S"}, {KeyCode::T, "T"},
        {KeyCode::U, "U"}, {KeyCode::V, "V"}, {KeyCode::W, "W"}, {KeyCode::X, "X"},
        {KeyCode::Y, "Y"}, {KeyCode::Z, "Z"},
        {KeyCode::Num0, "Num0"}, {KeyCode::Num1, "Num1"}, {KeyCode::Num2, "Num2"},
        {KeyCode::Num3, "Num3"}, {KeyCode::Num4, "Num4"}, {KeyCode::Num5, "Num5"},
        {KeyCode::Num6, "Num6"}, {KeyCode::Num7, "Num7"}, {KeyCode::Num8, "Num8"},
        {KeyCode::Num9, "Num9"},
        {KeyCode::Up, "Up"}, {KeyCode::Down, "Down"}, {KeyCode::Left, "Left"},
        {KeyCode::Right, "Right"},
        {KeyCode::Escape, "Escape"}, {KeyCode::Space, "Space"},
        {KeyCode::Enter, "Enter"}, {KeyCode::Tab, "Tab"},
        {KeyCode::Backspace, "Backspace"},
        {KeyCode::LeftShift, "LeftShift"}, {KeyCode::RightShift, "RightShift"},
        {KeyCode::LeftControl, "LeftControl"}, {KeyCode::RightControl, "RightControl"},
        {KeyCode::LeftAlt, "LeftAlt"}, {KeyCode::RightAlt, "RightAlt"},
        {KeyCode::Comma, "Comma"}, {KeyCode::Period, "Period"},
        {KeyCode::Slash, "Slash"}, {KeyCode::Semicolon, "Semicolon"},
        {KeyCode::Minus, "Minus"}, {KeyCode::Equal, "Equal"},
        {KeyCode::LeftBracket, "LeftBracket"},
        {KeyCode::RightBracket, "RightBracket"},
    };
    return table;
}

}  // namespace

KeyCode keyCodeFromName(const std::string& name) {
    for (const auto& entry : keyNameTable())
        if (name == entry.second) return entry.first;
    return KeyCode::Unknown;
}

const char* keyCodeName(KeyCode key) {
    for (const auto& entry : keyNameTable())
        if (key == entry.first) return entry.second;
    return "Unknown";
}

int InputMap::encodeKey(KeyCode key) {
    return static_cast<int>(key);
}

int InputMap::encodeMouse(MouseButton button) {
    return MOUSE_SOURCE_BASE + static_cast<int>(button);
}

int InputMap::encodeGamepadButton(GamepadButton button) {
    return GAMEPAD_BUTTON_BASE + static_cast<int>(button);
}

int InputMap::encodeXrButton(XrButton button) {
    return XR_BUTTON_BASE + static_cast<int>(button);
}

Real InputMap::applyDeadzone(Real value) const {
    Real magnitude = std::abs(value);
    if (magnitude < deadzone) return 0.0;
    // Rescale (deadzone, 1] back to (0, 1] so motion starts smoothly at the edge
    // of the dead region rather than jumping.
    Real sign = value < 0.0 ? -1.0 : 1.0;
    Real span = 1.0 - deadzone;
    if (span <= 0.0) return value;
    return sign * std::min(Real(1.0), (magnitude - deadzone) / span);
}

void InputMap::setDeadzone(Real value) {
    deadzone = std::clamp(value, Real(0.0), Real(0.95));
}

void InputMap::bindButton(const std::string& action, KeyCode key) {
    buttons[action].push_back(encodeKey(key));
}

void InputMap::bindButton(const std::string& action, MouseButton button) {
    buttons[action].push_back(encodeMouse(button));
}

void InputMap::bindButton(const std::string& action, GamepadButton button) {
    buttons[action].push_back(encodeGamepadButton(button));
}

void InputMap::bindButton(const std::string& action, XrButton button) {
    buttons[action].push_back(encodeXrButton(button));
}

void InputMap::bindAxis(const std::string& axis, KeyCode key, Real scale) {
    axes[axis].push_back({AxisContribution::Kind::Digital, encodeKey(key), scale});
}

void InputMap::bindAxis(const std::string& axis, GamepadAxis gamepadAxis,
                        Real scale) {
    axes[axis].push_back({AxisContribution::Kind::Gamepad,
                          static_cast<int>(gamepadAxis), scale});
}

bool InputMap::bindButtonByName(const std::string& action,
                                const std::string& keyName) {
    KeyCode key = keyCodeFromName(keyName);
    if (key == KeyCode::Unknown) return false;
    bindButton(action, key);
    return true;
}

bool InputMap::bindAxisByName(const std::string& axis, const std::string& keyName,
                              Real scale) {
    KeyCode key = keyCodeFromName(keyName);
    if (key == KeyCode::Unknown) return false;
    bindAxis(axis, key, scale);
    return true;
}

void InputMap::clearBindings() {
    buttons.clear();
    axes.clear();
}

namespace {

// Display names for the non-key source spaces (keys use keyCodeName).
const char* mouseButtonName(int index) {
    switch (static_cast<MouseButton>(index)) {
        case MouseButton::Left: return "Mouse Left";
        case MouseButton::Right: return "Mouse Right";
        case MouseButton::Middle: return "Mouse Middle";
    }
    return "Mouse ?";
}

const char* gamepadButtonName(int index) {
    static const char* names[] = {
        "Pad A", "Pad B", "Pad X", "Pad Y",
        "Pad LeftBumper", "Pad RightBumper",
        "Pad Back", "Pad Start", "Pad Guide",
        "Pad L3", "Pad R3",
        "Pad DpadUp", "Pad DpadRight", "Pad DpadDown", "Pad DpadLeft",
    };
    return (index >= 0 && index < static_cast<int>(GAMEPAD_BUTTON_COUNT))
               ? names[index]
               : "Pad ?";
}

const char* gamepadAxisName(int index) {
    static const char* names[] = {
        "Pad LeftX", "Pad LeftY", "Pad RightX", "Pad RightY",
        "Pad LeftTrigger", "Pad RightTrigger",
    };
    return (index >= 0 && index < static_cast<int>(GAMEPAD_AXIS_COUNT))
               ? names[index]
               : "Pad ?";
}

// Decode an encoded button/digital source back to a display name.
std::string sourceName(int encoded) {
    if (encoded >= XR_BUTTON_BASE) return "XR Pinch";
    if (encoded >= GAMEPAD_BUTTON_BASE)
        return gamepadButtonName(encoded - GAMEPAD_BUTTON_BASE);
    if (encoded >= MOUSE_SOURCE_BASE)
        return mouseButtonName(encoded - MOUSE_SOURCE_BASE);
    return keyCodeName(static_cast<KeyCode>(encoded));
}

}  // namespace

std::vector<InputMap::BindingDesc> InputMap::listBindings() const {
    std::vector<BindingDesc> out;
    for (const auto& entry : buttons)
        for (int source : entry.second)
            out.push_back({entry.first, sourceName(source), false, 0.0});
    for (const auto& entry : axes)
        for (const AxisContribution& c : entry.second)
            out.push_back({entry.first,
                           c.kind == AxisContribution::Kind::Digital
                               ? sourceName(c.source)
                               : gamepadAxisName(c.source),
                           true, c.scale});
    std::sort(out.begin(), out.end(),
              [](const BindingDesc& a, const BindingDesc& b) {
                  if (a.input != b.input) return a.input < b.input;
                  return a.action < b.action;
              });
    return out;
}

void InputMap::beginFrame() {
    pressedSources.clear();
    releasedSources.clear();
}

void InputMap::setTextInputCaptured(bool captured) {
    if (captured && !textCaptured) {
        // Keys held when the field took focus would otherwise stick down
        // (their releases arrive, but as text-field input in the user's mind).
        heldSources.clear();
    }
    textCaptured = captured;
}

void InputMap::processEvent(const Event& event) {
    switch (event.type) {
        case EventType::KeyPressed:
            // A held key repeats; only the initial press is an edge.
            if (event.repeat) break;
            if (textCaptured) break;   // typed into a text field, not an action
            heldSources.insert(encodeKey(event.key));
            pressedSources.insert(encodeKey(event.key));
            break;
        case EventType::KeyReleased:
            heldSources.erase(encodeKey(event.key));
            releasedSources.insert(encodeKey(event.key));
            break;
        case EventType::MouseButtonPressed:
            heldSources.insert(encodeMouse(event.button));
            pressedSources.insert(encodeMouse(event.button));
            break;
        case EventType::MouseButtonReleased:
            heldSources.erase(encodeMouse(event.button));
            releasedSources.insert(encodeMouse(event.button));
            break;
        case EventType::XrButtonPressed:
            heldSources.insert(encodeXrButton(event.xrButton));
            pressedSources.insert(encodeXrButton(event.xrButton));
            break;
        case EventType::XrButtonReleased:
            heldSources.erase(encodeXrButton(event.xrButton));
            releasedSources.insert(encodeXrButton(event.xrButton));
            break;
        case EventType::WindowUnfocused:
            // Dropping focus means we stop seeing key-up events, so clear held
            // state to avoid keys that stick down until the next press.
            heldSources.clear();
            break;
        default:
            break;
    }
}

void InputMap::updateGamepad(const GamepadState& pad) {
    // Buttons: derive edges by diffing this frame against the last. A
    // disconnected pad reads all-false, so its buttons release cleanly.
    for (std::size_t i = 0; i < GAMEPAD_BUTTON_COUNT; i++) {
        bool down = pad.connected && pad.buttons[i];
        bool was = prevGamepadButtons[i];
        int source = GAMEPAD_BUTTON_BASE + static_cast<int>(i);
        if (down && !was) {
            heldSources.insert(source);
            pressedSources.insert(source);
        } else if (!down && was) {
            heldSources.erase(source);
            releasedSources.insert(source);
        }
        prevGamepadButtons[i] = down;
    }

    // Axes: store deadzoned values for axis() to read.
    for (std::size_t i = 0; i < GAMEPAD_AXIS_COUNT; i++) {
        gamepadAxisValues[i] = pad.connected ? applyDeadzone(pad.axes[i]) : 0.0;
    }
}

bool InputMap::anyBoundSourceIn(const std::string& action,
                                const std::unordered_set<int>& sources) const {
    auto it = buttons.find(action);
    if (it == buttons.end()) return false;
    for (int source : it->second)
        if (sources.count(source)) return true;
    return false;
}

bool InputMap::held(const std::string& action) const {
    return anyBoundSourceIn(action, heldSources);
}

bool InputMap::pressed(const std::string& action) const {
    return anyBoundSourceIn(action, pressedSources);
}

bool InputMap::released(const std::string& action) const {
    return anyBoundSourceIn(action, releasedSources);
}

Real InputMap::axis(const std::string& name) const {
    auto it = axes.find(name);
    if (it == axes.end()) return 0.0;
    Real value = 0.0;
    for (const auto& contribution : it->second) {
        if (contribution.kind == AxisContribution::Kind::Digital) {
            if (heldSources.count(contribution.source)) value += contribution.scale;
        } else {
            value += contribution.scale * gamepadAxisValues[contribution.source];
        }
    }
    return std::clamp(value, Real(-1.0), Real(1.0));
}

}  // namespace engine

