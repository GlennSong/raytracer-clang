#include "input_map.h"

#include <algorithm>

namespace {

// Mouse buttons share the encoded-source space with keys; offset them past the
// KeyCode range so the two never collide.
constexpr int MOUSE_SOURCE_BASE = 100000;

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

void InputMap::bindButton(const std::string& action, KeyCode key) {
    buttons[action].push_back(encodeKey(key));
}

void InputMap::bindButton(const std::string& action, MouseButton button) {
    buttons[action].push_back(encodeMouse(button));
}

void InputMap::bindAxis(const std::string& axis, KeyCode key, Real scale) {
    axes[axis].emplace_back(encodeKey(key), scale);
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

void InputMap::beginFrame() {
    pressedSources.clear();
    releasedSources.clear();
}

void InputMap::processEvent(const Event& event) {
    switch (event.type) {
        case EventType::KeyPressed:
            // A held key repeats; only the initial press is an edge.
            if (event.repeat) break;
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
        case EventType::WindowUnfocused:
            // Dropping focus means we stop seeing key-up events, so clear held
            // state to avoid keys that stick down until the next press.
            heldSources.clear();
            break;
        default:
            break;
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
    for (const auto& contribution : it->second)
        if (heldSources.count(contribution.first)) value += contribution.second;
    return std::clamp(value, Real(-1.0), Real(1.0));
}
