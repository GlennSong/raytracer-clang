#include "gamepad_glfw.h"
#include "../log.h"

#include <GLFW/glfw3.h>

#include <fstream>
#include <sstream>

namespace engine {

namespace {

// Translate GLFW's gamepad snapshot into our backend-neutral one. Our
// GamepadButton/GamepadAxis enums mirror GLFW's standard layout order, but we
// map explicitly (and normalize triggers from GLFW's [-1, 1] to [0, 1]) so the
// neutral types stay decoupled from GLFW values.
void fillGamepadState(GamepadState& out, const GLFWgamepadstate& in) {
    out.connected = true;
    for (std::size_t i = 0; i < GAMEPAD_BUTTON_COUNT; i++)
        out.buttons[i] = (in.buttons[i] == GLFW_PRESS);

    out.axes[static_cast<std::size_t>(GamepadAxis::LeftX)] = in.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftY)] = in.axes[GLFW_GAMEPAD_AXIS_LEFT_Y];
    out.axes[static_cast<std::size_t>(GamepadAxis::RightX)] = in.axes[GLFW_GAMEPAD_AXIS_RIGHT_X];
    out.axes[static_cast<std::size_t>(GamepadAxis::RightY)] = in.axes[GLFW_GAMEPAD_AXIS_RIGHT_Y];
    out.axes[static_cast<std::size_t>(GamepadAxis::LeftTrigger)] =
        (in.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER] + 1.0f) * 0.5f;
    out.axes[static_cast<std::size_t>(GamepadAxis::RightTrigger)] =
        (in.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] + 1.0f) * 0.5f;
}

bool windowlessUp = false;

}  // namespace

bool loadGlfwGamepadMappings(const char* path) {
    std::ifstream file(path);
    if (!file) return false;
    std::ostringstream buf;
    buf << file.rdbuf();
    const std::string mappings = buf.str();
    if (glfwUpdateGamepadMappings(mappings.c_str())) {
        LOG_INFO << "Loaded gamepad mappings from " << path;
        return true;
    }
    LOG_WARN << path << " found but glfwUpdateGamepadMappings failed";
    return false;
}

void pollGlfwGamepads(GamepadSet& pads) {
    for (int jid = 0; jid < MAX_GAMEPADS; jid++) {
        GamepadState& slot = pads[static_cast<std::size_t>(jid)];
        GLFWgamepadstate gs;
        if (jid <= GLFW_JOYSTICK_LAST && glfwJoystickIsGamepad(jid) && glfwGetGamepadState(jid, &gs))
            fillGamepadState(slot, gs);
        else
            slot = GamepadState{};
    }
}

std::string glfwGamepadName(int jid) {
    if (jid < 0 || jid > GLFW_JOYSTICK_LAST || !glfwJoystickPresent(jid)) return "";
    if (glfwJoystickIsGamepad(jid)) {
        const char* n = glfwGetGamepadName(jid);
        if (n && *n) return n;
    }
    const char* n = glfwGetJoystickName(jid);
    return n ? n : "";
}

bool initWindowlessGlfwGamepads(const char* mappingsPath) {
    if (windowlessUp) return true;
    if (!glfwInit()) {
        const char* why = nullptr;
        glfwGetError(&why);
        LOG_WARN << "Gamepads unavailable: glfwInit failed" << (why ? std::string(" (") + why + ")" : "");
        return false;
    }
    windowlessUp = true;
    loadGlfwGamepadMappings(mappingsPath);
    for (int jid = 0; jid <= GLFW_JOYSTICK_LAST; jid++)
        if (glfwJoystickPresent(jid))
            LOG_INFO << "Gamepad " << jid << ": \"" << glfwGamepadName(jid) << "\""
                     << (glfwJoystickIsGamepad(jid) ? "" : " (no gamepad mapping: add its GUID to gamecontrollerdb.txt)");
    return true;
}

void pumpWindowlessGlfw() {
    if (windowlessUp) glfwPollEvents();
}

void shutdownWindowlessGlfwGamepads() {
    if (!windowlessUp) return;
    glfwTerminate();
    windowlessUp = false;
}

}  // namespace engine
