#ifndef RAYTRACER_GAMEPAD_GLFW_H
#define RAYTRACER_GAMEPAD_GLFW_H

// GLFW's gamepad path, shared by the viewer's GLFW window and by hosts that
// have no GLFW window at all: the Qt editor off macOS polled only the
// GCController layer (a no-op there), so a plugged-in pad did nothing on Linux.
// GLFW's joystick API needs glfwInit but no window; on Linux it reads evdev
// (/dev/input/event*), which the seat's uaccess ACL opens to the logged-in user.
// Not on the web build (Emscripten's GLFW shim has no gamepad state).

#include "gamepad.h"

#include <string>

namespace engine {

// Load SDL_GameControllerDB mappings (gamecontrollerdb.txt) into GLFW. Needs
// glfwInit. Returns false when the file is missing or GLFW rejects it.
bool loadGlfwGamepadMappings(const char* path);

// Every GLFW joystick slot into `pads`: a mapped gamepad's state, or cleared.
void pollGlfwGamepads(GamepadSet& pads);

// The device name GLFW reports for a slot ("" when none is present).
std::string glfwGamepadName(int jid);

// For a host with no GLFW window (the Qt editor): glfwInit with no window,
// plus the mappings. False if GLFW cannot start; the host then has no pads.
bool initWindowlessGlfwGamepads(const char* mappingsPath);
// Hot-plug: GLFW notices a pad arriving in its event pump, so a windowless host
// calls this (glfwPollEvents) before pollGlfwGamepads each frame.
void pumpWindowlessGlfw();
void shutdownWindowlessGlfwGamepads();

}  // namespace engine

#endif
