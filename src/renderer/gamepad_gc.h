#ifndef RAYTRACER_GAMEPAD_GC_H
#define RAYTRACER_GAMEPAD_GC_H

#include "gamepad.h"

namespace engine {

// Apple Game Controller framework polling (macOS). On modern macOS the system's
// DriverKit driver claims Xbox/PS controllers at the USB level and re-presents
// them with a vendor-specific HID descriptor that IOKit-based libraries (GLFW)
// cannot parse. GCController speaks the vendor protocol and is the only
// reliable path on macOS 13+.
//
// Usage: call gcPollGamepads() each frame *after* glfwPollEvents. For every
// slot where GCController reports a connected pad, it overwrites the
// GamepadState (filling axes/buttons that GLFW's IOKit path left at zero). GLFW
// still provides the joystick-present enumeration; this layer provides the
// actual input data.

#ifdef __APPLE__
void gcPollGamepads(GamepadSet& pads);
#else
inline void gcPollGamepads(GamepadSet&) {}   // no-op on other platforms
#endif

}  // namespace engine

#endif
