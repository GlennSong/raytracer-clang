#pragma once

// The debug UI's (ImGui's) clipboard, host by host.
//
// ImGui's own fallback clipboard is a PRIVATE buffer: a Copy button "works",
// the read-back says "same", and nothing ever reaches the OS. The GLFW
// backend bridges to the OS, but on KDE Wayland the compositor honours a
// client's set_selection only while it holds keyboard focus and never says
// when it refuses. The Qt-hosted window had no bridge at all. So every host
// installs an OS-backed clipboard explicitly and NAMES it in the log, and
// the Copy buttons say which one they went through.
namespace engine {

// Linux under Wayland with wl-copy/wl-paste on PATH: route ImGui's clipboard
// through them (the data-control protocol clipboard managers use — no
// keyboard-focus rule). Returns false (nothing installed) elsewhere.
bool installWaylandToolClipboard();

// What the debug UI's clipboard currently goes through ("wl-copy/wl-paste",
// "GLFW", "Qt", "ImGui private buffer") — for the log lines.
void setDebugUiClipboardName(const char* name);
const char* debugUiClipboardName();

}  // namespace engine
