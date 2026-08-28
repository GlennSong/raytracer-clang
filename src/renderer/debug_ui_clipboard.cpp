#include "debug_ui_clipboard.h"
#include "../log.h"

#include <string>

#ifdef RT_ENABLE_IMGUI
#include "imgui.h"
#endif
#if defined(__linux__)
#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#endif

namespace engine {

namespace {
std::string g_clipboardName = "ImGui private buffer";
#if defined(__linux__) && defined(RT_ENABLE_IMGUI)
std::string g_wlPasteBuffer;

bool wlToolsAvailable() {
    if (!std::getenv("WAYLAND_DISPLAY")) return false;
    return access("/usr/bin/wl-copy", X_OK) == 0 && access("/usr/bin/wl-paste", X_OK) == 0;
}

void wlSetClipboardText(ImGuiContext*, const char* text) {
    // wl-copy forks a child that serves the selection and returns at once.
    FILE* p = popen("wl-copy", "w");
    if (!p) { LOG_WARN << "clipboard: wl-copy could not start"; return; }
    std::fputs(text ? text : "", p);
    if (const int rc = pclose(p); rc != 0) LOG_WARN << "clipboard: wl-copy exited " << rc;
}

const char* wlGetClipboardText(ImGuiContext*) {
    g_wlPasteBuffer.clear();
    FILE* p = popen("wl-paste --no-newline 2>/dev/null", "r");
    if (!p) return nullptr;
    char buf[4096];
    std::size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), p)) > 0) g_wlPasteBuffer.append(buf, n);
    pclose(p);
    return g_wlPasteBuffer.c_str();
}
#endif
}  // namespace

bool installWaylandToolClipboard() {
#if defined(__linux__) && defined(RT_ENABLE_IMGUI)
    if (ImGui::GetCurrentContext() == nullptr || !wlToolsAvailable()) return false;
    ImGuiPlatformIO& pio = ImGui::GetPlatformIO();
    pio.Platform_SetClipboardTextFn = wlSetClipboardText;
    pio.Platform_GetClipboardTextFn = wlGetClipboardText;
    setDebugUiClipboardName("wl-copy/wl-paste");
    return true;
#else
    return false;
#endif
}

void setDebugUiClipboardName(const char* name) {
    g_clipboardName = name ? name : "?";
    LOG_INFO << "Debug UI clipboard: " << g_clipboardName;
}

const char* debugUiClipboardName() { return g_clipboardName.c_str(); }

}  // namespace engine
