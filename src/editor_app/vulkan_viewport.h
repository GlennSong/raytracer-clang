#ifndef RAYTRACER_EDITOR_VULKAN_VIEWPORT_H
#define RAYTRACER_EDITOR_VULKAN_VIEWPORT_H

// Vulkan surface creation for the Qt editor viewport (ADR-0057, editor-app
// plan). The Qt host (editor_main.cpp) owns the toolkit and extracts the
// platform-native handles; this translation unit owns the Vulkan + platform
// surface headers and turns those handles into a VkSurfaceKHR. The two are
// kept apart on purpose: Xlib's headers pollute the global namespace with
// macros (None/Status/Bool) that collide with Qt, so Qt and Xlib never share
// a TU. Built only on non-Apple targets where Vulkan is found.

#include <cstdint>
#include <string>
#include <vector>

namespace editor {

// Which native surface path the host resolved from the Qt platform plugin.
enum class VkPlatform { Win32, Xlib, Unsupported };

// Instance extensions the chosen platform surface needs (VK_KHR_surface plus
// the platform extension). Empty for Unsupported.
std::vector<std::string> vulkanInstanceExtensions(VkPlatform platform);

// Create a VkSurfaceKHR for the viewport. `instance` is an opaque VkInstance.
// For Win32 `display` is ignored (the module handle is fetched here) and
// `windowId` is the HWND; for Xlib `display` is the X11 Display* and `windowId`
// is the X11 Window XID. Returns false (and logs) on any failure or for an
// Unsupported platform. The created handle is written to `outSurface` as a
// uint64 (VkSurfaceKHR is always 64-bit).
bool createVulkanSurface(void* instance, VkPlatform platform, void* display,
                         unsigned long long windowId, uint64_t* outSurface);

}  // namespace editor

#endif
