#include "vulkan_viewport.h"

#include "../log.h"

// Pull in only the platform surface extension matching the build host. Each
// define makes <vulkan/vulkan.h> include the platform's window-system header
// and declare vkCreate*SurfaceKHR. We never enable more than one here, so the
// Xlib macro pollution stays contained to the Xlib build.
#if defined(_WIN32)
#define VK_USE_PLATFORM_WIN32_KHR
#include <windows.h>
#include <vulkan/vulkan.h>
#else
#define VK_USE_PLATFORM_XLIB_KHR
#include <vulkan/vulkan.h>
#endif

namespace editor {

std::vector<std::string> vulkanInstanceExtensions(VkPlatform platform) {
    switch (platform) {
#if defined(_WIN32)
        case VkPlatform::Win32:
            return {VK_KHR_SURFACE_EXTENSION_NAME,
                    VK_KHR_WIN32_SURFACE_EXTENSION_NAME};
#else
        case VkPlatform::Xlib:
            return {VK_KHR_SURFACE_EXTENSION_NAME,
                    VK_KHR_XLIB_SURFACE_EXTENSION_NAME};
#endif
        default:
            return {};
    }
}

bool createVulkanSurface(void* instance, VkPlatform platform, void* display,
                         unsigned long long windowId, uint64_t* outSurface) {
    auto vkInstance = reinterpret_cast<VkInstance>(instance);
    VkSurfaceKHR surface = VK_NULL_HANDLE;

#if defined(_WIN32)
    (void)display;
    if (platform != VkPlatform::Win32) {
        LOG_ERROR << "[editor] unsupported Vulkan surface platform on Windows";
        return false;
    }
    VkWin32SurfaceCreateInfoKHR ci{};
    ci.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
    ci.hinstance = GetModuleHandle(nullptr);
    ci.hwnd = reinterpret_cast<HWND>(static_cast<uintptr_t>(windowId));
    if (vkCreateWin32SurfaceKHR(vkInstance, &ci, nullptr, &surface) !=
        VK_SUCCESS) {
        LOG_ERROR << "[editor] vkCreateWin32SurfaceKHR failed";
        return false;
    }
#else
    if (platform != VkPlatform::Xlib || display == nullptr) {
        LOG_ERROR << "[editor] no Xlib display for the Vulkan surface "
                     "(Wayland not supported yet — run with QT_QPA_PLATFORM=xcb)";
        return false;
    }
    VkXlibSurfaceCreateInfoKHR ci{};
    ci.sType = VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR;
    ci.dpy = reinterpret_cast<Display*>(display);
    ci.window = static_cast<Window>(windowId);
    if (vkCreateXlibSurfaceKHR(vkInstance, &ci, nullptr, &surface) !=
        VK_SUCCESS) {
        LOG_ERROR << "[editor] vkCreateXlibSurfaceKHR failed";
        return false;
    }
#endif

    // VkSurfaceKHR is a 64-bit non-dispatchable handle: a pointer on 64-bit
    // builds, a uint64_t otherwise. A C-style cast covers both forms (matching
    // how the backend casts it back in createSurface).
    *outSurface = (uint64_t)surface;
    return true;
}

}  // namespace editor
