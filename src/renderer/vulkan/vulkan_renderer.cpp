// Vulkan backend — Phase 0 (ADR-0057, docs/vulkan-renderer-plan.md):
// instance/device/swapchain bring-up and a cleared frame. No shaders or draws
// yet — those land in Phase 1+. All Vulkan state is confined to this file
// (Impl); the rest of the engine sees only the Renderer seam.
//
// This backend targets Linux and Windows. It is written against the Vulkan 1.0
// core spec; surface creation and the required instance extensions come through
// the Window seam (Window::createVulkanSurface / requiredVulkanInstanceExtensions)
// so no GLFW symbol appears here.

#include "vulkan_renderer.h"

#include "../window.h"
#include "../../log.h"
#include "../../slot_map.h"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

namespace engine {

namespace {

constexpr int MAX_FRAMES_IN_FLIGHT = 2;

#if defined(NDEBUG)
constexpr bool kEnableValidation = false;
#else
constexpr bool kEnableValidation = true;
#endif

const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

// Per-mesh CPU-side record. Phase 0 keeps only what getMeshBounds needs; the GPU
// vertex/index buffers are created in Phase 1 when there is a pipeline to draw
// them with. Tracked in a SlotMap so handles stay stable and stale-safe.
struct GpuMesh {
    BoundingSphere bounds;
    uint32_t indexCount = 0;
};

bool hasValidationLayer() {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    for (const VkLayerProperties& l : layers) {
        if (std::strcmp(l.layerName, kValidationLayer) == 0) return true;
    }
    return false;
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT /*type*/,
    const VkDebugUtilsMessengerCallbackDataEXT* data, void* /*user*/) {
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT) {
        LOG_ERROR("[vulkan] %s", data->pMessage);
    } else if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT) {
        LOG_WARN("[vulkan] %s", data->pMessage);
    }
    return VK_FALSE;
}

}  // namespace

struct VulkanRenderer::Impl {
    Window* window = nullptr;
    int width = 0;
    int height = 0;

    VkInstance instance = VK_NULL_HANDLE;
    VkDebugUtilsMessengerEXT debugMessenger = VK_NULL_HANDLE;
    VkSurfaceKHR surface = VK_NULL_HANDLE;

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    uint32_t graphicsFamily = 0;
    uint32_t presentFamily = 0;
    VkDevice device = VK_NULL_HANDLE;
    VkQueue graphicsQueue = VK_NULL_HANDLE;
    VkQueue presentQueue = VK_NULL_HANDLE;

    VkSwapchainKHR swapchain = VK_NULL_HANDLE;
    VkFormat swapchainFormat = VK_FORMAT_UNDEFINED;
    VkExtent2D swapchainExtent{0, 0};
    std::vector<VkImage> swapchainImages;
    std::vector<VkImageView> swapchainImageViews;
    std::vector<VkFramebuffer> framebuffers;

    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    std::array<VkCommandBuffer, MAX_FRAMES_IN_FLIGHT> commandBuffers{};

    // Sync: image-available + in-flight fence are per frame-in-flight; the
    // render-finished semaphore is per swapchain image (signalled at submit,
    // waited at present) so it can't be reused while a present is still pending.
    std::array<VkSemaphore, MAX_FRAMES_IN_FLIGHT> imageAvailable{};
    std::array<VkFence, MAX_FRAMES_IN_FLIGHT> inFlightFences{};
    std::vector<VkSemaphore> renderFinished;     // one per swapchain image
    std::vector<VkFence> imagesInFlight;          // tracks the fence using an image
    uint32_t currentFrame = 0;

    bool framebufferResized = false;
    bool initialized = false;

    SlotMap<GpuMesh, MeshTag> meshes;
    SlotMap<uint8_t, TextureTag> textures;   // Phase 0 placeholder (no GPU upload)
    RenderStats stats;

    // ---- bring-up steps -------------------------------------------------
    bool createInstance();
    void setupDebugMessenger();
    bool createSurface();
    bool pickPhysicalDevice();
    bool createLogicalDevice();
    bool createSwapchain();
    bool createImageViews();
    bool createRenderPass();
    bool createFramebuffers();
    bool createCommandPool();
    bool createCommandBuffers();
    bool createSyncObjects();

    bool recreateSwapchain();
    void cleanupSwapchain();
    void recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex);
    void drawFrame();

    // helpers
    struct QueueFamilies {
        std::optional<uint32_t> graphics;
        std::optional<uint32_t> present;
        bool complete() const { return graphics.has_value() && present.has_value(); }
    };
    QueueFamilies findQueueFamilies(VkPhysicalDevice dev) const;
    bool deviceSupportsSwapchain(VkPhysicalDevice dev) const;
};

// ---------------------------------------------------------------------------

bool VulkanRenderer::Impl::createInstance() {
    if (!window) {
        LOG_ERROR("[vulkan] no window provided (setWindow not called)");
        return false;
    }

    VkApplicationInfo app{};
    app.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName = "raytracer-viewer";
    app.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    app.pEngineName = "raytracer-engine";
    app.engineVersion = VK_MAKE_VERSION(0, 1, 0);
    app.apiVersion = VK_API_VERSION_1_0;

    // Window-required surface extensions, plus debug utils when validating.
    std::vector<std::string> reqStr = window->requiredVulkanInstanceExtensions();
    if (reqStr.empty()) {
        LOG_ERROR("[vulkan] window reported no required instance extensions");
        return false;
    }
    std::vector<const char*> extensions;
    extensions.reserve(reqStr.size() + 1);
    for (const std::string& e : reqStr) extensions.push_back(e.c_str());

    const bool validate = kEnableValidation && hasValidationLayer();
    if (validate) extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
    if (kEnableValidation && !validate) {
        LOG_WARN("[vulkan] validation layer requested but unavailable; continuing without it");
    }

    VkInstanceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    info.pApplicationInfo = &app;
    info.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
    info.ppEnabledExtensionNames = extensions.data();
    if (validate) {
        info.enabledLayerCount = 1;
        info.ppEnabledLayerNames = &kValidationLayer;
    }

    if (vkCreateInstance(&info, nullptr, &instance) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateInstance failed");
        return false;
    }
    if (validate) setupDebugMessenger();
    return true;
}

void VulkanRenderer::Impl::setupDebugMessenger() {
    auto create = reinterpret_cast<PFN_vkCreateDebugUtilsMessengerEXT>(
        vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT"));
    if (!create) return;

    VkDebugUtilsMessengerCreateInfoEXT info{};
    info.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    info.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT |
                           VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    info.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT |
                       VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT |
                       VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    info.pfnUserCallback = debugCallback;
    create(instance, &info, nullptr, &debugMessenger);
}

bool VulkanRenderer::Impl::createSurface() {
    uint64_t handle = 0;
    if (!window->createVulkanSurface(reinterpret_cast<void*>(instance), &handle)) {
        LOG_ERROR("[vulkan] Window::createVulkanSurface failed");
        return false;
    }
    surface = reinterpret_cast<VkSurfaceKHR>(handle);
    return true;
}

VulkanRenderer::Impl::QueueFamilies
VulkanRenderer::Impl::findQueueFamilies(VkPhysicalDevice dev) const {
    QueueFamilies fam;
    uint32_t count = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(dev, &count, nullptr);
    std::vector<VkQueueFamilyProperties> props(count);
    vkGetPhysicalDeviceQueueFamilyProperties(dev, &count, props.data());
    for (uint32_t i = 0; i < count; ++i) {
        if (props[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) fam.graphics = i;
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface, &present);
        if (present) fam.present = i;
        if (fam.complete()) break;
    }
    return fam;
}

bool VulkanRenderer::Impl::deviceSupportsSwapchain(VkPhysicalDevice dev) const {
    uint32_t count = 0;
    vkEnumerateDeviceExtensionProperties(dev, nullptr, &count, nullptr);
    std::vector<VkExtensionProperties> exts(count);
    vkEnumerateDeviceExtensionProperties(dev, nullptr, &count, exts.data());
    bool hasSwapchain = false;
    for (const VkExtensionProperties& e : exts) {
        if (std::strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0)
            hasSwapchain = true;
    }
    if (!hasSwapchain) return false;

    uint32_t formats = 0, modes = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(dev, surface, &formats, nullptr);
    vkGetPhysicalDeviceSurfacePresentModesKHR(dev, surface, &modes, nullptr);
    return formats > 0 && modes > 0;
}

bool VulkanRenderer::Impl::pickPhysicalDevice() {
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance, &count, nullptr);
    if (count == 0) {
        LOG_ERROR("[vulkan] no GPUs with Vulkan support");
        return false;
    }
    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance, &count, devices.data());

    // Prefer a discrete GPU; otherwise take the first device that has a graphics
    // queue, present support, and the swapchain extension.
    VkPhysicalDevice fallback = VK_NULL_HANDLE;
    QueueFamilies fallbackFam;
    for (VkPhysicalDevice dev : devices) {
        QueueFamilies fam = findQueueFamilies(dev);
        if (!fam.complete() || !deviceSupportsSwapchain(dev)) continue;

        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(dev, &props);
        if (props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
            physicalDevice = dev;
            graphicsFamily = *fam.graphics;
            presentFamily = *fam.present;
            break;
        }
        if (fallback == VK_NULL_HANDLE) {
            fallback = dev;
            fallbackFam = fam;
        }
    }
    if (physicalDevice == VK_NULL_HANDLE && fallback != VK_NULL_HANDLE) {
        physicalDevice = fallback;
        graphicsFamily = *fallbackFam.graphics;
        presentFamily = *fallbackFam.present;
    }
    if (physicalDevice == VK_NULL_HANDLE) {
        LOG_ERROR("[vulkan] no suitable GPU (needs graphics + present + swapchain)");
        return false;
    }
    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physicalDevice, &props);
    LOG_INFO("[vulkan] using GPU: %s", props.deviceName);
    return true;
}

bool VulkanRenderer::Impl::createLogicalDevice() {
    std::set<uint32_t> uniqueFamilies{graphicsFamily, presentFamily};
    std::vector<VkDeviceQueueCreateInfo> queueInfos;
    float priority = 1.0f;
    for (uint32_t family : uniqueFamilies) {
        VkDeviceQueueCreateInfo q{};
        q.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        q.queueFamilyIndex = family;
        q.queueCount = 1;
        q.pQueuePriorities = &priority;
        queueInfos.push_back(q);
    }

    const char* deviceExt = VK_KHR_SWAPCHAIN_EXTENSION_NAME;
    VkPhysicalDeviceFeatures features{};

    VkDeviceCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    info.queueCreateInfoCount = static_cast<uint32_t>(queueInfos.size());
    info.pQueueCreateInfos = queueInfos.data();
    info.enabledExtensionCount = 1;
    info.ppEnabledExtensionNames = &deviceExt;
    info.pEnabledFeatures = &features;

    if (vkCreateDevice(physicalDevice, &info, nullptr, &device) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateDevice failed");
        return false;
    }
    vkGetDeviceQueue(device, graphicsFamily, 0, &graphicsQueue);
    vkGetDeviceQueue(device, presentFamily, 0, &presentQueue);
    return true;
}

bool VulkanRenderer::Impl::createSwapchain() {
    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &caps);

    uint32_t formatCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &formatCount, formats.data());

    VkSurfaceFormatKHR chosen = formats[0];
    for (const VkSurfaceFormatKHR& f : formats) {
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosen = f;
            break;
        }
    }

    // FIFO is always available (v-sync); good enough for Phase 0.
    VkPresentModeKHR present = VK_PRESENT_MODE_FIFO_KHR;

    VkExtent2D extent;
    if (caps.currentExtent.width != std::numeric_limits<uint32_t>::max()) {
        extent = caps.currentExtent;
    } else {
        extent.width = std::clamp(static_cast<uint32_t>(width),
                                  caps.minImageExtent.width, caps.maxImageExtent.width);
        extent.height = std::clamp(static_cast<uint32_t>(height),
                                   caps.minImageExtent.height, caps.maxImageExtent.height);
    }

    uint32_t imageCount = caps.minImageCount + 1;
    if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount)
        imageCount = caps.maxImageCount;

    VkSwapchainCreateInfoKHR info{};
    info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    info.surface = surface;
    info.minImageCount = imageCount;
    info.imageFormat = chosen.format;
    info.imageColorSpace = chosen.colorSpace;
    info.imageExtent = extent;
    info.imageArrayLayers = 1;
    info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    uint32_t families[2] = {graphicsFamily, presentFamily};
    if (graphicsFamily != presentFamily) {
        info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        info.queueFamilyIndexCount = 2;
        info.pQueueFamilyIndices = families;
    } else {
        info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }
    info.preTransform = caps.currentTransform;
    info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    info.presentMode = present;
    info.clipped = VK_TRUE;
    info.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(device, &info, nullptr, &swapchain) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateSwapchainKHR failed");
        return false;
    }

    vkGetSwapchainImagesKHR(device, swapchain, &imageCount, nullptr);
    swapchainImages.resize(imageCount);
    vkGetSwapchainImagesKHR(device, swapchain, &imageCount, swapchainImages.data());
    swapchainFormat = chosen.format;
    swapchainExtent = extent;
    return true;
}

bool VulkanRenderer::Impl::createImageViews() {
    swapchainImageViews.resize(swapchainImages.size());
    for (size_t i = 0; i < swapchainImages.size(); ++i) {
        VkImageViewCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        info.image = swapchainImages[i];
        info.viewType = VK_IMAGE_VIEW_TYPE_2D;
        info.format = swapchainFormat;
        info.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        info.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        info.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        info.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        info.subresourceRange.levelCount = 1;
        info.subresourceRange.layerCount = 1;
        if (vkCreateImageView(device, &info, nullptr, &swapchainImageViews[i]) != VK_SUCCESS) {
            LOG_ERROR("[vulkan] vkCreateImageView failed");
            return false;
        }
    }
    return true;
}

bool VulkanRenderer::Impl::createRenderPass() {
    VkAttachmentDescription color{};
    color.format = swapchainFormat;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorRef{};
    colorRef.attachment = 0;
    colorRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dep.srcAccessMask = 0;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkRenderPassCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    info.attachmentCount = 1;
    info.pAttachments = &color;
    info.subpassCount = 1;
    info.pSubpasses = &subpass;
    info.dependencyCount = 1;
    info.pDependencies = &dep;

    if (vkCreateRenderPass(device, &info, nullptr, &renderPass) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateRenderPass failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createFramebuffers() {
    framebuffers.resize(swapchainImageViews.size());
    for (size_t i = 0; i < swapchainImageViews.size(); ++i) {
        VkImageView attachments[] = {swapchainImageViews[i]};
        VkFramebufferCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        info.renderPass = renderPass;
        info.attachmentCount = 1;
        info.pAttachments = attachments;
        info.width = swapchainExtent.width;
        info.height = swapchainExtent.height;
        info.layers = 1;
        if (vkCreateFramebuffer(device, &info, nullptr, &framebuffers[i]) != VK_SUCCESS) {
            LOG_ERROR("[vulkan] vkCreateFramebuffer failed");
            return false;
        }
    }
    return true;
}

bool VulkanRenderer::Impl::createCommandPool() {
    VkCommandPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    info.queueFamilyIndex = graphicsFamily;
    if (vkCreateCommandPool(device, &info, nullptr, &commandPool) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateCommandPool failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createCommandBuffers() {
    VkCommandBufferAllocateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    info.commandPool = commandPool;
    info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    info.commandBufferCount = MAX_FRAMES_IN_FLIGHT;
    if (vkAllocateCommandBuffers(device, &info, commandBuffers.data()) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkAllocateCommandBuffers failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createSyncObjects() {
    renderFinished.resize(swapchainImages.size());
    imagesInFlight.assign(swapchainImages.size(), VK_NULL_HANDLE);

    VkSemaphoreCreateInfo sem{};
    sem.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    VkFenceCreateInfo fence{};
    fence.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fence.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        if (vkCreateSemaphore(device, &sem, nullptr, &imageAvailable[i]) != VK_SUCCESS ||
            vkCreateFence(device, &fence, nullptr, &inFlightFences[i]) != VK_SUCCESS) {
            LOG_ERROR("[vulkan] failed to create per-frame sync objects");
            return false;
        }
    }
    for (size_t i = 0; i < renderFinished.size(); ++i) {
        if (vkCreateSemaphore(device, &sem, nullptr, &renderFinished[i]) != VK_SUCCESS) {
            LOG_ERROR("[vulkan] failed to create render-finished semaphore");
            return false;
        }
    }
    return true;
}

void VulkanRenderer::Impl::cleanupSwapchain() {
    for (VkFramebuffer fb : framebuffers)
        if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    framebuffers.clear();
    for (VkImageView view : swapchainImageViews)
        if (view) vkDestroyImageView(device, view, nullptr);
    swapchainImageViews.clear();
    if (swapchain) {
        vkDestroySwapchainKHR(device, swapchain, nullptr);
        swapchain = VK_NULL_HANDLE;
    }
}

bool VulkanRenderer::Impl::recreateSwapchain() {
    // Pull the latest framebuffer size; skip while minimized (0-area).
    if (window) window->getFramebufferSize(width, height);
    if (width == 0 || height == 0) return true;  // stay idle until restored

    vkDeviceWaitIdle(device);

    // Render-finished semaphores are sized to the image count, which can change;
    // rebuild them with the rest of the swapchain-dependent objects.
    for (VkSemaphore s : renderFinished)
        if (s) vkDestroySemaphore(device, s, nullptr);
    renderFinished.clear();

    cleanupSwapchain();

    bool ok = createSwapchain() && createImageViews() && createFramebuffers();
    if (!ok) return false;

    renderFinished.resize(swapchainImages.size());
    imagesInFlight.assign(swapchainImages.size(), VK_NULL_HANDLE);
    VkSemaphoreCreateInfo sem{};
    sem.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    for (size_t i = 0; i < renderFinished.size(); ++i) {
        if (vkCreateSemaphore(device, &sem, nullptr, &renderFinished[i]) != VK_SUCCESS)
            return false;
    }
    framebufferResized = false;
    return true;
}

void VulkanRenderer::Impl::recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex) {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    vkBeginCommandBuffer(cmd, &begin);

    // Phase 0: clear to a calm blue-grey so a successful frame is visible. The
    // geometry + post pass graph (mirroring the Metal backend) lands in Phase 1+.
    VkClearValue clear{};
    clear.color = {{0.05f, 0.06f, 0.08f, 1.0f}};

    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = renderPass;
    rp.framebuffer = framebuffers[imageIndex];
    rp.renderArea.offset = {0, 0};
    rp.renderArea.extent = swapchainExtent;
    rp.clearValueCount = 1;
    rp.pClearValues = &clear;

    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdEndRenderPass(cmd);
    vkEndCommandBuffer(cmd);
}

void VulkanRenderer::Impl::drawFrame() {
    if (!initialized || width == 0 || height == 0) return;

    vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex = 0;
    VkResult acquire = vkAcquireNextImageKHR(
        device, swapchain, UINT64_MAX, imageAvailable[currentFrame],
        VK_NULL_HANDLE, &imageIndex);
    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) {
        recreateSwapchain();
        return;
    }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
        LOG_ERROR("[vulkan] vkAcquireNextImageKHR failed (%d)", static_cast<int>(acquire));
        return;
    }

    // If a previous frame is still using this image, wait on its fence.
    if (imagesInFlight[imageIndex] != VK_NULL_HANDLE)
        vkWaitForFences(device, 1, &imagesInFlight[imageIndex], VK_TRUE, UINT64_MAX);
    imagesInFlight[imageIndex] = inFlightFences[currentFrame];

    VkCommandBuffer cmd = commandBuffers[currentFrame];
    vkResetCommandBuffer(cmd, 0);
    recordCommandBuffer(cmd, imageIndex);

    VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.waitSemaphoreCount = 1;
    submit.pWaitSemaphores = &imageAvailable[currentFrame];
    submit.pWaitDstStageMask = &waitStage;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    submit.signalSemaphoreCount = 1;
    submit.pSignalSemaphores = &renderFinished[imageIndex];

    vkResetFences(device, 1, &inFlightFences[currentFrame]);
    if (vkQueueSubmit(graphicsQueue, 1, &submit, inFlightFences[currentFrame]) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkQueueSubmit failed");
        return;
    }

    VkPresentInfoKHR present{};
    present.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    present.waitSemaphoreCount = 1;
    present.pWaitSemaphores = &renderFinished[imageIndex];
    present.swapchainCount = 1;
    present.pSwapchains = &swapchain;
    present.pImageIndices = &imageIndex;

    VkResult result = vkQueuePresentKHR(presentQueue, &present);
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR ||
        framebufferResized) {
        recreateSwapchain();
    } else if (result != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkQueuePresentKHR failed (%d)", static_cast<int>(result));
    }

    currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

// ---------------------------------------------------------------------------
// Renderer interface

VulkanRenderer::VulkanRenderer() : impl(std::make_unique<Impl>()) {}
VulkanRenderer::~VulkanRenderer() { shutdown(); }

void VulkanRenderer::setWindow(Window* window) { impl->window = window; }

bool VulkanRenderer::initialize(void* /*windowHandle*/, int width, int height) {
    impl->width = width;
    impl->height = height;
    bool ok = impl->createInstance() &&
              impl->createSurface() &&
              impl->pickPhysicalDevice() &&
              impl->createLogicalDevice() &&
              impl->createSwapchain() &&
              impl->createImageViews() &&
              impl->createRenderPass() &&
              impl->createFramebuffers() &&
              impl->createCommandPool() &&
              impl->createCommandBuffers() &&
              impl->createSyncObjects();
    if (!ok) {
        LOG_ERROR("[vulkan] initialization failed");
        return false;
    }
    impl->initialized = true;
    LOG_INFO("[vulkan] backend initialized (%dx%d, Phase 0: cleared frame)", width, height);
    return true;
}

void VulkanRenderer::shutdown() {
    if (!impl || impl->device == VK_NULL_HANDLE) {
        if (impl && impl->instance) {
            vkDestroyInstance(impl->instance, nullptr);
            impl->instance = VK_NULL_HANDLE;
        }
        return;
    }
    vkDeviceWaitIdle(impl->device);

    for (VkSemaphore s : impl->renderFinished)
        if (s) vkDestroySemaphore(impl->device, s, nullptr);
    impl->renderFinished.clear();
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        if (impl->imageAvailable[i]) vkDestroySemaphore(impl->device, impl->imageAvailable[i], nullptr);
        if (impl->inFlightFences[i]) vkDestroyFence(impl->device, impl->inFlightFences[i], nullptr);
        impl->imageAvailable[i] = VK_NULL_HANDLE;
        impl->inFlightFences[i] = VK_NULL_HANDLE;
    }

    impl->cleanupSwapchain();

    if (impl->commandPool) vkDestroyCommandPool(impl->device, impl->commandPool, nullptr);
    if (impl->renderPass) vkDestroyRenderPass(impl->device, impl->renderPass, nullptr);
    impl->commandPool = VK_NULL_HANDLE;
    impl->renderPass = VK_NULL_HANDLE;

    vkDestroyDevice(impl->device, nullptr);
    impl->device = VK_NULL_HANDLE;

    if (impl->debugMessenger) {
        auto destroy = reinterpret_cast<PFN_vkDestroyDebugUtilsMessengerEXT>(
            vkGetInstanceProcAddr(impl->instance, "vkDestroyDebugUtilsMessengerEXT"));
        if (destroy) destroy(impl->instance, impl->debugMessenger, nullptr);
        impl->debugMessenger = VK_NULL_HANDLE;
    }
    if (impl->surface) vkDestroySurfaceKHR(impl->instance, impl->surface, nullptr);
    if (impl->instance) vkDestroyInstance(impl->instance, nullptr);
    impl->surface = VK_NULL_HANDLE;
    impl->instance = VK_NULL_HANDLE;
    impl->initialized = false;
}

void VulkanRenderer::resize(int width, int height) {
    impl->width = width;
    impl->height = height;
    impl->framebufferResized = true;
}

MeshHandle VulkanRenderer::uploadMesh(const RenderMesh& mesh) {
    // Phase 0: track bounds + index count so getMeshBounds/culling are valid.
    // GPU vertex/index buffers are created in Phase 1.
    GpuMesh record;
    record.bounds = computeBoundingSphere(mesh.vertices.data(), mesh.vertices.size());
    record.indexCount = static_cast<uint32_t>(mesh.indices.size());
    return impl->meshes.insert(record);
}

void VulkanRenderer::removeMesh(MeshHandle handle) { impl->meshes.erase(handle); }

BoundingSphere VulkanRenderer::getMeshBounds(MeshHandle handle) const {
    const GpuMesh* m = impl->meshes.get(handle);
    return m ? m->bounds : BoundingSphere{};
}

TextureHandle VulkanRenderer::uploadTexture(int /*width*/, int /*height*/,
                                            int /*channels*/, const uint8_t* /*data*/) {
    // Phase 0: no GPU upload yet; hand back a valid handle so the asset/material
    // layer keeps working until the texture path lands in Phase 2.
    return impl->textures.insert(0);
}

void VulkanRenderer::removeTexture(TextureHandle handle) { impl->textures.erase(handle); }

RenderStats VulkanRenderer::getRenderStats() const { return impl->stats; }

void VulkanRenderer::beginFrame() { impl->stats = RenderStats{}; }

void VulkanRenderer::setCamera(const CameraState& /*camera*/) {}
void VulkanRenderer::setLights(const SceneLighting& /*lighting*/) {}
void VulkanRenderer::drawMesh(MeshHandle /*handle*/, const Mat4& /*transform*/,
                              const RenderMaterial& /*material*/) {}

void VulkanRenderer::endFrame() { impl->drawFrame(); }

// The non-Apple factory. The Metal backend defines this on Apple; NullRenderer
// defines it when no GPU backend is compiled in. Exactly one is linked.
std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<VulkanRenderer>();
}

}  // namespace engine
