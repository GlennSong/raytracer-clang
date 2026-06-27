// Vulkan backend (ADR-0057, docs/vulkan-renderer-plan.md). Targets Linux and
// Windows. All Vulkan state is confined to this file (Impl); the rest of the
// engine sees only the Renderer seam. Surface creation and the required instance
// extensions come through the Window seam so no GLFW symbol appears here.
//
// Phase 0: instance/device/swapchain bring-up + a cleared frame.
// Phase 1: vertex/index buffer upload, a global UBO + per-frame descriptor set,
//          a forward graphics pipeline (SPIR-V loaded from RT_VULKAN_SHADER_DIR),
//          and lit single-mesh draws with push-constant model+material.
// Later phases add the full forward pass, shadows, IBL, and the post stack.

#include "vulkan_renderer.h"

#include "../window.h"
#include "../../log.h"
#include "../../slot_map.h"

#include <vulkan/vulkan.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <vector>

#ifndef RT_VULKAN_SHADER_DIR
#define RT_VULKAN_SHADER_DIR "shaders/vulkan"
#endif

namespace engine {

namespace {

constexpr int MAX_FRAMES_IN_FLIGHT = 2;
constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;

#if defined(NDEBUG)
constexpr bool kEnableValidation = false;
#else
constexpr bool kEnableValidation = true;
#endif

const char* kValidationLayer = "VK_LAYER_KHRONOS_validation";

// Packed float vertex uploaded to the GPU. The engine Vertex uses Real (double)
// Vec3s; the GPU wants tightly packed floats, so uploadMesh converts.
struct GpuVertex {
    float position[3];
    float normal[3];
    float tangent[3];
    float texcoord[2];
    float color[3];
};

// std140-compatible global uniforms (set 0, binding 0). Mirrors the Globals
// block in shaders/vulkan/mesh.{vert,frag}: mat4 + 4 vec4 = 128 bytes.
struct GlobalsUBO {
    float viewProjection[16];
    float cameraPosition[4];
    float sunDirection[4];
    float sunColor[4];
    float ambient[4];
};

// Per-draw push constants. Mirrors the Push block in the shaders (<=128 bytes).
struct MeshPush {
    float model[16];
    float albedoMetallic[4];   // rgb albedo, a metallic
    float emissionRough[4];    // rgb emission, a roughness
};

struct GpuMesh {
    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory = VK_NULL_HANDLE;
    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory = VK_NULL_HANDLE;
    uint32_t indexCount = 0;
    BoundingSphere bounds;
};

struct DrawItem {
    MeshHandle mesh;
    MeshPush push;
};

bool hasValidationLayer() {
    uint32_t count = 0;
    vkEnumerateInstanceLayerProperties(&count, nullptr);
    std::vector<VkLayerProperties> layers(count);
    vkEnumerateInstanceLayerProperties(&count, layers.data());
    for (const VkLayerProperties& l : layers)
        if (std::strcmp(l.layerName, kValidationLayer) == 0) return true;
    return false;
}

VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT, const VkDebugUtilsMessengerCallbackDataEXT* data, void*) {
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
        LOG_ERROR("[vulkan] %s", data->pMessage);
    else if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        LOG_WARN("[vulkan] %s", data->pMessage);
    return VK_FALSE;
}

std::vector<char> readFile(const std::string& path) {
    std::ifstream f(path, std::ios::ate | std::ios::binary);
    if (!f) return {};
    size_t size = static_cast<size_t>(f.tellg());
    std::vector<char> buf(size);
    f.seekg(0);
    f.read(buf.data(), static_cast<std::streamsize>(size));
    return buf;
}

// Pack a row-major engine Mat4 (M*v convention) into a column-major float[16] for
// GLSL, matching the Metal backend's toSimd transpose. flipY negates clip-space
// row 1, applying the Vulkan Y-flip once at upload so the shaders stay clean.
void packMat4(const Mat4& m, float* out, bool flipY) {
    for (int col = 0; col < 4; ++col)
        for (int row = 0; row < 4; ++row)
            out[col * 4 + row] = static_cast<float>(m.m[row][col]);
    if (flipY)
        for (int col = 0; col < 4; ++col) out[col * 4 + 1] = -out[col * 4 + 1];
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
    VkPhysicalDeviceMemoryProperties memProps{};
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

    VkImage depthImage = VK_NULL_HANDLE;
    VkDeviceMemory depthMemory = VK_NULL_HANDLE;
    VkImageView depthView = VK_NULL_HANDLE;

    VkRenderPass renderPass = VK_NULL_HANDLE;
    VkCommandPool commandPool = VK_NULL_HANDLE;
    std::array<VkCommandBuffer, MAX_FRAMES_IN_FLIGHT> commandBuffers{};

    std::array<VkSemaphore, MAX_FRAMES_IN_FLIGHT> imageAvailable{};
    std::array<VkFence, MAX_FRAMES_IN_FLIGHT> inFlightFences{};
    std::vector<VkSemaphore> renderFinished;     // one per swapchain image
    std::vector<VkFence> imagesInFlight;
    uint32_t currentFrame = 0;

    // Phase 1 pipeline + descriptors.
    VkDescriptorSetLayout descriptorSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool descriptorPool = VK_NULL_HANDLE;
    std::array<VkDescriptorSet, MAX_FRAMES_IN_FLIGHT> descriptorSets{};
    std::array<VkBuffer, MAX_FRAMES_IN_FLIGHT> globalsBuffers{};
    std::array<VkDeviceMemory, MAX_FRAMES_IN_FLIGHT> globalsMemory{};
    std::array<void*, MAX_FRAMES_IN_FLIGHT> globalsMapped{};
    VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
    VkPipeline meshPipeline = VK_NULL_HANDLE;

    bool framebufferResized = false;
    bool initialized = false;

    SlotMap<GpuMesh, MeshTag> meshes;
    SlotMap<uint8_t, TextureTag> textures;   // Phase 2 will give these real GPU data
    RenderStats stats;

    GlobalsUBO cpuGlobals{};
    std::vector<DrawItem> drawQueue;

    // ---- bring-up ----
    bool createInstance();
    void setupDebugMessenger();
    bool createSurface();
    bool pickPhysicalDevice();
    bool createLogicalDevice();
    bool createSwapchain();
    bool createImageViews();
    bool createDepthResources();
    bool createRenderPass();
    bool createFramebuffers();
    bool createCommandPool();
    bool createCommandBuffers();
    bool createSyncObjects();
    bool createDescriptorSetLayout();
    bool createGlobalsBuffers();
    bool createDescriptorPool();
    bool createDescriptorSets();
    bool createPipeline();

    bool recreateSwapchain();
    void cleanupSwapchain();
    void recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex);
    void drawFrame();

    // ---- helpers ----
    struct QueueFamilies {
        std::optional<uint32_t> graphics;
        std::optional<uint32_t> present;
        bool complete() const { return graphics.has_value() && present.has_value(); }
    };
    QueueFamilies findQueueFamilies(VkPhysicalDevice dev) const;
    bool deviceSupportsSwapchain(VkPhysicalDevice dev) const;
    uint32_t findMemoryType(uint32_t typeBits, VkMemoryPropertyFlags props) const;
    bool createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags props, VkBuffer& buffer, VkDeviceMemory& memory);
    bool createDeviceLocalBuffer(const void* data, VkDeviceSize size,
                                 VkBufferUsageFlags usage, VkBuffer& buffer, VkDeviceMemory& memory);
    VkShaderModule loadShaderModule(const std::string& path);
    void destroyMesh(GpuMesh& m);
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
    if (kEnableValidation && !validate)
        LOG_WARN("[vulkan] validation layer requested but unavailable; continuing without it");

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
    for (const VkExtensionProperties& e : exts)
        if (std::strcmp(e.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0) hasSwapchain = true;
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
        if (fallback == VK_NULL_HANDLE) { fallback = dev; fallbackFam = fam; }
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
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProps);
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
    for (const VkSurfaceFormatKHR& f : formats)
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { chosen = f; break; }

    VkPresentModeKHR present = VK_PRESENT_MODE_FIFO_KHR;  // always available, v-sync

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
    if (caps.maxImageCount > 0 && imageCount > caps.maxImageCount) imageCount = caps.maxImageCount;

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

bool VulkanRenderer::Impl::createDepthResources() {
    VkImageCreateInfo image{};
    image.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image.imageType = VK_IMAGE_TYPE_2D;
    image.extent = {swapchainExtent.width, swapchainExtent.height, 1};
    image.mipLevels = 1;
    image.arrayLayers = 1;
    image.format = kDepthFormat;
    image.tiling = VK_IMAGE_TILING_OPTIMAL;
    image.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    image.samples = VK_SAMPLE_COUNT_1_BIT;
    image.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &image, nullptr, &depthImage) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] depth vkCreateImage failed");
        return false;
    }
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device, depthImage, &req);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(device, &alloc, nullptr, &depthMemory) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] depth vkAllocateMemory failed");
        return false;
    }
    vkBindImageMemory(device, depthImage, depthMemory, 0);

    VkImageViewCreateInfo view{};
    view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view.image = depthImage;
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = kDepthFormat;
    view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    view.subresourceRange.levelCount = 1;
    view.subresourceRange.layerCount = 1;
    if (vkCreateImageView(device, &view, nullptr, &depthView) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] depth vkCreateImageView failed");
        return false;
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

    VkAttachmentDescription depth{};
    depth.format = kDepthFormat;
    depth.samples = VK_SAMPLE_COUNT_1_BIT;
    depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthRef{1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass = 0;
    dep.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                       VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                       VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = 0;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                        VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    std::array<VkAttachmentDescription, 2> attachments{color, depth};
    VkRenderPassCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    info.attachmentCount = static_cast<uint32_t>(attachments.size());
    info.pAttachments = attachments.data();
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
        std::array<VkImageView, 2> attachments{swapchainImageViews[i], depthView};
        VkFramebufferCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        info.renderPass = renderPass;
        info.attachmentCount = static_cast<uint32_t>(attachments.size());
        info.pAttachments = attachments.data();
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
    for (size_t i = 0; i < renderFinished.size(); ++i)
        if (vkCreateSemaphore(device, &sem, nullptr, &renderFinished[i]) != VK_SUCCESS) {
            LOG_ERROR("[vulkan] failed to create render-finished semaphore");
            return false;
        }
    return true;
}

bool VulkanRenderer::Impl::createDescriptorSetLayout() {
    VkDescriptorSetLayoutBinding binding{};
    binding.binding = 0;
    binding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    binding.descriptorCount = 1;
    binding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = 1;
    info.pBindings = &binding;
    if (vkCreateDescriptorSetLayout(device, &info, nullptr, &descriptorSetLayout) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateDescriptorSetLayout failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createGlobalsBuffers() {
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        if (!createBuffer(sizeof(GlobalsUBO), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                          globalsBuffers[i], globalsMemory[i]))
            return false;
        vkMapMemory(device, globalsMemory[i], 0, sizeof(GlobalsUBO), 0, &globalsMapped[i]);
    }
    return true;
}

bool VulkanRenderer::Impl::createDescriptorPool() {
    VkDescriptorPoolSize size{};
    size.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    size.descriptorCount = MAX_FRAMES_IN_FLIGHT;
    VkDescriptorPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    info.poolSizeCount = 1;
    info.pPoolSizes = &size;
    info.maxSets = MAX_FRAMES_IN_FLIGHT;
    if (vkCreateDescriptorPool(device, &info, nullptr, &descriptorPool) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateDescriptorPool failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createDescriptorSets() {
    std::array<VkDescriptorSetLayout, MAX_FRAMES_IN_FLIGHT> layouts;
    layouts.fill(descriptorSetLayout);
    VkDescriptorSetAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc.descriptorPool = descriptorPool;
    alloc.descriptorSetCount = MAX_FRAMES_IN_FLIGHT;
    alloc.pSetLayouts = layouts.data();
    if (vkAllocateDescriptorSets(device, &alloc, descriptorSets.data()) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkAllocateDescriptorSets failed");
        return false;
    }
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        VkDescriptorBufferInfo buf{globalsBuffers[i], 0, sizeof(GlobalsUBO)};
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = descriptorSets[i];
        write.dstBinding = 0;
        write.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        write.descriptorCount = 1;
        write.pBufferInfo = &buf;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }
    return true;
}

VkShaderModule VulkanRenderer::Impl::loadShaderModule(const std::string& path) {
    std::vector<char> code = readFile(path);
    if (code.empty()) {
        LOG_ERROR("[vulkan] could not read SPIR-V: %s", path.c_str());
        return VK_NULL_HANDLE;
    }
    VkShaderModuleCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    info.codeSize = code.size();
    info.pCode = reinterpret_cast<const uint32_t*>(code.data());
    VkShaderModule module = VK_NULL_HANDLE;
    if (vkCreateShaderModule(device, &info, nullptr, &module) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateShaderModule failed: %s", path.c_str());
        return VK_NULL_HANDLE;
    }
    return module;
}

bool VulkanRenderer::Impl::createPipeline() {
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(MeshPush);

    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = 1;
    layoutInfo.pSetLayouts = &descriptorSetLayout;
    layoutInfo.pushConstantRangeCount = 1;
    layoutInfo.pPushConstantRanges = &pushRange;
    if (vkCreatePipelineLayout(device, &layoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreatePipelineLayout failed");
        return false;
    }

    VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.vert.spv");
    VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.frag.spv");
    if (!vert || !frag) return false;

    VkPipelineShaderStageCreateInfo stages[2]{};
    stages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    stages[0].module = vert;
    stages[0].pName = "main";
    stages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    stages[1].module = frag;
    stages[1].pName = "main";

    VkVertexInputBindingDescription binding{};
    binding.binding = 0;
    binding.stride = sizeof(GpuVertex);
    binding.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

    std::array<VkVertexInputAttributeDescription, 5> attrs{};
    attrs[0] = {0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(GpuVertex, position)};
    attrs[1] = {1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(GpuVertex, normal)};
    attrs[2] = {2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(GpuVertex, tangent)};
    attrs[3] = {3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(GpuVertex, texcoord)};
    attrs[4] = {4, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(GpuVertex, color)};

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrs.size());
    vertexInput.pVertexAttributeDescriptions = attrs.data();

    VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
    inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo viewportState{};
    viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    viewportState.viewportCount = 1;
    viewportState.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    // Phase 1: cull nothing so the first mesh is visible regardless of winding.
    // Phase 2 enables VK_CULL_MODE_BACK_BIT once front-face winding is verified
    // on device (engine winds front faces clockwise; with the clip-space Y-flip
    // that should map to VK_FRONT_FACE_CLOCKWISE — confirm then enable).
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
    raster.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo multisample{};
    multisample.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    multisample.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo depthStencil{};
    depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    depthStencil.depthTestEnable = VK_TRUE;
    depthStencil.depthWriteEnable = VK_TRUE;
    depthStencil.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                     VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blendAttachment.blendEnable = VK_FALSE;
    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = 1;
    blend.pAttachments = &blendAttachment;

    std::array<VkDynamicState, 2> dynamics{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{};
    dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamic.dynamicStateCount = static_cast<uint32_t>(dynamics.size());
    dynamic.pDynamicStates = dynamics.data();

    VkGraphicsPipelineCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    info.stageCount = 2;
    info.pStages = stages;
    info.pVertexInputState = &vertexInput;
    info.pInputAssemblyState = &inputAssembly;
    info.pViewportState = &viewportState;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &multisample;
    info.pDepthStencilState = &depthStencil;
    info.pColorBlendState = &blend;
    info.pDynamicState = &dynamic;
    info.layout = pipelineLayout;
    info.renderPass = renderPass;
    info.subpass = 0;

    VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &meshPipeline);
    vkDestroyShaderModule(device, vert, nullptr);
    vkDestroyShaderModule(device, frag, nullptr);
    if (result != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateGraphicsPipelines failed");
        return false;
    }
    return true;
}

// ---- memory / buffer helpers ----

uint32_t VulkanRenderer::Impl::findMemoryType(uint32_t typeBits, VkMemoryPropertyFlags props) const {
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((typeBits & (1u << i)) &&
            (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    }
    LOG_ERROR("[vulkan] no suitable memory type");
    return 0;
}

bool VulkanRenderer::Impl::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                        VkMemoryPropertyFlags props, VkBuffer& buffer,
                                        VkDeviceMemory& memory) {
    VkBufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    info.size = size;
    info.usage = usage;
    info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &info, nullptr, &buffer) != VK_SUCCESS) return false;
    VkMemoryRequirements req;
    vkGetBufferMemoryRequirements(device, buffer, &req);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = findMemoryType(req.memoryTypeBits, props);
    if (vkAllocateMemory(device, &alloc, nullptr, &memory) != VK_SUCCESS) {
        vkDestroyBuffer(device, buffer, nullptr);
        buffer = VK_NULL_HANDLE;
        return false;
    }
    vkBindBufferMemory(device, buffer, memory, 0);
    return true;
}

bool VulkanRenderer::Impl::createDeviceLocalBuffer(const void* data, VkDeviceSize size,
                                                   VkBufferUsageFlags usage, VkBuffer& buffer,
                                                   VkDeviceMemory& memory) {
    VkBuffer staging = VK_NULL_HANDLE;
    VkDeviceMemory stagingMem = VK_NULL_HANDLE;
    if (!createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      staging, stagingMem))
        return false;
    void* mapped = nullptr;
    vkMapMemory(device, stagingMem, 0, size, 0, &mapped);
    std::memcpy(mapped, data, static_cast<size_t>(size));
    vkUnmapMemory(device, stagingMem);

    if (!createBuffer(size, VK_BUFFER_USAGE_TRANSFER_DST_BIT | usage,
                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, buffer, memory)) {
        vkDestroyBuffer(device, staging, nullptr);
        vkFreeMemory(device, stagingMem, nullptr);
        return false;
    }

    // One-time copy. Uploads are rare (level load), so a blocking submit is fine.
    VkCommandBufferAllocateInfo cba{};
    cba.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cba.commandPool = commandPool;
    cba.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cba.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(device, &cba, &cmd);
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &begin);
    VkBufferCopy copy{0, 0, size};
    vkCmdCopyBuffer(cmd, staging, buffer, 1, &copy);
    vkEndCommandBuffer(cmd);
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    vkQueueSubmit(graphicsQueue, 1, &submit, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);
    vkFreeCommandBuffers(device, commandPool, 1, &cmd);
    vkDestroyBuffer(device, staging, nullptr);
    vkFreeMemory(device, stagingMem, nullptr);
    return true;
}

void VulkanRenderer::Impl::destroyMesh(GpuMesh& m) {
    if (m.vertexBuffer) vkDestroyBuffer(device, m.vertexBuffer, nullptr);
    if (m.vertexMemory) vkFreeMemory(device, m.vertexMemory, nullptr);
    if (m.indexBuffer) vkDestroyBuffer(device, m.indexBuffer, nullptr);
    if (m.indexMemory) vkFreeMemory(device, m.indexMemory, nullptr);
    m = GpuMesh{};
}

// ---- swapchain lifecycle ----

void VulkanRenderer::Impl::cleanupSwapchain() {
    if (depthView) vkDestroyImageView(device, depthView, nullptr);
    if (depthImage) vkDestroyImage(device, depthImage, nullptr);
    if (depthMemory) vkFreeMemory(device, depthMemory, nullptr);
    depthView = VK_NULL_HANDLE;
    depthImage = VK_NULL_HANDLE;
    depthMemory = VK_NULL_HANDLE;
    for (VkFramebuffer fb : framebuffers) if (fb) vkDestroyFramebuffer(device, fb, nullptr);
    framebuffers.clear();
    for (VkImageView view : swapchainImageViews) if (view) vkDestroyImageView(device, view, nullptr);
    swapchainImageViews.clear();
    if (swapchain) { vkDestroySwapchainKHR(device, swapchain, nullptr); swapchain = VK_NULL_HANDLE; }
}

bool VulkanRenderer::Impl::recreateSwapchain() {
    if (window) window->getFramebufferSize(width, height);
    if (width == 0 || height == 0) return true;  // minimized: stay idle

    vkDeviceWaitIdle(device);
    for (VkSemaphore s : renderFinished) if (s) vkDestroySemaphore(device, s, nullptr);
    renderFinished.clear();
    cleanupSwapchain();

    if (!createSwapchain() || !createImageViews() || !createDepthResources() || !createFramebuffers())
        return false;

    renderFinished.resize(swapchainImages.size());
    imagesInFlight.assign(swapchainImages.size(), VK_NULL_HANDLE);
    VkSemaphoreCreateInfo sem{};
    sem.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    for (size_t i = 0; i < renderFinished.size(); ++i)
        if (vkCreateSemaphore(device, &sem, nullptr, &renderFinished[i]) != VK_SUCCESS) return false;
    framebufferResized = false;
    return true;
}

void VulkanRenderer::Impl::recordCommandBuffer(VkCommandBuffer cmd, uint32_t imageIndex) {
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    vkBeginCommandBuffer(cmd, &begin);

    std::array<VkClearValue, 2> clears{};
    clears[0].color = {{0.05f, 0.06f, 0.08f, 1.0f}};
    clears[1].depthStencil = {1.0f, 0};

    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = renderPass;
    rp.framebuffer = framebuffers[imageIndex];
    rp.renderArea.offset = {0, 0};
    rp.renderArea.extent = swapchainExtent;
    rp.clearValueCount = static_cast<uint32_t>(clears.size());
    rp.pClearValues = clears.data();
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport{};
    viewport.x = 0.0f;
    viewport.y = 0.0f;
    viewport.width = static_cast<float>(swapchainExtent.width);
    viewport.height = static_cast<float>(swapchainExtent.height);
    viewport.minDepth = 0.0f;
    viewport.maxDepth = 1.0f;
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    VkRect2D scissor{{0, 0}, swapchainExtent};
    vkCmdSetScissor(cmd, 0, 1, &scissor);

    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, meshPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1,
                            &descriptorSets[currentFrame], 0, nullptr);

    for (const DrawItem& item : drawQueue) {
        GpuMesh* m = meshes.get(item.mesh);
        if (!m || m->indexCount == 0) continue;
        vkCmdPushConstants(cmd, pipelineLayout,
                           VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                           0, sizeof(MeshPush), &item.push);
        VkDeviceSize offset = 0;
        vkCmdBindVertexBuffers(cmd, 0, 1, &m->vertexBuffer, &offset);
        vkCmdBindIndexBuffer(cmd, m->indexBuffer, 0, VK_INDEX_TYPE_UINT32);
        vkCmdDrawIndexed(cmd, m->indexCount, 1, 0, 0, 0);
        stats.drawCalls++;
        stats.trianglesDrawn += m->indexCount / 3;
    }

    vkCmdEndRenderPass(cmd);
    vkEndCommandBuffer(cmd);
}

void VulkanRenderer::Impl::drawFrame() {
    if (!initialized || width == 0 || height == 0) return;

    vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

    uint32_t imageIndex = 0;
    VkResult acquire = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX,
                                             imageAvailable[currentFrame], VK_NULL_HANDLE, &imageIndex);
    if (acquire == VK_ERROR_OUT_OF_DATE_KHR) { recreateSwapchain(); return; }
    if (acquire != VK_SUCCESS && acquire != VK_SUBOPTIMAL_KHR) {
        LOG_ERROR("[vulkan] vkAcquireNextImageKHR failed (%d)", static_cast<int>(acquire));
        return;
    }

    // Upload this frame's globals into its persistently mapped UBO.
    std::memcpy(globalsMapped[currentFrame], &cpuGlobals, sizeof(GlobalsUBO));

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
    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized)
        recreateSwapchain();
    else if (result != VK_SUCCESS)
        LOG_ERROR("[vulkan] vkQueuePresentKHR failed (%d)", static_cast<int>(result));

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
              impl->createDepthResources() &&
              impl->createFramebuffers() &&
              impl->createCommandPool() &&
              impl->createCommandBuffers() &&
              impl->createSyncObjects() &&
              impl->createDescriptorSetLayout() &&
              impl->createGlobalsBuffers() &&
              impl->createDescriptorPool() &&
              impl->createDescriptorSets() &&
              impl->createPipeline();
    if (!ok) {
        LOG_ERROR("[vulkan] initialization failed");
        return false;
    }
    impl->initialized = true;
    LOG_INFO("[vulkan] backend initialized (%dx%d, Phase 1: forward lit meshes)", width, height);
    return true;
}

void VulkanRenderer::shutdown() {
    if (!impl) return;
    if (impl->device == VK_NULL_HANDLE) {
        if (impl->instance) { vkDestroyInstance(impl->instance, nullptr); impl->instance = VK_NULL_HANDLE; }
        return;
    }
    vkDeviceWaitIdle(impl->device);

    impl->meshes.forEach([&](MeshHandle, GpuMesh& m) { impl->destroyMesh(m); });
    impl->meshes.clear();

    if (impl->meshPipeline) vkDestroyPipeline(impl->device, impl->meshPipeline, nullptr);
    if (impl->pipelineLayout) vkDestroyPipelineLayout(impl->device, impl->pipelineLayout, nullptr);
    if (impl->descriptorPool) vkDestroyDescriptorPool(impl->device, impl->descriptorPool, nullptr);
    if (impl->descriptorSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->descriptorSetLayout, nullptr);
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        if (impl->globalsMemory[i]) vkUnmapMemory(impl->device, impl->globalsMemory[i]);
        if (impl->globalsBuffers[i]) vkDestroyBuffer(impl->device, impl->globalsBuffers[i], nullptr);
        if (impl->globalsMemory[i]) vkFreeMemory(impl->device, impl->globalsMemory[i], nullptr);
    }
    impl->meshPipeline = VK_NULL_HANDLE;
    impl->pipelineLayout = VK_NULL_HANDLE;
    impl->descriptorPool = VK_NULL_HANDLE;
    impl->descriptorSetLayout = VK_NULL_HANDLE;

    for (VkSemaphore s : impl->renderFinished) if (s) vkDestroySemaphore(impl->device, s, nullptr);
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
    GpuMesh record;
    record.bounds = computeBoundingSphere(mesh.vertices.data(), mesh.vertices.size());
    record.indexCount = static_cast<uint32_t>(mesh.indices.size());

    if (impl->device && !mesh.vertices.empty() && !mesh.indices.empty()) {
        std::vector<GpuVertex> verts(mesh.vertices.size());
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            const Vertex& v = mesh.vertices[i];
            GpuVertex& g = verts[i];
            g.position[0] = static_cast<float>(v.position.x);
            g.position[1] = static_cast<float>(v.position.y);
            g.position[2] = static_cast<float>(v.position.z);
            g.normal[0] = static_cast<float>(v.normal.x);
            g.normal[1] = static_cast<float>(v.normal.y);
            g.normal[2] = static_cast<float>(v.normal.z);
            g.tangent[0] = static_cast<float>(v.tangent.x);
            g.tangent[1] = static_cast<float>(v.tangent.y);
            g.tangent[2] = static_cast<float>(v.tangent.z);
            g.texcoord[0] = v.u;
            g.texcoord[1] = v.v;
            g.color[0] = static_cast<float>(v.color.x);
            g.color[1] = static_cast<float>(v.color.y);
            g.color[2] = static_cast<float>(v.color.z);
        }
        VkDeviceSize vsize = verts.size() * sizeof(GpuVertex);
        VkDeviceSize isize = mesh.indices.size() * sizeof(uint32_t);
        bool ok = impl->createDeviceLocalBuffer(verts.data(), vsize,
                      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, record.vertexBuffer, record.vertexMemory) &&
                  impl->createDeviceLocalBuffer(mesh.indices.data(), isize,
                      VK_BUFFER_USAGE_INDEX_BUFFER_BIT, record.indexBuffer, record.indexMemory);
        if (!ok) {
            LOG_ERROR("[vulkan] uploadMesh buffer creation failed");
            impl->destroyMesh(record);
            record.indexCount = 0;
        }
    }
    return impl->meshes.insert(record);
}

void VulkanRenderer::removeMesh(MeshHandle handle) {
    GpuMesh* m = impl->meshes.get(handle);
    if (!m) return;
    if (impl->device) vkDeviceWaitIdle(impl->device);   // ensure no in-flight use
    impl->destroyMesh(*m);
    impl->meshes.erase(handle);
}

BoundingSphere VulkanRenderer::getMeshBounds(MeshHandle handle) const {
    const GpuMesh* m = impl->meshes.get(handle);
    return m ? m->bounds : BoundingSphere{};
}

TextureHandle VulkanRenderer::uploadTexture(int /*width*/, int /*height*/,
                                            int /*channels*/, const uint8_t* /*data*/) {
    // Phase 2 gives textures real GPU images + samplers; for now hand back a
    // valid handle so the material/asset layer keeps working.
    return impl->textures.insert(0);
}

void VulkanRenderer::removeTexture(TextureHandle handle) { impl->textures.erase(handle); }

RenderStats VulkanRenderer::getRenderStats() const { return impl->stats; }

void VulkanRenderer::beginFrame() {
    impl->stats = RenderStats{};
    impl->drawQueue.clear();
}

void VulkanRenderer::setCamera(const CameraState& camera) {
    Mat4 view = Mat4::lookAt(camera.position, camera.target, camera.up);
    float aspect = impl->swapchainExtent.height > 0
                       ? static_cast<float>(impl->swapchainExtent.width) /
                             static_cast<float>(impl->swapchainExtent.height)
                       : camera.aspectRatio;
    Mat4 proj;
    if (camera.projection == CameraProjection::Orthographic) {
        proj = Mat4::orthographic(camera.orthoHeight, aspect, camera.nearPlane, camera.farPlane);
    } else {
        proj = Mat4::perspective(camera.fovDegrees * 3.14159265358979 / 180.0, aspect,
                                 camera.nearPlane, camera.farPlane);
    }
    Mat4 vp = proj * view;
    packMat4(vp, impl->cpuGlobals.viewProjection, /*flipY=*/true);
    impl->cpuGlobals.cameraPosition[0] = static_cast<float>(camera.position.x);
    impl->cpuGlobals.cameraPosition[1] = static_cast<float>(camera.position.y);
    impl->cpuGlobals.cameraPosition[2] = static_cast<float>(camera.position.z);
    impl->cpuGlobals.cameraPosition[3] = 1.0f;
}

void VulkanRenderer::setLights(const SceneLighting& lighting) {
    const DirectionalLight& sun = lighting.sun;
    impl->cpuGlobals.sunDirection[0] = static_cast<float>(sun.direction.x);
    impl->cpuGlobals.sunDirection[1] = static_cast<float>(sun.direction.y);
    impl->cpuGlobals.sunDirection[2] = static_cast<float>(sun.direction.z);
    impl->cpuGlobals.sunDirection[3] = 0.0f;
    impl->cpuGlobals.sunColor[0] = static_cast<float>(sun.color.x) * sun.intensity;
    impl->cpuGlobals.sunColor[1] = static_cast<float>(sun.color.y) * sun.intensity;
    impl->cpuGlobals.sunColor[2] = static_cast<float>(sun.color.z) * sun.intensity;
    impl->cpuGlobals.sunColor[3] = 1.0f;
    float amb = lighting.ambientMultiplier;
    impl->cpuGlobals.ambient[0] = static_cast<float>(lighting.ambientTint.x) * amb;
    impl->cpuGlobals.ambient[1] = static_cast<float>(lighting.ambientTint.y) * amb;
    impl->cpuGlobals.ambient[2] = static_cast<float>(lighting.ambientTint.z) * amb;
    impl->cpuGlobals.ambient[3] = 1.0f;
}

void VulkanRenderer::drawMesh(MeshHandle handle, const Mat4& transform,
                              const RenderMaterial& material) {
    DrawItem item;
    item.mesh = handle;
    packMat4(transform, item.push.model, /*flipY=*/false);
    item.push.albedoMetallic[0] = static_cast<float>(material.albedo.x);
    item.push.albedoMetallic[1] = static_cast<float>(material.albedo.y);
    item.push.albedoMetallic[2] = static_cast<float>(material.albedo.z);
    item.push.albedoMetallic[3] = material.metallic;
    item.push.emissionRough[0] = static_cast<float>(material.emission.x);
    item.push.emissionRough[1] = static_cast<float>(material.emission.y);
    item.push.emissionRough[2] = static_cast<float>(material.emission.z);
    item.push.emissionRough[3] = material.roughness;
    impl->drawQueue.push_back(item);
    impl->stats.entitiesSubmitted++;
}

void VulkanRenderer::endFrame() { impl->drawFrame(); }

// The non-Apple factory. Exactly one Renderer::create() is linked per target.
std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<VulkanRenderer>();
}

}  // namespace engine
