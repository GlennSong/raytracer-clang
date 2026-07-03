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

#ifdef RT_ENABLE_IMGUI
#include "imgui.h"
#include "backends/imgui_impl_vulkan.h"
#endif

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
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
constexpr int RT_MAX_CASCADES = 4;
constexpr VkFormat kDepthFormat = VK_FORMAT_D32_SFLOAT;
constexpr VkFormat kHdrFormat = VK_FORMAT_R16G16B16A16_SFLOAT;

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

// One light, packed into 4 vec4 (std140). Mirrors the Light struct in the
// shaders. Maps the engine's GPULight fields: positionIntensity = (pos, intensity),
// directionInner = (dir, innerCos), colorOuter = (color, outerCos),
// typeRange = (type, range, _, _). type: 0 point, 1 directional, 2 spot.
struct GpuLight {
    float positionIntensity[4];
    float directionInner[4];
    float colorOuter[4];
    float typeRange[4];
};

// std140-compatible global uniforms (set 0, binding 0). Mirrors the Globals
// block in shaders/vulkan/{mesh,sky}.*. Offsets: viewProjection 0, view 64,
// invViewProjection 128, cascadeVP 192, cameraPosition 448, ambient 464,
// cascadeSplit 480, counts 496, shadowParams 512, sky* 528..608, lights 624.
struct GlobalsUBO {
    float    viewProjection[16];
    float    view[16];
    float    invViewProjection[16];  // inverse of the flipped VP (skybox ray, IBL)
    float    cascadeVP[4][16];       // per-cascade light view-projection (CSM)
    float    cameraPosition[4];
    float    ambient[4];             // rgb = ambient tint * multiplier (IBL strength)
    float    cascadeSplit[4];        // far view-space depth of cascades 0..3
    int32_t  counts[4];              // x lightCount, y cascadeCount, z envMode, w debugView
    float    shadowParams[4];        // x normalBias, y pcfRadius, z mapSize, w strength
    // Procedural sky (ADR-0016), mirrored from SceneLighting::sky.
    float    skySunDir[4];           // xyz toward sun, w sun disc intensity
    float    skySunColor[4];         // rgb
    float    skyZenith[4];
    float    skyHorizon[4];
    float    skyGround[4];
    float    skyCloud[4];            // x coverage, y density, z scale, w time
    GpuLight lights[32];
    // Appended after the lights array so every earlier offset is unchanged (and
    // shaders that don't read these can omit them).
    float    fog[4];                 // aerial-perspective fog (ADR-0016); w 0 = off
    float    shadowTint[4];          // rgb artistic shadow tint, w ambientStrength
    float    wind1[4];               // xyz wind direction, w wind time (seconds)
    float    wind2[4];               // x frequency, y height, z amplitude (FLAG_WIND)
};

// Per-draw push constants for the forward pass (120 <= 128 B).
struct MeshPush {
    float    model[16];
    float    albedoMetallic[4];  // rgb albedo, a metallic
    float    emissionRough[4];   // rgb emission, a roughness
    uint32_t surfaceFlags[4];    // x surfaceId, y rawFlags, z textureFlags
    float    morphStart;         // CDLOD terrain morph band (terrain.vert only)
    float    morphEnd;
};

// Per-draw push constants for the shadow (depth-only) pass: the cascade's light
// view-projection and the model matrix (128 B = the guaranteed push limit).
struct ShadowPush {
    float lightViewProj[16];
    float model[16];
};

// Composite (tonemap) push constants.
struct CompositePush {
    float    exposure;
    int32_t  tonemapOp;      // 0 = ACES, 1 = AgX
    float    gradeContrast;
    float    gradeSaturation;
    int32_t  bloomEnabled;
    float    bloomIntensity;
    int32_t  ssaoEnabled;
    float    aoFloor;
    int32_t  ssrEnabled;
    int32_t  lensEnabled;
    float    lensK1;          // Brown radial distortion
    float    lensK2;
    float    lensCA;          // lateral chromatic aberration
    float    lensVignette;
    float    lensAspect;      // width / height
    int32_t  debugView;      // 0 normal; 1 AO/2 SSR/4 normals composite-side,
                             // 3/5/6/7/8 raw HDR (mesh.frag writes them)
    int32_t  dofEnabled;     // 1 → read the DOF-blurred scene instead of HDR
};

// Bloom pass push constants. dir is the blur direction in texels (0 for the
// bright pass); threshold/knee used by the bright pass only.
struct BloomPush {
    float texelX, texelY;
    float dirX, dirY;
    float threshold;
    float knee;
    float brightPass;        // 1.0 = bright-pass (threshold), 0.0 = blur
    float _pad;
};

// SSAO push constants.
struct SsaoPush {
    float radius;
    float bias;
    float intensity;
    float _pad;
};

// SSAO blur push constants (AO texel size).
struct SsaoBlurPush {
    float texelX;
    float texelY;
};

// SSR push constants.
struct SsrPush {
    float maxRayDist;
    float thickness;
    float maxRoughness;
    float blendStrength;
};

// Depth-of-field push constants (thin-lens CoC + gather params). Mirrors Metal's
// DOFUniforms; texel is 1/resolution for the gather offsets.
struct DofPush {
    float focusDistance;   // world units
    float focalLength;     // meters
    float aperture;        // diameter, meters
    float cocScale;        // sensor meters → pixels
    float maxCocPixels;
    float texelX;
    float texelY;
    float _pad;
};

struct GpuMesh {
    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory = VK_NULL_HANDLE;
    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory = VK_NULL_HANDLE;
    uint32_t indexCount = 0;
    BoundingSphere bounds;
};

struct GpuTexture {
    VkImage image = VK_NULL_HANDLE;
    VkDeviceMemory memory = VK_NULL_HANDLE;
    VkImageView view = VK_NULL_HANDLE;
};

// Material texture slots, in the order the fragment shader samples them and the
// textureFlags bits are assigned (matching the Metal backend): 0 albedo,
// 1 metallic-roughness, 2 normal, 3 AO, 4 emissive.
struct DrawItem {
    MeshHandle mesh;
    MeshPush push;
    std::array<TextureHandle, 5> textures;   // albedo, MR, normal, AO, emissive
    float opacity = 1.0f;                     // < 1 → transparent pass (back-to-front)
    bool  terrain = false;                    // → terrainPipeline (CDLOD morph in vert)
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

// Minimal IEEE-754 float→half (RGBA16F env upload). Ignores subnormals/rounding
// — fine for an environment map.
uint16_t floatToHalf(float f) {
    uint32_t x;
    std::memcpy(&x, &f, 4);
    uint32_t sign = (x >> 16) & 0x8000u;
    int32_t exp = static_cast<int32_t>((x >> 23) & 0xff) - 127 + 15;
    uint32_t mant = x & 0x7fffffu;
    if (exp <= 0) return static_cast<uint16_t>(sign);
    if (exp >= 31) return static_cast<uint16_t>(sign | 0x7c00u);
    return static_cast<uint16_t>(sign | (static_cast<uint32_t>(exp) << 10) | (mant >> 13));
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

    // Offscreen HDR scene target (Phase 5): geometry + sky render here in linear
    // HDR; the composite pass tonemaps it into the swapchain. Single target,
    // matching the existing single shared depth buffer (frames-in-flight aliasing
    // of scene targets is pre-existing debt — see plan).
    VkImage hdrImage = VK_NULL_HANDLE;
    VkDeviceMemory hdrMemory = VK_NULL_HANDLE;
    VkImageView hdrView = VK_NULL_HANDLE;
    VkFramebuffer sceneFramebuffer = VK_NULL_HANDLE;   // HDR + depth (scene pass)

    VkRenderPass compositeRenderPass = VK_NULL_HANDLE;
    VkSampler compositeSampler = VK_NULL_HANDLE;
    VkDescriptorSetLayout compositeSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool compositePool = VK_NULL_HANDLE;
    VkDescriptorSet compositeSet = VK_NULL_HANDLE;
    VkPipelineLayout compositePipelineLayout = VK_NULL_HANDLE;
    VkPipeline compositePipeline = VK_NULL_HANDLE;
    float sceneExposure = 1.0f;
    int   tonemapOp = 0;          // mirrored from Renderer::tonemapOperator each frame
    float gradeContrast = 1.0f;
    float gradeSaturation = 1.0f;

    // Bloom (Phase 5b): bright-pass → separable Gaussian (H,V) at half res, added
    // back in composite. Two half-res RGBA16F targets ping-pong. (Metal uses a
    // 5-mip pyramid; a single blurred level is the simpler first cut.)
    VkImage bloomImage[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
    VkDeviceMemory bloomMemory[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
    VkImageView bloomView[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
    VkFramebuffer bloomFramebuffer[2] = {VK_NULL_HANDLE, VK_NULL_HANDLE};
    VkExtent2D bloomExtent{0, 0};
    VkRenderPass bloomRenderPass = VK_NULL_HANDLE;
    VkSampler bloomSampler = VK_NULL_HANDLE;        // linear clamp
    VkDescriptorSetLayout bloomSetLayout = VK_NULL_HANDLE;
    VkDescriptorPool bloomPool = VK_NULL_HANDLE;
    VkDescriptorSet bloomSetBright = VK_NULL_HANDLE;   // samples HDR
    VkDescriptorSet bloomSetH = VK_NULL_HANDLE;        // samples bloomView[0]
    VkDescriptorSet bloomSetV = VK_NULL_HANDLE;        // samples bloomView[1]
    VkPipelineLayout bloomPipelineLayout = VK_NULL_HANDLE;
    VkPipeline bloomPipeline = VK_NULL_HANDLE;   // one shader; bright vs blur via push
    bool  bloomEnabledFrame = true;
    float bloomThreshold = 1.0f;
    float bloomKnee = 0.5f;
    float bloomIntensity = 0.3f;

    // SSAO (Phase 5b): the scene pass also writes world-space normals to a
    // second color attachment; an SSAO pass reads depth + normals to a half-res
    // AO target the composite multiplies in. World-space reconstruction reuses
    // viewProjection / invViewProjection (already in the UBO) — no extra matrix.
    VkImage normalImage = VK_NULL_HANDLE;
    VkDeviceMemory normalMemory = VK_NULL_HANDLE;
    VkImageView normalView = VK_NULL_HANDLE;     // RGBA8 world normal *0.5+0.5
    VkSampler gbufferSampler = VK_NULL_HANDLE;   // nearest clamp (depth + normal)
    VkImage aoImage = VK_NULL_HANDLE;
    VkDeviceMemory aoMemory = VK_NULL_HANDLE;
    VkImageView aoView = VK_NULL_HANDLE;         // half-res R8 AO (raw)
    VkFramebuffer aoFramebuffer = VK_NULL_HANDLE;
    VkImage aoBlurImage = VK_NULL_HANDLE;        // box-blurred AO (composite reads this)
    VkDeviceMemory aoBlurMemory = VK_NULL_HANDLE;
    VkImageView aoBlurView = VK_NULL_HANDLE;
    VkFramebuffer aoBlurFramebuffer = VK_NULL_HANDLE;
    VkExtent2D aoExtent{0, 0};
    VkRenderPass ssaoRenderPass = VK_NULL_HANDLE;
    VkDescriptorSetLayout gbufferSetLayout = VK_NULL_HANDLE;  // set 1: depth + normal
    VkDescriptorSetLayout ssaoBlurSetLayout = VK_NULL_HANDLE; // single sampler (raw AO)
    VkDescriptorPool ssaoPool = VK_NULL_HANDLE;
    VkDescriptorSet gbufferSet = VK_NULL_HANDLE;
    VkDescriptorSet ssaoBlurSet = VK_NULL_HANDLE;
    VkPipelineLayout ssaoPipelineLayout = VK_NULL_HANDLE;
    VkPipeline ssaoPipeline = VK_NULL_HANDLE;
    VkPipelineLayout ssaoBlurPipelineLayout = VK_NULL_HANDLE;
    VkPipeline ssaoBlurPipeline = VK_NULL_HANDLE;
    bool  ssaoEnabledFrame = true;
    float ssaoRadius = 1.5f;
    float ssaoIntensity = 0.8f;
    float ssaoBias = 0.05f;
    float ssaoFloor = 0.15f;

    // SSR (Phase 5b): world-space ray march of the reflection ray against the
    // depth buffer, sampling the HDR scene at the hit. Half-res; composite adds
    // it by confidence. Reuses the normal G-buffer (roughness packed in .a).
    VkImage ssrImage = VK_NULL_HANDLE;
    VkDeviceMemory ssrMemory = VK_NULL_HANDLE;
    VkImageView ssrView = VK_NULL_HANDLE;          // half-res RGBA16F (rgb + confidence)
    VkFramebuffer ssrFramebuffer = VK_NULL_HANDLE;
    VkExtent2D ssrExtent{0, 0};
    VkRenderPass ssrRenderPass = VK_NULL_HANDLE;
    VkDescriptorSetLayout ssrSetLayout = VK_NULL_HANDLE;   // set 1: depth, normal, hdr
    VkDescriptorPool ssrPool = VK_NULL_HANDLE;
    VkDescriptorSet ssrSet = VK_NULL_HANDLE;
    VkPipelineLayout ssrPipelineLayout = VK_NULL_HANDLE;
    VkPipeline ssrPipeline = VK_NULL_HANDLE;
    bool  ssrEnabledFrame = true;
    float ssrMaxRayDist = 20.0f;
    float ssrThickness = 0.3f;
    float ssrMaxRoughness = 0.6f;
    float ssrBlendStrength = 0.5f;

    // Depth of field (Phase 5b): a single fullscreen scatter-as-gather on the HDR
    // scene, before composite (composite reads dofView at binding 6 when enabled).
    // Full-res; reads the globals UBO (set 0) for view-depth reconstruction and
    // hdr+depth (set 1). Off by default (Renderer::dofEnabled + a real aperture).
    VkImage dofImage = VK_NULL_HANDLE;
    VkDeviceMemory dofMemory = VK_NULL_HANDLE;
    VkImageView dofView = VK_NULL_HANDLE;          // full-res RGBA16F (blurred scene)
    VkFramebuffer dofFramebuffer = VK_NULL_HANDLE;
    VkExtent2D dofExtent{0, 0};
    VkRenderPass dofRenderPass = VK_NULL_HANDLE;
    VkDescriptorSetLayout dofSetLayout = VK_NULL_HANDLE;   // set 1: hdr, depth
    VkDescriptorPool dofPool = VK_NULL_HANDLE;
    VkDescriptorSet dofSet = VK_NULL_HANDLE;
    VkPipelineLayout dofPipelineLayout = VK_NULL_HANDLE;
    VkPipeline dofPipeline = VK_NULL_HANDLE;
    bool  dofEnabledFrame = false;
    float dofFocusDistance = 10.0f;
    float dofFocalLength = 0.05f;   // meters
    float dofAperture = 0.0f;       // meters (0 = pinhole → pass skipped)
    float dofSensorHeight = 24.0f;  // mm

    // Lens effects (Phase 5b): distortion + chromatic aberration + vignette,
    // folded into the composite. From the active camera's LensParams.
    bool  lensEnabledFrame = true;
    float lensK1 = 0.0f, lensK2 = 0.0f, lensCA = 0.0f, lensVignette = 0.0f, lensAspect = 1.0f;
    int   debugViewFrame = 0;   // mirrored from Renderer::debugView
    int   wireframeFrame = 0;   // mirrored from Renderer::wireframe (0 off,1 only,2 overlay)
    float wireColor[3] = {0.1f, 1.0f, 0.5f};
    std::chrono::steady_clock::time_point windStart = std::chrono::steady_clock::now();

    // HDR environment (Phase 4b): an equirectangular RGBA16F map (with mips for
    // roughness-LOD specular) sampled for the skybox + IBL when bound. Bound at
    // set 0 binding 2; a 1x1 default stands in when none is set. The full
    // GGX-prefiltered-cubemap split-sum + reflection probes are a later refinement.
    VkSampler envSampler = VK_NULL_HANDLE;   // linear, wrap-u/clamp-v, mipped
    VkImageView envView = VK_NULL_HANDLE;     // current equirect (or default)
    bool envBound = false;

    // Split-sum BRDF integration LUT (set 0 binding 3), baked once at init.
    VkImage brdfLutImage = VK_NULL_HANDLE;
    VkDeviceMemory brdfLutMemory = VK_NULL_HANDLE;
    VkImageView brdfLutView = VK_NULL_HANDLE;
    VkSampler brdfLutSampler = VK_NULL_HANDLE;

    // Dear ImGui (ADR-0011); only used when RT_ENABLE_IMGUI is defined.
    VkDescriptorPool imguiPool = VK_NULL_HANDLE;
    bool imguiInitialized = false;

    VkRenderPass renderPass = VK_NULL_HANDLE;   // scene pass (HDR color + depth)
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
    VkPipeline wirePipeline = VK_NULL_HANDLE;          // VK_POLYGON_MODE_LINE debug view
    VkPipeline transparentPipeline = VK_NULL_HANDLE;   // alpha blend, no depth write
    VkPipeline overlayPipeline = VK_NULL_HANDLE;       // FLAG_OVERLAY: no depth test/write, on top
    VkPipeline terrainPipeline = VK_NULL_HANDLE;       // CDLOD morph (terrain.vert)

    // Procedural-sky skybox (fullscreen triangle, no vertex buffer).
    VkPipelineLayout skyPipelineLayout = VK_NULL_HANDLE;
    VkPipeline skyPipeline = VK_NULL_HANDLE;

    // Material textures (set 1): a per-frame transient descriptor pool reset each
    // frame, so each draw gets a fresh set with no cross-frame lifetime issues.
    VkDescriptorSetLayout materialSetLayout = VK_NULL_HANDLE;
    std::array<VkDescriptorPool, MAX_FRAMES_IN_FLIGHT> materialPools{};
    VkSampler textureSampler = VK_NULL_HANDLE;
    GpuTexture defaultTexture;   // 1x1 white, stands in for absent maps
    bool materialPoolExhaustedWarned = false;

    // Cascaded shadow maps (sun). One depth array image with a layer per cascade;
    // per-layer views for rendering, one array view + a compare sampler for the
    // PCF lookup in the lit pass.
    static constexpr int SHADOW_MAP_SIZE = 2048;
    VkImage shadowImage = VK_NULL_HANDLE;
    VkDeviceMemory shadowMemory = VK_NULL_HANDLE;
    VkImageView shadowArrayView = VK_NULL_HANDLE;
    std::array<VkImageView, RT_MAX_CASCADES> shadowLayerViews{};
    std::array<VkFramebuffer, RT_MAX_CASCADES> shadowFramebuffers{};
    VkRenderPass shadowRenderPass = VK_NULL_HANDLE;
    VkPipelineLayout shadowPipelineLayout = VK_NULL_HANDLE;
    VkPipeline shadowPipeline = VK_NULL_HANDLE;
    VkSampler shadowSampler = VK_NULL_HANDLE;
    int activeCascadeCount = 0;
    float shadowDepthBiasConst = 1.25f;
    float shadowDepthBiasSlope = 1.75f;

    // Camera state captured in setCamera, needed by setLights for the cascade fit.
    Mat4 camViewProj;       // un-flipped (forward-Z) view-projection
    Vec3 camPos;
    float camNear = 0.1f;
    float camFar = 1000.0f;

    bool framebufferResized = false;
    bool initialized = false;

    SlotMap<GpuMesh, MeshTag> meshes;
    SlotMap<GpuTexture, TextureTag> textures;
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
    bool createHdrResources();
    bool createRenderPass();
    bool createCompositeRenderPass();
    bool createSceneFramebuffer();
    bool createFramebuffers();           // composite framebuffers (per swapchain image)
    bool createCompositeResources();     // sampler, descriptor, pipeline
    void updateCompositeDescriptor();
    bool createBloomResources();
    void updateBloomDescriptors();
    void recordBloom(VkCommandBuffer cmd);
    bool createSsaoResources();
    void updateSsaoDescriptors();
    void recordSsao(VkCommandBuffer cmd);
    bool createSsrResources();
    void updateSsrDescriptors();
    void recordSsr(VkCommandBuffer cmd);
    bool createDofResources();
    void updateDofDescriptors();
    void recordDof(VkCommandBuffer cmd);
    bool createColorTarget(uint32_t w, uint32_t h, VkFormat fmt, VkImageUsageFlags usage,
                           VkImage& image, VkDeviceMemory& memory, VkImageView& view);
    void updateGlobalEnvDescriptor();   // write env equirect into set 0 binding 2
    bool createBrdfLut();               // bake the split-sum LUT (set 0 binding 3)
    bool createCommandPool();
    bool createCommandBuffers();
    bool createSyncObjects();
    bool createDescriptorSetLayout();
    bool createGlobalsBuffers();
    bool createDescriptorPool();
    bool createDescriptorSets();
    bool createMaterialResources();
    bool createShadowResources();
    bool createShadowPipeline();
    bool createPipeline();
    bool createSkyPipeline();
    void recordShadowPass(VkCommandBuffer cmd);

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

    bool createImageRGBA8(const uint8_t* rgba, uint32_t w, uint32_t h, GpuTexture& out);
    void transitionImageLayout(VkCommandBuffer cmd, VkImage image,
                               VkImageLayout from, VkImageLayout to);
    void destroyTexture(GpuTexture& t);
    VkImageView textureViewOr(TextureHandle h) const;   // view, or default white
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
    // The MRT scene pass masks the normal attachment on the sky pipeline (writes
    // color, not normals), so its per-attachment blend state differs from the HDR
    // attachment — which requires independentBlend (VUID-...-pAttachments-00605).
    // Widely supported; verified needed by the first Phase 5b device run.
    features.independentBlend = VK_TRUE;
    // VK_POLYGON_MODE_LINE for the wireframe debug view (Renderer::wireframe).
    // Core-but-optional; effectively universal on desktop GPUs.
    features.fillModeNonSolid = VK_TRUE;
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
    // Prefer a UNORM swapchain: the composite tonemap (ACES/AgX) folds the sRGB
    // encode into its output, so the presented image is already encoded — an sRGB
    // swapchain would gamma it twice. (Pre-Phase-5 wrote linear to an sRGB
    // swapchain; that path is gone now that everything goes through composite.)
    VkSurfaceFormatKHR chosen = formats[0];
    for (const VkSurfaceFormatKHR& f : formats)
        if (f.format == VK_FORMAT_B8G8R8A8_UNORM &&
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
    image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
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

bool VulkanRenderer::Impl::createHdrResources() {
    VkImageCreateInfo image{};
    image.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image.imageType = VK_IMAGE_TYPE_2D;
    image.extent = {swapchainExtent.width, swapchainExtent.height, 1};
    image.mipLevels = 1;
    image.arrayLayers = 1;
    image.format = kHdrFormat;
    image.tiling = VK_IMAGE_TILING_OPTIMAL;
    image.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image.usage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    image.samples = VK_SAMPLE_COUNT_1_BIT;
    image.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &image, nullptr, &hdrImage) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] HDR vkCreateImage failed");
        return false;
    }
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device, hdrImage, &req);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(device, &alloc, nullptr, &hdrMemory) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] HDR vkAllocateMemory failed");
        return false;
    }
    vkBindImageMemory(device, hdrImage, hdrMemory, 0);

    VkImageViewCreateInfo view{};
    view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view.image = hdrImage;
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = kHdrFormat;
    view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view.subresourceRange.levelCount = 1;
    view.subresourceRange.layerCount = 1;
    if (vkCreateImageView(device, &view, nullptr, &hdrView) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] HDR vkCreateImageView failed");
        return false;
    }
    // World-normal G-buffer (scene-pass color attachment 1), RGBA8.
    if (!createColorTarget(swapchainExtent.width, swapchainExtent.height,
                           VK_FORMAT_R8G8B8A8_UNORM,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                           normalImage, normalMemory, normalView)) {
        LOG_ERROR("[vulkan] normal G-buffer creation failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createRenderPass() {
    // Two color attachments: 0 = HDR scene, 1 = world-normal G-buffer. Both are
    // sampled afterward (composite reads HDR; SSAO reads normals + depth), so
    // their final layout is SHADER_READ_ONLY and depth's is DEPTH_READ_ONLY.
    VkAttachmentDescription color{};
    color.format = kHdrFormat;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

    VkAttachmentDescription normal = color;
    normal.format = VK_FORMAT_R8G8B8A8_UNORM;

    VkAttachmentDescription depth{};
    depth.format = kDepthFormat;
    depth.samples = VK_SAMPLE_COUNT_1_BIT;
    depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;   // SSAO samples it
    depth.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;

    std::array<VkAttachmentReference, 2> colorRefs{
        VkAttachmentReference{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL},
        VkAttachmentReference{1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL}};
    VkAttachmentReference depthRef{2, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = static_cast<uint32_t>(colorRefs.size());
    subpass.pColorAttachments = colorRefs.data();
    subpass.pDepthStencilAttachment = &depthRef;

    std::array<VkSubpassDependency, 2> deps{};
    deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    deps[0].dstSubpass = 0;
    deps[0].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                           VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                           VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    deps[0].srcAccessMask = 0;
    deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    // Scene color + depth writes must be visible to the SSAO/composite reads.
    deps[1].srcSubpass = 0;
    deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                           VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    std::array<VkAttachmentDescription, 3> attachments{color, normal, depth};
    VkRenderPassCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    info.attachmentCount = static_cast<uint32_t>(attachments.size());
    info.pAttachments = attachments.data();
    info.subpassCount = 1;
    info.pSubpasses = &subpass;
    info.dependencyCount = static_cast<uint32_t>(deps.size());
    info.pDependencies = deps.data();

    if (vkCreateRenderPass(device, &info, nullptr, &renderPass) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] vkCreateRenderPass failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createCompositeRenderPass() {
    VkAttachmentDescription color{};
    color.format = swapchainFormat;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;   // composite writes every pixel
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkAttachmentReference colorRef{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
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
    if (vkCreateRenderPass(device, &info, nullptr, &compositeRenderPass) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] composite render pass creation failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createSceneFramebuffer() {
    std::array<VkImageView, 3> attachments{hdrView, normalView, depthView};
    VkFramebufferCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    info.renderPass = renderPass;
    info.attachmentCount = static_cast<uint32_t>(attachments.size());
    info.pAttachments = attachments.data();
    info.width = swapchainExtent.width;
    info.height = swapchainExtent.height;
    info.layers = 1;
    if (vkCreateFramebuffer(device, &info, nullptr, &sceneFramebuffer) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] scene framebuffer creation failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createFramebuffers() {
    // Composite framebuffers — one per swapchain image (color only).
    framebuffers.resize(swapchainImageViews.size());
    for (size_t i = 0; i < swapchainImageViews.size(); ++i) {
        VkFramebufferCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        info.renderPass = compositeRenderPass;
        info.attachmentCount = 1;
        info.pAttachments = &swapchainImageViews[i];
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

void VulkanRenderer::Impl::updateCompositeDescriptor() {
    VkDescriptorImageInfo imgs[7]{};
    // 0 HDR, 1 bloom, 2 AO, 3 SSR, 4 depth, 5 normals, 6 DOF (when dofEnabled).
    VkImageView views[7] = {hdrView, bloomView[0], aoBlurView, ssrView, depthView,
                            normalView, dofView};
    for (uint32_t i = 0; i < 7; ++i) {
        imgs[i].sampler = compositeSampler;
        imgs[i].imageView = views[i];
        // Binding 4 is the depth attachment, which lives in DEPTH_READ_ONLY.
        imgs[i].imageLayout = (i == 4) ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
                                       : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    }
    std::array<VkWriteDescriptorSet, 7> writes{};
    for (uint32_t i = 0; i < 7; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = compositeSet;
        writes[i].dstBinding = i;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i].descriptorCount = 1;
        writes[i].pImageInfo = &imgs[i];
    }
    vkUpdateDescriptorSets(device, 7, writes.data(), 0, nullptr);
}

bool VulkanRenderer::Impl::createColorTarget(uint32_t w, uint32_t h, VkFormat fmt,
                                             VkImageUsageFlags usage, VkImage& image,
                                             VkDeviceMemory& memory, VkImageView& view) {
    VkImageCreateInfo ci{};
    ci.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    ci.imageType = VK_IMAGE_TYPE_2D;
    ci.extent = {w, h, 1};
    ci.mipLevels = 1;
    ci.arrayLayers = 1;
    ci.format = fmt;
    ci.tiling = VK_IMAGE_TILING_OPTIMAL;
    ci.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    ci.usage = usage;
    ci.samples = VK_SAMPLE_COUNT_1_BIT;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &ci, nullptr, &image) != VK_SUCCESS) return false;
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device, image, &req);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(device, &alloc, nullptr, &memory) != VK_SUCCESS) return false;
    vkBindImageMemory(device, image, memory, 0);
    VkImageViewCreateInfo vi{};
    vi.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    vi.image = image;
    vi.viewType = VK_IMAGE_VIEW_TYPE_2D;
    vi.format = fmt;
    vi.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    vi.subresourceRange.levelCount = 1;
    vi.subresourceRange.layerCount = 1;
    return vkCreateImageView(device, &vi, nullptr, &view) == VK_SUCCESS;
}

void VulkanRenderer::Impl::updateBloomDescriptors() {
    VkDescriptorSet sets[3] = {bloomSetBright, bloomSetH, bloomSetV};
    VkImageView views[3] = {hdrView, bloomView[0], bloomView[1]};
    for (int i = 0; i < 3; ++i) {
        VkDescriptorImageInfo img{};
        img.sampler = bloomSampler;
        img.imageView = views[i];
        img.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = sets[i];
        write.dstBinding = 0;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.descriptorCount = 1;
        write.pImageInfo = &img;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }
}

bool VulkanRenderer::Impl::createBloomResources() {
    bloomExtent = {std::max(1u, swapchainExtent.width / 2),
                   std::max(1u, swapchainExtent.height / 2)};
    for (int i = 0; i < 2; ++i)
        if (!createColorTarget(bloomExtent.width, bloomExtent.height, kHdrFormat,
                               VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                               bloomImage[i], bloomMemory[i], bloomView[i]))
            return false;

    // Render pass shared by all three bloom passes (each overwrites its target).
    if (bloomRenderPass == VK_NULL_HANDLE) {
        VkAttachmentDescription color{};
        color.format = kHdrFormat;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &ref;
        std::array<VkSubpassDependency, 2> deps{};
        deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        deps[0].dstSubpass = 0;
        deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].srcSubpass = 0;
        deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo rp{};
        rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rp.attachmentCount = 1;
        rp.pAttachments = &color;
        rp.subpassCount = 1;
        rp.pSubpasses = &subpass;
        rp.dependencyCount = static_cast<uint32_t>(deps.size());
        rp.pDependencies = deps.data();
        if (vkCreateRenderPass(device, &rp, nullptr, &bloomRenderPass) != VK_SUCCESS) return false;
    }

    for (int i = 0; i < 2; ++i) {
        VkFramebufferCreateInfo fb{};
        fb.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fb.renderPass = bloomRenderPass;
        fb.attachmentCount = 1;
        fb.pAttachments = &bloomView[i];
        fb.width = bloomExtent.width;
        fb.height = bloomExtent.height;
        fb.layers = 1;
        if (vkCreateFramebuffer(device, &fb, nullptr, &bloomFramebuffer[i]) != VK_SUCCESS)
            return false;
    }

    if (bloomSampler == VK_NULL_HANDLE) {
        VkSamplerCreateInfo s{};
        s.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        s.magFilter = VK_FILTER_LINEAR;
        s.minFilter = VK_FILTER_LINEAR;
        s.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        s.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        s.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        s.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        if (vkCreateSampler(device, &s, nullptr, &bloomSampler) != VK_SUCCESS) return false;
    }

    if (bloomSetLayout == VK_NULL_HANDLE) {
        VkDescriptorSetLayoutBinding b{};
        b.binding = 0;
        b.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        b.descriptorCount = 1;
        b.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo l{};
        l.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        l.bindingCount = 1;
        l.pBindings = &b;
        if (vkCreateDescriptorSetLayout(device, &l, nullptr, &bloomSetLayout) != VK_SUCCESS)
            return false;

        VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3};
        VkDescriptorPoolCreateInfo p{};
        p.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        p.poolSizeCount = 1;
        p.pPoolSizes = &size;
        p.maxSets = 3;
        if (vkCreateDescriptorPool(device, &p, nullptr, &bloomPool) != VK_SUCCESS) return false;
        std::array<VkDescriptorSetLayout, 3> layouts{bloomSetLayout, bloomSetLayout, bloomSetLayout};
        VkDescriptorSet sets[3];
        VkDescriptorSetAllocateInfo a{};
        a.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        a.descriptorPool = bloomPool;
        a.descriptorSetCount = 3;
        a.pSetLayouts = layouts.data();
        if (vkAllocateDescriptorSets(device, &a, sets) != VK_SUCCESS) return false;
        bloomSetBright = sets[0];
        bloomSetH = sets[1];
        bloomSetV = sets[2];
    }
    updateBloomDescriptors();

    if (bloomPipeline == VK_NULL_HANDLE) {
        VkPushConstantRange pr{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(BloomPush)};
        VkPipelineLayoutCreateInfo pl{};
        pl.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pl.setLayoutCount = 1;
        pl.pSetLayouts = &bloomSetLayout;
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pr;
        if (vkCreatePipelineLayout(device, &pl, nullptr, &bloomPipelineLayout) != VK_SUCCESS)
            return false;

        VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
        VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/bloom.frag.spv");
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
        VkPipelineVertexInputStateCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        VkPipelineInputAssemblyStateCreateInfo ia{};
        ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{};
        vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{};
        raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{};
        ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{};
        ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        VkPipelineColorBlendAttachmentState ba{};
        ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo blend{};
        blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        blend.attachmentCount = 1;
        blend.pAttachments = &ba;
        std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{};
        dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dynamic.pDynamicStates = dyn.data();
        VkGraphicsPipelineCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        info.stageCount = 2;
        info.pStages = stages;
        info.pVertexInputState = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState = &vp;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &ms;
        info.pDepthStencilState = &ds;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = bloomPipelineLayout;
        info.renderPass = bloomRenderPass;
        info.subpass = 0;
        VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                    &bloomPipeline);
        vkDestroyShaderModule(device, vert, nullptr);
        vkDestroyShaderModule(device, frag, nullptr);
        if (result != VK_SUCCESS) {
            LOG_ERROR("[vulkan] bloom pipeline creation failed");
            return false;
        }
    }
    return true;
}

void VulkanRenderer::Impl::recordBloom(VkCommandBuffer cmd) {
    VkViewport vp{0, 0, float(bloomExtent.width), float(bloomExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, bloomExtent};
    float tx = 1.0f / float(bloomExtent.width);
    float ty = 1.0f / float(bloomExtent.height);

    auto pass = [&](VkFramebuffer fb, VkDescriptorSet src, BloomPush push) {
        VkRenderPassBeginInfo rp{};
        rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        rp.renderPass = bloomRenderPass;
        rp.framebuffer = fb;
        rp.renderArea.extent = bloomExtent;
        vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
        vkCmdSetViewport(cmd, 0, 1, &vp);
        vkCmdSetScissor(cmd, 0, 1, &sc);
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, bloomPipeline);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, bloomPipelineLayout, 0, 1,
                                &src, 0, nullptr);
        vkCmdPushConstants(cmd, bloomPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                           sizeof(BloomPush), &push);
        vkCmdDraw(cmd, 3, 1, 0, 0);
        vkCmdEndRenderPass(cmd);
    };

    // Bright-pass: HDR → bloomImage[0].
    BloomPush bright{tx, ty, 0.0f, 0.0f, bloomThreshold, bloomKnee, 1.0f, 0.0f};
    pass(bloomFramebuffer[0], bloomSetBright, bright);
    // Blur H: bloomImage[0] → bloomImage[1].
    BloomPush bh{tx, ty, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    pass(bloomFramebuffer[1], bloomSetH, bh);
    // Blur V: bloomImage[1] → bloomImage[0] (final, sampled by composite).
    BloomPush bv{tx, ty, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    pass(bloomFramebuffer[0], bloomSetV, bv);
}

void VulkanRenderer::Impl::updateSsaoDescriptors() {
    VkDescriptorImageInfo imgs[2]{};
    imgs[0].sampler = gbufferSampler;
    imgs[0].imageView = depthView;
    imgs[0].imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    imgs[1].sampler = gbufferSampler;
    imgs[1].imageView = normalView;
    imgs[1].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    // The blur set samples the raw AO target (aoView); recreated each swapchain.
    VkDescriptorImageInfo aoInfo{};
    aoInfo.sampler = gbufferSampler;
    aoInfo.imageView = aoView;
    aoInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    std::array<VkWriteDescriptorSet, 3> writes{};
    for (uint32_t i = 0; i < 2; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = gbufferSet;
        writes[i].dstBinding = i;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i].descriptorCount = 1;
        writes[i].pImageInfo = &imgs[i];
    }
    writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
    writes[2].dstSet = ssaoBlurSet;
    writes[2].dstBinding = 0;
    writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    writes[2].descriptorCount = 1;
    writes[2].pImageInfo = &aoInfo;
    vkUpdateDescriptorSets(device, 3, writes.data(), 0, nullptr);
}

bool VulkanRenderer::Impl::createSsaoResources() {
    aoExtent = {std::max(1u, swapchainExtent.width / 2),
                std::max(1u, swapchainExtent.height / 2)};
    if (!createColorTarget(aoExtent.width, aoExtent.height, VK_FORMAT_R8_UNORM,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                           aoImage, aoMemory, aoView))
        return false;
    if (!createColorTarget(aoExtent.width, aoExtent.height, VK_FORMAT_R8_UNORM,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                           aoBlurImage, aoBlurMemory, aoBlurView))
        return false;

    if (ssaoRenderPass == VK_NULL_HANDLE) {
        VkAttachmentDescription color{};
        color.format = VK_FORMAT_R8_UNORM;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &ref;
        std::array<VkSubpassDependency, 2> deps{};
        deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        deps[0].dstSubpass = 0;
        deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].srcSubpass = 0;
        deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo rp{};
        rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rp.attachmentCount = 1;
        rp.pAttachments = &color;
        rp.subpassCount = 1;
        rp.pSubpasses = &subpass;
        rp.dependencyCount = static_cast<uint32_t>(deps.size());
        rp.pDependencies = deps.data();
        if (vkCreateRenderPass(device, &rp, nullptr, &ssaoRenderPass) != VK_SUCCESS) return false;
    }

    VkFramebufferCreateInfo fb{};
    fb.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fb.renderPass = ssaoRenderPass;
    fb.attachmentCount = 1;
    fb.pAttachments = &aoView;
    fb.width = aoExtent.width;
    fb.height = aoExtent.height;
    fb.layers = 1;
    if (vkCreateFramebuffer(device, &fb, nullptr, &aoFramebuffer) != VK_SUCCESS) return false;
    fb.pAttachments = &aoBlurView;
    if (vkCreateFramebuffer(device, &fb, nullptr, &aoBlurFramebuffer) != VK_SUCCESS) return false;

    if (gbufferSampler == VK_NULL_HANDLE) {
        VkSamplerCreateInfo s{};
        s.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        s.magFilter = VK_FILTER_NEAREST;
        s.minFilter = VK_FILTER_NEAREST;
        s.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        s.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        s.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        s.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        if (vkCreateSampler(device, &s, nullptr, &gbufferSampler) != VK_SUCCESS) return false;
    }

    if (gbufferSetLayout == VK_NULL_HANDLE) {
        std::array<VkDescriptorSetLayoutBinding, 2> b{};   // depth, normal
        for (uint32_t i = 0; i < 2; ++i) {
            b[i].binding = i;
            b[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            b[i].descriptorCount = 1;
            b[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo l{};
        l.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        l.bindingCount = static_cast<uint32_t>(b.size());
        l.pBindings = b.data();
        if (vkCreateDescriptorSetLayout(device, &l, nullptr, &gbufferSetLayout) != VK_SUCCESS)
            return false;

        // Blur set layout: a single sampler over the raw AO target.
        VkDescriptorSetLayoutBinding bb{};
        bb.binding = 0;
        bb.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bb.descriptorCount = 1;
        bb.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        VkDescriptorSetLayoutCreateInfo bl{};
        bl.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        bl.bindingCount = 1;
        bl.pBindings = &bb;
        if (vkCreateDescriptorSetLayout(device, &bl, nullptr, &ssaoBlurSetLayout) != VK_SUCCESS)
            return false;

        // Pool feeds two sets: the G-buffer set (2 samplers) and the blur set (1).
        VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3};
        VkDescriptorPoolCreateInfo p{};
        p.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        p.poolSizeCount = 1;
        p.pPoolSizes = &size;
        p.maxSets = 2;
        if (vkCreateDescriptorPool(device, &p, nullptr, &ssaoPool) != VK_SUCCESS) return false;
        VkDescriptorSetAllocateInfo a{};
        a.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        a.descriptorPool = ssaoPool;
        a.descriptorSetCount = 1;
        a.pSetLayouts = &gbufferSetLayout;
        if (vkAllocateDescriptorSets(device, &a, &gbufferSet) != VK_SUCCESS) return false;
        a.pSetLayouts = &ssaoBlurSetLayout;
        if (vkAllocateDescriptorSets(device, &a, &ssaoBlurSet) != VK_SUCCESS) return false;
    }
    updateSsaoDescriptors();

    if (ssaoPipeline == VK_NULL_HANDLE) {
        VkPushConstantRange pr{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(SsaoPush)};
        std::array<VkDescriptorSetLayout, 2> setLayouts{descriptorSetLayout, gbufferSetLayout};
        VkPipelineLayoutCreateInfo pl{};
        pl.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pl.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
        pl.pSetLayouts = setLayouts.data();
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pr;
        if (vkCreatePipelineLayout(device, &pl, nullptr, &ssaoPipelineLayout) != VK_SUCCESS)
            return false;

        VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
        VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/ssao.frag.spv");
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
        VkPipelineVertexInputStateCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        VkPipelineInputAssemblyStateCreateInfo ia{};
        ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{};
        vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{};
        raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{};
        ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{};
        ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        VkPipelineColorBlendAttachmentState ba{};
        ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
        VkPipelineColorBlendStateCreateInfo blend{};
        blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        blend.attachmentCount = 1;
        blend.pAttachments = &ba;
        std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{};
        dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dynamic.pDynamicStates = dyn.data();
        VkGraphicsPipelineCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        info.stageCount = 2;
        info.pStages = stages;
        info.pVertexInputState = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState = &vp;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &ms;
        info.pDepthStencilState = &ds;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = ssaoPipelineLayout;
        info.renderPass = ssaoRenderPass;
        info.subpass = 0;
        VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                    &ssaoPipeline);
        vkDestroyShaderModule(device, vert, nullptr);
        vkDestroyShaderModule(device, frag, nullptr);
        if (result != VK_SUCCESS) {
            LOG_ERROR("[vulkan] SSAO pipeline creation failed");
            return false;
        }
    }

    if (ssaoBlurPipeline == VK_NULL_HANDLE) {
        VkPushConstantRange pr{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(SsaoBlurPush)};
        VkPipelineLayoutCreateInfo pl{};
        pl.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pl.setLayoutCount = 1;
        pl.pSetLayouts = &ssaoBlurSetLayout;
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pr;
        if (vkCreatePipelineLayout(device, &pl, nullptr, &ssaoBlurPipelineLayout) != VK_SUCCESS)
            return false;

        VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
        VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/ssao_blur.frag.spv");
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
        VkPipelineVertexInputStateCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        VkPipelineInputAssemblyStateCreateInfo ia{};
        ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{};
        vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{};
        raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{};
        ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{};
        ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        VkPipelineColorBlendAttachmentState ba{};
        ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT;
        VkPipelineColorBlendStateCreateInfo blend{};
        blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        blend.attachmentCount = 1;
        blend.pAttachments = &ba;
        std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{};
        dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dynamic.pDynamicStates = dyn.data();
        VkGraphicsPipelineCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        info.stageCount = 2;
        info.pStages = stages;
        info.pVertexInputState = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState = &vp;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &ms;
        info.pDepthStencilState = &ds;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = ssaoBlurPipelineLayout;
        info.renderPass = ssaoRenderPass;
        info.subpass = 0;
        VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                    &ssaoBlurPipeline);
        vkDestroyShaderModule(device, vert, nullptr);
        vkDestroyShaderModule(device, frag, nullptr);
        if (result != VK_SUCCESS) {
            LOG_ERROR("[vulkan] SSAO blur pipeline creation failed");
            return false;
        }
    }
    return true;
}

void VulkanRenderer::Impl::recordSsao(VkCommandBuffer cmd) {
    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = ssaoRenderPass;
    rp.framebuffer = aoFramebuffer;
    rp.renderArea.extent = aoExtent;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
    VkViewport vp{0, 0, float(aoExtent.width), float(aoExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, aoExtent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ssaoPipeline);
    std::array<VkDescriptorSet, 2> sets{descriptorSets[currentFrame], gbufferSet};
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ssaoPipelineLayout, 0,
                            static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);
    SsaoPush push{ssaoRadius, ssaoBias, ssaoIntensity, 0.0f};
    vkCmdPushConstants(cmd, ssaoPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                       sizeof(SsaoPush), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);

    // Blur pass: raw AO (aoImage) → aoBlurImage. The render pass external
    // dependencies serialise the color write above with this fragment read.
    VkRenderPassBeginInfo brp{};
    brp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    brp.renderPass = ssaoRenderPass;
    brp.framebuffer = aoBlurFramebuffer;
    brp.renderArea.extent = aoExtent;
    vkCmdBeginRenderPass(cmd, &brp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ssaoBlurPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ssaoBlurPipelineLayout, 0,
                            1, &ssaoBlurSet, 0, nullptr);
    SsaoBlurPush bpush{1.0f / float(aoExtent.width), 1.0f / float(aoExtent.height)};
    vkCmdPushConstants(cmd, ssaoBlurPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                       sizeof(SsaoBlurPush), &bpush);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
}

void VulkanRenderer::Impl::updateSsrDescriptors() {
    VkDescriptorImageInfo imgs[3]{};
    imgs[0].sampler = gbufferSampler;
    imgs[0].imageView = depthView;
    imgs[0].imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    imgs[1].sampler = gbufferSampler;
    imgs[1].imageView = normalView;
    imgs[1].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    // createCompositeResources (which owns this sampler) runs after createSsrResources
    // — SSR feeds the composite — so create it on first use here, else this write
    // binds a null sampler (VUID-…-00325). Mirrors the lazy gbuffer/bloom samplers.
    if (compositeSampler == VK_NULL_HANDLE) {
        VkSamplerCreateInfo samp{};
        samp.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samp.magFilter = VK_FILTER_LINEAR;
        samp.minFilter = VK_FILTER_LINEAR;
        samp.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
        samp.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samp.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samp.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        vkCreateSampler(device, &samp, nullptr, &compositeSampler);
    }
    imgs[2].sampler = compositeSampler;   // linear, for the HDR color fetch
    imgs[2].imageView = hdrView;
    imgs[2].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    std::array<VkWriteDescriptorSet, 3> writes{};
    for (uint32_t i = 0; i < 3; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = ssrSet;
        writes[i].dstBinding = i;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i].descriptorCount = 1;
        writes[i].pImageInfo = &imgs[i];
    }
    vkUpdateDescriptorSets(device, 3, writes.data(), 0, nullptr);
}

bool VulkanRenderer::Impl::createSsrResources() {
    ssrExtent = {std::max(1u, swapchainExtent.width / 2),
                 std::max(1u, swapchainExtent.height / 2)};
    if (!createColorTarget(ssrExtent.width, ssrExtent.height, kHdrFormat,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                           ssrImage, ssrMemory, ssrView))
        return false;

    if (ssrRenderPass == VK_NULL_HANDLE) {
        VkAttachmentDescription color{};
        color.format = kHdrFormat;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &ref;
        std::array<VkSubpassDependency, 2> deps{};
        deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        deps[0].dstSubpass = 0;
        deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].srcSubpass = 0;
        deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo rp{};
        rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rp.attachmentCount = 1;
        rp.pAttachments = &color;
        rp.subpassCount = 1;
        rp.pSubpasses = &subpass;
        rp.dependencyCount = static_cast<uint32_t>(deps.size());
        rp.pDependencies = deps.data();
        if (vkCreateRenderPass(device, &rp, nullptr, &ssrRenderPass) != VK_SUCCESS) return false;
    }

    VkFramebufferCreateInfo fb{};
    fb.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fb.renderPass = ssrRenderPass;
    fb.attachmentCount = 1;
    fb.pAttachments = &ssrView;
    fb.width = ssrExtent.width;
    fb.height = ssrExtent.height;
    fb.layers = 1;
    if (vkCreateFramebuffer(device, &fb, nullptr, &ssrFramebuffer) != VK_SUCCESS) return false;

    if (ssrSetLayout == VK_NULL_HANDLE) {
        std::array<VkDescriptorSetLayoutBinding, 3> b{};   // depth, normal, hdr
        for (uint32_t i = 0; i < 3; ++i) {
            b[i].binding = i;
            b[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            b[i].descriptorCount = 1;
            b[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo l{};
        l.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        l.bindingCount = static_cast<uint32_t>(b.size());
        l.pBindings = b.data();
        if (vkCreateDescriptorSetLayout(device, &l, nullptr, &ssrSetLayout) != VK_SUCCESS)
            return false;
        VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 3};
        VkDescriptorPoolCreateInfo p{};
        p.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        p.poolSizeCount = 1;
        p.pPoolSizes = &size;
        p.maxSets = 1;
        if (vkCreateDescriptorPool(device, &p, nullptr, &ssrPool) != VK_SUCCESS) return false;
        VkDescriptorSetAllocateInfo a{};
        a.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        a.descriptorPool = ssrPool;
        a.descriptorSetCount = 1;
        a.pSetLayouts = &ssrSetLayout;
        if (vkAllocateDescriptorSets(device, &a, &ssrSet) != VK_SUCCESS) return false;
    }
    updateSsrDescriptors();

    if (ssrPipeline == VK_NULL_HANDLE) {
        VkPushConstantRange pr{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(SsrPush)};
        std::array<VkDescriptorSetLayout, 2> setLayouts{descriptorSetLayout, ssrSetLayout};
        VkPipelineLayoutCreateInfo pl{};
        pl.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pl.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
        pl.pSetLayouts = setLayouts.data();
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pr;
        if (vkCreatePipelineLayout(device, &pl, nullptr, &ssrPipelineLayout) != VK_SUCCESS)
            return false;

        VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
        VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/ssr.frag.spv");
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
        VkPipelineVertexInputStateCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        VkPipelineInputAssemblyStateCreateInfo ia{};
        ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{};
        vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{};
        raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{};
        ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{};
        ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        VkPipelineColorBlendAttachmentState ba{};
        ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo blend{};
        blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        blend.attachmentCount = 1;
        blend.pAttachments = &ba;
        std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{};
        dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dynamic.pDynamicStates = dyn.data();
        VkGraphicsPipelineCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        info.stageCount = 2;
        info.pStages = stages;
        info.pVertexInputState = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState = &vp;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &ms;
        info.pDepthStencilState = &ds;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = ssrPipelineLayout;
        info.renderPass = ssrRenderPass;
        info.subpass = 0;
        VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                    &ssrPipeline);
        vkDestroyShaderModule(device, vert, nullptr);
        vkDestroyShaderModule(device, frag, nullptr);
        if (result != VK_SUCCESS) {
            LOG_ERROR("[vulkan] SSR pipeline creation failed");
            return false;
        }
    }
    return true;
}

void VulkanRenderer::Impl::recordSsr(VkCommandBuffer cmd) {
    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = ssrRenderPass;
    rp.framebuffer = ssrFramebuffer;
    rp.renderArea.extent = ssrExtent;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
    VkViewport vp{0, 0, float(ssrExtent.width), float(ssrExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, ssrExtent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ssrPipeline);
    std::array<VkDescriptorSet, 2> sets{descriptorSets[currentFrame], ssrSet};
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, ssrPipelineLayout, 0,
                            static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);
    SsrPush push{ssrMaxRayDist, ssrThickness, ssrMaxRoughness, ssrBlendStrength};
    vkCmdPushConstants(cmd, ssrPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                       sizeof(SsrPush), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
}

void VulkanRenderer::Impl::updateDofDescriptors() {
    // The composite sampler (linear, clamp) is created lazily by SSR/composite;
    // it exists by the time DOF resources are built (DOF is created after SSR).
    VkDescriptorImageInfo imgs[2]{};
    imgs[0].sampler = compositeSampler;
    imgs[0].imageView = hdrView;
    imgs[0].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    imgs[1].sampler = gbufferSampler;
    imgs[1].imageView = depthView;
    imgs[1].imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    std::array<VkWriteDescriptorSet, 2> writes{};
    for (uint32_t i = 0; i < 2; ++i) {
        writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[i].dstSet = dofSet;
        writes[i].dstBinding = i;
        writes[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[i].descriptorCount = 1;
        writes[i].pImageInfo = &imgs[i];
    }
    vkUpdateDescriptorSets(device, 2, writes.data(), 0, nullptr);
}

bool VulkanRenderer::Impl::createDofResources() {
    dofExtent = swapchainExtent;   // full-res
    if (!createColorTarget(dofExtent.width, dofExtent.height, kHdrFormat,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                           dofImage, dofMemory, dofView))
        return false;

    // The composite statically samples dofView every frame, but the DOF pass
    // (which leaves it SHADER_READ) only runs when enabled. Put it in a valid
    // sampled layout up front so the DOF-off frames don't read an UNDEFINED image.
    {
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
        VkImageMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        b.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = dofImage;
        b.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        b.subresourceRange.levelCount = 1;
        b.subresourceRange.layerCount = 1;
        b.srcAccessMask = 0;
        b.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
        vkEndCommandBuffer(cmd);
        VkSubmitInfo submit{};
        submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submit.commandBufferCount = 1;
        submit.pCommandBuffers = &cmd;
        vkQueueSubmit(graphicsQueue, 1, &submit, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);
        vkFreeCommandBuffers(device, commandPool, 1, &cmd);
    }

    if (dofRenderPass == VK_NULL_HANDLE) {
        VkAttachmentDescription color{};
        color.format = kHdrFormat;
        color.samples = VK_SAMPLE_COUNT_1_BIT;
        color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
        color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
        VkSubpassDescription subpass{};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 1;
        subpass.pColorAttachments = &ref;
        std::array<VkSubpassDependency, 2> deps{};
        deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        deps[0].dstSubpass = 0;
        deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[0].dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        deps[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].srcSubpass = 0;
        deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        deps[1].srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
        deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        deps[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
        deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        VkRenderPassCreateInfo rp{};
        rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rp.attachmentCount = 1;
        rp.pAttachments = &color;
        rp.subpassCount = 1;
        rp.pSubpasses = &subpass;
        rp.dependencyCount = static_cast<uint32_t>(deps.size());
        rp.pDependencies = deps.data();
        if (vkCreateRenderPass(device, &rp, nullptr, &dofRenderPass) != VK_SUCCESS) return false;
    }

    VkFramebufferCreateInfo fb{};
    fb.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fb.renderPass = dofRenderPass;
    fb.attachmentCount = 1;
    fb.pAttachments = &dofView;
    fb.width = dofExtent.width;
    fb.height = dofExtent.height;
    fb.layers = 1;
    if (vkCreateFramebuffer(device, &fb, nullptr, &dofFramebuffer) != VK_SUCCESS) return false;

    if (dofSetLayout == VK_NULL_HANDLE) {
        std::array<VkDescriptorSetLayoutBinding, 2> b{};   // hdr, depth
        for (uint32_t i = 0; i < 2; ++i) {
            b[i].binding = i;
            b[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
            b[i].descriptorCount = 1;
            b[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
        }
        VkDescriptorSetLayoutCreateInfo l{};
        l.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        l.bindingCount = static_cast<uint32_t>(b.size());
        l.pBindings = b.data();
        if (vkCreateDescriptorSetLayout(device, &l, nullptr, &dofSetLayout) != VK_SUCCESS)
            return false;
        VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 2};
        VkDescriptorPoolCreateInfo p{};
        p.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        p.poolSizeCount = 1;
        p.pPoolSizes = &size;
        p.maxSets = 1;
        if (vkCreateDescriptorPool(device, &p, nullptr, &dofPool) != VK_SUCCESS) return false;
        VkDescriptorSetAllocateInfo a{};
        a.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        a.descriptorPool = dofPool;
        a.descriptorSetCount = 1;
        a.pSetLayouts = &dofSetLayout;
        if (vkAllocateDescriptorSets(device, &a, &dofSet) != VK_SUCCESS) return false;
    }
    updateDofDescriptors();

    if (dofPipeline == VK_NULL_HANDLE) {
        VkPushConstantRange pr{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(DofPush)};
        std::array<VkDescriptorSetLayout, 2> setLayouts{descriptorSetLayout, dofSetLayout};
        VkPipelineLayoutCreateInfo pl{};
        pl.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pl.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
        pl.pSetLayouts = setLayouts.data();
        pl.pushConstantRangeCount = 1;
        pl.pPushConstantRanges = &pr;
        if (vkCreatePipelineLayout(device, &pl, nullptr, &dofPipelineLayout) != VK_SUCCESS)
            return false;

        VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
        VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/dof.frag.spv");
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
        VkPipelineVertexInputStateCreateInfo vi{};
        vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        VkPipelineInputAssemblyStateCreateInfo ia{};
        ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        VkPipelineViewportStateCreateInfo vp{};
        vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        vp.viewportCount = 1;
        vp.scissorCount = 1;
        VkPipelineRasterizationStateCreateInfo raster{};
        raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        raster.polygonMode = VK_POLYGON_MODE_FILL;
        raster.cullMode = VK_CULL_MODE_NONE;
        raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
        raster.lineWidth = 1.0f;
        VkPipelineMultisampleStateCreateInfo ms{};
        ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        VkPipelineDepthStencilStateCreateInfo ds{};
        ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        VkPipelineColorBlendAttachmentState ba{};
        ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                            VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        VkPipelineColorBlendStateCreateInfo blend{};
        blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        blend.attachmentCount = 1;
        blend.pAttachments = &ba;
        std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
        VkPipelineDynamicStateCreateInfo dynamic{};
        dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
        dynamic.pDynamicStates = dyn.data();
        VkGraphicsPipelineCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        info.stageCount = 2;
        info.pStages = stages;
        info.pVertexInputState = &vi;
        info.pInputAssemblyState = &ia;
        info.pViewportState = &vp;
        info.pRasterizationState = &raster;
        info.pMultisampleState = &ms;
        info.pDepthStencilState = &ds;
        info.pColorBlendState = &blend;
        info.pDynamicState = &dynamic;
        info.layout = dofPipelineLayout;
        info.renderPass = dofRenderPass;
        info.subpass = 0;
        VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                    &dofPipeline);
        vkDestroyShaderModule(device, vert, nullptr);
        vkDestroyShaderModule(device, frag, nullptr);
        if (result != VK_SUCCESS) {
            LOG_ERROR("[vulkan] DOF pipeline creation failed");
            return false;
        }
    }
    return true;
}

void VulkanRenderer::Impl::recordDof(VkCommandBuffer cmd) {
    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = dofRenderPass;
    rp.framebuffer = dofFramebuffer;
    rp.renderArea.extent = dofExtent;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);
    VkViewport vp{0, 0, float(dofExtent.width), float(dofExtent.height), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, dofExtent};
    vkCmdSetViewport(cmd, 0, 1, &vp);
    vkCmdSetScissor(cmd, 0, 1, &sc);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, dofPipeline);
    std::array<VkDescriptorSet, 2> sets{descriptorSets[currentFrame], dofSet};
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, dofPipelineLayout, 0,
                            static_cast<uint32_t>(sets.size()), sets.data(), 0, nullptr);
    DofPush push{dofFocusDistance, dofFocalLength, dofAperture,
                 dofExtent.height * 1000.0f / dofSensorHeight, 16.0f,
                 1.0f / float(dofExtent.width), 1.0f / float(dofExtent.height), 0.0f};
    vkCmdPushConstants(cmd, dofPipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT, 0,
                       sizeof(DofPush), &push);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
}

bool VulkanRenderer::Impl::createCompositeResources() {
    VkSamplerCreateInfo samp{};
    samp.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    samp.magFilter = VK_FILTER_LINEAR;
    samp.minFilter = VK_FILTER_LINEAR;
    samp.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    samp.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samp.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    samp.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    if (compositeSampler == VK_NULL_HANDLE &&
        vkCreateSampler(device, &samp, nullptr, &compositeSampler) != VK_SUCCESS) return false;

    // 0 HDR, 1 bloom, 2 AO, 3 SSR, 4 depth, 5 normals (4,5 for debug views),
    // 6 DOF (the blurred scene, read instead of HDR when dofEnabled).
    std::array<VkDescriptorSetLayoutBinding, 7> bindings{};
    for (uint32_t i = 0; i < 7; ++i) {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layout{};
    layout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layout.bindingCount = static_cast<uint32_t>(bindings.size());
    layout.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(device, &layout, nullptr, &compositeSetLayout) != VK_SUCCESS)
        return false;

    VkDescriptorPoolSize size{VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 7};
    VkDescriptorPoolCreateInfo pool{};
    pool.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    pool.poolSizeCount = 1;
    pool.pPoolSizes = &size;
    pool.maxSets = 1;
    if (vkCreateDescriptorPool(device, &pool, nullptr, &compositePool) != VK_SUCCESS) return false;

    VkDescriptorSetAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    alloc.descriptorPool = compositePool;
    alloc.descriptorSetCount = 1;
    alloc.pSetLayouts = &compositeSetLayout;
    if (vkAllocateDescriptorSets(device, &alloc, &compositeSet) != VK_SUCCESS) return false;
    updateCompositeDescriptor();

    VkPushConstantRange pushRange{VK_SHADER_STAGE_FRAGMENT_BIT, 0, sizeof(CompositePush)};
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = 1;
    layoutInfo.pSetLayouts = &compositeSetLayout;
    layoutInfo.pushConstantRangeCount = 1;
    layoutInfo.pPushConstantRanges = &pushRange;
    if (vkCreatePipelineLayout(device, &layoutInfo, nullptr, &compositePipelineLayout) != VK_SUCCESS)
        return false;

    VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
    VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.frag.spv");
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

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkPipelineViewportStateCreateInfo vp{};
    vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.scissorCount = 1;
    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
    raster.lineWidth = 1.0f;
    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo ds{};
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    VkPipelineColorBlendAttachmentState ba{};
    ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                        VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = 1;
    blend.pAttachments = &ba;
    std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{};
    dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
    dynamic.pDynamicStates = dyn.data();
    VkGraphicsPipelineCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    info.stageCount = 2;
    info.pStages = stages;
    info.pVertexInputState = &vertexInput;
    info.pInputAssemblyState = &ia;
    info.pViewportState = &vp;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &ms;
    info.pDepthStencilState = &ds;
    info.pColorBlendState = &blend;
    info.pDynamicState = &dynamic;
    info.layout = compositePipelineLayout;
    info.renderPass = compositeRenderPass;
    info.subpass = 0;
    VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                &compositePipeline);
    vkDestroyShaderModule(device, vert, nullptr);
    vkDestroyShaderModule(device, frag, nullptr);
    if (result != VK_SUCCESS) {
        LOG_ERROR("[vulkan] composite pipeline creation failed");
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
    std::array<VkDescriptorSetLayoutBinding, 4> bindings{};
    bindings[0].binding = 0;   // global UBO
    bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    bindings[0].descriptorCount = 1;
    bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[1].binding = 1;   // cascaded shadow map (sampler2DArrayShadow)
    bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[1].descriptorCount = 1;
    bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[2].binding = 2;   // HDR environment equirect (skybox + IBL)
    bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[2].descriptorCount = 1;
    bindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    bindings[3].binding = 3;   // split-sum BRDF integration LUT
    bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
    bindings[3].descriptorCount = 1;
    bindings[3].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    VkDescriptorSetLayoutCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    info.bindingCount = static_cast<uint32_t>(bindings.size());
    info.pBindings = bindings.data();
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
    std::array<VkDescriptorPoolSize, 2> sizes{};
    sizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
    sizes[0].descriptorCount = MAX_FRAMES_IN_FLIGHT;
    sizes[1].type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;   // shadow map + env + BRDF LUT
    sizes[1].descriptorCount = MAX_FRAMES_IN_FLIGHT * 3;
    VkDescriptorPoolCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    info.poolSizeCount = static_cast<uint32_t>(sizes.size());
    info.pPoolSizes = sizes.data();
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
        VkDescriptorImageInfo shadow{};
        shadow.sampler = shadowSampler;
        shadow.imageView = shadowArrayView;
        shadow.imageLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
        VkDescriptorImageInfo lut{};
        lut.sampler = brdfLutSampler;
        lut.imageView = brdfLutView;
        lut.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        std::array<VkWriteDescriptorSet, 3> writes{};
        writes[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[0].dstSet = descriptorSets[i];
        writes[0].dstBinding = 0;
        writes[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
        writes[0].descriptorCount = 1;
        writes[0].pBufferInfo = &buf;
        writes[1].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[1].dstSet = descriptorSets[i];
        writes[1].dstBinding = 1;
        writes[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[1].descriptorCount = 1;
        writes[1].pImageInfo = &shadow;
        writes[2].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        writes[2].dstSet = descriptorSets[i];
        writes[2].dstBinding = 3;   // BRDF LUT (binding 2 = env, written later)
        writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        writes[2].descriptorCount = 1;
        writes[2].pImageInfo = &lut;
        vkUpdateDescriptorSets(device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
    }

    // Env equirect sampler (wrap-u/clamp-v, linear + mips for roughness LOD).
    VkSamplerCreateInfo s{};
    s.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    s.magFilter = VK_FILTER_LINEAR;
    s.minFilter = VK_FILTER_LINEAR;
    s.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    s.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    s.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    s.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    s.maxLod = VK_LOD_CLAMP_NONE;
    if (vkCreateSampler(device, &s, nullptr, &envSampler) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] env sampler creation failed");
        return false;
    }
    // Binding 2 (env) is written after createMaterialResources, once the default
    // texture exists (see initialize()).
    return true;
}

void VulkanRenderer::Impl::updateGlobalEnvDescriptor() {
    VkImageView view = envView ? envView : defaultTexture.view;
    if (!view) return;
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        VkDescriptorImageInfo img{};
        img.sampler = envSampler;
        img.imageView = view;
        img.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstSet = descriptorSets[i];
        write.dstBinding = 2;
        write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        write.descriptorCount = 1;
        write.pImageInfo = &img;
        vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
    }
}

bool VulkanRenderer::Impl::createBrdfLut() {
    const uint32_t SIZE = 256;
    const VkFormat fmt = VK_FORMAT_R16G16_SFLOAT;
    if (!createColorTarget(SIZE, SIZE, fmt,
                           VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                           brdfLutImage, brdfLutMemory, brdfLutView))
        return false;

    VkSamplerCreateInfo ss{};
    ss.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    ss.magFilter = VK_FILTER_LINEAR;
    ss.minFilter = VK_FILTER_LINEAR;
    ss.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    ss.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    ss.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    ss.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    if (vkCreateSampler(device, &ss, nullptr, &brdfLutSampler) != VK_SUCCESS) return false;

    // One-time bake: a fullscreen pass integrating the split-sum BRDF (no inputs).
    VkAttachmentDescription color{};
    color.format = fmt;
    color.samples = VK_SAMPLE_COUNT_1_BIT;
    color.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    color.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    color.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    color.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    color.finalLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
    VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &ref;
    VkRenderPassCreateInfo rpci{};
    rpci.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rpci.attachmentCount = 1;
    rpci.pAttachments = &color;
    rpci.subpassCount = 1;
    rpci.pSubpasses = &subpass;
    VkRenderPass rp = VK_NULL_HANDLE;
    if (vkCreateRenderPass(device, &rpci, nullptr, &rp) != VK_SUCCESS) return false;

    VkFramebufferCreateInfo fbci{};
    fbci.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
    fbci.renderPass = rp;
    fbci.attachmentCount = 1;
    fbci.pAttachments = &brdfLutView;
    fbci.width = SIZE;
    fbci.height = SIZE;
    fbci.layers = 1;
    VkFramebuffer fb = VK_NULL_HANDLE;
    if (vkCreateFramebuffer(device, &fbci, nullptr, &fb) != VK_SUCCESS) return false;

    VkPipelineLayoutCreateInfo plci{};
    plci.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;   // no sets, no push
    VkPipelineLayout pl = VK_NULL_HANDLE;
    if (vkCreatePipelineLayout(device, &plci, nullptr, &pl) != VK_SUCCESS) return false;

    VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/composite.vert.spv");
    VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/brdf_lut.frag.spv");
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
    VkPipelineVertexInputStateCreateInfo vi{};
    vi.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
    VkViewport vpr{0, 0, float(SIZE), float(SIZE), 0.0f, 1.0f};
    VkRect2D sc{{0, 0}, {SIZE, SIZE}};
    VkPipelineViewportStateCreateInfo vp{};
    vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.pViewports = &vpr;
    vp.scissorCount = 1;
    vp.pScissors = &sc;
    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
    raster.lineWidth = 1.0f;
    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    VkPipelineDepthStencilStateCreateInfo ds{};
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    VkPipelineColorBlendAttachmentState ba{};
    ba.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT;
    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = 1;
    blend.pAttachments = &ba;
    VkGraphicsPipelineCreateInfo gpi{};
    gpi.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    gpi.stageCount = 2;
    gpi.pStages = stages;
    gpi.pVertexInputState = &vi;
    gpi.pInputAssemblyState = &ia;
    gpi.pViewportState = &vp;
    gpi.pRasterizationState = &raster;
    gpi.pMultisampleState = &ms;
    gpi.pDepthStencilState = &ds;
    gpi.pColorBlendState = &blend;
    gpi.layout = pl;
    gpi.renderPass = rp;
    VkPipeline pipe = VK_NULL_HANDLE;
    VkResult pr = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &gpi, nullptr, &pipe);
    vkDestroyShaderModule(device, vert, nullptr);
    vkDestroyShaderModule(device, frag, nullptr);
    if (pr != VK_SUCCESS) { LOG_ERROR("[vulkan] BRDF LUT pipeline creation failed"); return false; }

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
    VkRenderPassBeginInfo rpb{};
    rpb.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rpb.renderPass = rp;
    rpb.framebuffer = fb;
    rpb.renderArea.extent = {SIZE, SIZE};
    vkCmdBeginRenderPass(cmd, &rpb, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
    vkCmdDraw(cmd, 3, 1, 0, 0);
    vkCmdEndRenderPass(cmd);
    vkEndCommandBuffer(cmd);
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    vkQueueSubmit(graphicsQueue, 1, &submit, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);
    vkFreeCommandBuffers(device, commandPool, 1, &cmd);

    vkDestroyPipeline(device, pipe, nullptr);
    vkDestroyPipelineLayout(device, pl, nullptr);
    vkDestroyFramebuffer(device, fb, nullptr);
    vkDestroyRenderPass(device, rp, nullptr);
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

    std::array<VkDescriptorSetLayout, 2> setLayouts{descriptorSetLayout, materialSetLayout};
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
    layoutInfo.pSetLayouts = setLayouts.data();
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
    depthStencil.depthCompareOp = VK_COMPARE_OP_GREATER_OR_EQUAL;   // reverse-Z (near=1, far=0)

    // Two attachments (HDR color + world-normal G-buffer), both opaque writes.
    VkPipelineColorBlendAttachmentState blendAttachment{};
    blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                     VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blendAttachment.blendEnable = VK_FALSE;
    std::array<VkPipelineColorBlendAttachmentState, 2> blendAttachments{blendAttachment, blendAttachment};
    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = static_cast<uint32_t>(blendAttachments.size());
    blend.pAttachments = blendAttachments.data();

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

    // Wireframe variant (Renderer::wireframe debug view): same layout/state as the
    // mesh pipeline but LINE polygon mode and a flat-color fragment shader. Used
    // for both wireframe-only (mode 1, drawn in place of the fills) and overlay
    // (mode 2, re-drawn on top — LESS_OR_EQUAL + depth write lets coincident edges
    // pass). Needs the fillModeNonSolid device feature (enabled above).
    VkShaderModule wvert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.vert.spv");
    VkShaderModule wfrag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh_wire.frag.spv");
    if (!wvert || !wfrag) return false;
    stages[0].module = wvert;
    stages[1].module = wfrag;
    raster.polygonMode = VK_POLYGON_MODE_LINE;
    VkResult wresult = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &wirePipeline);
    vkDestroyShaderModule(device, wvert, nullptr);
    vkDestroyShaderModule(device, wfrag, nullptr);
    if (wresult != VK_SUCCESS) {
        LOG_ERROR("[vulkan] wireframe pipeline creation failed");
        return false;
    }

    // Transparent variant (material opacity < 1): the same mesh shaders, but alpha
    // blend onto the HDR attachment, no depth write, and mask the normal G-buffer
    // (attachment 1) so SSAO/SSR keep the opaque surface seen through the glass.
    // Needs independentBlend (already enabled). Drawn back-to-front after opaque.
    VkShaderModule tvert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.vert.spv");
    VkShaderModule tfrag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.frag.spv");
    if (!tvert || !tfrag) return false;
    stages[0].module = tvert;
    stages[1].module = tfrag;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    depthStencil.depthWriteEnable = VK_FALSE;
    blendAttachments[0].blendEnable = VK_TRUE;
    blendAttachments[0].srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
    blendAttachments[0].dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blendAttachments[0].colorBlendOp = VK_BLEND_OP_ADD;
    blendAttachments[0].srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
    blendAttachments[0].dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
    blendAttachments[0].alphaBlendOp = VK_BLEND_OP_ADD;
    blendAttachments[1].colorWriteMask = 0;   // don't write the normal G-buffer
    VkResult tresult = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                 &transparentPipeline);
    vkDestroyShaderModule(device, tvert, nullptr);
    vkDestroyShaderModule(device, tfrag, nullptr);
    if (tresult != VK_SUCCESS) {
        LOG_ERROR("[vulkan] transparent pipeline creation failed");
        return false;
    }

    // CDLOD terrain variant: opaque (reset the transparent state above) but with
    // terrain.vert, which morphs each vertex toward its coarser-LOD position by
    // camera distance. Same layout + mesh.frag shading.
    VkShaderModule cvert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/terrain.vert.spv");
    VkShaderModule cfrag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.frag.spv");
    if (!cvert || !cfrag) return false;
    stages[0].module = cvert;
    stages[1].module = cfrag;
    depthStencil.depthWriteEnable = VK_TRUE;
    blendAttachments[0].blendEnable = VK_FALSE;
    blendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blendAttachments[1].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    VkResult cresult = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                 &terrainPipeline);
    vkDestroyShaderModule(device, cvert, nullptr);
    vkDestroyShaderModule(device, cfrag, nullptr);
    if (cresult != VK_SUCCESS) {
        LOG_ERROR("[vulkan] terrain pipeline creation failed");
        return false;
    }

    // Overlay variant (RenderMaterial::FLAG_OVERLAY — debug gizmos, ADR-0061): the
    // same opaque mesh shaders, but depth test AND write are off so the geometry
    // always draws over everything already in the frame (parity with the Metal
    // depthStateOverlay: Always-compare, no write). Drawn last, after opaque /
    // terrain / transparent. The terrain block above already reset color/blend to
    // the clean opaque state, so we only flip the depth flags here.
    VkShaderModule overt = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.vert.spv");
    VkShaderModule ofrag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh.frag.spv");
    if (!overt || !ofrag) return false;
    stages[0].module = overt;
    stages[1].module = ofrag;
    depthStencil.depthTestEnable = VK_FALSE;
    depthStencil.depthWriteEnable = VK_FALSE;
    VkResult oresult = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr,
                                                 &overlayPipeline);
    vkDestroyShaderModule(device, overt, nullptr);
    vkDestroyShaderModule(device, ofrag, nullptr);
    if (oresult != VK_SUCCESS) {
        LOG_ERROR("[vulkan] overlay pipeline creation failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createSkyPipeline() {
    // Fullscreen-triangle skybox; reads only the global UBO (set 0), no push, no
    // vertex buffer. Drawn first in the main pass with depth test/write off so it
    // fills the background; geometry then overwrites with depth.
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.setLayoutCount = 1;
    layoutInfo.pSetLayouts = &descriptorSetLayout;
    if (vkCreatePipelineLayout(device, &layoutInfo, nullptr, &skyPipelineLayout) != VK_SUCCESS)
        return false;

    VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/sky.vert.spv");
    VkShaderModule frag = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/sky.frag.spv");
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

    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{};
    vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
    raster.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{};
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    ds.depthTestEnable = VK_FALSE;
    ds.depthWriteEnable = VK_FALSE;

    // Scene pass has 2 color attachments; the sky writes color (0), not the
    // normal G-buffer (1). SSAO ignores sky pixels via depth==far.
    std::array<VkPipelineColorBlendAttachmentState, 2> blendAttachments{};
    blendAttachments[0].colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                         VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
    blendAttachments[1].colorWriteMask = 0;
    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = static_cast<uint32_t>(blendAttachments.size());
    blend.pAttachments = blendAttachments.data();

    std::array<VkDynamicState, 2> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamic{};
    dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
    dynamic.pDynamicStates = dyn.data();

    VkGraphicsPipelineCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    info.stageCount = 2;
    info.pStages = stages;
    info.pVertexInputState = &vertexInput;
    info.pInputAssemblyState = &ia;
    info.pViewportState = &vp;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &ms;
    info.pDepthStencilState = &ds;
    info.pColorBlendState = &blend;
    info.pDynamicState = &dynamic;
    info.layout = skyPipelineLayout;
    info.renderPass = renderPass;
    info.subpass = 0;
    VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &skyPipeline);
    vkDestroyShaderModule(device, vert, nullptr);
    vkDestroyShaderModule(device, frag, nullptr);
    if (result != VK_SUCCESS) {
        LOG_ERROR("[vulkan] sky pipeline creation failed");
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

void VulkanRenderer::Impl::transitionImageLayout(VkCommandBuffer cmd, VkImage image,
                                                 VkImageLayout from, VkImageLayout to) {
    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = from;
    barrier.newLayout = to;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = image;
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.layerCount = 1;

    VkPipelineStageFlags srcStage, dstStage;
    if (from == VK_IMAGE_LAYOUT_UNDEFINED && to == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
        barrier.srcAccessMask = 0;
        barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        srcStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
        dstStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    } else {  // TRANSFER_DST -> SHADER_READ_ONLY
        barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        srcStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        dstStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    }
    vkCmdPipelineBarrier(cmd, srcStage, dstStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);
}

bool VulkanRenderer::Impl::createImageRGBA8(const uint8_t* rgba, uint32_t w, uint32_t h,
                                            GpuTexture& out) {
    VkDeviceSize size = static_cast<VkDeviceSize>(w) * h * 4;
    VkBuffer staging = VK_NULL_HANDLE;
    VkDeviceMemory stagingMem = VK_NULL_HANDLE;
    if (!createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                      staging, stagingMem))
        return false;
    void* mapped = nullptr;
    vkMapMemory(device, stagingMem, 0, size, 0, &mapped);
    std::memcpy(mapped, rgba, static_cast<size_t>(size));
    vkUnmapMemory(device, stagingMem);

    // Full mip chain: floor(log2(max(w,h)))+1 levels, generated by blit downsample
    // (parity with Metal's generateMipmaps; kills minification aliasing/shimmer).
    uint32_t mipLevels = 1;
    for (uint32_t d = std::max(w, h); d > 1; d >>= 1) ++mipLevels;

    VkImageCreateInfo image{};
    image.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image.imageType = VK_IMAGE_TYPE_2D;
    image.extent = {w, h, 1};
    image.mipLevels = mipLevels;
    image.arrayLayers = 1;
    image.format = VK_FORMAT_R8G8B8A8_UNORM;   // matches Metal's RGBA8Unorm (no sRGB decode)
    image.tiling = VK_IMAGE_TILING_OPTIMAL;
    image.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    // TRANSFER_SRC too: each mip is the blit source for the next.
    image.usage = VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT |
                  VK_IMAGE_USAGE_SAMPLED_BIT;
    image.samples = VK_SAMPLE_COUNT_1_BIT;
    image.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &image, nullptr, &out.image) != VK_SUCCESS) {
        vkDestroyBuffer(device, staging, nullptr);
        vkFreeMemory(device, stagingMem, nullptr);
        return false;
    }
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device, out.image, &req);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(device, &alloc, nullptr, &out.memory) != VK_SUCCESS) {
        vkDestroyImage(device, out.image, nullptr);
        vkDestroyBuffer(device, staging, nullptr);
        vkFreeMemory(device, stagingMem, nullptr);
        out.image = VK_NULL_HANDLE;
        return false;
    }
    vkBindImageMemory(device, out.image, out.memory, 0);

    // One-time upload: transition, copy, transition to shader-read.
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

    // Helper: barrier one mip level between layouts/accesses.
    auto levelBarrier = [&](uint32_t level, VkImageLayout from, VkImageLayout to,
                            VkAccessFlags srcA, VkAccessFlags dstA,
                            VkPipelineStageFlags srcS, VkPipelineStageFlags dstS) {
        VkImageMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        b.oldLayout = from;
        b.newLayout = to;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = out.image;
        b.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        b.subresourceRange.baseMipLevel = level;
        b.subresourceRange.levelCount = 1;
        b.subresourceRange.layerCount = 1;
        b.srcAccessMask = srcA;
        b.dstAccessMask = dstA;
        vkCmdPipelineBarrier(cmd, srcS, dstS, 0, 0, nullptr, 0, nullptr, 1, &b);
    };

    // All levels UNDEFINED -> TRANSFER_DST, then upload mip 0.
    {
        VkImageMemoryBarrier b{};
        b.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        b.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        b.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
        b.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = out.image;
        b.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        b.subresourceRange.levelCount = mipLevels;
        b.subresourceRange.layerCount = 1;
        b.srcAccessMask = 0;
        b.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
        vkCmdPipelineBarrier(cmd, VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT,
                             VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0, nullptr, 1, &b);
    }
    VkBufferImageCopy region{};
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.layerCount = 1;
    region.imageExtent = {w, h, 1};
    vkCmdCopyBufferToImage(cmd, staging, out.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

    // Blit each level down from the previous, leaving each source in SHADER_READ.
    int32_t mw = static_cast<int32_t>(w), mh = static_cast<int32_t>(h);
    for (uint32_t i = 1; i < mipLevels; ++i) {
        levelBarrier(i - 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                     VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                     VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT,
                     VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT);
        int32_t nw = mw > 1 ? mw / 2 : 1, nh = mh > 1 ? mh / 2 : 1;
        VkImageBlit blit{};
        blit.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, i - 1, 0, 1};
        blit.srcOffsets[1] = {mw, mh, 1};
        blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, i, 0, 1};
        blit.dstOffsets[1] = {nw, nh, 1};
        vkCmdBlitImage(cmd, out.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                       out.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit, VK_FILTER_LINEAR);
        levelBarrier(i - 1, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                     VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                     VK_ACCESS_TRANSFER_READ_BIT, VK_ACCESS_SHADER_READ_BIT,
                     VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
        mw = nw; mh = nh;
    }
    // Last level is still TRANSFER_DST -> SHADER_READ.
    levelBarrier(mipLevels - 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                 VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                 VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT,
                 VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT);
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

    VkImageViewCreateInfo view{};
    view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    view.image = out.image;
    view.viewType = VK_IMAGE_VIEW_TYPE_2D;
    view.format = VK_FORMAT_R8G8B8A8_UNORM;
    view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    view.subresourceRange.levelCount = mipLevels;
    view.subresourceRange.layerCount = 1;
    if (vkCreateImageView(device, &view, nullptr, &out.view) != VK_SUCCESS) {
        destroyTexture(out);
        return false;
    }
    return true;
}

void VulkanRenderer::Impl::destroyTexture(GpuTexture& t) {
    if (t.view) vkDestroyImageView(device, t.view, nullptr);
    if (t.image) vkDestroyImage(device, t.image, nullptr);
    if (t.memory) vkFreeMemory(device, t.memory, nullptr);
    t = GpuTexture{};
}

VkImageView VulkanRenderer::Impl::textureViewOr(TextureHandle h) const {
    const GpuTexture* t = textures.get(h);
    return (t && t->view) ? t->view : defaultTexture.view;
}

bool VulkanRenderer::Impl::createMaterialResources() {
    // 5 combined image samplers (albedo, MR, normal, AO, emissive).
    std::array<VkDescriptorSetLayoutBinding, 5> bindings{};
    for (uint32_t i = 0; i < bindings.size(); ++i) {
        bindings[i].binding = i;
        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        bindings[i].descriptorCount = 1;
        bindings[i].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
    }
    VkDescriptorSetLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
    layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
    layoutInfo.pBindings = bindings.data();
    if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &materialSetLayout) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] material set layout creation failed");
        return false;
    }

    constexpr uint32_t MAX_MATERIAL_SETS = 2048;
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        VkDescriptorPoolSize size{};
        size.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
        size.descriptorCount = MAX_MATERIAL_SETS * 5;
        VkDescriptorPoolCreateInfo info{};
        info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        info.poolSizeCount = 1;
        info.pPoolSizes = &size;
        info.maxSets = MAX_MATERIAL_SETS;
        if (vkCreateDescriptorPool(device, &info, nullptr, &materialPools[i]) != VK_SUCCESS) {
            LOG_ERROR("[vulkan] material descriptor pool creation failed");
            return false;
        }
    }

    VkSamplerCreateInfo sampler{};
    sampler.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    sampler.magFilter = VK_FILTER_LINEAR;
    sampler.minFilter = VK_FILTER_LINEAR;
    sampler.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
    sampler.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
    sampler.maxLod = VK_LOD_CLAMP_NONE;
    if (vkCreateSampler(device, &sampler, nullptr, &textureSampler) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] sampler creation failed");
        return false;
    }

    const uint8_t white[4] = {255, 255, 255, 255};
    if (!createImageRGBA8(white, 1, 1, defaultTexture)) {
        LOG_ERROR("[vulkan] default texture creation failed");
        return false;
    }
    return true;
}

bool VulkanRenderer::Impl::createShadowResources() {
    // Depth array image: one layer per cascade.
    VkImageCreateInfo image{};
    image.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    image.imageType = VK_IMAGE_TYPE_2D;
    image.extent = {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE, 1};
    image.mipLevels = 1;
    image.arrayLayers = RT_MAX_CASCADES;
    image.format = kDepthFormat;
    image.tiling = VK_IMAGE_TILING_OPTIMAL;
    image.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
    image.samples = VK_SAMPLE_COUNT_1_BIT;
    image.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateImage(device, &image, nullptr, &shadowImage) != VK_SUCCESS) return false;
    VkMemoryRequirements req;
    vkGetImageMemoryRequirements(device, shadowImage, &req);
    VkMemoryAllocateInfo alloc{};
    alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    alloc.allocationSize = req.size;
    alloc.memoryTypeIndex = findMemoryType(req.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    if (vkAllocateMemory(device, &alloc, nullptr, &shadowMemory) != VK_SUCCESS) return false;
    vkBindImageMemory(device, shadowImage, shadowMemory, 0);

    // Array view for sampling; per-layer views for rendering each cascade.
    VkImageViewCreateInfo arrayView{};
    arrayView.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    arrayView.image = shadowImage;
    arrayView.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
    arrayView.format = kDepthFormat;
    arrayView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    arrayView.subresourceRange.levelCount = 1;
    arrayView.subresourceRange.layerCount = RT_MAX_CASCADES;
    if (vkCreateImageView(device, &arrayView, nullptr, &shadowArrayView) != VK_SUCCESS) return false;

    // Depth-only render pass; finalLayout READ_ONLY so the lit pass can sample it.
    VkAttachmentDescription depth{};
    depth.format = kDepthFormat;
    depth.samples = VK_SAMPLE_COUNT_1_BIT;
    depth.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depth.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    depth.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depth.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depth.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    depth.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
    VkAttachmentReference depthRef{0, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};
    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.pDepthStencilAttachment = &depthRef;
    std::array<VkSubpassDependency, 2> deps{};
    deps[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    deps[0].dstSubpass = 0;
    deps[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    deps[0].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    deps[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
    deps[0].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    deps[1].srcSubpass = 0;
    deps[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    deps[1].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
    deps[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
    deps[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
    deps[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    VkRenderPassCreateInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    rp.attachmentCount = 1;
    rp.pAttachments = &depth;
    rp.subpassCount = 1;
    rp.pSubpasses = &subpass;
    rp.dependencyCount = static_cast<uint32_t>(deps.size());
    rp.pDependencies = deps.data();
    if (vkCreateRenderPass(device, &rp, nullptr, &shadowRenderPass) != VK_SUCCESS) return false;

    for (int c = 0; c < RT_MAX_CASCADES; ++c) {
        VkImageViewCreateInfo lv{};
        lv.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        lv.image = shadowImage;
        lv.viewType = VK_IMAGE_VIEW_TYPE_2D;
        lv.format = kDepthFormat;
        lv.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
        lv.subresourceRange.baseArrayLayer = static_cast<uint32_t>(c);
        lv.subresourceRange.levelCount = 1;
        lv.subresourceRange.layerCount = 1;
        if (vkCreateImageView(device, &lv, nullptr, &shadowLayerViews[c]) != VK_SUCCESS) return false;
        VkFramebufferCreateInfo fb{};
        fb.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fb.renderPass = shadowRenderPass;
        fb.attachmentCount = 1;
        fb.pAttachments = &shadowLayerViews[c];
        fb.width = SHADOW_MAP_SIZE;
        fb.height = SHADOW_MAP_SIZE;
        fb.layers = 1;
        if (vkCreateFramebuffer(device, &fb, nullptr, &shadowFramebuffers[c]) != VK_SUCCESS) return false;
    }

    // Comparison sampler for hardware PCF (sampler2DArrayShadow).
    VkSamplerCreateInfo s{};
    s.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
    s.magFilter = VK_FILTER_LINEAR;
    s.minFilter = VK_FILTER_LINEAR;
    s.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
    s.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    s.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    s.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
    s.compareEnable = VK_TRUE;
    s.compareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
    if (vkCreateSampler(device, &s, nullptr, &shadowSampler) != VK_SUCCESS) return false;
    return true;
}

bool VulkanRenderer::Impl::createShadowPipeline() {
    VkPushConstantRange pushRange{};
    pushRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
    pushRange.offset = 0;
    pushRange.size = sizeof(ShadowPush);
    VkPipelineLayoutCreateInfo layoutInfo{};
    layoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    layoutInfo.pushConstantRangeCount = 1;
    layoutInfo.pPushConstantRanges = &pushRange;
    if (vkCreatePipelineLayout(device, &layoutInfo, nullptr, &shadowPipelineLayout) != VK_SUCCESS)
        return false;

    VkShaderModule vert = loadShaderModule(std::string(RT_VULKAN_SHADER_DIR) + "/mesh_shadow.vert.spv");
    if (!vert) return false;
    VkPipelineShaderStageCreateInfo stage{};
    stage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    stage.stage = VK_SHADER_STAGE_VERTEX_BIT;
    stage.module = vert;
    stage.pName = "main";

    // Position-only vertex input: the shadow vertex shader reads nothing else, so
    // declaring the full layout just triggers "attribute N not consumed"
    // validation warnings. The binding stride stays sizeof(GpuVertex) — the same
    // interleaved vertex buffer is bound, we only read position at offset 0.
    VkVertexInputBindingDescription binding{0, sizeof(GpuVertex), VK_VERTEX_INPUT_RATE_VERTEX};
    VkVertexInputAttributeDescription posAttr{0, 0, VK_FORMAT_R32G32B32_SFLOAT,
                                              offsetof(GpuVertex, position)};
    VkPipelineVertexInputStateCreateInfo vertexInput{};
    vertexInput.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
    vertexInput.vertexBindingDescriptionCount = 1;
    vertexInput.pVertexBindingDescriptions = &binding;
    vertexInput.vertexAttributeDescriptionCount = 1;
    vertexInput.pVertexAttributeDescriptions = &posAttr;

    VkPipelineInputAssemblyStateCreateInfo ia{};
    ia.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
    ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;

    VkPipelineViewportStateCreateInfo vp{};
    vp.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
    vp.viewportCount = 1;
    vp.scissorCount = 1;

    VkPipelineRasterizationStateCreateInfo raster{};
    raster.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
    raster.polygonMode = VK_POLYGON_MODE_FILL;
    raster.cullMode = VK_CULL_MODE_NONE;
    raster.frontFace = VK_FRONT_FACE_CLOCKWISE;
    raster.depthBiasEnable = VK_TRUE;   // set dynamically per shadow pass
    raster.lineWidth = 1.0f;

    VkPipelineMultisampleStateCreateInfo ms{};
    ms.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
    ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

    VkPipelineDepthStencilStateCreateInfo ds{};
    ds.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
    ds.depthTestEnable = VK_TRUE;
    ds.depthWriteEnable = VK_TRUE;
    ds.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;

    VkPipelineColorBlendStateCreateInfo blend{};
    blend.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
    blend.attachmentCount = 0;   // depth-only

    std::array<VkDynamicState, 3> dyn{VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR,
                                      VK_DYNAMIC_STATE_DEPTH_BIAS};
    VkPipelineDynamicStateCreateInfo dynamic{};
    dynamic.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
    dynamic.dynamicStateCount = static_cast<uint32_t>(dyn.size());
    dynamic.pDynamicStates = dyn.data();

    VkGraphicsPipelineCreateInfo info{};
    info.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
    info.stageCount = 1;
    info.pStages = &stage;
    info.pVertexInputState = &vertexInput;
    info.pInputAssemblyState = &ia;
    info.pViewportState = &vp;
    info.pRasterizationState = &raster;
    info.pMultisampleState = &ms;
    info.pDepthStencilState = &ds;
    info.pColorBlendState = &blend;
    info.pDynamicState = &dynamic;
    info.layout = shadowPipelineLayout;
    info.renderPass = shadowRenderPass;
    info.subpass = 0;
    VkResult result = vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &info, nullptr, &shadowPipeline);
    vkDestroyShaderModule(device, vert, nullptr);
    if (result != VK_SUCCESS) {
        LOG_ERROR("[vulkan] shadow pipeline creation failed");
        return false;
    }
    return true;
}

void VulkanRenderer::Impl::recordShadowPass(VkCommandBuffer cmd) {
    // Render every cascade layer so all are in READ_ONLY layout for sampling;
    // inactive cascades are cleared only (no draws). cascadeCount==0 => sun casts
    // no shadow, but the clears still leave the array valid to sample (lit).
    VkViewport viewport{0, 0, float(SHADOW_MAP_SIZE), float(SHADOW_MAP_SIZE), 0.0f, 1.0f};
    VkRect2D scissor{{0, 0}, {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE}};
    for (int c = 0; c < RT_MAX_CASCADES; ++c) {
        VkClearValue clear{};
        clear.depthStencil = {1.0f, 0};
        VkRenderPassBeginInfo rp{};
        rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        rp.renderPass = shadowRenderPass;
        rp.framebuffer = shadowFramebuffers[c];
        rp.renderArea.extent = {SHADOW_MAP_SIZE, SHADOW_MAP_SIZE};
        rp.clearValueCount = 1;
        rp.pClearValues = &clear;
        vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

        if (c < activeCascadeCount) {
            vkCmdSetViewport(cmd, 0, 1, &viewport);
            vkCmdSetScissor(cmd, 0, 1, &scissor);
            vkCmdSetDepthBias(cmd, shadowDepthBiasConst, 0.0f, shadowDepthBiasSlope);
            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, shadowPipeline);
            for (const DrawItem& item : drawQueue) {
                GpuMesh* m = meshes.get(item.mesh);
                if (!m || m->indexCount == 0) continue;
                ShadowPush push;
                std::memcpy(push.lightViewProj, cpuGlobals.cascadeVP[c], sizeof(push.lightViewProj));
                std::memcpy(push.model, item.push.model, sizeof(push.model));
                vkCmdPushConstants(cmd, shadowPipelineLayout, VK_SHADER_STAGE_VERTEX_BIT,
                                   0, sizeof(ShadowPush), &push);
                VkDeviceSize offset = 0;
                vkCmdBindVertexBuffers(cmd, 0, 1, &m->vertexBuffer, &offset);
                vkCmdBindIndexBuffer(cmd, m->indexBuffer, 0, VK_INDEX_TYPE_UINT32);
                vkCmdDrawIndexed(cmd, m->indexCount, 1, 0, 0, 0);
            }
        }
        vkCmdEndRenderPass(cmd);
    }
}

// ---- swapchain lifecycle ----

void VulkanRenderer::Impl::cleanupSwapchain() {
    for (int i = 0; i < 2; ++i) {
        if (bloomFramebuffer[i]) vkDestroyFramebuffer(device, bloomFramebuffer[i], nullptr);
        if (bloomView[i]) vkDestroyImageView(device, bloomView[i], nullptr);
        if (bloomImage[i]) vkDestroyImage(device, bloomImage[i], nullptr);
        if (bloomMemory[i]) vkFreeMemory(device, bloomMemory[i], nullptr);
        bloomFramebuffer[i] = VK_NULL_HANDLE;
        bloomView[i] = VK_NULL_HANDLE;
        bloomImage[i] = VK_NULL_HANDLE;
        bloomMemory[i] = VK_NULL_HANDLE;
    }
    if (ssrFramebuffer) vkDestroyFramebuffer(device, ssrFramebuffer, nullptr);
    if (ssrView) vkDestroyImageView(device, ssrView, nullptr);
    if (ssrImage) vkDestroyImage(device, ssrImage, nullptr);
    if (ssrMemory) vkFreeMemory(device, ssrMemory, nullptr);
    ssrFramebuffer = VK_NULL_HANDLE;
    ssrView = VK_NULL_HANDLE;
    ssrImage = VK_NULL_HANDLE;
    ssrMemory = VK_NULL_HANDLE;
    if (dofFramebuffer) vkDestroyFramebuffer(device, dofFramebuffer, nullptr);
    if (dofView) vkDestroyImageView(device, dofView, nullptr);
    if (dofImage) vkDestroyImage(device, dofImage, nullptr);
    if (dofMemory) vkFreeMemory(device, dofMemory, nullptr);
    dofFramebuffer = VK_NULL_HANDLE;
    dofView = VK_NULL_HANDLE;
    dofImage = VK_NULL_HANDLE;
    dofMemory = VK_NULL_HANDLE;
    if (aoFramebuffer) vkDestroyFramebuffer(device, aoFramebuffer, nullptr);
    if (aoView) vkDestroyImageView(device, aoView, nullptr);
    if (aoImage) vkDestroyImage(device, aoImage, nullptr);
    if (aoMemory) vkFreeMemory(device, aoMemory, nullptr);
    aoFramebuffer = VK_NULL_HANDLE;
    aoView = VK_NULL_HANDLE;
    aoImage = VK_NULL_HANDLE;
    aoMemory = VK_NULL_HANDLE;
    if (aoBlurFramebuffer) vkDestroyFramebuffer(device, aoBlurFramebuffer, nullptr);
    if (aoBlurView) vkDestroyImageView(device, aoBlurView, nullptr);
    if (aoBlurImage) vkDestroyImage(device, aoBlurImage, nullptr);
    if (aoBlurMemory) vkFreeMemory(device, aoBlurMemory, nullptr);
    aoBlurFramebuffer = VK_NULL_HANDLE;
    aoBlurView = VK_NULL_HANDLE;
    aoBlurImage = VK_NULL_HANDLE;
    aoBlurMemory = VK_NULL_HANDLE;
    if (normalView) vkDestroyImageView(device, normalView, nullptr);
    if (normalImage) vkDestroyImage(device, normalImage, nullptr);
    if (normalMemory) vkFreeMemory(device, normalMemory, nullptr);
    normalView = VK_NULL_HANDLE;
    normalImage = VK_NULL_HANDLE;
    normalMemory = VK_NULL_HANDLE;
    if (sceneFramebuffer) vkDestroyFramebuffer(device, sceneFramebuffer, nullptr);
    sceneFramebuffer = VK_NULL_HANDLE;
    if (hdrView) vkDestroyImageView(device, hdrView, nullptr);
    if (hdrImage) vkDestroyImage(device, hdrImage, nullptr);
    if (hdrMemory) vkFreeMemory(device, hdrMemory, nullptr);
    hdrView = VK_NULL_HANDLE;
    hdrImage = VK_NULL_HANDLE;
    hdrMemory = VK_NULL_HANDLE;
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

    if (!createSwapchain() || !createImageViews() || !createDepthResources() ||
        !createHdrResources() || !createSceneFramebuffer() || !createFramebuffers() ||
        !createBloomResources() || !createSsaoResources() || !createSsrResources() ||
        !createDofResources())
        return false;
    updateCompositeDescriptor();   // HDR + bloom + AO + SSR + DOF views were recreated

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

    // Cascaded shadow maps first (writes the depth array the lit pass samples).
    recordShadowPass(cmd);

    // Scene pass → offscreen HDR + world-normal G-buffer (sky + geometry).
    std::array<VkClearValue, 3> clears{};
    clears[0].color = {{0.05f, 0.06f, 0.08f, 1.0f}};   // HDR
    clears[1].color = {{0.5f, 0.5f, 1.0f, 0.0f}};      // normal (+Z, unused for sky)
    clears[2].depthStencil = {0.0f, 0};                 // reverse-Z: far = 0

    VkRenderPassBeginInfo rp{};
    rp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass = renderPass;
    rp.framebuffer = sceneFramebuffer;
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

    // Skybox first (fills the background; depth untouched), then geometry.
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, skyPipelineLayout, 0, 1,
                            &descriptorSets[currentFrame], 0, nullptr);
    vkCmdDraw(cmd, 3, 1, 0, 0);

    // Geometry. The wireframe debug view (Renderer::wireframe) shares the mesh
    // pipeline layout: mode 1 replaces the fills with the LINE-mode wire pipeline;
    // mode 2 draws the shaded fills then re-draws edges on top. Wire draws read a
    // flat color from the push (mesh_wire.frag) and skip the material set.
    // Split opaque (depth write, no blend) from transparent (alpha blend, no depth
    // write, sorted back-to-front). Wireframe modes draw everything as lines.
    std::vector<const DrawItem*> opaque, terrainItems, transparent, overlay;
    for (const DrawItem& item : drawQueue) {
        GpuMesh* m = meshes.get(item.mesh);
        if (!m || m->indexCount == 0) continue;
        // FLAG_OVERLAY draws last with depth off, regardless of opacity/terrain.
        if (item.push.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY) overlay.push_back(&item);
        else if (item.terrain) terrainItems.push_back(&item);
        else if (item.opacity < 1.0f) transparent.push_back(&item);
        else opaque.push_back(&item);
    }
    const float cx = cpuGlobals.cameraPosition[0], cy = cpuGlobals.cameraPosition[1],
                cz = cpuGlobals.cameraPosition[2];
    auto camDistSq = [&](const DrawItem* it) {
        float dx = it->push.model[12] - cx, dy = it->push.model[13] - cy,
              dz = it->push.model[14] - cz;
        return dx * dx + dy * dy + dz * dz;
    };
    std::sort(transparent.begin(), transparent.end(),
              [&](const DrawItem* a, const DrawItem* b) { return camDistSq(a) > camDistSq(b); });
    std::vector<const DrawItem*> allItems = opaque;
    allItems.insert(allItems.end(), terrainItems.begin(), terrainItems.end());
    allItems.insert(allItems.end(), transparent.begin(), transparent.end());
    allItems.insert(allItems.end(), overlay.begin(), overlay.end());

    auto recordGeometry = [&](const std::vector<const DrawItem*>& items, VkPipeline pipe,
                              bool wire, bool countStats) {
        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe);
        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1,
                                &descriptorSets[currentFrame], 0, nullptr);
        for (const DrawItem* itemPtr : items) {
            const DrawItem& item = *itemPtr;
            GpuMesh* m = meshes.get(item.mesh);
            if (!m || m->indexCount == 0) continue;

            MeshPush push = item.push;
            if (wire) {
                push.albedoMetallic[0] = wireColor[0];   // flat line color
                push.albedoMetallic[1] = wireColor[1];
                push.albedoMetallic[2] = wireColor[2];
            } else {
                // Material textures (set 1): a transient set from this frame's pool.
                VkDescriptorSet matSet = VK_NULL_HANDLE;
                VkDescriptorSetAllocateInfo dsa{};
                dsa.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                dsa.descriptorPool = materialPools[currentFrame];
                dsa.descriptorSetCount = 1;
                dsa.pSetLayouts = &materialSetLayout;
                if (vkAllocateDescriptorSets(device, &dsa, &matSet) != VK_SUCCESS) {
                    if (!materialPoolExhaustedWarned) {
                        LOG_WARN("[vulkan] material descriptor pool exhausted; some draws skipped this frame");
                        materialPoolExhaustedWarned = true;
                    }
                    continue;
                }
                std::array<VkDescriptorImageInfo, 5> imgs{};
                std::array<VkWriteDescriptorSet, 5> writes{};
                for (uint32_t k = 0; k < 5; ++k) {
                    imgs[k].sampler = textureSampler;
                    imgs[k].imageView = textureViewOr(item.textures[k]);
                    imgs[k].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    writes[k].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                    writes[k].dstSet = matSet;
                    writes[k].dstBinding = k;
                    writes[k].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    writes[k].descriptorCount = 1;
                    writes[k].pImageInfo = &imgs[k];
                }
                vkUpdateDescriptorSets(device, 5, writes.data(), 0, nullptr);
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 1, 1,
                                        &matSet, 0, nullptr);
            }

            vkCmdPushConstants(cmd, pipelineLayout,
                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                               0, sizeof(MeshPush), &push);
            VkDeviceSize offset = 0;
            vkCmdBindVertexBuffers(cmd, 0, 1, &m->vertexBuffer, &offset);
            vkCmdBindIndexBuffer(cmd, m->indexBuffer, 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(cmd, m->indexCount, 1, 0, 0, 0);
            if (countStats) {
                stats.drawCalls++;
                stats.trianglesDrawn += m->indexCount / 3;
            }
        }
    };

    if (wireframeFrame == 1) {
        recordGeometry(allItems, wirePipeline, /*wire=*/true, /*countStats=*/true);
    } else {
        recordGeometry(opaque, meshPipeline, /*wire=*/false, /*countStats=*/true);
        recordGeometry(terrainItems, terrainPipeline, /*wire=*/false, /*countStats=*/true);
        recordGeometry(transparent, transparentPipeline, /*wire=*/false, /*countStats=*/true);
        // Debug gizmos on top, after everything, with depth off (ADR-0061).
        recordGeometry(overlay, overlayPipeline, /*wire=*/false, /*countStats=*/true);
        if (wireframeFrame == 2)
            recordGeometry(allItems, wirePipeline, /*wire=*/true, /*countStats=*/false);
    }

    vkCmdEndRenderPass(cmd);

    // Bloom + SSAO read the scene outputs (always run so their views stay
    // sampleable; composite gates whether they're applied).
    recordBloom(cmd);
    recordSsao(cmd);
    recordSsr(cmd);
    // DOF (off by default) blurs the HDR scene; composite reads dofView when on.
    if (dofEnabledFrame) recordDof(cmd);

    // Composite pass → swapchain: tonemap (ACES/AgX) + grade + exposure of the
    // HDR scene target, plus bloom. (Phase 5b continues with SSAO/SSR/lens/DOF.)
    VkRenderPassBeginInfo crp{};
    crp.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    crp.renderPass = compositeRenderPass;
    crp.framebuffer = framebuffers[imageIndex];
    crp.renderArea.offset = {0, 0};
    crp.renderArea.extent = swapchainExtent;
    crp.clearValueCount = 0;
    vkCmdBeginRenderPass(cmd, &crp, VK_SUBPASS_CONTENTS_INLINE);
    vkCmdSetViewport(cmd, 0, 1, &viewport);
    vkCmdSetScissor(cmd, 0, 1, &scissor);
    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipeline);
    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, compositePipelineLayout, 0, 1,
                            &compositeSet, 0, nullptr);
    CompositePush cpush{};
    cpush.exposure = sceneExposure;
    cpush.tonemapOp = tonemapOp;          // mirrored from the Renderer in endFrame
    cpush.gradeContrast = gradeContrast;
    cpush.gradeSaturation = gradeSaturation;
    cpush.bloomEnabled = bloomEnabledFrame ? 1 : 0;
    cpush.bloomIntensity = bloomIntensity;
    cpush.ssaoEnabled = ssaoEnabledFrame ? 1 : 0;
    cpush.aoFloor = ssaoFloor;
    cpush.ssrEnabled = ssrEnabledFrame ? 1 : 0;
    cpush.lensEnabled = lensEnabledFrame ? 1 : 0;
    cpush.lensK1 = lensK1;
    cpush.lensK2 = lensK2;
    cpush.lensCA = lensCA;
    cpush.lensVignette = lensVignette;
    cpush.lensAspect = lensAspect;
    cpush.debugView = debugViewFrame;
    cpush.dofEnabled = dofEnabledFrame ? 1 : 0;
    vkCmdPushConstants(cmd, compositePipelineLayout, VK_SHADER_STAGE_FRAGMENT_BIT,
                       0, sizeof(CompositePush), &cpush);
    vkCmdDraw(cmd, 3, 1, 0, 0);
#ifdef RT_ENABLE_IMGUI
    // ImGui overlay, into the same swapchain pass (draw data finalized in endFrame).
    if (imguiInitialized) {
        ImDrawData* dd = ImGui::GetDrawData();
        if (dd) ImGui_ImplVulkan_RenderDrawData(dd, cmd);
    }
#endif
    vkCmdEndRenderPass(cmd);

    vkEndCommandBuffer(cmd);
}

void VulkanRenderer::Impl::drawFrame() {
    if (!initialized || width == 0 || height == 0) return;

    vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

    // This frame's prior submission is done: recycle its transient material sets.
    vkResetDescriptorPool(device, materialPools[currentFrame], 0);
    materialPoolExhaustedWarned = false;

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
              impl->createDepthResources() &&
              impl->createHdrResources() &&
              impl->createRenderPass() &&
              impl->createCompositeRenderPass() &&
              impl->createSceneFramebuffer() &&
              impl->createFramebuffers() &&
              impl->createCommandPool() &&
              impl->createCommandBuffers() &&
              impl->createSyncObjects() &&
              impl->createShadowResources() &&
              impl->createDescriptorSetLayout() &&
              impl->createGlobalsBuffers() &&
              impl->createDescriptorPool() &&
              impl->createBrdfLut() &&
              impl->createDescriptorSets() &&
              impl->createMaterialResources() &&
              impl->createBloomResources() &&
              impl->createSsaoResources() &&
              impl->createSsrResources() &&
              impl->createDofResources() &&
              impl->createCompositeResources() &&
              impl->createShadowPipeline() &&
              impl->createPipeline() &&
              impl->createSkyPipeline();
    if (!ok) {
        LOG_ERROR("[vulkan] initialization failed");
        return false;
    }
    // Env binding 2 defaults to the 1x1 white texture until setEnvironmentMap.
    impl->envView = impl->defaultTexture.view;
    impl->updateGlobalEnvDescriptor();
    impl->initialized = true;
    LOG_INFO("[vulkan] backend initialized (%dx%d, Phase 4b: + HDR equirect IBL)", width, height);
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
    impl->textures.forEach([&](TextureHandle, GpuTexture& t) { impl->destroyTexture(t); });
    impl->textures.clear();
    impl->destroyTexture(impl->defaultTexture);
    if (impl->envSampler) vkDestroySampler(impl->device, impl->envSampler, nullptr);
    impl->envSampler = VK_NULL_HANDLE;
    if (impl->brdfLutSampler) vkDestroySampler(impl->device, impl->brdfLutSampler, nullptr);
    if (impl->brdfLutView) vkDestroyImageView(impl->device, impl->brdfLutView, nullptr);
    if (impl->brdfLutImage) vkDestroyImage(impl->device, impl->brdfLutImage, nullptr);
    if (impl->brdfLutMemory) vkFreeMemory(impl->device, impl->brdfLutMemory, nullptr);
    impl->brdfLutSampler = VK_NULL_HANDLE;
    impl->brdfLutView = VK_NULL_HANDLE;
    impl->brdfLutImage = VK_NULL_HANDLE;
    impl->brdfLutMemory = VK_NULL_HANDLE;
    if (impl->textureSampler) vkDestroySampler(impl->device, impl->textureSampler, nullptr);
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i)
        if (impl->materialPools[i]) vkDestroyDescriptorPool(impl->device, impl->materialPools[i], nullptr);
    if (impl->materialSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->materialSetLayout, nullptr);
    impl->textureSampler = VK_NULL_HANDLE;
    impl->materialSetLayout = VK_NULL_HANDLE;

    // Shadow resources.
    if (impl->shadowPipeline) vkDestroyPipeline(impl->device, impl->shadowPipeline, nullptr);
    if (impl->shadowPipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->shadowPipelineLayout, nullptr);
    if (impl->shadowSampler) vkDestroySampler(impl->device, impl->shadowSampler, nullptr);
    for (int c = 0; c < RT_MAX_CASCADES; ++c) {
        if (impl->shadowFramebuffers[c]) vkDestroyFramebuffer(impl->device, impl->shadowFramebuffers[c], nullptr);
        if (impl->shadowLayerViews[c]) vkDestroyImageView(impl->device, impl->shadowLayerViews[c], nullptr);
    }
    if (impl->shadowArrayView) vkDestroyImageView(impl->device, impl->shadowArrayView, nullptr);
    if (impl->shadowRenderPass) vkDestroyRenderPass(impl->device, impl->shadowRenderPass, nullptr);
    if (impl->shadowImage) vkDestroyImage(impl->device, impl->shadowImage, nullptr);
    if (impl->shadowMemory) vkFreeMemory(impl->device, impl->shadowMemory, nullptr);
    impl->shadowPipeline = VK_NULL_HANDLE;
    impl->shadowPipelineLayout = VK_NULL_HANDLE;
    impl->shadowSampler = VK_NULL_HANDLE;
    impl->shadowArrayView = VK_NULL_HANDLE;
    impl->shadowRenderPass = VK_NULL_HANDLE;
    impl->shadowImage = VK_NULL_HANDLE;
    impl->shadowMemory = VK_NULL_HANDLE;

    if (impl->skyPipeline) vkDestroyPipeline(impl->device, impl->skyPipeline, nullptr);
    if (impl->skyPipelineLayout) vkDestroyPipelineLayout(impl->device, impl->skyPipelineLayout, nullptr);
    impl->skyPipeline = VK_NULL_HANDLE;
    impl->skyPipelineLayout = VK_NULL_HANDLE;
    if (impl->meshPipeline) vkDestroyPipeline(impl->device, impl->meshPipeline, nullptr);
    if (impl->wirePipeline) vkDestroyPipeline(impl->device, impl->wirePipeline, nullptr);
    impl->wirePipeline = VK_NULL_HANDLE;
    if (impl->transparentPipeline) vkDestroyPipeline(impl->device, impl->transparentPipeline, nullptr);
    impl->transparentPipeline = VK_NULL_HANDLE;
    if (impl->overlayPipeline) vkDestroyPipeline(impl->device, impl->overlayPipeline, nullptr);
    impl->overlayPipeline = VK_NULL_HANDLE;
    if (impl->terrainPipeline) vkDestroyPipeline(impl->device, impl->terrainPipeline, nullptr);
    impl->terrainPipeline = VK_NULL_HANDLE;
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

    // SSR resources (size-independent; target/framebuffer go via cleanupSwapchain).
    if (impl->ssrPipeline) vkDestroyPipeline(impl->device, impl->ssrPipeline, nullptr);
    if (impl->ssrPipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->ssrPipelineLayout, nullptr);
    if (impl->ssrPool) vkDestroyDescriptorPool(impl->device, impl->ssrPool, nullptr);
    if (impl->ssrSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->ssrSetLayout, nullptr);
    if (impl->ssrRenderPass) vkDestroyRenderPass(impl->device, impl->ssrRenderPass, nullptr);
    impl->ssrPipeline = VK_NULL_HANDLE;
    impl->ssrPipelineLayout = VK_NULL_HANDLE;
    impl->ssrPool = VK_NULL_HANDLE;
    impl->ssrSetLayout = VK_NULL_HANDLE;
    impl->ssrRenderPass = VK_NULL_HANDLE;

    // DOF resources (size-independent; target/framebuffer go via cleanupSwapchain).
    if (impl->dofPipeline) vkDestroyPipeline(impl->device, impl->dofPipeline, nullptr);
    if (impl->dofPipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->dofPipelineLayout, nullptr);
    if (impl->dofPool) vkDestroyDescriptorPool(impl->device, impl->dofPool, nullptr);
    if (impl->dofSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->dofSetLayout, nullptr);
    if (impl->dofRenderPass) vkDestroyRenderPass(impl->device, impl->dofRenderPass, nullptr);
    impl->dofPipeline = VK_NULL_HANDLE;
    impl->dofPipelineLayout = VK_NULL_HANDLE;
    impl->dofPool = VK_NULL_HANDLE;
    impl->dofSetLayout = VK_NULL_HANDLE;
    impl->dofRenderPass = VK_NULL_HANDLE;

    // SSAO resources (size-independent; targets/framebuffer go via cleanupSwapchain).
    if (impl->ssaoPipeline) vkDestroyPipeline(impl->device, impl->ssaoPipeline, nullptr);
    if (impl->ssaoPipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->ssaoPipelineLayout, nullptr);
    if (impl->ssaoBlurPipeline) vkDestroyPipeline(impl->device, impl->ssaoBlurPipeline, nullptr);
    if (impl->ssaoBlurPipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->ssaoBlurPipelineLayout, nullptr);
    if (impl->ssaoPool) vkDestroyDescriptorPool(impl->device, impl->ssaoPool, nullptr);
    if (impl->gbufferSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->gbufferSetLayout, nullptr);
    if (impl->ssaoBlurSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->ssaoBlurSetLayout, nullptr);
    if (impl->gbufferSampler) vkDestroySampler(impl->device, impl->gbufferSampler, nullptr);
    if (impl->ssaoRenderPass) vkDestroyRenderPass(impl->device, impl->ssaoRenderPass, nullptr);
    impl->ssaoPipeline = VK_NULL_HANDLE;
    impl->ssaoPipelineLayout = VK_NULL_HANDLE;
    impl->ssaoBlurPipeline = VK_NULL_HANDLE;
    impl->ssaoBlurPipelineLayout = VK_NULL_HANDLE;
    impl->ssaoPool = VK_NULL_HANDLE;
    impl->gbufferSetLayout = VK_NULL_HANDLE;
    impl->ssaoBlurSetLayout = VK_NULL_HANDLE;
    impl->gbufferSampler = VK_NULL_HANDLE;
    impl->ssaoRenderPass = VK_NULL_HANDLE;

    // Bloom resources (size-independent; the targets/framebuffers go via cleanupSwapchain).
    if (impl->bloomPipeline) vkDestroyPipeline(impl->device, impl->bloomPipeline, nullptr);
    if (impl->bloomPipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->bloomPipelineLayout, nullptr);
    if (impl->bloomPool) vkDestroyDescriptorPool(impl->device, impl->bloomPool, nullptr);
    if (impl->bloomSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->bloomSetLayout, nullptr);
    if (impl->bloomSampler) vkDestroySampler(impl->device, impl->bloomSampler, nullptr);
    if (impl->bloomRenderPass) vkDestroyRenderPass(impl->device, impl->bloomRenderPass, nullptr);
    impl->bloomPipeline = VK_NULL_HANDLE;
    impl->bloomPipelineLayout = VK_NULL_HANDLE;
    impl->bloomPool = VK_NULL_HANDLE;
    impl->bloomSetLayout = VK_NULL_HANDLE;
    impl->bloomSampler = VK_NULL_HANDLE;
    impl->bloomRenderPass = VK_NULL_HANDLE;

    // Composite (tonemap) resources.
    if (impl->compositePipeline) vkDestroyPipeline(impl->device, impl->compositePipeline, nullptr);
    if (impl->compositePipelineLayout)
        vkDestroyPipelineLayout(impl->device, impl->compositePipelineLayout, nullptr);
    if (impl->compositePool) vkDestroyDescriptorPool(impl->device, impl->compositePool, nullptr);
    if (impl->compositeSetLayout)
        vkDestroyDescriptorSetLayout(impl->device, impl->compositeSetLayout, nullptr);
    if (impl->compositeSampler) vkDestroySampler(impl->device, impl->compositeSampler, nullptr);
    if (impl->compositeRenderPass)
        vkDestroyRenderPass(impl->device, impl->compositeRenderPass, nullptr);
    impl->compositePipeline = VK_NULL_HANDLE;
    impl->compositePipelineLayout = VK_NULL_HANDLE;
    impl->compositePool = VK_NULL_HANDLE;
    impl->compositeSetLayout = VK_NULL_HANDLE;
    impl->compositeSampler = VK_NULL_HANDLE;
    impl->compositeRenderPass = VK_NULL_HANDLE;

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

TextureHandle VulkanRenderer::uploadTexture(int width, int height, int channels,
                                            const uint8_t* data) {
    GpuTexture tex;
    if (impl->device && data && width > 0 && height > 0) {
        // Expand to RGBA8 (the engine hands us 1..4 channels; the GPU image is
        // always 4-channel to match the Metal backend's upload path).
        std::vector<uint8_t> rgba(static_cast<size_t>(width) * height * 4, 255);
        for (size_t p = 0; p < static_cast<size_t>(width) * height; ++p) {
            for (int c = 0; c < 4; ++c) {
                rgba[p * 4 + c] = (c < channels) ? data[p * channels + c]
                                                 : (c == 3 ? 255 : 0);
            }
        }
        if (!impl->createImageRGBA8(rgba.data(), static_cast<uint32_t>(width),
                                    static_cast<uint32_t>(height), tex)) {
            LOG_ERROR("[vulkan] uploadTexture failed");
            tex = GpuTexture{};
        }
    }
    return impl->textures.insert(tex);
}

TextureHandle VulkanRenderer::uploadTextureHDR(int width, int height, int channels,
                                               const float* data) {
    // RGBA16F equirectangular env map, single mip (roughness blur is approximated
    // in the shader by blending toward the N-direction sample; a real GGX-
    // prefiltered mip chain is a later refinement).
    GpuTexture tex;
    if (impl->device && data && width > 0 && height > 0) {
        uint32_t w = static_cast<uint32_t>(width), h = static_cast<uint32_t>(height);
        std::vector<uint16_t> half(static_cast<size_t>(w) * h * 4);
        for (size_t p = 0; p < static_cast<size_t>(w) * h; ++p)
            for (int c = 0; c < 4; ++c) {
                float v = (c < channels) ? data[p * channels + c] : (c == 3 ? 1.0f : 0.0f);
                half[p * 4 + c] = floatToHalf(v);
            }
        VkDeviceSize size = half.size() * sizeof(uint16_t);
        VkBuffer staging = VK_NULL_HANDLE;
        VkDeviceMemory stagingMem = VK_NULL_HANDLE;
        if (impl->createBuffer(size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                staging, stagingMem)) {
            void* mapped = nullptr;
            vkMapMemory(impl->device, stagingMem, 0, size, 0, &mapped);
            std::memcpy(mapped, half.data(), static_cast<size_t>(size));
            vkUnmapMemory(impl->device, stagingMem);

            VkImageCreateInfo image{};
            image.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
            image.imageType = VK_IMAGE_TYPE_2D;
            image.extent = {w, h, 1};
            image.mipLevels = 1;
            image.arrayLayers = 1;
            image.format = kHdrFormat;
            image.tiling = VK_IMAGE_TILING_OPTIMAL;
            image.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
            image.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
            image.samples = VK_SAMPLE_COUNT_1_BIT;
            image.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
            if (vkCreateImage(impl->device, &image, nullptr, &tex.image) == VK_SUCCESS) {
                VkMemoryRequirements req;
                vkGetImageMemoryRequirements(impl->device, tex.image, &req);
                VkMemoryAllocateInfo alloc{};
                alloc.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
                alloc.allocationSize = req.size;
                alloc.memoryTypeIndex = impl->findMemoryType(req.memoryTypeBits,
                                                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
                vkAllocateMemory(impl->device, &alloc, nullptr, &tex.memory);
                vkBindImageMemory(impl->device, tex.image, tex.memory, 0);

                VkCommandBufferAllocateInfo cba{};
                cba.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                cba.commandPool = impl->commandPool;
                cba.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cba.commandBufferCount = 1;
                VkCommandBuffer cmd = VK_NULL_HANDLE;
                vkAllocateCommandBuffers(impl->device, &cba, &cmd);
                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                vkBeginCommandBuffer(cmd, &begin);
                impl->transitionImageLayout(cmd, tex.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                            VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
                VkBufferImageCopy region{};
                region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                region.imageSubresource.layerCount = 1;
                region.imageExtent = {w, h, 1};
                vkCmdCopyBufferToImage(cmd, staging, tex.image,
                                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);
                impl->transitionImageLayout(cmd, tex.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                            VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
                vkEndCommandBuffer(cmd);
                VkSubmitInfo submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
                submit.commandBufferCount = 1;
                submit.pCommandBuffers = &cmd;
                vkQueueSubmit(impl->graphicsQueue, 1, &submit, VK_NULL_HANDLE);
                vkQueueWaitIdle(impl->graphicsQueue);
                vkFreeCommandBuffers(impl->device, impl->commandPool, 1, &cmd);

                VkImageViewCreateInfo view{};
                view.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view.image = tex.image;
                view.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view.format = kHdrFormat;
                view.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view.subresourceRange.levelCount = 1;
                view.subresourceRange.layerCount = 1;
                vkCreateImageView(impl->device, &view, nullptr, &tex.view);
            }
            vkDestroyBuffer(impl->device, staging, nullptr);
            vkFreeMemory(impl->device, stagingMem, nullptr);
        }
        if (!tex.view) { impl->destroyTexture(tex); LOG_ERROR("[vulkan] uploadTextureHDR failed"); }
    }
    return impl->textures.insert(tex);
}

void VulkanRenderer::setEnvironmentMap(TextureHandle equirect) {
    GpuTexture* t = impl->textures.get(equirect);
    if (impl->device) vkDeviceWaitIdle(impl->device);   // descriptor in-use safety
    if (t && t->view) {
        impl->envView = t->view;
        impl->envBound = true;
    } else {
        impl->envView = impl->defaultTexture.view;   // restore procedural sky
        impl->envBound = false;
    }
    impl->updateGlobalEnvDescriptor();
}

void VulkanRenderer::removeTexture(TextureHandle handle) {
    GpuTexture* t = impl->textures.get(handle);
    if (!t) return;
    if (impl->device) vkDeviceWaitIdle(impl->device);
    impl->destroyTexture(*t);
    impl->textures.erase(handle);
}

RenderStats VulkanRenderer::getRenderStats() const { return impl->stats; }

void VulkanRenderer::beginFrame() {
    impl->stats = RenderStats{};
    impl->drawQueue.clear();
#ifdef RT_ENABLE_IMGUI
    // Backend new-frame here; the GLFW new-frame ran in Window::pollEvents, and
    // ImGui::NewFrame() must come after both (mirrors the Metal backend).
    if (impl->imguiInitialized) {
        ImGui_ImplVulkan_NewFrame();
        // Window::pollEvents normally runs ImGui_ImplGlfw_NewFrame first (it sets
        // io.DisplaySize from the window size). But beginFrame also fires via the
        // window draw callback during show/resize — before pollEvents has run this
        // frame — leaving DisplaySize at ImGui's (-1,-1) default, which asserts in
        // NewFrame(). Fall back to the swapchain extent when it's unset; a normal
        // frame's GLFW new-frame then restores the proper logical size + DPI scale.
        ImGuiIO& io = ImGui::GetIO();
        if (io.DisplaySize.x < 0.0f || io.DisplaySize.y < 0.0f)
            io.DisplaySize = ImVec2(static_cast<float>(impl->swapchainExtent.width),
                                    static_cast<float>(impl->swapchainExtent.height));
        ImGui::NewFrame();
    }
#endif
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
    // Reverse-Z: remap clip depth so the near plane is at NDC z=1 and the far
    // plane at z=0. With a float depth buffer this puts the precision where it's
    // needed (distant geometry no longer z-fights — fixes CDLOD terrain flicker),
    // matching the Metal backend. R: clip.z' = clip.w - clip.z.
    Mat4 reverseZ;
    reverseZ.m[2][2] = -1.0;
    reverseZ.m[2][3] = 1.0;
    proj = reverseZ * proj;

    Mat4 vp = proj * view;
    packMat4(vp, impl->cpuGlobals.viewProjection, /*flipY=*/true);
    packMat4(view, impl->cpuGlobals.view, /*flipY=*/false);   // for view-space depth

    // Inverse of the *flipped* clip transform (Flip*vp), so the skybox/IBL ray
    // reconstruction is the exact inverse of what geometry uses. Flip negates
    // clip row 1; it is its own inverse.
    Mat4 flip;
    flip.m[1][1] = -1.0;
    Mat4 invFlippedVP = (flip * vp).inverse();
    packMat4(invFlippedVP, impl->cpuGlobals.invViewProjection, /*flipY=*/false);
    impl->cpuGlobals.cameraPosition[0] = static_cast<float>(camera.position.x);
    impl->cpuGlobals.cameraPosition[1] = static_cast<float>(camera.position.y);
    impl->cpuGlobals.cameraPosition[2] = static_cast<float>(camera.position.z);
    impl->cpuGlobals.cameraPosition[3] = 1.0f;

    // Captured for the cascade fit in setLights (un-flipped, reverse-Z VP).
    impl->camViewProj = vp;
    impl->camPos = camera.position;
    impl->camNear = camera.nearPlane;
    impl->camFar = camera.farPlane;

    // Lens effects from the active camera (applied in composite).
    impl->lensK1 = static_cast<float>(camera.lens.distortionK1);
    impl->lensK2 = static_cast<float>(camera.lens.distortionK2);
    impl->lensCA = static_cast<float>(camera.lens.chromaticAberration);
    impl->lensVignette = static_cast<float>(camera.lens.vignette);
    impl->lensAspect = aspect;

    // Depth-of-field inputs (thin-lens). cocScale is derived per-frame in recordDof
    // from the framebuffer height and sensorHeight.
    impl->dofFocusDistance = static_cast<float>(camera.lens.focusDistance);
    impl->dofFocalLength = static_cast<float>(camera.lens.focalLength / 1000.0);   // mm → m
    impl->dofAperture = static_cast<float>(camera.lens.apertureDiameter());        // meters
    impl->dofSensorHeight = static_cast<float>(camera.lens.sensorHeight);          // mm
}

void VulkanRenderer::setLights(const SceneLighting& lighting) {
    float amb = lighting.ambientMultiplier;
    impl->cpuGlobals.ambient[0] = static_cast<float>(lighting.ambientTint.x) * amb;
    impl->cpuGlobals.ambient[1] = static_cast<float>(lighting.ambientTint.y) * amb;
    impl->cpuGlobals.ambient[2] = static_cast<float>(lighting.ambientTint.z) * amb;
    impl->cpuGlobals.ambient[3] = 1.0f;

    // Procedural sky (ADR-0016) → skybox + analytic IBL. HDR equirect mode is a
    // later refinement, so envMode is always procedural (0) for now.
    const ProceduralSky& sky = lighting.sky;
    Vec3 sd = normalize(sky.sunDirection);
    auto set3 = [](float* d, const Vec3& v, float w) {
        d[0] = static_cast<float>(v.x); d[1] = static_cast<float>(v.y);
        d[2] = static_cast<float>(v.z); d[3] = w;
    };
    set3(impl->cpuGlobals.skySunDir, sd, sky.sunDiscIntensity);
    set3(impl->cpuGlobals.skySunColor, sky.sunColor, 0.0f);
    set3(impl->cpuGlobals.skyZenith, sky.zenithColor, 0.0f);
    set3(impl->cpuGlobals.skyHorizon, sky.horizonColor, 0.0f);
    set3(impl->cpuGlobals.skyGround, sky.groundColor, 0.0f);
    impl->cpuGlobals.skyCloud[0] = sky.cloudCoverage;
    impl->cpuGlobals.skyCloud[1] = sky.cloudsEnabled ? sky.cloudDensity : 0.0f;
    impl->cpuGlobals.skyCloud[2] = sky.cloudScale;
    impl->cpuGlobals.skyCloud[3] = sky.cloudTime;
    // envMode: 1 = HDR equirect (when bound and the live toggle is on), else 0 = procedural.
    impl->cpuGlobals.counts[2] = (impl->envBound && environmentMapEnabled) ? 1 : 0;
    impl->sceneExposure = lighting.exposure;

    // Aerial-perspective fog (matches Metal's lightData.fog* + the offline tracer).
    impl->cpuGlobals.fog[0] = static_cast<float>(lighting.fog.color.x);
    impl->cpuGlobals.fog[1] = static_cast<float>(lighting.fog.color.y);
    impl->cpuGlobals.fog[2] = static_cast<float>(lighting.fog.color.z);
    impl->cpuGlobals.fog[3] = lighting.fog.enabled ? lighting.fog.density : 0.0f;

    int n = 0;
    auto setColor = [](float* dst, const Vec3& c, float w) {
        dst[0] = static_cast<float>(c.x);
        dst[1] = static_cast<float>(c.y);
        dst[2] = static_cast<float>(c.z);
        dst[3] = w;
    };

    // Directional sun (type 1): intensity in positionIntensity.w, dir normalized.
    if (n < 32) {
        GpuLight& l = impl->cpuGlobals.lights[n++];
        setColor(l.positionIntensity, Vec3(0, 0, 0), lighting.sun.intensity);
        setColor(l.directionInner, normalize(lighting.sun.direction), 0.0f);
        setColor(l.colorOuter, lighting.sun.color, 0.0f);
        l.typeRange[0] = 1.0f; l.typeRange[1] = 0.0f; l.typeRange[2] = 0.0f; l.typeRange[3] = 0.0f;
    }
    // Point lights (type 0).
    for (const PointLight& p : lighting.pointLights) {
        if (n >= 32) break;
        GpuLight& l = impl->cpuGlobals.lights[n++];
        setColor(l.positionIntensity, p.position, p.intensity);
        setColor(l.directionInner, Vec3(0, 0, 0), 0.0f);
        setColor(l.colorOuter, p.color, 0.0f);
        l.typeRange[0] = 0.0f; l.typeRange[1] = p.range; l.typeRange[2] = 0.0f; l.typeRange[3] = 0.0f;
    }
    // Spot lights (type 2): cones stored as cosines.
    for (const SpotLight& s : lighting.spotLights) {
        if (n >= 32) break;
        GpuLight& l = impl->cpuGlobals.lights[n++];
        setColor(l.positionIntensity, s.position, s.intensity);
        setColor(l.directionInner, normalize(s.direction), std::cos(s.innerConeAngle));
        setColor(l.colorOuter, s.color, std::cos(s.outerConeAngle));
        l.typeRange[0] = 2.0f; l.typeRange[1] = s.range; l.typeRange[2] = 0.0f; l.typeRange[3] = 0.0f;
    }
    impl->cpuGlobals.counts[0] = n;

    // ---- Cascaded shadow maps for the sun (port of the Metal cascade fit) ----
    impl->activeCascadeCount = 0;
    impl->cpuGlobals.counts[1] = 0;
    const DirectionalLight& sun = lighting.sun;
    if (sun.castsShadow && lighting.shadow.enabled) {
        float distance = (lighting.shadow.distance > 0.0f) ? lighting.shadow.distance
                                                           : shadowParams.distance;
        int cc = (lighting.shadow.cascadeCount > 0) ? lighting.shadow.cascadeCount
                                                     : shadowParams.cascadeCount;
        cc = std::max(1, std::min(cc, RT_MAX_CASCADES));
        Real lambda = shadowParams.splitLambda;
        Real camNear = impl->camNear, camFar = impl->camFar;
        Real shadowDist = std::max(std::min(static_cast<Real>(distance), camFar), camNear * 2.0);

        Vec3 sunDir = normalize(sun.direction);
        Vec3 up = (std::abs(sunDir.y) > 0.99) ? Vec3(0, 0, 1) : Vec3(0, 1, 0);
        Mat4 invVP = impl->camViewProj.inverse();
        // Reverse-Z NDC: near plane at z=1, far at z=0 (camViewProj is reverse-Z).
        auto corner = [&](Real x, Real y, Real z) { return invVP.transformPoint(Vec3(x, y, z)); };
        Vec3 nearC[4] = {corner(-1,-1,1), corner(1,-1,1), corner(1,1,1), corner(-1,1,1)};
        Vec3 farC[4]  = {corner(-1,-1,0), corner(1,-1,0), corner(1,1,0), corner(-1,1,0)};

        Real prevFar = camNear;
        for (int c = 0; c < cc; ++c) {
            Real f = Real(c + 1) / Real(cc);
            Real uni = camNear + (shadowDist - camNear) * f;
            Real lg = camNear * std::pow(shadowDist / camNear, f);
            Real zFar = lambda * lg + (1.0 - lambda) * uni;
            Real zNear = prevFar;
            prevFar = zFar;

            Real fN = (zNear - camNear) / (camFar - camNear);
            Real fF = (zFar - camNear) / (camFar - camNear);
            Vec3 corners[8];
            for (int k = 0; k < 4; ++k) {
                corners[k]     = nearC[k] + (farC[k] - nearC[k]) * fN;
                corners[k + 4] = nearC[k] + (farC[k] - nearC[k]) * fF;
            }
            Vec3 center = impl->camPos;   // camera-centered fit: stable when turning
            Real radius = 0.01;
            for (const Vec3& p : corners) radius = std::max(radius, (p - center).length());
            radius = std::ceil(radius * 16.0) / 16.0;

            Real pullback = radius + 50.0;
            Real texelWorld = (radius * 2.0) / static_cast<Real>(Impl::SHADOW_MAP_SIZE);
            Mat4 lightView = Mat4::lookAt(center + sunDir * pullback, center, up);
            Vec3 centerLS = lightView.transformPoint(center);
            centerLS.x = std::round(centerLS.x / texelWorld) * texelWorld;
            centerLS.y = std::round(centerLS.y / texelWorld) * texelWorld;
            Vec3 snapped = lightView.inverse().transformPoint(centerLS);

            lightView = Mat4::lookAt(snapped + sunDir * pullback, snapped, up);
            Mat4 lightProj = Mat4::orthographic(radius * 2.0, 1.0, 0.1, pullback + radius);
            Mat4 lightVP = lightProj * lightView;
            packMat4(lightVP, impl->cpuGlobals.cascadeVP[c], /*flipY=*/false);
            impl->cpuGlobals.cascadeSplit[c] = static_cast<float>(zFar);
        }
        impl->cpuGlobals.counts[1] = cc;
        impl->activeCascadeCount = cc;
        impl->cpuGlobals.shadowParams[0] = lighting.shadow.normalBias;
        impl->cpuGlobals.shadowParams[1] = lighting.shadow.pcfRadius;
        impl->cpuGlobals.shadowParams[2] = static_cast<float>(Impl::SHADOW_MAP_SIZE);
        impl->cpuGlobals.shadowParams[3] = lighting.shadowArtistic.strength;
        impl->cpuGlobals.shadowTint[0] = static_cast<float>(lighting.shadowArtistic.tint.x);
        impl->cpuGlobals.shadowTint[1] = static_cast<float>(lighting.shadowArtistic.tint.y);
        impl->cpuGlobals.shadowTint[2] = static_cast<float>(lighting.shadowArtistic.tint.z);
        impl->cpuGlobals.shadowTint[3] = lighting.shadowArtistic.ambientStrength;
    }
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
    // Material texture slots in shader order: albedo, MR, normal, AO, emissive.
    item.textures = {material.albedoMap, material.metallicRoughnessMap,
                     material.normalMap, material.aoMap, material.emissiveMap};
    uint32_t textureFlags = 0;
    for (uint32_t k = 0; k < item.textures.size(); ++k)
        if (item.textures[k].valid()) textureFlags |= (1u << k);

    item.push.surfaceFlags[0] = static_cast<uint32_t>(material.surface());
    item.push.surfaceFlags[1] = material.flags;
    item.push.surfaceFlags[2] = textureFlags;
    // Stash opacity (float bits) in the spare push slot so mesh.frag can write it
    // as the output alpha for the transparent blend pass.
    std::memcpy(&item.push.surfaceFlags[3], &material.opacity, sizeof(float));
    item.push.morphStart = 0.0f;   // terrain-only (drawTerrain sets these)
    item.push.morphEnd = 0.0f;
    item.opacity = material.opacity;
    impl->drawQueue.push_back(item);
    impl->stats.entitiesSubmitted++;
}

void VulkanRenderer::drawTerrain(MeshHandle handle, const RenderMaterial& material,
                                 float morphStart, float morphEnd) {
    // CDLOD node (ADR-0036): world-space mesh, identity model; terrain.vert morphs
    // each vertex toward its coarser-LOD position (packed in the tangent slot) over
    // [morphStart, morphEnd] camera distance. Shares the lit fragment + material.
    DrawItem item;
    item.mesh = handle;
    packMat4(Mat4(), item.push.model, /*flipY=*/false);   // identity (verts are world-space)
    item.push.albedoMetallic[0] = static_cast<float>(material.albedo.x);
    item.push.albedoMetallic[1] = static_cast<float>(material.albedo.y);
    item.push.albedoMetallic[2] = static_cast<float>(material.albedo.z);
    item.push.albedoMetallic[3] = material.metallic;
    item.push.emissionRough[0] = static_cast<float>(material.emission.x);
    item.push.emissionRough[1] = static_cast<float>(material.emission.y);
    item.push.emissionRough[2] = static_cast<float>(material.emission.z);
    item.push.emissionRough[3] = material.roughness;
    item.textures = {material.albedoMap, material.metallicRoughnessMap,
                     material.normalMap, material.aoMap, material.emissiveMap};
    uint32_t textureFlags = 0;
    for (uint32_t k = 0; k < item.textures.size(); ++k)
        if (item.textures[k].valid()) textureFlags |= (1u << k);
    item.push.surfaceFlags[0] = static_cast<uint32_t>(material.surface());
    item.push.surfaceFlags[1] = material.flags;
    item.push.surfaceFlags[2] = textureFlags;
    float one = 1.0f;
    std::memcpy(&item.push.surfaceFlags[3], &one, sizeof(float));   // opaque
    item.push.morphStart = morphStart;
    item.push.morphEnd = morphEnd;
    item.terrain = true;
    impl->drawQueue.push_back(item);
    impl->stats.entitiesSubmitted++;
}

void VulkanRenderer::endFrame() {
    // Mirror the live tonemap/grade/bloom knobs (Renderer base members).
    impl->tonemapOp = tonemapOperator;
    impl->gradeContrast = gradeParams.contrast;
    impl->gradeSaturation = gradeParams.saturation;
    impl->bloomEnabledFrame = bloomEnabled;
    impl->bloomThreshold = bloomParams.threshold;
    impl->bloomKnee = bloomParams.knee;
    impl->bloomIntensity = bloomParams.intensity;
    impl->ssaoEnabledFrame = ssaoEnabled;
    impl->ssaoRadius = ssaoParams.radius;
    impl->ssaoIntensity = ssaoParams.intensity;
    impl->ssaoBias = ssaoParams.bias;
    impl->ssaoFloor = ssaoParams.aoFloor;
    impl->ssrEnabledFrame = ssrEnabled;
    impl->ssrMaxRayDist = ssrParams.maxRayDist;
    impl->ssrThickness = ssrParams.thickness;
    impl->ssrMaxRoughness = ssrParams.maxRoughness;
    impl->ssrBlendStrength = ssrParams.blendStrength;
    impl->lensEnabledFrame = lensEffectsEnabled;
    impl->debugViewFrame = debugView;
    // Carry the debug-view selector into the globals UBO (the unused counts.w) so
    // the lit pass can write debug content (shadow/albedo/facing/cascades/depth)
    // into the HDR target; composite shows those raw. Buffer views (AO/SSR/
    // normals) stay composite-side. See shaders/vulkan/{mesh,composite}.frag.
    impl->cpuGlobals.counts[3] = debugView;
    impl->wireframeFrame = wireframe;
    impl->wireColor[0] = static_cast<float>(wireframeColor.x);
    impl->wireColor[1] = static_cast<float>(wireframeColor.y);
    impl->wireColor[2] = static_cast<float>(wireframeColor.z);
    // DOF runs only with a real aperture (pinhole = perfectly sharp → no pass).
    impl->dofEnabledFrame = dofEnabled && impl->dofAperture > 0.0f;
    // Wind sway (FLAG_WIND vegetation) — same constants as the Metal backend; the
    // vertex shader applies it per-instance from the model matrix base.
    float windTime = std::chrono::duration<float>(
                         std::chrono::steady_clock::now() - impl->windStart).count();
    impl->cpuGlobals.wind1[0] = 0.85f;  impl->cpuGlobals.wind1[1] = 0.0f;
    impl->cpuGlobals.wind1[2] = 0.30f;  impl->cpuGlobals.wind1[3] = windTime;
    impl->cpuGlobals.wind2[0] = 1.6f;   // frequency
    impl->cpuGlobals.wind2[1] = 2.5f;   // height
    impl->cpuGlobals.wind2[2] = 0.12f;  // amplitude
    impl->cpuGlobals.wind2[3] = 0.0f;
#ifdef RT_ENABLE_IMGUI
    // Finalize the ImGui draw data (built by systems during render) before the
    // command buffer records it inside the composite pass.
    if (impl->imguiInitialized) ImGui::Render();
#endif
    impl->drawFrame();
}

// Defined unconditionally (the header declares the overrides); the body is a
// no-op unless RT_ENABLE_IMGUI is set, so the default build links cleanly.
void VulkanRenderer::initDebugUi(void* windowHandle) {
    (void)windowHandle;
#ifdef RT_ENABLE_IMGUI
    // Create the ImGui context here (the non-Apple path never did, which crashed
    // at the first NewFrame). Window::initDebugUi attaches the GLFW backend after.
    if (!impl->device) return;
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    // A generously-sized descriptor pool for ImGui's font/texture descriptors.
    // ImGui 1.92's imgui_impl_vulkan splits the combined image+sampler into
    // separate VK_DESCRIPTOR_TYPE_SAMPLER + VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE
    // descriptors, so the pool must offer all three types it may allocate.
    VkDescriptorPoolSize sizes[]{
        {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 64},
        {VK_DESCRIPTOR_TYPE_SAMPLER, 64},
        {VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 64},
    };
    VkDescriptorPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
    poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
    poolInfo.maxSets = 64;
    poolInfo.poolSizeCount = 3;
    poolInfo.pPoolSizes = sizes;
    if (vkCreateDescriptorPool(impl->device, &poolInfo, nullptr, &impl->imguiPool) != VK_SUCCESS) {
        LOG_ERROR("[vulkan] ImGui descriptor pool creation failed");
        return;
    }

    ImGui_ImplVulkan_InitInfo init{};
    init.Instance = impl->instance;
    init.PhysicalDevice = impl->physicalDevice;
    init.Device = impl->device;
    init.QueueFamily = impl->graphicsFamily;
    init.Queue = impl->graphicsQueue;
    init.DescriptorPool = impl->imguiPool;
    init.MinImageCount = 2;
    init.ImageCount = static_cast<uint32_t>(impl->swapchainImages.size());
    // ImGui draws into the composite (swapchain) render pass. Where the render
    // pass + MSAA fields live in ImGui_ImplVulkan_InitInfo has shifted twice, so
    // this is version-guarded:
    //   < 1.90    : RenderPass is the 2nd arg to Init; fonts via a command buffer.
    //   1.90–1.91 : RenderPass/MSAASamples flat in InitInfo; CreateFontsTexture().
    //   >= 1.92   : both moved into init.PipelineInfoMain, and the explicit
    //               CreateFontsTexture() was removed (fonts built lazily on first
    //               use). IMGUI_VERSION_NUM 19200 == 1.92.0.
#if defined(IMGUI_VERSION_NUM) && IMGUI_VERSION_NUM >= 19200
    init.PipelineInfoMain.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    init.PipelineInfoMain.RenderPass = impl->compositeRenderPass;
    ImGui_ImplVulkan_Init(&init);            // fonts auto-managed in 1.92+
#elif defined(IMGUI_VERSION_NUM) && IMGUI_VERSION_NUM >= 19000
    init.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    init.RenderPass = impl->compositeRenderPass;
    ImGui_ImplVulkan_Init(&init);
    ImGui_ImplVulkan_CreateFontsTexture();   // 1.90+: no command buffer needed
#else
    init.MSAASamples = VK_SAMPLE_COUNT_1_BIT;
    ImGui_ImplVulkan_Init(&init, impl->compositeRenderPass);
    // Older ImGui: upload fonts via a one-time command buffer.
    VkCommandBufferAllocateInfo cba{};
    cba.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    cba.commandPool = impl->commandPool;
    cba.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    cba.commandBufferCount = 1;
    VkCommandBuffer cmd = VK_NULL_HANDLE;
    vkAllocateCommandBuffers(impl->device, &cba, &cmd);
    VkCommandBufferBeginInfo begin{};
    begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &begin);
    ImGui_ImplVulkan_CreateFontsTexture(cmd);
    vkEndCommandBuffer(cmd);
    VkSubmitInfo submit{};
    submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    submit.commandBufferCount = 1;
    submit.pCommandBuffers = &cmd;
    vkQueueSubmit(impl->graphicsQueue, 1, &submit, VK_NULL_HANDLE);
    vkQueueWaitIdle(impl->graphicsQueue);
    vkFreeCommandBuffers(impl->device, impl->commandPool, 1, &cmd);
    ImGui_ImplVulkan_DestroyFontUploadObjects();
#endif
    impl->imguiInitialized = true;
    LOG_INFO("[vulkan] ImGui backend initialized");
#endif
}

void VulkanRenderer::shutdownDebugUi() {
#ifdef RT_ENABLE_IMGUI
    if (!impl->imguiInitialized) return;
    vkDeviceWaitIdle(impl->device);
    ImGui_ImplVulkan_Shutdown();
    if (impl->imguiPool) vkDestroyDescriptorPool(impl->device, impl->imguiPool, nullptr);
    impl->imguiPool = VK_NULL_HANDLE;
    ImGui::DestroyContext();   // Window::shutdownDebugUi (GLFW) ran first
    impl->imguiInitialized = false;
#endif
}

// The non-Apple factory. Exactly one Renderer::create() is linked per target.
std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<VulkanRenderer>();
}

}  // namespace engine
