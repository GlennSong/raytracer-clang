// WebGPU renderer backend (ADR-0058) — the web target's implementation of the
// `Renderer` seam (../../renderer/renderer.h), compiled under Emscripten and
// driving the browser's WebGPU device. Feature parity with the Vulkan backend:
// multi-light Cook-Torrance + texture/material maps (with mipmaps), cascaded
// shadow maps (PCF), procedural sky or HDR equirect environment with a baked
// split-sum BRDF LUT, an offscreen HDR pipeline (tone map/grade, bloom, SSAO,
// SSR over a material G-buffer), hardware instancing, CDLOD terrain morph, and
// wind. See docs/webgpu-renderer-plan.md and this directory's AGENTS.md.
//
// Structure mirrors src/renderer/vulkan/vulkan_renderer.cpp: pack the engine's
// (double) Vertex to a float GpuVertex, queue draws during the frame, and record
// the whole render pass in endFrame(). WebGPU has no push constants, so per-draw
// data rides a single dynamic uniform buffer (one 256-byte slot per draw).
//
// Targets the **emdawnwebgpu** port (Dawn's standardized webgpu.h), which is how
// Emscripten 4.0.10+/6.x ship WebGPU — the legacy `-sUSE_WEBGPU`/
// `emscripten_webgpu_get_device()` binding was removed. Built with `-sASYNCIFY`
// so the async adapter/device request can be awaited inside the synchronous
// Renderer::initialize() seam (emscripten_sleep yields to the browser until the
// callbacks fire). Compiles + links against emsdk 6.0.1; in-browser behaviour is
// still unverified (no GPU in CI).

#include "../renderer.h"
#include "../cascade_fit.h"
#include "../../log.h"

#include <webgpu/webgpu.h>
#include <emscripten/emscripten.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <unordered_map>
#include <vector>

// Last camera eye position, exported for the page's debug readout (lets a headless
// run confirm input actually moves the camera).
static float g_webCamEye[3] = {0, 0, 0};
extern "C" EMSCRIPTEN_KEEPALIVE float rt_web_cam(int i) {
    return (i >= 0 && i < 3) ? g_webCamEye[i] : 0.0f;
}

namespace engine {

namespace {

// One slot of per-draw uniform data, padded so each draw sits on a 256-byte
// dynamic-offset boundary (conservative minUniformBufferOffsetAlignment).
constexpr uint64_t kDrawStride = 256;

// Phase 1 swapchain format. navigator.gpu.getPreferredCanvasFormat() returns
// "bgra8unorm" on every current platform, so we hardcode the non-sRGB form and
// do the linear->sRGB encode in the shader (see WGSL fs_main). A later phase can
// query surface capabilities and switch to a *-srgb view to drop the gamma.
constexpr WGPUTextureFormat kSwapFormat = WGPUTextureFormat_BGRA8Unorm;
constexpr WGPUTextureFormat kDepthFormat = WGPUTextureFormat_Depth24Plus;
// Scene renders into a linear HDR target; a composite pass tone-maps it to the
// swapchain. RGBA16Float is renderable + filterable everywhere WebGPU runs.
constexpr WGPUTextureFormat kHdrFormat = WGPUTextureFormat_RGBA16Float;
constexpr WGPUTextureFormat kShadowFormat = WGPUTextureFormat_Depth32Float;

WGPUStringView sv(const char* s) {
    WGPUStringView v;
    v.data = s;
    v.length = s ? std::strlen(s) : 0;
    return v;
}

// Float vertex as the GPU sees it (the engine Vertex is double-precision). Must
// match the vertex layout below and the @location inputs in the WGSL.
struct GpuVertex {
    float position[3];
    float normal[3];
    float tangent[3];
    float texcoord[2];
    float color[3];
};

// One light, packed into 4 vec4 (mirrors the GLSL Light / Vulkan GpuLight):
// positionIntensity = (pos, intensity); directionInner = (dir, innerCos);
// colorOuter = (color, outerCos); typeRange = (type, range, _, _).
// type: 0 point, 1 directional, 2 spot.
struct GpuLight {
    float positionIntensity[4];
    float directionInner[4];
    float colorOuter[4];
    float typeRange[4];
};

// Scene globals (group 0, binding 0). Field order/alignment must match the WGSL
// `Globals` struct (every field is vec4/mat4 → 16-byte aligned, std140-style).
struct GpuGlobals {
    float    viewProjection[16];  // column-major
    float    view[16];            // for view-space depth (debug view 3)
    float    invViewProjection[16]; // reconstruct world rays (sky background)
    float    cameraPosition[4];
    float    ambient[4];          // rgb ambient term (tint * multiplier)
    float    skySunDir[4];        // xyz dir, w disc intensity
    float    skySunColor[4];
    float    skyZenith[4];
    float    skyHorizon[4];
    float    skyGround[4];
    float    fog[4];              // rgb color, w density (0 = off)
    int32_t  counts[4];           // x lightCount, y debugView, z shadowMapSize, w cascadeCount
    float    cascadeVP[4][16];    // per-cascade sun shadow matrices (CSM)
    float    cascadeSplit[4];     // far view-space depth of cascades 0..3
    float    shadowParams[4];     // x enabled, y depthBias, z normalBias, w pcfTexels
    float    postParams[4];       // x exposure, y tonemapOp, z contrast, w saturation
    float    skyMoonDir[4];       // xyz toward the moon, w disc radiance (0 = down)
    float    skyMoonSun[4];       // xyz TRUE sun direction (lights the disc), w lit fraction
    float    skyCelX[4];          // the celestial frame in local space (stars)
    float    skyCelY[4];
    float    skyCelZ[4];
    float    skyStars[4];         // x visibility, y Milky Way strength
    float    skyCity[4];          // xy unit XZ toward the city, z light pollution
    GpuLight lights[32];
};

// Composite uniform (matches the WGSL `Post`): the view-transform knobs + the
// debug-view selector. Separate from Globals so the composite pass binds a small
// buffer instead of the whole scene block.
struct GpuPost {
    float    postParams[4];   // x exposure, y tonemapOp, z contrast, w saturation
    int32_t  debugView[4];    // x = debug view (0 = normal)
    float    effects[4];      // x = bloom intensity (0 = off)
};

// Bloom pass uniform (matches the WGSL `BloomU`).
struct GpuBloom {
    float params[4];          // x threshold, y knee, z intensity, w mode
    float texel[4];           // xy = blur step in uv
};

// SSAO pass uniform (matches the WGSL `SsaoU`).
struct GpuSsao {
    float invViewProjection[16];
    float viewProjection[16];
    float cameraPosition[4];
    float params[4];          // x radius, y bias, z intensity, w aoFloor
    float texel[4];           // xy = 1/resolution
};

// SSR pass uniform (matches the WGSL `SsrU`).
struct GpuSsr {
    float invViewProjection[16];
    float viewProjection[16];
    float cameraPosition[4];
    float params[4];          // x maxDist, y thickness, z thicknessFar, w maxRoughness
    float params2[4];         // x camNear, y camFar, z pixel stride
    float texel[4];           // xy = 1/effectRes, zw = full resolution
};

// Atmosphere pass uniform (matches the WGSL `Atmosphere` in atmosphere.wgsl).
// vec4/mat4-aligned (std140). Filled from globals_ + the AtmosphereRenderParams.
struct GpuAtmosphere {
    float invViewProjection[16];
    float cameraPosition[4];   // xyz
    float sunDirection[4];     // xyz toward the sun
    float planetCenter[4];     // xyz
    float sunColor[4];         // rgb, w = intensity
    float rayleighCoeff[4];    // rgb per length
    float radii[4];            // x planetRadius, y atmosphereRadius, z rayleighH, w mieH
    float mie[4];              // x mieCoeff, y mieG, z viewSamples, w lightSamples
};

// Per-draw uniforms (group 0, binding 1, dynamic). Matches the WGSL `DrawData`.
struct GpuDraw {
    float    model[16];          // column-major
    float    albedoMetallic[4];  // rgb albedo, a metallic
    float    emissionRough[4];   // rgb emission, a roughness
    uint32_t surfaceFlags[4];    // x surfaceId, y rawFlags, z mapBits, w unused
    float    terrainMorph[4];    // x morphStart, y morphEnd (CDLOD terrain path)
};

// Engine Mat4 is row-major (m[row][col]); WGSL/WebGPU matrices are column-major,
// so transpose on the way to the GPU. WebGPU clip space is Y-up with depth in
// [0,1] (same as Metal/D3D), and Mat4::perspective already targets [0,1], so —
// unlike the Vulkan backend — no clip-space Y-flip is baked in here.
void packMat4(const Mat4& m, float* out) {
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r)
            out[c * 4 + r] = static_cast<float>(m.m[r][c]);
}

// WGSL shader sources live in shaders/webgpu/*.wgsl; the build embeds
// them into this generated header (cmake/embed_wgsl.cmake) so the wasm
// needs no runtime fetch. Edit the .wgsl files, not the header.
#include "webgpu_shaders_gen.h"


// IEEE-754 float32 -> float16 (binary16), for RGBA16Float HDR env uploads.
// Round-to-nearest-even; flushes subnormal results to zero, clamps overflow to
// infinity. Enough for equirect environment maps (no NaN handling needed).
uint16_t floatToHalf(float f) {
    uint32_t x;
    std::memcpy(&x, &f, sizeof(x));
    uint32_t sign = (x >> 16) & 0x8000u;
    int32_t exp = static_cast<int32_t>((x >> 23) & 0xFF) - 127 + 15;
    uint32_t mant = x & 0x7FFFFFu;
    if (exp <= 0) return static_cast<uint16_t>(sign);              // underflow -> 0
    if (exp >= 0x1F) return static_cast<uint16_t>(sign | 0x7C00u); // overflow -> inf
    uint32_t half = sign | (static_cast<uint32_t>(exp) << 10) | (mant >> 13);
    if (mant & 0x1000u) ++half;                                   // round to nearest-even
    return static_cast<uint16_t>(half);
}








class WebGpuRenderer final : public Renderer {
public:
    struct GpuTexture {
        WGPUTexture texture = nullptr;
        WGPUTextureView view = nullptr;
        uint32_t generation = 0;
    };
    // The five material maps as a hashable key (0 = none -> a default texture).
    struct MaterialKey {
        uint32_t albedo, normal, mr, emissive, ao;
        bool operator==(const MaterialKey& o) const {
            return albedo == o.albedo && normal == o.normal && mr == o.mr
                && emissive == o.emissive && ao == o.ao;
        }
    };
    struct MaterialKeyHash {
        size_t operator()(const MaterialKey& k) const {
            uint64_t h = 1469598103934665603ull;
            for (uint32_t v : {k.albedo, k.normal, k.mr, k.emissive, k.ao}) {
                h ^= v; h *= 1099511628211ull;
            }
            return static_cast<size_t>(h);
        }
    };

    bool initialize(void* /*windowHandle*/, int width, int height) override {
        width_ = width > 0 ? width : 1;
        height_ = height > 0 ? height : 1;

        instance_ = wgpuCreateInstance(nullptr);
        if (!instance_) {
            LOG_ERROR("WebGPU: wgpuCreateInstance failed");
            return false;
        }

        // Adapter + device are acquired asynchronously (the only API the
        // standardized webgpu.h offers); -sASYNCIFY lets us await them here so
        // the Renderer seam stays synchronous. emscripten_sleep yields to the
        // browser event loop, where the AllowSpontaneous callbacks resolve.
        WGPURequestAdapterCallbackInfo aci = {};
        aci.mode = WGPUCallbackMode_AllowSpontaneous;
        aci.callback = &WebGpuRenderer::onAdapter;
        aci.userdata1 = this;
        wgpuInstanceRequestAdapter(instance_, nullptr, aci);
        while (!adapterDone_) emscripten_sleep(1);
        if (!adapter_) {
            LOG_ERROR("WebGPU: no GPU adapter (navigator.gpu unavailable?)");
            return false;
        }

        // Route WebGPU validation/uncaptured errors to the log — otherwise they
        // are silently swallowed and a bad pipeline/draw just renders nothing.
        WGPUDeviceDescriptor deviceDesc = {};
        deviceDesc.uncapturedErrorCallbackInfo.callback = &WebGpuRenderer::onUncapturedError;
        WGPURequestDeviceCallbackInfo dci = {};
        dci.mode = WGPUCallbackMode_AllowSpontaneous;
        dci.callback = &WebGpuRenderer::onDevice;
        dci.userdata1 = this;
        wgpuAdapterRequestDevice(adapter_, &deviceDesc, dci);
        while (!deviceDone_) emscripten_sleep(1);
        if (!device_) {
            LOG_ERROR("WebGPU: failed to acquire a device");
            return false;
        }
        queue_ = wgpuDeviceGetQueue(device_);

        if (!createSurface()) return false;
        configureSurface();
        createDepthTarget();
        if (!createPipeline()) return false;
        createUniformResources();
        createShadowResources();
        createMaterialDefaults();

        LOG_INFO("WebGPU backend initialized (%dx%d)", width_, height_);
        return true;
    }

    void shutdown() override {
        releaseDepthTarget();
        if (bindGroup_) { wgpuBindGroupRelease(bindGroup_); bindGroup_ = nullptr; }
        if (globalBuf_) { wgpuBufferRelease(globalBuf_); globalBuf_ = nullptr; }
        if (drawBuf_)   { wgpuBufferRelease(drawBuf_);   drawBuf_ = nullptr; }
        if (pipeline_)  { wgpuRenderPipelineRelease(pipeline_); pipeline_ = nullptr; }
        if (overlayPipeline_) { wgpuRenderPipelineRelease(overlayPipeline_); overlayPipeline_ = nullptr; }
        if (instancedPipeline_) { wgpuRenderPipelineRelease(instancedPipeline_); instancedPipeline_ = nullptr; }
        if (instancedOverlayPipeline_) { wgpuRenderPipelineRelease(instancedOverlayPipeline_); instancedOverlayPipeline_ = nullptr; }
        if (instancedShadowPipeline_) { wgpuRenderPipelineRelease(instancedShadowPipeline_); instancedShadowPipeline_ = nullptr; }
        if (terrainPipeline_) { wgpuRenderPipelineRelease(terrainPipeline_); terrainPipeline_ = nullptr; }
        if (terrainShadowPipeline_) { wgpuRenderPipelineRelease(terrainShadowPipeline_); terrainShadowPipeline_ = nullptr; }
        if (instanceBuf_) { wgpuBufferRelease(instanceBuf_); instanceBuf_ = nullptr; }
        if (skyBindGroup_) { wgpuBindGroupRelease(skyBindGroup_); skyBindGroup_ = nullptr; }
        if (skyPipeline_) { wgpuRenderPipelineRelease(skyPipeline_); skyPipeline_ = nullptr; }
        if (skyLayout_) { wgpuBindGroupLayoutRelease(skyLayout_); skyLayout_ = nullptr; }
        if (compositeBindGroup_) { wgpuBindGroupRelease(compositeBindGroup_); compositeBindGroup_ = nullptr; }
        if (compositePipeline_) { wgpuRenderPipelineRelease(compositePipeline_); compositePipeline_ = nullptr; }
        if (compositeLayout_) { wgpuBindGroupLayoutRelease(compositeLayout_); compositeLayout_ = nullptr; }
        if (postBuf_) { wgpuBufferRelease(postBuf_); postBuf_ = nullptr; }
        for (WGPUBindGroup* g : {&bloomBrightGroup_, &bloomBlurHGroup_, &bloomBlurVGroup_})
            if (*g) { wgpuBindGroupRelease(*g); *g = nullptr; }
        for (WGPUBuffer* b : {&bloomUboBright_, &bloomUboH_, &bloomUboV_})
            if (*b) { wgpuBufferRelease(*b); *b = nullptr; }
        if (bloomBrightPipeline_) { wgpuRenderPipelineRelease(bloomBrightPipeline_); bloomBrightPipeline_ = nullptr; }
        if (bloomBlurPipeline_) { wgpuRenderPipelineRelease(bloomBlurPipeline_); bloomBlurPipeline_ = nullptr; }
        if (bloomLayout_) { wgpuBindGroupLayoutRelease(bloomLayout_); bloomLayout_ = nullptr; }
        if (linearSampler_) { wgpuSamplerRelease(linearSampler_); linearSampler_ = nullptr; }
        if (ssaoGroup_) { wgpuBindGroupRelease(ssaoGroup_); ssaoGroup_ = nullptr; }
        if (ssaoUbo_) { wgpuBufferRelease(ssaoUbo_); ssaoUbo_ = nullptr; }
        if (ssaoPipeline_) { wgpuRenderPipelineRelease(ssaoPipeline_); ssaoPipeline_ = nullptr; }
        if (ssaoLayout_) { wgpuBindGroupLayoutRelease(ssaoLayout_); ssaoLayout_ = nullptr; }
        if (ssrGroup_) { wgpuBindGroupRelease(ssrGroup_); ssrGroup_ = nullptr; }
        if (ssrUbo_) { wgpuBufferRelease(ssrUbo_); ssrUbo_ = nullptr; }
        if (ssrPipeline_) { wgpuRenderPipelineRelease(ssrPipeline_); ssrPipeline_ = nullptr; }
        if (ssrLayout_) { wgpuBindGroupLayoutRelease(ssrLayout_); ssrLayout_ = nullptr; }
        if (atmosphereGroup_) { wgpuBindGroupRelease(atmosphereGroup_); atmosphereGroup_ = nullptr; }
        if (atmosphereUbo_) { wgpuBufferRelease(atmosphereUbo_); atmosphereUbo_ = nullptr; }
        if (atmospherePipeline_) { wgpuRenderPipelineRelease(atmospherePipeline_); atmospherePipeline_ = nullptr; }
        if (atmosphereLayout_) { wgpuBindGroupLayoutRelease(atmosphereLayout_); atmosphereLayout_ = nullptr; }
        if (shadowPipeline_) { wgpuRenderPipelineRelease(shadowPipeline_); shadowPipeline_ = nullptr; }
        if (shadowSampleGroup_) { wgpuBindGroupRelease(shadowSampleGroup_); shadowSampleGroup_ = nullptr; }
        if (shadowIdxGroup_) { wgpuBindGroupRelease(shadowIdxGroup_); shadowIdxGroup_ = nullptr; }
        if (shadowIdxBuf_) { wgpuBufferRelease(shadowIdxBuf_); shadowIdxBuf_ = nullptr; }
        if (shadowSampler_) { wgpuSamplerRelease(shadowSampler_); shadowSampler_ = nullptr; }
        if (shadowArrayView_) { wgpuTextureViewRelease(shadowArrayView_); shadowArrayView_ = nullptr; }
        for (auto& v : shadowLayerViews_) { if (v) { wgpuTextureViewRelease(v); v = nullptr; } }
        if (shadowTexture_) { wgpuTextureRelease(shadowTexture_); shadowTexture_ = nullptr; }
        if (shadowSampleLayout_) { wgpuBindGroupLayoutRelease(shadowSampleLayout_); shadowSampleLayout_ = nullptr; }
        if (shadowVsLayout_) { wgpuBindGroupLayoutRelease(shadowVsLayout_); shadowVsLayout_ = nullptr; }
        if (bindLayout_) { wgpuBindGroupLayoutRelease(bindLayout_); bindLayout_ = nullptr; }
        for (auto& m : meshes_) freeMesh(m);
        meshes_.clear();
        for (auto& kv : materialGroups_) if (kv.second) wgpuBindGroupRelease(kv.second);
        materialGroups_.clear();
        for (auto& t : textures_) {
            if (t.view) wgpuTextureViewRelease(t.view);
            if (t.texture) wgpuTextureRelease(t.texture);
        }
        textures_.clear();
        if (whiteView_) { wgpuTextureViewRelease(whiteView_); whiteView_ = nullptr; }
        if (whiteTex_) { wgpuTextureRelease(whiteTex_); whiteTex_ = nullptr; }
        if (flatNormalView_) { wgpuTextureViewRelease(flatNormalView_); flatNormalView_ = nullptr; }
        if (flatNormalTex_) { wgpuTextureRelease(flatNormalTex_); flatNormalTex_ = nullptr; }
        if (materialSampler_) { wgpuSamplerRelease(materialSampler_); materialSampler_ = nullptr; }
        if (envSampler_) { wgpuSamplerRelease(envSampler_); envSampler_ = nullptr; }
        if (envDefaultView_) { wgpuTextureViewRelease(envDefaultView_); envDefaultView_ = nullptr; }
        if (envDefaultTex_) { wgpuTextureRelease(envDefaultTex_); envDefaultTex_ = nullptr; }
        if (brdfLutView_) { wgpuTextureViewRelease(brdfLutView_); brdfLutView_ = nullptr; }
        if (brdfLutTex_) { wgpuTextureRelease(brdfLutTex_); brdfLutTex_ = nullptr; }
        envCurrentView_ = nullptr;
        if (materialLayout_) { wgpuBindGroupLayoutRelease(materialLayout_); materialLayout_ = nullptr; }
        if (blitPipeline_) { wgpuRenderPipelineRelease(blitPipeline_); blitPipeline_ = nullptr; }
        if (blitLayout_) { wgpuBindGroupLayoutRelease(blitLayout_); blitLayout_ = nullptr; }
        if (surface_) { wgpuSurfaceRelease(surface_); surface_ = nullptr; }
        if (queue_)   { wgpuQueueRelease(queue_); queue_ = nullptr; }
        if (device_)  { wgpuDeviceRelease(device_); device_ = nullptr; }
        if (adapter_) { wgpuAdapterRelease(adapter_); adapter_ = nullptr; }
        if (instance_) { wgpuInstanceRelease(instance_); instance_ = nullptr; }
    }

    void resize(int width, int height) override {
        if (width <= 0 || height <= 0) return;
        if (width == width_ && height == height_) return;
        width_ = width;
        height_ = height;
        configureSurface();
        releaseDepthTarget();
        createDepthTarget();
        if (postBuf_) {
            rebuildCompositeBindGroup(); rebuildBloomGroups();
            rebuildSsaoGroup(); rebuildSsrGroup();
        }
    }

    MeshHandle uploadMesh(const RenderMesh& mesh) override {
        GpuMesh gpu;
        gpu.indexCount = static_cast<uint32_t>(mesh.indices.size());
        gpu.bounds = computeBoundingSphere(mesh.vertices.data(), mesh.vertices.size());

        std::vector<GpuVertex> packed(mesh.vertices.size());
        for (size_t i = 0; i < mesh.vertices.size(); ++i) {
            const Vertex& v = mesh.vertices[i];
            GpuVertex& o = packed[i];
            o.position[0] = (float)v.position.x; o.position[1] = (float)v.position.y; o.position[2] = (float)v.position.z;
            o.normal[0]   = (float)v.normal.x;   o.normal[1]   = (float)v.normal.y;   o.normal[2]   = (float)v.normal.z;
            o.tangent[0]  = (float)v.tangent.x;  o.tangent[1]  = (float)v.tangent.y;  o.tangent[2]  = (float)v.tangent.z;
            o.texcoord[0] = v.u; o.texcoord[1] = v.v;
            o.color[0] = (float)v.color.x; o.color[1] = (float)v.color.y; o.color[2] = (float)v.color.z;
        }

        gpu.vertexBuffer = createBuffer(WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst,
                                        packed.data(), packed.size() * sizeof(GpuVertex));
        gpu.indexBuffer = createBuffer(WGPUBufferUsage_Index | WGPUBufferUsage_CopyDst,
                                       mesh.indices.data(), mesh.indices.size() * sizeof(uint32_t));

        // Reuse a freed slot if one exists, else append. Handle index is slot+1
        // so 0 stays the invalid handle.
        uint32_t slot;
        if (!freeSlots_.empty()) {
            slot = freeSlots_.back();
            freeSlots_.pop_back();
            meshes_[slot] = gpu;
        } else {
            slot = static_cast<uint32_t>(meshes_.size());
            meshes_.push_back(gpu);
        }
        meshes_[slot].generation = ++generationCounter_;

        MeshHandle handle;
        handle.index = slot + 1;
        handle.generation = meshes_[slot].generation;
        return handle;
    }

    void removeMesh(MeshHandle handle) override {
        GpuMesh* m = resolve(handle);
        if (!m) return;
        freeMesh(*m);
        m->generation = 0;
        freeSlots_.push_back(handle.index - 1);
    }

    BoundingSphere getMeshBounds(MeshHandle handle) const override {
        const GpuMesh* m = resolve(handle);
        return m ? m->bounds : BoundingSphere{};
    }

    TextureHandle uploadTexture(int width, int height, int channels,
                                const uint8_t* data) override {
        if (width <= 0 || height <= 0 || !data) return TextureHandle{};
        // WebGPU sampled textures are RGBA8; expand 1/3-channel source to RGBA
        // (4-channel input is already the GPU layout — copy it straight through).
        std::vector<uint8_t> rgba(static_cast<size_t>(width) * height * 4);
        if (channels == 4) {
            std::memcpy(rgba.data(), data, rgba.size());
        } else {
            for (int i = 0; i < width * height; ++i) {
                uint8_t r = data[i * channels + 0];
                uint8_t g = channels >= 3 ? data[i * channels + 1] : r;
                uint8_t b = channels >= 3 ? data[i * channels + 2] : r;
                rgba[i * 4 + 0] = r; rgba[i * 4 + 1] = g; rgba[i * 4 + 2] = b; rgba[i * 4 + 3] = 255;
            }
        }
        int levels = 1;
        while ((std::max(width, height) >> levels) > 0) ++levels;   // full mip chain
        GpuTexture t;
        t.texture = createTexture2D(width, height, WGPUTextureFormat_RGBA8Unorm, rgba.data(),
                                    static_cast<size_t>(width) * 4, levels,
                                    levels > 1 ? WGPUTextureUsage_RenderAttachment
                                               : WGPUTextureUsage_None);
        if (levels > 1) generateMips(t.texture, levels);
        t.view = wgpuTextureCreateView(t.texture, nullptr);
        t.generation = ++textureCounter_;

        uint32_t slot;
        if (!freeTextures_.empty()) { slot = freeTextures_.back(); freeTextures_.pop_back(); textures_[slot] = t; }
        else { slot = static_cast<uint32_t>(textures_.size()); textures_.push_back(t); }
        TextureHandle h;
        h.index = slot + 1;
        h.generation = t.generation;
        return h;
    }
    void removeTexture(TextureHandle handle) override {
        if (handle.index == 0 || handle.index > textures_.size()) return;
        GpuTexture& t = textures_[handle.index - 1];
        if (t.generation != handle.generation) return;
        // If this texture is the bound environment, fall back to the procedural
        // sky BEFORE releasing the view — envCurrentView_ would otherwise dangle
        // and the next group-0/sky bind-group rebuild would use a dead view.
        if (t.view && t.view == envCurrentView_) {
            envMode_ = false;
            envCurrentView_ = envDefaultView_;
            if (bindGroup_) rebuildBindGroup();
            if (skyBindGroup_) rebuildSkyBindGroup();
        }
        if (t.view) { wgpuTextureViewRelease(t.view); t.view = nullptr; }
        if (t.texture) { wgpuTextureRelease(t.texture); t.texture = nullptr; }
        t.generation = 0;
        // Any cached material bind group may reference the removed texture. The
        // groups must be RELEASED, not just dropped from the map — each holds a
        // ref to its texture views, so leaking them pins the texture's VRAM.
        for (auto& kv : materialGroups_)
            if (kv.second) wgpuBindGroupRelease(kv.second);
        materialGroups_.clear();
        freeTextures_.push_back(handle.index - 1);
    }

    // HDR (float) texture upload — equirectangular environment maps decoded from
    // Radiance .hdr. `data` is linear RGB(A); stored as RGBA16Float (half) so it
    // stays filterable in core WebGPU. Mirrors uploadTexture's slot bookkeeping.
    TextureHandle uploadTextureHDR(int width, int height, int channels,
                                   const float* data) override {
        if (width <= 0 || height <= 0 || !data) return TextureHandle{};
        std::vector<uint16_t> half(static_cast<size_t>(width) * height * 4);
        for (int i = 0; i < width * height; ++i) {
            float r = data[i * channels + 0];
            float g = channels >= 3 ? data[i * channels + 1] : r;
            float b = channels >= 3 ? data[i * channels + 2] : r;
            float a = channels >= 4 ? data[i * channels + 3] : 1.0f;
            half[i * 4 + 0] = floatToHalf(r); half[i * 4 + 1] = floatToHalf(g);
            half[i * 4 + 2] = floatToHalf(b); half[i * 4 + 3] = floatToHalf(a);
        }
        GpuTexture t;
        t.texture = createTexture2D(width, height, WGPUTextureFormat_RGBA16Float,
                                    half.data(), static_cast<size_t>(width) * 8);
        t.view = wgpuTextureCreateView(t.texture, nullptr);
        t.generation = ++textureCounter_;

        uint32_t slot;
        if (!freeTextures_.empty()) { slot = freeTextures_.back(); freeTextures_.pop_back(); textures_[slot] = t; }
        else { slot = static_cast<uint32_t>(textures_.size()); textures_.push_back(t); }
        TextureHandle h;
        h.index = slot + 1;
        h.generation = t.generation;
        return h;
    }

    // Bind an equirectangular HDR as the scene environment (drives the sky
    // background + IBL). An invalid handle restores the procedural sky. Sets the
    // envMode flag (skySunColor.w) and rebuilds the group-0 / sky bind groups.
    void setEnvironmentMap(TextureHandle equirect) override {
        WGPUTextureView view = nullptr;
        if (equirect.index != 0 && equirect.index <= textures_.size()) {
            GpuTexture& t = textures_[equirect.index - 1];
            if (t.generation == equirect.generation) view = t.view;
        }
        envMode_ = (view != nullptr);
        envCurrentView_ = view ? view : envDefaultView_;
        if (bindGroup_) rebuildBindGroup();
        if (skyBindGroup_) rebuildSkyBindGroup();
    }

    // Planetary atmosphere glow (procedural-planet-plan P3). Stored here and
    // consumed by recordAtmosphere() each frame; disabled params skip the pass.
    void setAtmosphere(const AtmosphereRenderParams& atmosphere) override {
        atmosphere_ = atmosphere;
    }

    RenderStats getRenderStats() const override { return stats_; }

    void beginFrame() override {
        // Apply a live postEffectScale change (debug slider) before any pass by
        // rebuilding the SSAO/SSR buffers. Cheap targets, and only on change.
        float want = std::min(std::max(postEffectScale, 0.25f), 1.0f);
        if (want != appliedFxScale_ && postBuf_) {
            releaseDepthTarget();
            createDepthTarget();   // recomputes fxW_/fxH_, sets appliedFxScale_
            rebuildCompositeBindGroup(); rebuildBloomGroups();
            rebuildSsaoGroup(); rebuildSsrGroup();
        }
        draws_.clear();
        instancedGroups_.clear();
        instanceStaging_.clear();
        terrainDraws_.clear();
        stats_ = RenderStats{};
    }

    void setCamera(const CameraState& camera) override {
        Mat4 view = Mat4::lookAt(camera.position, camera.target, camera.up);
        Mat4 proj = (camera.projection == CameraProjection::Perspective)
            ? Mat4::perspective(camera.fovDegrees * static_cast<Real>(M_PI) / 180.0,
                                camera.aspectRatio, camera.nearPlane, camera.farPlane)
            : Mat4::orthographic(camera.orthoHeight, camera.aspectRatio,
                                 camera.nearPlane, camera.farPlane);
        Mat4 vp = proj * view;
        packMat4(vp, globals_.viewProjection);
        packMat4(view, globals_.view);
        packMat4(vp.inverse(), globals_.invViewProjection);
        camVP_ = vp;                       // for the cascade fit in endFrame
        camNear_ = camera.nearPlane;
        camFar_ = camera.farPlane;
        cameraEye_ = camera.position;
        g_webCamEye[0] = (float)camera.position.x;
        g_webCamEye[1] = (float)camera.position.y;
        g_webCamEye[2] = (float)camera.position.z;
        globals_.cameraPosition[0] = (float)camera.position.x;
        globals_.cameraPosition[1] = (float)camera.position.y;
        globals_.cameraPosition[2] = (float)camera.position.z;
        globals_.cameraPosition[3] = 1.0f;
    }

    void setLights(const SceneLighting& lighting) override {
        auto set4 = [](float* o, float x, float y, float z, float w) {
            o[0] = x; o[1] = y; o[2] = z; o[3] = w;
        };
        float amb = lighting.ambientMultiplier;
        set4(globals_.ambient, (float)lighting.ambientTint.x * amb,
             (float)lighting.ambientTint.y * amb, (float)lighting.ambientTint.z * amb, 0.0f);

        // Procedural sky (drives IBL + the clear color).
        const ProceduralSky& sky = lighting.sky;
        Vec3 sd = normalize(sky.sunDirection);
        set4(globals_.skySunDir, (float)sd.x, (float)sd.y, (float)sd.z, sky.sunDiscIntensity);
        Vec3 md = normalize(sky.moonDirection), ts = normalize(sky.sunTrueDirection);
        set4(globals_.skyMoonDir, (float)md.x, (float)md.y, (float)md.z, sky.moonDiscIntensity);
        set4(globals_.skyMoonSun, (float)ts.x, (float)ts.y, (float)ts.z, sky.moonIllumination);
        set4(globals_.skyCelX, (float)sky.celestialX.x, (float)sky.celestialX.y, (float)sky.celestialX.z, 0.0f);
        set4(globals_.skyCelY, (float)sky.celestialY.x, (float)sky.celestialY.y, (float)sky.celestialY.z, 0.0f);
        set4(globals_.skyCelZ, (float)sky.celestialZ.x, (float)sky.celestialZ.y, (float)sky.celestialZ.z, 0.0f);
        set4(globals_.skyStars, sky.starVisibility, sky.milkyWay, 0.0f, 0.0f);
        set4(globals_.skyCity, (float)sky.cityDirection.x, (float)sky.cityDirection.z, sky.lightPollution, 0.0f);
        // skySunColor.w doubles as the env mode: >0.5 = sample the bound HDR
        // equirect for the sky + IBL, else the analytic procedural sky.
        set4(globals_.skySunColor, (float)sky.sunColor.x, (float)sky.sunColor.y,
             (float)sky.sunColor.z, envMode_ ? 1.0f : 0.0f);
        set4(globals_.skyZenith, (float)sky.zenithColor.x, (float)sky.zenithColor.y, (float)sky.zenithColor.z, 0.0f);
        set4(globals_.skyHorizon, (float)sky.horizonColor.x, (float)sky.horizonColor.y, (float)sky.horizonColor.z, 0.0f);
        set4(globals_.skyGround, (float)sky.groundColor.x, (float)sky.groundColor.y, (float)sky.groundColor.z, 0.0f);

        // Aerial-perspective fog.
        const FogParams& fog = lighting.fog;
        float density = fog.enabled ? fog.density : 0.0f;
        set4(globals_.fog, (float)fog.color.x, (float)fog.color.y, (float)fog.color.z, density);

        // View transform: scene-referred exposure + the live grade/tonemap knobs
        // (Renderer::tonemapOperator/gradeParams, driven by the debug panel).
        set4(globals_.postParams, (float)lighting.exposure, (float)tonemapOperator,
             gradeParams.contrast, gradeParams.saturation);

        // Lights: sun first (directional), then point, then spot, up to 32.
        // Sun shadow config (cascaded). Driven off the level's ShadowConfig
        // + the live Renderer::shadowParams (debug overlay distance override).
        const DirectionalLight& sun = lighting.sun;
        const ShadowConfig& shc = lighting.shadow;
        sunDir_ = normalize(sun.direction);
        shadowOn_ = shc.enabled && sun.castsShadow && sun.intensity > 0.0f;
        shadowDistance_ = shc.distance > 0.0f ? shc.distance
                        : (shadowParams.distance > 0.0f ? shadowParams.distance : 150.0f);
        shadowDepthBias_ = shc.bias;
        shadowNormalBias_ = shc.normalBias;
        shadowPcf_ = shc.pcfRadius;
        shadowCascadeCount_ = shc.cascadeCount > 0 ? shc.cascadeCount
                            : (shadowParams.cascadeCount > 0 ? shadowParams.cascadeCount : 3);

        int n = 0;
        if (sun.intensity > 0.0f && n < 32) {
            Vec3 dir = normalize(sun.direction);
            GpuLight& L = globals_.lights[n++];
            set4(L.positionIntensity, 0, 0, 0, sun.intensity);
            set4(L.directionInner, (float)dir.x, (float)dir.y, (float)dir.z, 0.0f);
            set4(L.colorOuter, (float)sun.color.x, (float)sun.color.y, (float)sun.color.z, 0.0f);
            set4(L.typeRange, 1.0f, 0.0f, 0.0f, 0.0f);
        }
        for (const PointLight& p : lighting.pointLights) {
            if (n >= 32) break;
            GpuLight& L = globals_.lights[n++];
            set4(L.positionIntensity, (float)p.position.x, (float)p.position.y, (float)p.position.z, p.intensity);
            set4(L.directionInner, 0, 0, 0, 0);
            set4(L.colorOuter, (float)p.color.x, (float)p.color.y, (float)p.color.z, 0.0f);
            set4(L.typeRange, 0.0f, p.range, 0.0f, 0.0f);
        }
        for (const SpotLight& s : lighting.spotLights) {
            if (n >= 32) break;
            Vec3 sdir = normalize(s.direction);
            GpuLight& L = globals_.lights[n++];
            set4(L.positionIntensity, (float)s.position.x, (float)s.position.y, (float)s.position.z, s.intensity);
            set4(L.directionInner, (float)sdir.x, (float)sdir.y, (float)sdir.z, std::cos(s.innerConeAngle));
            set4(L.colorOuter, (float)s.color.x, (float)s.color.y, (float)s.color.z, std::cos(s.outerConeAngle));
            set4(L.typeRange, 2.0f, s.range, 0.0f, 0.0f);
        }
        globals_.counts[0] = n;
        globals_.counts[1] = debugView;   // Renderer::debugView (debug panel)
        globals_.counts[2] = kShadowMapSize;
        globals_.counts[3] = 0;

        // Clear color: the procedural sky's horizon tint (gamma-encoded to match
        // the shader), so empty regions read as sky.
        const Vec3& h = sky.horizonColor;
        clearColor_ = {std::pow(std::max(0.0, (double)h.x), 1.0 / 2.2),
                       std::pow(std::max(0.0, (double)h.y), 1.0 / 2.2),
                       std::pow(std::max(0.0, (double)h.z), 1.0 / 2.2), 1.0};
    }

    void drawMesh(MeshHandle handle, const Mat4& transform,
                  const RenderMaterial& material) override {
        const GpuMesh* m = resolve(handle);
        if (!m || m->indexCount == 0) return;

        QueuedDraw qd;
        qd.mesh = handle.index - 1;
        packMat4(transform, qd.data.model);
        qd.data.albedoMetallic[0] = (float)material.albedo.x;
        qd.data.albedoMetallic[1] = (float)material.albedo.y;
        qd.data.albedoMetallic[2] = (float)material.albedo.z;
        qd.data.albedoMetallic[3] = material.metallic;
        qd.data.emissionRough[0] = (float)material.emission.x;
        qd.data.emissionRough[1] = (float)material.emission.y;
        qd.data.emissionRough[2] = (float)material.emission.z;
        qd.data.emissionRough[3] = material.roughness;
        qd.data.surfaceFlags[0] = static_cast<uint32_t>(material.surface());
        qd.data.surfaceFlags[1] = material.flags;
        qd.data.surfaceFlags[2] = mapBits(material);
        qd.data.surfaceFlags[3] = 0;
        qd.mat = matKeyOf(material);
        // World-space bounding sphere for shadow-caster culling: transform the
        // mesh-local sphere center and scale the radius by the largest column
        // norm of the upper 3x3 (a conservative bound for non-uniform scale).
        qd.wCenter = transform.transformPoint(m->bounds.center);
        Real maxScaleSq = 0;
        for (int c = 0; c < 3; ++c) {
            Real s = transform.m[0][c] * transform.m[0][c]
                   + transform.m[1][c] * transform.m[1][c]
                   + transform.m[2][c] * transform.m[2][c];
            maxScaleSq = std::max(maxScaleSq, s);
        }
        qd.wRadius = m->bounds.radius * std::sqrt(maxScaleSq);
        draws_.push_back(qd);
    }

    // The five map handles as a cache key (0 = none).
    static MaterialKey matKeyOf(const RenderMaterial& m) {
        return {m.albedoMap.index, m.normalMap.index, m.metallicRoughnessMap.index,
                m.emissiveMap.index, m.aoMap.index};
    }
    // Which maps are present (bit0 albedo, 1 normal, 2 MR, 3 emissive, 4 AO), so
    // the shader multiplies by a sampled map only when there is one.
    static uint32_t mapBits(const RenderMaterial& m) {
        return (m.albedoMap.index ? 1u : 0u) | (m.normalMap.index ? 2u : 0u)
             | (m.metallicRoughnessMap.index ? 4u : 0u) | (m.emissiveMap.index ? 8u : 0u)
             | (m.aoMap.index ? 16u : 0u);
    }

    // Fill a GpuDraw's material fields (model unused for instanced draws).
    static void fillMaterial(GpuDraw& d, const RenderMaterial& material) {
        d.albedoMetallic[0] = (float)material.albedo.x;
        d.albedoMetallic[1] = (float)material.albedo.y;
        d.albedoMetallic[2] = (float)material.albedo.z;
        d.albedoMetallic[3] = material.metallic;
        d.emissionRough[0] = (float)material.emission.x;
        d.emissionRough[1] = (float)material.emission.y;
        d.emissionRough[2] = (float)material.emission.z;
        d.emissionRough[3] = material.roughness;
        d.surfaceFlags[0] = static_cast<uint32_t>(material.surface());
        d.surfaceFlags[1] = material.flags;
        d.surfaceFlags[2] = mapBits(material);
        d.surfaceFlags[3] = 0;
    }

    // Real GPU instancing: one draw per group, per-instance models in a packed
    // instance buffer (uploaded in endFrame). Overrides the drawMesh loop.
    void drawMeshInstanced(MeshHandle handle, const std::vector<Mat4>& transforms,
                           const RenderMaterial& material) override {
        const GpuMesh* m = resolve(handle);
        if (!m || m->indexCount == 0 || transforms.empty()) return;
        InstancedGroup g;
        g.mesh = handle.index - 1;
        g.instanceOffset = static_cast<uint32_t>(instanceStaging_.size() / 16);
        g.instanceCount = static_cast<uint32_t>(transforms.size());
        fillMaterial(g.data, material);
        g.mat = matKeyOf(material);
        instanceStaging_.resize(instanceStaging_.size() + transforms.size() * 16);
        float* dst = instanceStaging_.data() + g.instanceOffset * 16;
        for (const Mat4& t : transforms) { packMat4(t, dst); dst += 16; }
        instancedGroups_.push_back(g);
    }

    // CDLOD terrain node (identity transform; morph target baked in tangent).
    void drawTerrain(MeshHandle handle, const RenderMaterial& material,
                     float morphStart, float morphEnd) override {
        const GpuMesh* m = resolve(handle);
        if (!m || m->indexCount == 0) return;
        QueuedDraw qd;
        qd.mesh = handle.index - 1;
        packMat4(Mat4(), qd.data.model);       // identity: verts are world-space
        fillMaterial(qd.data, material);
        qd.data.terrainMorph[0] = morphStart;
        qd.data.terrainMorph[1] = morphEnd;
        qd.mat = matKeyOf(material);
        terrainDraws_.push_back(qd);
    }

    void endFrame() override {
        if (!device_ || !surface_) return;

        // Orchestration only — each render-graph stage lives in its own
        // record*/update* method below. Order matters: the cascade fit feeds
        // the globals upload, the draw sort feeds the uniform staging, and all
        // passes record into one command encoder submitted once at the end.
        FramePlan plan;
        updateCascades(plan);
        uploadFrameData(plan);
        planPostEffects(plan);

        WGPUSurfaceTexture surfaceTexture = {};
        wgpuSurfaceGetCurrentTexture(surface_, &surfaceTexture);
        if (!frameDiagLogged_) {
            frameDiagLogged_ = true;
            LOG_INFO("WebGPU frame0: %zu draws, status=%d, %dx%d, shadow=%d dist=%.0f, lights=%d",
                     draws_.size(), static_cast<int>(surfaceTexture.status),
                     width_, height_, shadowOn_ ? 1 : 0, shadowDistance_, globals_.counts[0]);
        }
        if (!surfaceTexture.texture) {
            LOG_WARN("WebGPU: no current surface texture this frame");
            return;
        }
        WGPUTextureView backbuffer = wgpuTextureCreateView(surfaceTexture.texture, nullptr);

        WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device_, nullptr);

        recordShadowPasses(encoder, plan);
        recordMainPass(encoder, plan);
        recordAtmosphere(encoder, plan);   // additive glow into HDR, before post
        recordPostStack(encoder, plan);
        recordComposite(encoder, backbuffer, plan);

        WGPUCommandBuffer commands = wgpuCommandEncoderFinish(encoder, nullptr);
        wgpuQueueSubmit(queue_, 1, &commands);

        wgpuCommandBufferRelease(commands);
        wgpuCommandEncoderRelease(encoder);
        wgpuTextureViewRelease(backbuffer);
        wgpuTextureRelease(surfaceTexture.texture);
        // The browser presents automatically after the queue work resolves;
        // there is no wgpuSurfacePresent on the web.
    }

private:
    // Per-frame pass plan: which post passes run, the dynamic-buffer slot
    // layout, the cascade cull data, and the shadow-cull counters — computed
    // once per frame and threaded through the record* stages.
    struct FramePlan {
        size_t terrainBase = 0;              // first terrain slot in the draw buffer
        bool ssaoOn = false, ssaoForDebug = false, ssrOn = false, bloomOn = false;
        bool needGbuf = false;               // main pass stores depth + G-buffer?
        Mat4 cascadeCullView[4];             // per-cascade light view (cull space)
        Real cascadeCullRadius[4] = {0, 0, 0, 0};
        size_t shadowCulled = 0, shadowConsidered = 0;
    };

    // Cascade fit -> globals_ (cascadeVP/split/shadowParams) + the plan's cull
    // data. The split/fit math itself is the shared, unit-tested
    // computeCascadeFits (../cascade_fit.h) — the same function the other
    // backends are meant to adopt (WebGPU standard-Z: NDC near 0, far 1).
    void updateCascades(FramePlan& plan) {
        activeCascades_ = 0;
        if (!shadowOn_) {
            globals_.counts[3] = 0;
            globals_.shadowParams[0] = 0.0f;
            return;
        }
        CascadeFitInput in;
        in.camViewProj = camVP_;
        in.cameraEye = cameraEye_;
        in.sunDir = sunDir_;
        in.camNear = camNear_;
        in.camFar = camFar_;
        in.shadowDistance = static_cast<Real>(shadowDistance_);
        in.splitLambda = shadowParams.splitLambda;
        in.cascadeCount = shadowCascadeCount_;
        in.shadowMapSize = kShadowMapSize;
        CascadeFit fits[4];
        int cc = computeCascadeFits(in, fits);
        for (int c = 0; c < cc; ++c) {
            packMat4(fits[c].viewProj, globals_.cascadeVP[c]);
            globals_.cascadeSplit[c] = static_cast<float>(fits[c].splitFar);
            plan.cascadeCullView[c] = fits[c].lightView;
            plan.cascadeCullRadius[c] = fits[c].radius;
        }
        activeCascades_ = cc;
        globals_.counts[3] = cc;
        globals_.shadowParams[0] = 1.0f;
        globals_.shadowParams[1] = shadowDepthBias_;
        globals_.shadowParams[2] = shadowNormalBias_;
        globals_.shadowParams[3] = shadowPcf_;
    }

    // Wind clock, scene globals upload, the material/mesh draw sort, the
    // strided per-draw uniform staging, and the instance-model upload.
    void uploadFrameData(FramePlan& plan) {
        // Wind clock (ambient.w): a monotonic time for FLAG_WIND vertex sway.
        // Advance by real elapsed time — rAF runs at the display rate (60/120/144
        // Hz, or throttled), so a fixed 1/60 step would tie sway speed to fps.
        double nowMs = emscripten_get_now();
        if (windLastMs_ > 0.0)
            windTime_ += static_cast<float>(std::min((nowMs - windLastMs_) / 1000.0, 0.1));
        windLastMs_ = nowMs;
        globals_.ambient[3] = windTime_;

        // Upload scene globals.
        wgpuQueueWriteBuffer(queue_, globalBuf_, 0, &globals_, sizeof(GpuGlobals));

        // Sort draws by (material, mesh) so consecutive draws share bindings and
        // the loops below can skip redundant setBindGroup/setVertex/setIndex
        // calls. Under Emscripten every encoder call crosses the WASM->JS
        // boundary — the dominant per-draw CPU cost — so batching identical
        // state adjacently is the cheapest big win. Safe to reorder: the main
        // pass is opaque-only (alpha-test discards, no blending), so draw order
        // does not affect the image. Must happen BEFORE the uniform staging
        // below (slot i in the dynamic buffer = draws_[i] after the sort).
        std::sort(draws_.begin(), draws_.end(),
                  [](const QueuedDraw& a, const QueuedDraw& b) {
                      if (a.mat.albedo != b.mat.albedo) return a.mat.albedo < b.mat.albedo;
                      if (a.mat.normal != b.mat.normal) return a.mat.normal < b.mat.normal;
                      if (a.mat.mr != b.mat.mr) return a.mat.mr < b.mat.mr;
                      if (a.mat.emissive != b.mat.emissive) return a.mat.emissive < b.mat.emissive;
                      if (a.mat.ao != b.mat.ao) return a.mat.ao < b.mat.ao;
                      return a.mesh < b.mesh;
                  });

        // Grow the per-draw uniform buffer (and rebuild the bind group bound to
        // it) if this frame needs more slots than the current capacity. Instanced
        // groups ride the same dynamic buffer (their material) after the regular
        // draws, at slots [draws_.size() .. draws_.size()+instancedGroups_.size()).
        // Slot layout in the dynamic draw buffer: regular draws, then instanced
        // group materials, then terrain draws.
        plan.terrainBase = draws_.size() + instancedGroups_.size();
        size_t totalSlots = plan.terrainBase + terrainDraws_.size();
        ensureDrawCapacity(totalSlots);

        if (totalSlots > 0) {
            // Reused member scratch (grows monotonically): a fresh
            // vector<uint8_t>(N, 0) here would heap-allocate and zero-fill
            // 256 B x draws every frame. Only the GpuDraw prefix of each slot is
            // ever read by the shader, so the stride padding can stay stale.
            if (drawStaging_.size() < totalSlots * kDrawStride)
                drawStaging_.resize(totalSlots * kDrawStride);
            uint8_t* staging = drawStaging_.data();
            for (size_t i = 0; i < draws_.size(); ++i)
                std::memcpy(staging + i * kDrawStride, &draws_[i].data, sizeof(GpuDraw));
            for (size_t i = 0; i < instancedGroups_.size(); ++i)
                std::memcpy(staging + (draws_.size() + i) * kDrawStride,
                            &instancedGroups_[i].data, sizeof(GpuDraw));
            for (size_t i = 0; i < terrainDraws_.size(); ++i)
                std::memcpy(staging + (plan.terrainBase + i) * kDrawStride,
                            &terrainDraws_[i].data, sizeof(GpuDraw));
            wgpuQueueWriteBuffer(queue_, drawBuf_, 0, staging, totalSlots * kDrawStride);
        }

        // Upload per-instance models (grow the instance buffer as needed).
        size_t instanceCount = instanceStaging_.size() / 16;
        if (instanceCount > 0) {
            if (instanceCount > instanceCapacity_) {
                if (instanceBuf_) wgpuBufferRelease(instanceBuf_);
                instanceCapacity_ = std::max<size_t>(instanceCount, instanceCapacity_ * 2);
                WGPUBufferDescriptor id = {};
                id.usage = WGPUBufferUsage_Vertex | WGPUBufferUsage_CopyDst;
                id.size = instanceCapacity_ * 64;
                instanceBuf_ = wgpuDeviceCreateBuffer(device_, &id);
            }
            wgpuQueueWriteBuffer(queue_, instanceBuf_, 0, instanceStaging_.data(),
                                 instanceStaging_.size() * sizeof(float));
        }

    }

    // Which post passes actually run this frame (zero-strength = off), and
    // whether the main pass needs to store depth + G-buffer at all.
    void planPostEffects(FramePlan& plan) {
        // Which post passes actually run this frame — decided up front so the
        // main pass can skip storing attachments nothing will read. Zero-strength
        // effects are treated as off (their passes would render into targets the
        // composite multiplies by 0).
        plan.ssaoOn = ssaoEnabled && globals_.counts[1] == 0 && ssaoParams.intensity > 0.0f;
        plan.ssaoForDebug = globals_.counts[1] == 1;   // AO debug view shows AO even if disabled
        plan.ssrOn = ssrEnabled && globals_.counts[1] == 0 && ssrParams.blendStrength > 0.0f;
        plan.bloomOn = bloomEnabled && globals_.counts[1] == 0 && bloomParams.intensity > 0.0f;
        // Depth + G-buffer are only ever sampled by SSAO/SSR; when neither runs,
        // Discard skips the attachment writeback (a real win on tiler GPUs).
        plan.needGbuf = plan.ssaoOn || plan.ssaoForDebug || plan.ssrOn;

    }

    // Depth-only sun passes, one per cascade, with per-cascade caster culling.
    void recordShadowPasses(WGPUCommandEncoder encoder, FramePlan& plan) {
        // Cascaded shadow depth passes (sun): one depth pass per cascade layer,
        // before the main pass. The cascade index rides shadowIdxGroup_ via a
        // dynamic offset; vs_shadow uses it to pick cascadeVP[c].
        if (shadowOn_ && (!draws_.empty() || !instancedGroups_.empty() || !terrainDraws_.empty())) {
            for (int c = 0; c < activeCascades_; ++c) {
                // Caster culling: skip draws whose world sphere misses this
                // cascade's ortho box in light-space XY. Z is never culled — a
                // caster outside the slice toward the light still casts into it.
                // Near cascades have small XY extents, so this drops most of the
                // scene from cascade 0/1. Instanced groups are not culled (no
                // per-instance bounds).
                const Mat4& cullView = plan.cascadeCullView[c];
                const Real cullR = plan.cascadeCullRadius[c];
                auto casterVisible = [&](const Vec3& wc, Real wr) {
                    Vec3 p = cullView.transformPoint(wc);
                    return std::abs(p.x) <= cullR + wr && std::abs(p.y) <= cullR + wr;
                };
                WGPURenderPassDepthStencilAttachment sdepth = {};
                sdepth.view = shadowLayerViews_[c];
                sdepth.depthLoadOp = WGPULoadOp_Clear;
                sdepth.depthStoreOp = WGPUStoreOp_Store;
                sdepth.depthClearValue = 1.0f;
                WGPURenderPassDescriptor sPass = {};
                sPass.colorAttachmentCount = 0;
                sPass.depthStencilAttachment = &sdepth;
                WGPURenderPassEncoder spass = wgpuCommandEncoderBeginRenderPass(encoder, &sPass);
                wgpuRenderPassEncoderSetPipeline(spass, shadowPipeline_);
                uint32_t cascadeOff = static_cast<uint32_t>(c * kDrawStride);
                // Group 1 (cascade index) persists across the pipeline switches
                // below — the shadow pipelines share one layout. Bind it once.
                wgpuRenderPassEncoderSetBindGroup(spass, 1, shadowIdxGroup_, 1, &cascadeOff);
                // Draws are material/mesh-sorted; skip re-binding a mesh the
                // previous shadow draw already bound (materials don't exist in
                // the depth-only pass).
                uint32_t sLastMesh = UINT32_MAX;
                for (size_t i = 0; i < draws_.size(); ++i) {
                    const GpuMesh& m = meshes_[draws_[i].mesh];
                    if (!m.vertexBuffer) continue;   // removed mid-frame
                    // Debug-gizmo overlays never cast shadows.
                    if (draws_[i].data.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY) continue;
                    ++plan.shadowConsidered;
                    if (!casterVisible(draws_[i].wCenter, draws_[i].wRadius)) {
                        ++plan.shadowCulled;
                        continue;
                    }
                    uint32_t dynOffset = static_cast<uint32_t>(i * kDrawStride);
                    wgpuRenderPassEncoderSetBindGroup(spass, 0, bindGroup_, 1, &dynOffset);
                    if (draws_[i].mesh != sLastMesh) {
                        wgpuRenderPassEncoderSetVertexBuffer(spass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                        wgpuRenderPassEncoderSetIndexBuffer(spass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                            0, WGPU_WHOLE_SIZE);
                        sLastMesh = draws_[i].mesh;
                    }
                    wgpuRenderPassEncoderDrawIndexed(spass, m.indexCount, 1, 0, 0, 0);
                }
                // Instanced groups cast shadows too — except FLAG_OVERLAY
                // gizmos, which never cast (Metal/Vulkan parity).
                if (!instancedGroups_.empty()) {
                    wgpuRenderPassEncoderSetPipeline(spass, instancedShadowPipeline_);
                    for (size_t i = 0; i < instancedGroups_.size(); ++i) {
                        const InstancedGroup& g = instancedGroups_[i];
                        if (g.data.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY)
                            continue;
                        const GpuMesh& m = meshes_[g.mesh];
                        if (!m.vertexBuffer) continue;   // removed mid-frame
                        uint32_t dynOffset = static_cast<uint32_t>((draws_.size() + i) * kDrawStride);
                        wgpuRenderPassEncoderSetBindGroup(spass, 0, bindGroup_, 1, &dynOffset);
                        wgpuRenderPassEncoderSetVertexBuffer(spass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                        wgpuRenderPassEncoderSetVertexBuffer(spass, 1, instanceBuf_,
                                                             static_cast<uint64_t>(g.instanceOffset) * 64, WGPU_WHOLE_SIZE);
                        wgpuRenderPassEncoderSetIndexBuffer(spass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                            0, WGPU_WHOLE_SIZE);
                        wgpuRenderPassEncoderDrawIndexed(spass, m.indexCount, g.instanceCount, 0, 0, 0);
                    }
                }
                // Terrain nodes cast morphed shadows (matching the receiver).
                // Their vertices are world-space (identity transform), so the
                // mesh bounds ARE the world bounds — cull directly on those.
                if (!terrainDraws_.empty()) {
                    wgpuRenderPassEncoderSetPipeline(spass, terrainShadowPipeline_);
                    for (size_t i = 0; i < terrainDraws_.size(); ++i) {
                        const GpuMesh& m = meshes_[terrainDraws_[i].mesh];
                        if (!m.vertexBuffer) continue;   // removed mid-frame
                        ++plan.shadowConsidered;
                        if (!casterVisible(m.bounds.center, m.bounds.radius)) {
                            ++plan.shadowCulled;
                            continue;
                        }
                        uint32_t dynOffset = static_cast<uint32_t>((plan.terrainBase + i) * kDrawStride);
                        wgpuRenderPassEncoderSetBindGroup(spass, 0, bindGroup_, 1, &dynOffset);
                        wgpuRenderPassEncoderSetVertexBuffer(spass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                        wgpuRenderPassEncoderSetIndexBuffer(spass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                            0, WGPU_WHOLE_SIZE);
                        wgpuRenderPassEncoderDrawIndexed(spass, m.indexCount, 1, 0, 0, 0);
                    }
                }
                wgpuRenderPassEncoderEnd(spass);
                wgpuRenderPassEncoderRelease(spass);
            }
        }

    }

    // Sky + lit geometry (regular, instanced, terrain) into the HDR target +
    // G-buffer MRT, with sorted-state bind skipping.
    void recordMainPass(WGPUCommandEncoder encoder, const FramePlan& plan) {
        WGPURenderPassColorAttachment colorAtt[2] = {};
        colorAtt[0].view = hdrView_;   // scene -> HDR target; composite -> swapchain
        colorAtt[0].depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        colorAtt[0].loadOp = WGPULoadOp_Clear;
        colorAtt[0].storeOp = WGPUStoreOp_Store;
        colorAtt[0].clearValue = clearColor_;
        colorAtt[1].view = gbufView_;  // material G-buffer (normal, roughness)
        colorAtt[1].depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        colorAtt[1].loadOp = WGPULoadOp_Clear;
        colorAtt[1].storeOp = plan.needGbuf ? WGPUStoreOp_Store : WGPUStoreOp_Discard;
        colorAtt[1].clearValue = {0.0, 0.0, 0.0, 1.0};

        WGPURenderPassDepthStencilAttachment depth = {};
        depth.view = depthView_;
        depth.depthLoadOp = WGPULoadOp_Clear;
        depth.depthStoreOp = plan.needGbuf ? WGPUStoreOp_Store : WGPUStoreOp_Discard;
        depth.depthClearValue = 1.0f;

        WGPURenderPassDescriptor passDesc = {};
        passDesc.colorAttachmentCount = 2;
        passDesc.colorAttachments = colorAtt;
        passDesc.depthStencilAttachment = &depth;

        WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);

        // Procedural sky background first (depth-compare Always, no write), so
        // the meshes paint over it where they pass the depth test.
        wgpuRenderPassEncoderSetPipeline(pass, skyPipeline_);
        wgpuRenderPassEncoderSetBindGroup(pass, 0, skyBindGroup_, 0, nullptr);
        wgpuRenderPassEncoderDraw(pass, 3, 1, 0, 0);

        wgpuRenderPassEncoderSetPipeline(pass, pipeline_);
        wgpuRenderPassEncoderSetBindGroup(pass, 1, shadowSampleGroup_, 0, nullptr);

        // Draws are sorted by (material, mesh) — skip re-binding state a
        // consecutive draw shares. The per-draw dynamic offset (group 0) always
        // changes; everything else is elided when unchanged. Group 2 bindings
        // persist across the pipeline switches below (identical pipeline
        // layouts), so the material cache carries through all three loops.
        // lastMat also short-circuits the per-draw hash lookup in
        // materialGroupFor. Bind counters feed a one-shot diagnostic.
        const MaterialKey* lastMat = nullptr;
        uint32_t lastMesh = UINT32_MAX;
        size_t matBinds = 0, meshBinds = 0;
        auto bindMaterial = [&](const MaterialKey& k) {
            if (lastMat && k == *lastMat) return;
            wgpuRenderPassEncoderSetBindGroup(pass, 2, materialGroupFor(k), 0, nullptr);
            lastMat = &k;
            ++matBinds;
        };

        bool anyOverlay = false;
        for (size_t i = 0; i < draws_.size(); ++i) {
            const GpuMesh& m = meshes_[draws_[i].mesh];
            if (!m.vertexBuffer) continue;   // removed mid-frame
            // FLAG_OVERLAY gizmos draw LAST with the depth-off pipeline (below).
            if (draws_[i].data.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY) {
                anyOverlay = true;
                continue;
            }
            uint32_t dynOffset = static_cast<uint32_t>(i * kDrawStride);
            wgpuRenderPassEncoderSetBindGroup(pass, 0, bindGroup_, 1, &dynOffset);
            bindMaterial(draws_[i].mat);
            if (draws_[i].mesh != lastMesh) {
                wgpuRenderPassEncoderSetVertexBuffer(pass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetIndexBuffer(pass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                    0, WGPU_WHOLE_SIZE);
                lastMesh = draws_[i].mesh;
                ++meshBinds;
            }
            wgpuRenderPassEncoderDrawIndexed(pass, m.indexCount, 1, 0, 0, 0);
            stats_.drawCalls++;
            stats_.trianglesDrawn += m.indexCount / 3;
        }

        // Instanced groups: one draw each. Group 1 (shadow sampler) persists
        // across the pipeline switch (identical layouts) — no re-bind needed.
        // The instance vertex buffer (slot 1) invalidates nothing in slot 0, but
        // each group binds its own slot-0 mesh, so reset the mesh cache.
        if (!instancedGroups_.empty()) {
            wgpuRenderPassEncoderSetPipeline(pass, instancedPipeline_);
            lastMesh = UINT32_MAX;
            for (size_t i = 0; i < instancedGroups_.size(); ++i) {
                const InstancedGroup& g = instancedGroups_[i];
                // FLAG_OVERLAY gizmo groups draw LAST, depth-off (below).
                if (g.data.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY)
                    continue;
                const GpuMesh& m = meshes_[g.mesh];
                if (!m.vertexBuffer) continue;   // removed mid-frame
                uint32_t dynOffset = static_cast<uint32_t>((draws_.size() + i) * kDrawStride);
                wgpuRenderPassEncoderSetBindGroup(pass, 0, bindGroup_, 1, &dynOffset);
                bindMaterial(g.mat);
                wgpuRenderPassEncoderSetVertexBuffer(pass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetVertexBuffer(pass, 1, instanceBuf_,
                                                     static_cast<uint64_t>(g.instanceOffset) * 64, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetIndexBuffer(pass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                    0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderDrawIndexed(pass, m.indexCount, g.instanceCount, 0, 0, 0);
                stats_.instancedDrawCalls++;
                stats_.totalInstances += g.instanceCount;
                stats_.trianglesDrawn += (m.indexCount / 3) * g.instanceCount;
            }
        }

        // Terrain nodes (CDLOD morph). Group 1 persists here too.
        if (!terrainDraws_.empty()) {
            wgpuRenderPassEncoderSetPipeline(pass, terrainPipeline_);
            for (size_t i = 0; i < terrainDraws_.size(); ++i) {
                const GpuMesh& m = meshes_[terrainDraws_[i].mesh];
                if (!m.vertexBuffer) continue;   // removed mid-frame
                uint32_t dynOffset = static_cast<uint32_t>((plan.terrainBase + i) * kDrawStride);
                wgpuRenderPassEncoderSetBindGroup(pass, 0, bindGroup_, 1, &dynOffset);
                bindMaterial(terrainDraws_[i].mat);
                wgpuRenderPassEncoderSetVertexBuffer(pass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetIndexBuffer(pass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                    0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderDrawIndexed(pass, m.indexCount, 1, 0, 0, 0);
                stats_.drawCalls++;
                stats_.trianglesDrawn += m.indexCount / 3;
            }
        }

        // Debug-gizmo overlay (FLAG_OVERLAY): drawn after everything with depth
        // test Always / no write, so agent widgets and plan outlines stay
        // visible through the road solid and buildings (Vulkan/Metal parity).
        if (anyOverlay && overlayPipeline_) {
            wgpuRenderPassEncoderSetPipeline(pass, overlayPipeline_);
            lastMesh = UINT32_MAX;
            for (size_t i = 0; i < draws_.size(); ++i) {
                if (!(draws_[i].data.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY)) continue;
                const GpuMesh& m = meshes_[draws_[i].mesh];
                if (!m.vertexBuffer) continue;
                uint32_t dynOffset = static_cast<uint32_t>(i * kDrawStride);
                wgpuRenderPassEncoderSetBindGroup(pass, 0, bindGroup_, 1, &dynOffset);
                bindMaterial(draws_[i].mat);
                if (draws_[i].mesh != lastMesh) {
                    wgpuRenderPassEncoderSetVertexBuffer(pass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                    wgpuRenderPassEncoderSetIndexBuffer(pass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                        0, WGPU_WHOLE_SIZE);
                    lastMesh = draws_[i].mesh;
                }
                wgpuRenderPassEncoderDrawIndexed(pass, m.indexCount, 1, 0, 0, 0);
                stats_.drawCalls++;
                stats_.trianglesDrawn += m.indexCount / 3;
            }
        }

        // Instanced FLAG_OVERLAY gizmos (citysim widgets — rings, arrows, plan
        // outlines): drawn after everything, depth Always / no write, so they
        // stay visible over the CDLOD terrain and the road solid.
        if (instancedOverlayPipeline_) {
            bool bound = false;
            for (size_t i = 0; i < instancedGroups_.size(); ++i) {
                const InstancedGroup& g = instancedGroups_[i];
                if (!(g.data.surfaceFlags[1] & RenderMaterial::FLAG_OVERLAY))
                    continue;
                const GpuMesh& m = meshes_[g.mesh];
                if (!m.vertexBuffer) continue;
                if (!bound) {
                    wgpuRenderPassEncoderSetPipeline(pass, instancedOverlayPipeline_);
                    bound = true;
                }
                uint32_t dynOffset = static_cast<uint32_t>((draws_.size() + i) * kDrawStride);
                wgpuRenderPassEncoderSetBindGroup(pass, 0, bindGroup_, 1, &dynOffset);
                bindMaterial(g.mat);
                wgpuRenderPassEncoderSetVertexBuffer(pass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetVertexBuffer(pass, 1, instanceBuf_,
                                                     static_cast<uint64_t>(g.instanceOffset) * 64, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetIndexBuffer(pass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                    0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderDrawIndexed(pass, m.indexCount, g.instanceCount, 0, 0, 0);
                stats_.instancedDrawCalls++;
                stats_.totalInstances += g.instanceCount;
                stats_.trianglesDrawn += (m.indexCount / 3) * g.instanceCount;
            }
        }

        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);

        if (!bindDiagLogged_ && (!draws_.empty() || !terrainDraws_.empty())) {
            bindDiagLogged_ = true;
            LOG_INFO("WebGPU binds: %zu draws -> %zu material binds, %zu mesh binds; "
                     "shadow casters culled %zu/%zu (across %d cascades)",
                     draws_.size(), matBinds, meshBinds,
                     plan.shadowCulled, plan.shadowConsidered, activeCascades_);
        }

    }

    // Screen-space effects reading the main pass outputs: SSAO, SSR, bloom.
    void recordPostStack(WGPUCommandEncoder encoder, const FramePlan& plan) {
        // SSAO: reconstruct from the just-written depth into the AO target.
        if (plan.ssaoOn || plan.ssaoForDebug) {
            GpuSsao us = {};
            std::memcpy(us.invViewProjection, globals_.invViewProjection, sizeof(us.invViewProjection));
            std::memcpy(us.viewProjection, globals_.viewProjection, sizeof(us.viewProjection));
            std::memcpy(us.cameraPosition, globals_.cameraPosition, sizeof(us.cameraPosition));
            us.params[0] = ssaoParams.radius;
            us.params[1] = ssaoParams.bias;
            us.params[2] = ssaoParams.intensity;
            us.params[3] = ssaoParams.aoFloor;
            // xy = 1/effectRes (uv from the scaled AO frag coords);
            // zw = full resolution (to index the full-res depth/G-buffer).
            us.texel[0] = 1.0f / static_cast<float>(fxW_);
            us.texel[1] = 1.0f / static_cast<float>(fxH_);
            us.texel[2] = static_cast<float>(width_);
            us.texel[3] = static_cast<float>(height_);
            wgpuQueueWriteBuffer(queue_, ssaoUbo_, 0, &us, sizeof(us));

            WGPURenderPassColorAttachment a = {};
            a.view = ssaoView_; a.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
            a.loadOp = WGPULoadOp_Clear; a.storeOp = WGPUStoreOp_Store;
            a.clearValue = {1.0, 1.0, 1.0, 1.0};   // 1 = no occlusion
            WGPURenderPassDescriptor pd = {};
            pd.colorAttachmentCount = 1; pd.colorAttachments = &a;
            WGPURenderPassEncoder e = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
            wgpuRenderPassEncoderSetPipeline(e, ssaoPipeline_);
            wgpuRenderPassEncoderSetBindGroup(e, 0, ssaoGroup_, 0, nullptr);
            wgpuRenderPassEncoderDraw(e, 3, 1, 0, 0);
            wgpuRenderPassEncoderEnd(e);
            wgpuRenderPassEncoderRelease(e);
        }

        // SSR: reflect the HDR color off the G-buffer into the SSR target.
        if (plan.ssrOn) {
            GpuSsr ur = {};
            std::memcpy(ur.invViewProjection, globals_.invViewProjection, sizeof(ur.invViewProjection));
            std::memcpy(ur.viewProjection, globals_.viewProjection, sizeof(ur.viewProjection));
            std::memcpy(ur.cameraPosition, globals_.cameraPosition, sizeof(ur.cameraPosition));
            ur.params[0] = ssrParams.maxRayDist;
            ur.params[1] = ssrParams.thickness;
            ur.params[2] = ssrParams.thicknessFar;
            ur.params[3] = ssrParams.maxRoughness;
            ur.params2[0] = static_cast<float>(camNear_);
            ur.params2[1] = static_cast<float>(camFar_);
            ur.params2[2] = ssrParams.stride;
            // xy = 1/effectRes (uv from the scaled SSR frag coords);
            // zw = full resolution (to index the full-res depth/G-buffer/HDR).
            ur.texel[0] = 1.0f / static_cast<float>(fxW_);
            ur.texel[1] = 1.0f / static_cast<float>(fxH_);
            ur.texel[2] = static_cast<float>(width_);
            ur.texel[3] = static_cast<float>(height_);
            wgpuQueueWriteBuffer(queue_, ssrUbo_, 0, &ur, sizeof(ur));

            WGPURenderPassColorAttachment a = {};
            a.view = ssrView_; a.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
            a.loadOp = WGPULoadOp_Clear; a.storeOp = WGPUStoreOp_Store;
            a.clearValue = {0.0, 0.0, 0.0, 0.0};
            WGPURenderPassDescriptor pd = {};
            pd.colorAttachmentCount = 1; pd.colorAttachments = &a;
            WGPURenderPassEncoder e = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
            wgpuRenderPassEncoderSetPipeline(e, ssrPipeline_);
            wgpuRenderPassEncoderSetBindGroup(e, 0, ssrGroup_, 0, nullptr);
            wgpuRenderPassEncoderDraw(e, 3, 1, 0, 0);
            wgpuRenderPassEncoderEnd(e);
            wgpuRenderPassEncoderRelease(e);
        }

        // Bloom: bright-pass (HDR -> bloomA) then a separable blur (A->B->A).
        // Only when enabled and not in a debug view (debug bypasses post).
        if (plan.bloomOn) {
            GpuBloom ub = {};
            ub.params[0] = bloomParams.threshold;
            ub.params[1] = bloomParams.knee;
            ub.params[2] = bloomParams.intensity;
            float tx = 1.0f / static_cast<float>(bloomW_);
            float ty = 1.0f / static_cast<float>(bloomH_);
            // Bright pass: no blur step. H/V passes carry their texel direction.
            wgpuQueueWriteBuffer(queue_, bloomUboBright_, 0, &ub, sizeof(ub));
            ub.texel[0] = tx; ub.texel[1] = 0.0f;
            wgpuQueueWriteBuffer(queue_, bloomUboH_, 0, &ub, sizeof(ub));
            ub.texel[0] = 0.0f; ub.texel[1] = ty;
            wgpuQueueWriteBuffer(queue_, bloomUboV_, 0, &ub, sizeof(ub));

            auto bloomPass = [&](WGPUTextureView dst, WGPURenderPipeline pipe, WGPUBindGroup grp) {
                WGPURenderPassColorAttachment a = {};
                a.view = dst; a.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
                a.loadOp = WGPULoadOp_Clear; a.storeOp = WGPUStoreOp_Store;
                a.clearValue = {0.0, 0.0, 0.0, 1.0};
                WGPURenderPassDescriptor pd = {};
                pd.colorAttachmentCount = 1; pd.colorAttachments = &a;
                WGPURenderPassEncoder e = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
                wgpuRenderPassEncoderSetPipeline(e, pipe);
                wgpuRenderPassEncoderSetBindGroup(e, 0, grp, 0, nullptr);
                wgpuRenderPassEncoderDraw(e, 3, 1, 0, 0);
                wgpuRenderPassEncoderEnd(e);
                wgpuRenderPassEncoderRelease(e);
            };
            bloomPass(bloomViewA_, bloomBrightPipeline_, bloomBrightGroup_);  // HDR -> A
            bloomPass(bloomViewB_, bloomBlurPipeline_, bloomBlurHGroup_);     // A -> B (H)
            bloomPass(bloomViewA_, bloomBlurPipeline_, bloomBlurVGroup_);     // B -> A (V)
        }

    }

    // Planetary atmosphere glow (procedural-planet-plan P3): a fullscreen triangle
    // that raymarches the atmosphere shell and ADDITIVELY blends the in-scattered
    // light into the HDR scene — after geometry, before post, so the limb halo
    // blooms (mirrors the Metal integration). No scene fetch: the additive blend
    // (One, One) composites, so the pass reads only its uniform and writes hdrView_
    // with loadOp Load. Camera + sun come from this frame's globals_; centre/radii/
    // coeffs from setAtmosphere().
    void recordAtmosphere(WGPUCommandEncoder encoder, const FramePlan& /*plan*/) {
        if (!atmosphere_.enabled || !atmospherePipeline_ || !hdrView_) return;
        const AtmosphereRenderParams& ap = atmosphere_;

        GpuAtmosphere ua = {};
        std::memcpy(ua.invViewProjection, globals_.invViewProjection,
                    sizeof(ua.invViewProjection));
        std::memcpy(ua.cameraPosition, globals_.cameraPosition, sizeof(ua.cameraPosition));
        // Sun direction/colour from the scene's primary directional light.
        ua.sunDirection[0] = (float)sunDir_.x;
        ua.sunDirection[1] = (float)sunDir_.y;
        ua.sunDirection[2] = (float)sunDir_.z;
        ua.sunDirection[3] = 0.0f;
        ua.planetCenter[0] = (float)ap.planetCenter.x;
        ua.planetCenter[1] = (float)ap.planetCenter.y;
        ua.planetCenter[2] = (float)ap.planetCenter.z;
        ua.planetCenter[3] = 0.0f;
        // Sun colour: the directional light's colour (lights[0], the sun when
        // present), matching the Metal integration — not the sky's sun tint.
        ua.sunColor[0] = globals_.lights[0].colorOuter[0];
        ua.sunColor[1] = globals_.lights[0].colorOuter[1];
        ua.sunColor[2] = globals_.lights[0].colorOuter[2];
        ua.sunColor[3] = ap.sunIntensity;
        ua.rayleighCoeff[0] = (float)ap.rayleighCoeff.x;
        ua.rayleighCoeff[1] = (float)ap.rayleighCoeff.y;
        ua.rayleighCoeff[2] = (float)ap.rayleighCoeff.z;
        ua.rayleighCoeff[3] = 0.0f;
        ua.radii[0] = ap.planetRadius;
        ua.radii[1] = ap.atmosphereRadius;
        ua.radii[2] = ap.rayleighScaleHeight;
        ua.radii[3] = ap.mieScaleHeight;
        ua.mie[0] = ap.mieCoeff;
        ua.mie[1] = ap.mieG;
        ua.mie[2] = (float)ap.viewSamples;
        ua.mie[3] = (float)ap.lightSamples;
        wgpuQueueWriteBuffer(queue_, atmosphereUbo_, 0, &ua, sizeof(ua));

        WGPURenderPassColorAttachment a = {};
        a.view = hdrView_; a.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        a.loadOp = WGPULoadOp_Load; a.storeOp = WGPUStoreOp_Store;   // keep the scene
        WGPURenderPassDescriptor pd = {};
        pd.colorAttachmentCount = 1; pd.colorAttachments = &a;
        WGPURenderPassEncoder e = wgpuCommandEncoderBeginRenderPass(encoder, &pd);
        wgpuRenderPassEncoderSetPipeline(e, atmospherePipeline_);
        wgpuRenderPassEncoderSetBindGroup(e, 0, atmosphereGroup_, 0, nullptr);
        wgpuRenderPassEncoderDraw(e, 3, 1, 0, 0);
        wgpuRenderPassEncoderEnd(e);
        wgpuRenderPassEncoderRelease(e);
    }

    // Tone map / grade the HDR scene (+ AO/SSR/bloom) onto the swapchain.
    void recordComposite(WGPUCommandEncoder encoder, WGPUTextureView backbuffer,
                         const FramePlan& plan) {
        // Composite the HDR target to the swapchain (view transform / post).
        GpuPost post = {};
        post.postParams[0] = globals_.postParams[0];
        post.postParams[1] = globals_.postParams[1];
        post.postParams[2] = globals_.postParams[2];
        post.postParams[3] = globals_.postParams[3];
        post.debugView[0] = globals_.counts[1];
        post.effects[0] = plan.bloomOn ? bloomParams.intensity : 0.0f;
        post.effects[1] = plan.ssaoOn ? 1.0f : 0.0f;   // SSAO strength (0..1)
        post.effects[2] = plan.ssrOn ? ssrParams.blendStrength : 0.0f;
        wgpuQueueWriteBuffer(queue_, postBuf_, 0, &post, sizeof(post));

        WGPURenderPassColorAttachment ccolor = {};
        ccolor.view = backbuffer;
        ccolor.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        ccolor.loadOp = WGPULoadOp_Clear;
        ccolor.storeOp = WGPUStoreOp_Store;
        ccolor.clearValue = {0.0, 0.0, 0.0, 1.0};
        WGPURenderPassDescriptor cpassDesc = {};
        cpassDesc.colorAttachmentCount = 1;
        cpassDesc.colorAttachments = &ccolor;
        WGPURenderPassEncoder cpass = wgpuCommandEncoderBeginRenderPass(encoder, &cpassDesc);
        wgpuRenderPassEncoderSetPipeline(cpass, compositePipeline_);
        wgpuRenderPassEncoderSetBindGroup(cpass, 0, compositeBindGroup_, 0, nullptr);
        wgpuRenderPassEncoderDraw(cpass, 3, 1, 0, 0);
        wgpuRenderPassEncoderEnd(cpass);
        wgpuRenderPassEncoderRelease(cpass);

    }

    struct GpuMesh {
        WGPUBuffer vertexBuffer = nullptr;
        WGPUBuffer indexBuffer = nullptr;
        uint32_t indexCount = 0;
        uint32_t generation = 0;
        BoundingSphere bounds;
    };
    struct QueuedDraw {
        uint32_t mesh;
        GpuDraw data;
        MaterialKey mat;
        Vec3 wCenter;      // world-space bounding sphere (mesh bounds x transform),
        Real wRadius = 0;  // for per-cascade shadow-caster culling
    };
    struct InstancedGroup {
        uint32_t mesh;
        GpuDraw data;             // shared material (model unused)
        uint32_t instanceOffset;  // first instance in instanceBuf_
        uint32_t instanceCount;
        MaterialKey mat;
    };

    // ---- async device acquisition callbacks --------------------------------

    static void onAdapter(WGPURequestAdapterStatus status, WGPUAdapter adapter,
                          WGPUStringView /*message*/, void* userdata1, void*) {
        auto* self = static_cast<WebGpuRenderer*>(userdata1);
        if (status == WGPURequestAdapterStatus_Success) self->adapter_ = adapter;
        self->adapterDone_ = true;
    }
    static void onDevice(WGPURequestDeviceStatus status, WGPUDevice device,
                         WGPUStringView /*message*/, void* userdata1, void*) {
        auto* self = static_cast<WebGpuRenderer*>(userdata1);
        if (status == WGPURequestDeviceStatus_Success) self->device_ = device;
        self->deviceDone_ = true;
    }
    static void onUncapturedError(WGPUDevice const*, WGPUErrorType type,
                                  WGPUStringView message, void*, void*) {
        LOG_ERROR("WebGPU uncaptured error (type %d): %.*s",
                  static_cast<int>(type),
                  static_cast<int>(message.length), message.data ? message.data : "");
    }

    // ---- resource helpers --------------------------------------------------

    WGPUBuffer createBuffer(WGPUBufferUsage usage, const void* data, size_t size) {
        // WebGPU requires buffer sizes (and mapped writes) to be 4-byte aligned.
        size_t aligned = (size + 3) & ~size_t(3);
        WGPUBufferDescriptor desc = {};
        desc.usage = usage;
        desc.size = aligned;
        WGPUBuffer buf = wgpuDeviceCreateBuffer(device_, &desc);
        if (data && size > 0) wgpuQueueWriteBuffer(queue_, buf, 0, data, size);
        return buf;
    }

    // Create a sampled 2D texture and upload the base RGBA8 mip level.
    WGPUTexture createTexture2D(int w, int h, WGPUTextureFormat fmt,
                                const void* data, size_t bytesPerRow,
                                int mipLevels = 1, WGPUTextureUsage extraUsage = WGPUTextureUsage_None) {
        WGPUTextureDescriptor td = {};
        td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_CopyDst | extraUsage;
        td.dimension = WGPUTextureDimension_2D;
        td.size = {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1};
        td.format = fmt;
        td.mipLevelCount = static_cast<uint32_t>(mipLevels);
        td.sampleCount = 1;
        WGPUTexture tex = wgpuDeviceCreateTexture(device_, &td);
        if (data) {
            WGPUTexelCopyTextureInfo dst = {};
            dst.texture = tex;
            dst.mipLevel = 0;
            dst.aspect = WGPUTextureAspect_All;
            WGPUTexelCopyBufferLayout layout = {};
            layout.offset = 0;
            layout.bytesPerRow = static_cast<uint32_t>(bytesPerRow);
            layout.rowsPerImage = static_cast<uint32_t>(h);
            WGPUExtent3D ext = {static_cast<uint32_t>(w), static_cast<uint32_t>(h), 1};
            wgpuQueueWriteTexture(queue_, &dst, data, bytesPerRow * h, &layout, &ext);
        }
        return tex;
    }

    void freeMesh(GpuMesh& m) {
        if (m.vertexBuffer) { wgpuBufferRelease(m.vertexBuffer); m.vertexBuffer = nullptr; }
        if (m.indexBuffer)  { wgpuBufferRelease(m.indexBuffer);  m.indexBuffer = nullptr; }
        m.indexCount = 0;
    }

    GpuMesh* resolve(MeshHandle h) {
        if (h.index == 0 || h.index > meshes_.size()) return nullptr;
        GpuMesh& m = meshes_[h.index - 1];
        if (m.generation != h.generation || !m.vertexBuffer) return nullptr;
        return &m;
    }
    const GpuMesh* resolve(MeshHandle h) const {
        if (h.index == 0 || h.index > meshes_.size()) return nullptr;
        const GpuMesh& m = meshes_[h.index - 1];
        if (m.generation != h.generation || !m.vertexBuffer) return nullptr;
        return &m;
    }

    bool createSurface() {
        WGPUEmscriptenSurfaceSourceCanvasHTMLSelector canvasDesc = {};
        canvasDesc.chain.sType = WGPUSType_EmscriptenSurfaceSourceCanvasHTMLSelector;
        canvasDesc.selector = sv("#canvas");
        WGPUSurfaceDescriptor surfaceDesc = {};
        surfaceDesc.nextInChain = &canvasDesc.chain;
        surface_ = wgpuInstanceCreateSurface(instance_, &surfaceDesc);
        if (!surface_) {
            LOG_ERROR("WebGPU: failed to create surface from canvas #canvas");
            return false;
        }
        return true;
    }

    void configureSurface() {
        if (!surface_) return;
        WGPUSurfaceConfiguration config = {};
        config.device = device_;
        config.format = kSwapFormat;
        config.usage = WGPUTextureUsage_RenderAttachment;
        config.width = static_cast<uint32_t>(width_);
        config.height = static_cast<uint32_t>(height_);
        config.presentMode = WGPUPresentMode_Fifo;
        config.alphaMode = WGPUCompositeAlphaMode_Opaque;
        wgpuSurfaceConfigure(surface_, &config);
    }

    void createDepthTarget() {
        WGPUTextureDescriptor desc = {};
        // Sampled by the SSAO pass (texture_depth_2d) after the main pass writes it.
        desc.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
        desc.dimension = WGPUTextureDimension_2D;
        desc.size = {static_cast<uint32_t>(width_), static_cast<uint32_t>(height_), 1};
        desc.format = kDepthFormat;
        desc.mipLevelCount = 1;
        desc.sampleCount = 1;
        depthTexture_ = wgpuDeviceCreateTexture(device_, &desc);
        depthView_ = wgpuTextureCreateView(depthTexture_, nullptr);

        // Linear HDR scene target (sampled by the composite pass).
        WGPUTextureDescriptor hd = {};
        hd.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
        hd.dimension = WGPUTextureDimension_2D;
        hd.size = {static_cast<uint32_t>(width_), static_cast<uint32_t>(height_), 1};
        hd.format = kHdrFormat;
        hd.mipLevelCount = 1;
        hd.sampleCount = 1;
        hdrTexture_ = wgpuDeviceCreateTexture(device_, &hd);
        hdrView_ = wgpuTextureCreateView(hdrTexture_, nullptr);

        // Material G-buffer (world normal in xyz, roughness in w) for SSAO + SSR.
        gbufTexture_ = wgpuDeviceCreateTexture(device_, &hd);   // same descriptor (RGBA16F)
        gbufView_ = wgpuTextureCreateView(gbufTexture_, nullptr);

        // Half-res bloom ping-pong targets (sampled, render-to).
        bloomW_ = std::max(1, width_ / 2);
        bloomH_ = std::max(1, height_ / 2);
        WGPUTextureDescriptor bd = {};
        bd.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
        bd.dimension = WGPUTextureDimension_2D;
        bd.size = {static_cast<uint32_t>(bloomW_), static_cast<uint32_t>(bloomH_), 1};
        bd.format = kHdrFormat;
        bd.mipLevelCount = 1;
        bd.sampleCount = 1;
        bloomTexA_ = wgpuDeviceCreateTexture(device_, &bd);
        bloomViewA_ = wgpuTextureCreateView(bloomTexA_, nullptr);
        bloomTexB_ = wgpuDeviceCreateTexture(device_, &bd);
        bloomViewB_ = wgpuTextureCreateView(bloomTexB_, nullptr);

        // Screen-space effect buffers (SSAO + SSR) at a fraction of the
        // framebuffer (postEffectScale). These passes are the heaviest per-pixel
        // work (SSAO: 12 taps × 2 matrix-mults; SSR: a 32-step march), and AO /
        // reflections are low/medium frequency — so render them small and let the
        // composite linear-sample them back up (a free upscale-blur). Full-res
        // depth/G-buffer are still read per effect texel, so quality holds.
        appliedFxScale_ = std::min(std::max(postEffectScale, 0.25f), 1.0f);
        fxW_ = std::max(1, static_cast<int>(std::lround(width_ * appliedFxScale_)));
        fxH_ = std::max(1, static_cast<int>(std::lround(height_ * appliedFxScale_)));

        // SSAO target (single channel).
        WGPUTextureDescriptor ad = {};
        ad.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
        ad.dimension = WGPUTextureDimension_2D;
        ad.size = {static_cast<uint32_t>(fxW_), static_cast<uint32_t>(fxH_), 1};
        ad.format = WGPUTextureFormat_R8Unorm;
        ad.mipLevelCount = 1;
        ad.sampleCount = 1;
        ssaoTex_ = wgpuDeviceCreateTexture(device_, &ad);
        ssaoView_ = wgpuTextureCreateView(ssaoTex_, nullptr);

        // SSR target (RGBA16F: reflected color premultiplied by confidence, in a).
        WGPUTextureDescriptor rd = hd;   // RGBA16F, but at the effect resolution
        rd.size = {static_cast<uint32_t>(fxW_), static_cast<uint32_t>(fxH_), 1};
        ssrTexture_ = wgpuDeviceCreateTexture(device_, &rd);
        ssrView_ = wgpuTextureCreateView(ssrTexture_, nullptr);
    }

    void releaseDepthTarget() {
        if (depthView_) { wgpuTextureViewRelease(depthView_); depthView_ = nullptr; }
        if (depthTexture_) { wgpuTextureRelease(depthTexture_); depthTexture_ = nullptr; }
        if (hdrView_) { wgpuTextureViewRelease(hdrView_); hdrView_ = nullptr; }
        if (hdrTexture_) { wgpuTextureRelease(hdrTexture_); hdrTexture_ = nullptr; }
        if (gbufView_) { wgpuTextureViewRelease(gbufView_); gbufView_ = nullptr; }
        if (gbufTexture_) { wgpuTextureRelease(gbufTexture_); gbufTexture_ = nullptr; }
        if (bloomViewA_) { wgpuTextureViewRelease(bloomViewA_); bloomViewA_ = nullptr; }
        if (bloomTexA_) { wgpuTextureRelease(bloomTexA_); bloomTexA_ = nullptr; }
        if (bloomViewB_) { wgpuTextureViewRelease(bloomViewB_); bloomViewB_ = nullptr; }
        if (bloomTexB_) { wgpuTextureRelease(bloomTexB_); bloomTexB_ = nullptr; }
        if (ssaoView_) { wgpuTextureViewRelease(ssaoView_); ssaoView_ = nullptr; }
        if (ssaoTex_) { wgpuTextureRelease(ssaoTex_); ssaoTex_ = nullptr; }
        if (ssrView_) { wgpuTextureViewRelease(ssrView_); ssrView_ = nullptr; }
        if (ssrTexture_) { wgpuTextureRelease(ssrTexture_); ssrTexture_ = nullptr; }
    }

    bool createPipeline() {
        WGPUShaderSourceWGSL wgslDesc = {};
        wgslDesc.chain.sType = WGPUSType_ShaderSourceWGSL;
        wgslDesc.code = sv(kMeshWgsl);
        WGPUShaderModuleDescriptor moduleDesc = {};
        moduleDesc.nextInChain = &wgslDesc.chain;
        WGPUShaderModule module = wgpuDeviceCreateShaderModule(device_, &moduleDesc);
        if (!module) {
            LOG_ERROR("WebGPU: shader module creation failed");
            return false;
        }

        // Bind group layout: globals (uniform) + per-draw (dynamic uniform) +
        // the scene environment (equirect HDR + sampler) and the BRDF LUT.
        WGPUBindGroupLayoutEntry entries[5] = {};
        entries[0].binding = 0;
        entries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
        entries[0].buffer.type = WGPUBufferBindingType_Uniform;
        entries[0].buffer.minBindingSize = sizeof(GpuGlobals);
        entries[1].binding = 1;
        entries[1].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
        entries[1].buffer.type = WGPUBufferBindingType_Uniform;
        entries[1].buffer.hasDynamicOffset = true;
        entries[1].buffer.minBindingSize = sizeof(GpuDraw);
        entries[2].binding = 2;   // env equirect (HDR when bound, else a 1x1 default)
        entries[2].visibility = WGPUShaderStage_Fragment;
        entries[2].texture.sampleType = WGPUTextureSampleType_Float;
        entries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
        entries[3].binding = 3;   // env + LUT sampler (linear, U-repeat/V-clamp)
        entries[3].visibility = WGPUShaderStage_Fragment;
        entries[3].sampler.type = WGPUSamplerBindingType_Filtering;
        entries[4].binding = 4;   // split-sum BRDF LUT (RG16Float)
        entries[4].visibility = WGPUShaderStage_Fragment;
        entries[4].texture.sampleType = WGPUTextureSampleType_Float;
        entries[4].texture.viewDimension = WGPUTextureViewDimension_2D;

        WGPUBindGroupLayoutDescriptor blDesc = {};
        blDesc.entryCount = 5;
        blDesc.entries = entries;
        bindLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &blDesc);

        // Group 1 (main pass only): the cascaded sun shadow map (array) + a
        // comparison sampler.
        WGPUBindGroupLayoutEntry shEntries[2] = {};
        shEntries[0].binding = 0;
        shEntries[0].visibility = WGPUShaderStage_Fragment;
        shEntries[0].texture.sampleType = WGPUTextureSampleType_Depth;
        shEntries[0].texture.viewDimension = WGPUTextureViewDimension_2DArray;
        shEntries[1].binding = 1;
        shEntries[1].visibility = WGPUShaderStage_Fragment;
        shEntries[1].sampler.type = WGPUSamplerBindingType_Comparison;
        WGPUBindGroupLayoutDescriptor shblDesc = {};
        shblDesc.entryCount = 2;
        shblDesc.entries = shEntries;
        shadowSampleLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &shblDesc);

        // Group 1 (shadow pass only): the per-cascade index uniform (dynamic),
        // binding 2 so it never collides with the lit pass's group-1 texture.
        WGPUBindGroupLayoutEntry sviEntry = {};
        sviEntry.binding = 2;
        sviEntry.visibility = WGPUShaderStage_Vertex;
        sviEntry.buffer.type = WGPUBufferBindingType_Uniform;
        sviEntry.buffer.hasDynamicOffset = true;
        sviEntry.buffer.minBindingSize = sizeof(int32_t) * 4;
        WGPUBindGroupLayoutDescriptor sviblDesc = {};
        sviblDesc.entryCount = 1;
        sviblDesc.entries = &sviEntry;
        shadowVsLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &sviblDesc);

        // Group 2 (main + instanced pass): material maps + a sampler.
        WGPUBindGroupLayoutEntry matEntries[6] = {};
        for (int i = 0; i < 5; ++i) {
            matEntries[i].binding = static_cast<uint32_t>(i);
            matEntries[i].visibility = WGPUShaderStage_Fragment;
            matEntries[i].texture.sampleType = WGPUTextureSampleType_Float;
            matEntries[i].texture.viewDimension = WGPUTextureViewDimension_2D;
        }
        matEntries[5].binding = 5;
        matEntries[5].visibility = WGPUShaderStage_Fragment;
        matEntries[5].sampler.type = WGPUSamplerBindingType_Filtering;
        WGPUBindGroupLayoutDescriptor matblDesc = {};
        matblDesc.entryCount = 6;
        matblDesc.entries = matEntries;
        materialLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &matblDesc);

        WGPUBindGroupLayout mainGroups[3] = {bindLayout_, shadowSampleLayout_, materialLayout_};
        WGPUPipelineLayoutDescriptor plDesc = {};
        plDesc.bindGroupLayoutCount = 3;
        plDesc.bindGroupLayouts = mainGroups;
        WGPUPipelineLayout pipelineLayout = wgpuDeviceCreatePipelineLayout(device_, &plDesc);

        // Vertex layout — must match GpuVertex / the WGSL @location inputs. Set
        // fields by name (WGPUVertexAttribute leads with nextInChain).
        WGPUVertexAttribute attrs[5] = {};
        attrs[0].format = WGPUVertexFormat_Float32x3; attrs[0].offset = offsetof(GpuVertex, position); attrs[0].shaderLocation = 0;
        attrs[1].format = WGPUVertexFormat_Float32x3; attrs[1].offset = offsetof(GpuVertex, normal);   attrs[1].shaderLocation = 1;
        attrs[2].format = WGPUVertexFormat_Float32x3; attrs[2].offset = offsetof(GpuVertex, tangent);  attrs[2].shaderLocation = 2;
        attrs[3].format = WGPUVertexFormat_Float32x2; attrs[3].offset = offsetof(GpuVertex, texcoord); attrs[3].shaderLocation = 3;
        attrs[4].format = WGPUVertexFormat_Float32x3; attrs[4].offset = offsetof(GpuVertex, color);    attrs[4].shaderLocation = 4;
        WGPUVertexBufferLayout vbLayout = {};
        vbLayout.arrayStride = sizeof(GpuVertex);
        vbLayout.stepMode = WGPUVertexStepMode_Vertex;
        vbLayout.attributeCount = 5;
        vbLayout.attributes = attrs;

        // Two targets: HDR color + the material G-buffer (both RGBA16F).
        WGPUColorTargetState colorTargets[2] = {};
        colorTargets[0].format = kHdrFormat;
        colorTargets[0].writeMask = WGPUColorWriteMask_All;
        colorTargets[1].format = kHdrFormat;
        colorTargets[1].writeMask = WGPUColorWriteMask_All;

        WGPUFragmentState fragment = {};
        fragment.module = module;
        fragment.entryPoint = sv("fs_main");
        fragment.targetCount = 2;
        fragment.targets = colorTargets;

        WGPUDepthStencilState depthState = {};
        depthState.format = kDepthFormat;
        depthState.depthWriteEnabled = WGPUOptionalBool_True;
        depthState.depthCompare = WGPUCompareFunction_LessEqual;
        // Depth-only format: a canonical no-stencil face state (Always/Keep) so
        // the zero-init doesn't leave Undefined compare values.
        depthState.stencilFront.compare = WGPUCompareFunction_Always;
        depthState.stencilFront.failOp = WGPUStencilOperation_Keep;
        depthState.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
        depthState.stencilFront.passOp = WGPUStencilOperation_Keep;
        depthState.stencilBack = depthState.stencilFront;

        WGPURenderPipelineDescriptor desc = {};
        desc.layout = pipelineLayout;
        desc.vertex.module = module;
        desc.vertex.entryPoint = sv("vs_main");
        desc.vertex.bufferCount = 1;
        desc.vertex.buffers = &vbLayout;
        desc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        desc.primitive.frontFace = WGPUFrontFace_CCW;
        // Culling stays OFF permanently: alpha-test foliage cards are
        // single-sided, so global back-face culling would drop leaves (a
        // per-material two-sided flag is the eventual fix; matches Vulkan).
        // (matches the Vulkan Phase-1 choice); a later phase turns it on.
        desc.primitive.cullMode = WGPUCullMode_None;
        desc.depthStencil = &depthState;
        desc.fragment = &fragment;
        desc.multisample.count = 1;
        desc.multisample.mask = 0xFFFFFFFF;

        pipeline_ = wgpuDeviceCreateRenderPipeline(device_, &desc);

        // Overlay variant (RenderMaterial::FLAG_OVERLAY — debug gizmos): same
        // shader/layout, but depth test Always + no depth write, drawn LAST in
        // the main pass so gizmos sit on top of world geometry (Vulkan/Metal
        // parity: overlayPipeline / depthStateOverlay).
        depthState.depthWriteEnabled = WGPUOptionalBool_False;
        depthState.depthCompare = WGPUCompareFunction_Always;
        overlayPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &desc);
        depthState.depthWriteEnabled = WGPUOptionalBool_True;
        depthState.depthCompare = WGPUCompareFunction_LessEqual;
        wgpuPipelineLayoutRelease(pipelineLayout);

        // Shadow pipeline: depth-only (no fragment), vs_shadow. Group 0 = globals
        // + per-draw model; group 1 = the per-cascade index (binding 2).
        WGPUBindGroupLayout shadowGroups[2] = {bindLayout_, shadowVsLayout_};
        WGPUPipelineLayoutDescriptor splDesc = {};
        splDesc.bindGroupLayoutCount = 2;
        splDesc.bindGroupLayouts = shadowGroups;
        WGPUPipelineLayout shadowLayout = wgpuDeviceCreatePipelineLayout(device_, &splDesc);

        WGPUDepthStencilState shadowDepth = {};
        shadowDepth.format = kShadowFormat;
        shadowDepth.depthWriteEnabled = WGPUOptionalBool_True;
        shadowDepth.depthCompare = WGPUCompareFunction_LessEqual;
        shadowDepth.stencilFront.compare = WGPUCompareFunction_Always;
        shadowDepth.stencilFront.failOp = WGPUStencilOperation_Keep;
        shadowDepth.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
        shadowDepth.stencilFront.passOp = WGPUStencilOperation_Keep;
        shadowDepth.stencilBack = shadowDepth.stencilFront;

        WGPURenderPipelineDescriptor sdesc = {};
        sdesc.layout = shadowLayout;
        sdesc.vertex.module = module;
        sdesc.vertex.entryPoint = sv("vs_shadow");
        sdesc.vertex.bufferCount = 1;
        sdesc.vertex.buffers = &vbLayout;
        sdesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        sdesc.primitive.frontFace = WGPUFrontFace_CCW;
        sdesc.primitive.cullMode = WGPUCullMode_None;
        sdesc.depthStencil = &shadowDepth;
        sdesc.fragment = nullptr;   // depth-only
        sdesc.multisample.count = 1;
        sdesc.multisample.mask = 0xFFFFFFFF;
        shadowPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &sdesc);
        wgpuPipelineLayoutRelease(shadowLayout);

        // Instanced pipelines: a second vertex buffer carries the per-instance
        // model (locations 5-8 = its four columns, stepMode Instance), so one
        // draw covers a whole InstanceGroup. Same bind-group layouts as the
        // non-instanced pipelines; the model comes from the attribute, not d.
        WGPUVertexAttribute iattrs[4] = {};
        for (int i = 0; i < 4; ++i) {
            iattrs[i].format = WGPUVertexFormat_Float32x4;
            iattrs[i].offset = static_cast<uint64_t>(i) * 16;
            iattrs[i].shaderLocation = static_cast<uint32_t>(5 + i);
        }
        WGPUVertexBufferLayout iLayout = {};
        iLayout.arrayStride = 64;          // 4 columns * 16 bytes
        iLayout.stepMode = WGPUVertexStepMode_Instance;
        iLayout.attributeCount = 4;
        iLayout.attributes = iattrs;
        WGPUVertexBufferLayout instMainBufs[2] = {vbLayout, iLayout};

        WGPUBindGroupLayout mainGroups2[3] = {bindLayout_, shadowSampleLayout_, materialLayout_};
        WGPUPipelineLayoutDescriptor iplDesc = {};
        iplDesc.bindGroupLayoutCount = 3;
        iplDesc.bindGroupLayouts = mainGroups2;
        WGPUPipelineLayout iPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &iplDesc);
        WGPURenderPipelineDescriptor idesc = desc;   // reuse fragment/depth/primitive
        idesc.layout = iPipeLayout;
        idesc.vertex.entryPoint = sv("vs_instanced");
        idesc.vertex.bufferCount = 2;
        idesc.vertex.buffers = instMainBufs;
        instancedPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &idesc);
        // Instanced OVERLAY variant (FLAG_OVERLAY debug widgets — the citysim
        // rings/arrows/plan outlines are InstanceGroups): depth Always + no
        // write, drawn last. Without it the instanced pass depth-tested the
        // gizmos, and the CDLOD terrain (whose drawn surface morphs above the
        // analytic ground the widgets are baked on) swallowed every ground
        // strip (device: "the living city visualization doesn't show up").
        depthState.depthWriteEnabled = WGPUOptionalBool_False;
        depthState.depthCompare = WGPUCompareFunction_Always;
        instancedOverlayPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &idesc);
        depthState.depthWriteEnabled = WGPUOptionalBool_True;
        depthState.depthCompare = WGPUCompareFunction_LessEqual;
        wgpuPipelineLayoutRelease(iPipeLayout);

        WGPUBindGroupLayout shadowGroups2[2] = {bindLayout_, shadowVsLayout_};
        WGPUPipelineLayoutDescriptor isplDesc = {};
        isplDesc.bindGroupLayoutCount = 2;
        isplDesc.bindGroupLayouts = shadowGroups2;
        WGPUPipelineLayout iShadowLayout = wgpuDeviceCreatePipelineLayout(device_, &isplDesc);
        WGPURenderPipelineDescriptor isdesc = sdesc;
        isdesc.layout = iShadowLayout;
        isdesc.vertex.entryPoint = sv("vs_shadow_instanced");
        isdesc.vertex.bufferCount = 2;
        isdesc.vertex.buffers = instMainBufs;
        instancedShadowPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &isdesc);
        wgpuPipelineLayoutRelease(iShadowLayout);

        // Terrain pipelines: single vertex buffer (like the non-instanced path),
        // but vs_terrain / vs_shadow_terrain apply the CDLOD morph. Same layouts.
        WGPUBindGroupLayout tGroups[3] = {bindLayout_, shadowSampleLayout_, materialLayout_};
        WGPUPipelineLayoutDescriptor tplDesc = {};
        tplDesc.bindGroupLayoutCount = 3;
        tplDesc.bindGroupLayouts = tGroups;
        WGPUPipelineLayout tPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &tplDesc);
        WGPURenderPipelineDescriptor tdesc = desc;
        tdesc.layout = tPipeLayout;
        tdesc.vertex.entryPoint = sv("vs_terrain");
        tdesc.vertex.bufferCount = 1;
        tdesc.vertex.buffers = &vbLayout;
        terrainPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &tdesc);
        wgpuPipelineLayoutRelease(tPipeLayout);

        WGPUBindGroupLayout tsGroups[2] = {bindLayout_, shadowVsLayout_};
        WGPUPipelineLayoutDescriptor tsplDesc = {};
        tsplDesc.bindGroupLayoutCount = 2;
        tsplDesc.bindGroupLayouts = tsGroups;
        WGPUPipelineLayout tShadowLayout = wgpuDeviceCreatePipelineLayout(device_, &tsplDesc);
        WGPURenderPipelineDescriptor tsdesc = sdesc;
        tsdesc.layout = tShadowLayout;
        tsdesc.vertex.entryPoint = sv("vs_shadow_terrain");
        tsdesc.vertex.bufferCount = 1;
        tsdesc.vertex.buffers = &vbLayout;
        terrainShadowPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &tsdesc);
        wgpuPipelineLayoutRelease(tShadowLayout);

        // Sky pipeline: fullscreen triangle, globals only (its own one-binding
        // layout so it doesn't depend on the per-draw dynamic buffer). No vertex
        // buffer; depth-compare Always + no write so meshes paint over it.
        WGPUBindGroupLayoutEntry skyEntry[3] = {};
        skyEntry[0].binding = 0;
        skyEntry[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
        skyEntry[0].buffer.type = WGPUBufferBindingType_Uniform;
        skyEntry[0].buffer.minBindingSize = sizeof(GpuGlobals);
        skyEntry[1].binding = 2;   // env equirect (fs_sky background)
        skyEntry[1].visibility = WGPUShaderStage_Fragment;
        skyEntry[1].texture.sampleType = WGPUTextureSampleType_Float;
        skyEntry[1].texture.viewDimension = WGPUTextureViewDimension_2D;
        skyEntry[2].binding = 3;   // env sampler
        skyEntry[2].visibility = WGPUShaderStage_Fragment;
        skyEntry[2].sampler.type = WGPUSamplerBindingType_Filtering;
        WGPUBindGroupLayoutDescriptor skyblDesc = {};
        skyblDesc.entryCount = 3;
        skyblDesc.entries = skyEntry;
        skyLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &skyblDesc);

        WGPUPipelineLayoutDescriptor skyplDesc = {};
        skyplDesc.bindGroupLayoutCount = 1;
        skyplDesc.bindGroupLayouts = &skyLayout_;
        WGPUPipelineLayout skyPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &skyplDesc);

        WGPUDepthStencilState skyDepth = {};
        skyDepth.format = kDepthFormat;
        skyDepth.depthWriteEnabled = WGPUOptionalBool_False;
        skyDepth.depthCompare = WGPUCompareFunction_Always;
        skyDepth.stencilFront.compare = WGPUCompareFunction_Always;
        skyDepth.stencilFront.failOp = WGPUStencilOperation_Keep;
        skyDepth.stencilFront.depthFailOp = WGPUStencilOperation_Keep;
        skyDepth.stencilFront.passOp = WGPUStencilOperation_Keep;
        skyDepth.stencilBack = skyDepth.stencilFront;

        WGPUColorTargetState skyColors[2] = {};
        skyColors[0].format = kHdrFormat;     // sky renders into the HDR target too
        skyColors[0].writeMask = WGPUColorWriteMask_All;
        skyColors[1].format = kHdrFormat;     // + the G-buffer (sentinel)
        skyColors[1].writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState skyFrag = {};
        skyFrag.module = module;
        skyFrag.entryPoint = sv("fs_sky");
        skyFrag.targetCount = 2;
        skyFrag.targets = skyColors;

        WGPURenderPipelineDescriptor skyPipeDesc = {};
        skyPipeDesc.layout = skyPipeLayout;
        skyPipeDesc.vertex.module = module;
        skyPipeDesc.vertex.entryPoint = sv("vs_sky");
        skyPipeDesc.vertex.bufferCount = 0;        // fullscreen triangle from vertex_index
        skyPipeDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        skyPipeDesc.primitive.frontFace = WGPUFrontFace_CCW;
        skyPipeDesc.primitive.cullMode = WGPUCullMode_None;
        skyPipeDesc.depthStencil = &skyDepth;
        skyPipeDesc.fragment = &skyFrag;
        skyPipeDesc.multisample.count = 1;
        skyPipeDesc.multisample.mask = 0xFFFFFFFF;
        skyPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &skyPipeDesc);
        wgpuPipelineLayoutRelease(skyPipeLayout);

        wgpuShaderModuleRelease(module);

        // Composite pipeline (its own module): HDR target -> swapchain. Group 0 =
        // { hdr texture, post uniform }. No vertex buffer, no depth.
        WGPUShaderSourceWGSL cWgsl = {};
        cWgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
        cWgsl.code = sv(kCompositeWgsl);
        WGPUShaderModuleDescriptor cModDesc = {};
        cModDesc.nextInChain = &cWgsl.chain;
        WGPUShaderModule cModule = wgpuDeviceCreateShaderModule(device_, &cModDesc);

        WGPUBindGroupLayoutEntry cEntries[6] = {};
        cEntries[0].binding = 0;
        cEntries[0].visibility = WGPUShaderStage_Fragment;
        cEntries[0].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;  // textureLoad
        cEntries[0].texture.viewDimension = WGPUTextureViewDimension_2D;
        cEntries[1].binding = 1;
        cEntries[1].visibility = WGPUShaderStage_Fragment;
        cEntries[1].buffer.type = WGPUBufferBindingType_Uniform;
        cEntries[1].buffer.minBindingSize = sizeof(GpuPost);
        cEntries[2].binding = 2;                              // bloom (sampled, filterable)
        cEntries[2].visibility = WGPUShaderStage_Fragment;
        cEntries[2].texture.sampleType = WGPUTextureSampleType_Float;
        cEntries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
        cEntries[3].binding = 3;
        cEntries[3].visibility = WGPUShaderStage_Fragment;
        cEntries[3].sampler.type = WGPUSamplerBindingType_Filtering;
        cEntries[4].binding = 4;                              // SSAO (half-res, filtered upscale)
        cEntries[4].visibility = WGPUShaderStage_Fragment;
        cEntries[4].texture.sampleType = WGPUTextureSampleType_Float;   // R8Unorm, filterable
        cEntries[4].texture.viewDimension = WGPUTextureViewDimension_2D;
        cEntries[5].binding = 5;                              // SSR (scaled, filtered upscale)
        cEntries[5].visibility = WGPUShaderStage_Fragment;
        cEntries[5].texture.sampleType = WGPUTextureSampleType_Float;   // RGBA16F, filterable
        cEntries[5].texture.viewDimension = WGPUTextureViewDimension_2D;
        WGPUBindGroupLayoutDescriptor cblDesc = {};
        cblDesc.entryCount = 6;
        cblDesc.entries = cEntries;
        compositeLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &cblDesc);

        // A linear sampler shared by bloom + the composite's bloom upsample.
        WGPUSamplerDescriptor lsd = {};
        lsd.magFilter = WGPUFilterMode_Linear;
        lsd.minFilter = WGPUFilterMode_Linear;
        lsd.addressModeU = WGPUAddressMode_ClampToEdge;
        lsd.addressModeV = WGPUAddressMode_ClampToEdge;
        lsd.addressModeW = WGPUAddressMode_ClampToEdge;
        lsd.maxAnisotropy = 1;
        linearSampler_ = wgpuDeviceCreateSampler(device_, &lsd);

        WGPUPipelineLayoutDescriptor cplDesc = {};
        cplDesc.bindGroupLayoutCount = 1;
        cplDesc.bindGroupLayouts = &compositeLayout_;
        WGPUPipelineLayout cPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &cplDesc);

        WGPUColorTargetState cColor = {};
        cColor.format = kSwapFormat;       // composite writes the swapchain
        cColor.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState cFrag = {};
        cFrag.module = cModule;
        cFrag.entryPoint = sv("fs_composite");
        cFrag.targetCount = 1;
        cFrag.targets = &cColor;

        WGPURenderPipelineDescriptor cDesc = {};
        cDesc.layout = cPipeLayout;
        cDesc.vertex.module = cModule;
        cDesc.vertex.entryPoint = sv("vs_composite");
        cDesc.vertex.bufferCount = 0;
        cDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        cDesc.primitive.frontFace = WGPUFrontFace_CCW;
        cDesc.primitive.cullMode = WGPUCullMode_None;
        cDesc.depthStencil = nullptr;      // composite ignores depth
        cDesc.fragment = &cFrag;
        cDesc.multisample.count = 1;
        cDesc.multisample.mask = 0xFFFFFFFF;
        compositePipeline_ = wgpuDeviceCreateRenderPipeline(device_, &cDesc);
        wgpuPipelineLayoutRelease(cPipeLayout);
        wgpuShaderModuleRelease(cModule);

        // Bloom pipelines (its own module): bright-pass + separable blur. One
        // layout { src tex, sampler, uniform }; both write the half-res HDR format.
        WGPUShaderSourceWGSL bWgsl = {};
        bWgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
        bWgsl.code = sv(kBloomWgsl);
        WGPUShaderModuleDescriptor bModDesc = {};
        bModDesc.nextInChain = &bWgsl.chain;
        WGPUShaderModule bModule = wgpuDeviceCreateShaderModule(device_, &bModDesc);

        WGPUBindGroupLayoutEntry bEntries[3] = {};
        bEntries[0].binding = 0;
        bEntries[0].visibility = WGPUShaderStage_Fragment;
        bEntries[0].texture.sampleType = WGPUTextureSampleType_Float;
        bEntries[0].texture.viewDimension = WGPUTextureViewDimension_2D;
        bEntries[1].binding = 1;
        bEntries[1].visibility = WGPUShaderStage_Fragment;
        bEntries[1].sampler.type = WGPUSamplerBindingType_Filtering;
        bEntries[2].binding = 2;
        bEntries[2].visibility = WGPUShaderStage_Fragment;
        bEntries[2].buffer.type = WGPUBufferBindingType_Uniform;
        bEntries[2].buffer.minBindingSize = sizeof(GpuBloom);
        WGPUBindGroupLayoutDescriptor bblDesc = {};
        bblDesc.entryCount = 3;
        bblDesc.entries = bEntries;
        bloomLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &bblDesc);

        WGPUPipelineLayoutDescriptor bplDesc = {};
        bplDesc.bindGroupLayoutCount = 1;
        bplDesc.bindGroupLayouts = &bloomLayout_;
        WGPUPipelineLayout bPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &bplDesc);

        WGPUColorTargetState bColor = {};
        bColor.format = kHdrFormat;
        bColor.writeMask = WGPUColorWriteMask_All;
        auto makeBloomPipe = [&](const char* fsName) {
            WGPUFragmentState fs = {};
            fs.module = bModule; fs.entryPoint = sv(fsName);
            fs.targetCount = 1; fs.targets = &bColor;
            WGPURenderPipelineDescriptor d = {};
            d.layout = bPipeLayout;
            d.vertex.module = bModule; d.vertex.entryPoint = sv("vs_bloom");
            d.vertex.bufferCount = 0;
            d.primitive.topology = WGPUPrimitiveTopology_TriangleList;
            d.primitive.frontFace = WGPUFrontFace_CCW;
            d.primitive.cullMode = WGPUCullMode_None;
            d.depthStencil = nullptr;
            d.fragment = &fs;
            d.multisample.count = 1; d.multisample.mask = 0xFFFFFFFF;
            return wgpuDeviceCreateRenderPipeline(device_, &d);
        };
        bloomBrightPipeline_ = makeBloomPipe("fs_bright");
        bloomBlurPipeline_ = makeBloomPipe("fs_blur");
        wgpuPipelineLayoutRelease(bPipeLayout);
        wgpuShaderModuleRelease(bModule);

        // SSAO pipeline (its own module): depth -> single-channel AO. Group 0 =
        // { depth texture, uniform }.
        WGPUShaderSourceWGSL aWgsl = {};
        aWgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
        aWgsl.code = sv(kSsaoWgsl);
        WGPUShaderModuleDescriptor aModDesc = {};
        aModDesc.nextInChain = &aWgsl.chain;
        WGPUShaderModule aModule = wgpuDeviceCreateShaderModule(device_, &aModDesc);

        WGPUBindGroupLayoutEntry aEntries[3] = {};
        aEntries[0].binding = 0;
        aEntries[0].visibility = WGPUShaderStage_Fragment;
        aEntries[0].texture.sampleType = WGPUTextureSampleType_Depth;
        aEntries[0].texture.viewDimension = WGPUTextureViewDimension_2D;
        aEntries[1].binding = 1;
        aEntries[1].visibility = WGPUShaderStage_Fragment;
        aEntries[1].buffer.type = WGPUBufferBindingType_Uniform;
        aEntries[1].buffer.minBindingSize = sizeof(GpuSsao);
        aEntries[2].binding = 2;                              // G-buffer normal
        aEntries[2].visibility = WGPUShaderStage_Fragment;
        aEntries[2].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
        aEntries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
        WGPUBindGroupLayoutDescriptor ablDesc = {};
        ablDesc.entryCount = 3;
        ablDesc.entries = aEntries;
        ssaoLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &ablDesc);

        WGPUPipelineLayoutDescriptor aplDesc = {};
        aplDesc.bindGroupLayoutCount = 1;
        aplDesc.bindGroupLayouts = &ssaoLayout_;
        WGPUPipelineLayout aPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &aplDesc);

        WGPUColorTargetState aColor = {};
        aColor.format = WGPUTextureFormat_R8Unorm;
        aColor.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState aFrag = {};
        aFrag.module = aModule; aFrag.entryPoint = sv("fs_ssao");
        aFrag.targetCount = 1; aFrag.targets = &aColor;
        WGPURenderPipelineDescriptor aDesc = {};
        aDesc.layout = aPipeLayout;
        aDesc.vertex.module = aModule; aDesc.vertex.entryPoint = sv("vs_ssao");
        aDesc.vertex.bufferCount = 0;
        aDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        aDesc.primitive.frontFace = WGPUFrontFace_CCW;
        aDesc.primitive.cullMode = WGPUCullMode_None;
        aDesc.depthStencil = nullptr;
        aDesc.fragment = &aFrag;
        aDesc.multisample.count = 1; aDesc.multisample.mask = 0xFFFFFFFF;
        ssaoPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &aDesc);
        wgpuPipelineLayoutRelease(aPipeLayout);
        wgpuShaderModuleRelease(aModule);

        // SSR pipeline: { hdr, depth, gbuffer, uniform } -> RGBA16F reflection.
        WGPUShaderSourceWGSL rWgsl = {};
        rWgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
        rWgsl.code = sv(kSsrWgsl);
        WGPUShaderModuleDescriptor rModDesc = {};
        rModDesc.nextInChain = &rWgsl.chain;
        WGPUShaderModule rModule = wgpuDeviceCreateShaderModule(device_, &rModDesc);

        WGPUBindGroupLayoutEntry rEntries[4] = {};
        rEntries[0].binding = 0;
        rEntries[0].visibility = WGPUShaderStage_Fragment;
        rEntries[0].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
        rEntries[0].texture.viewDimension = WGPUTextureViewDimension_2D;
        rEntries[1].binding = 1;
        rEntries[1].visibility = WGPUShaderStage_Fragment;
        rEntries[1].texture.sampleType = WGPUTextureSampleType_Depth;
        rEntries[1].texture.viewDimension = WGPUTextureViewDimension_2D;
        rEntries[2].binding = 2;
        rEntries[2].visibility = WGPUShaderStage_Fragment;
        rEntries[2].texture.sampleType = WGPUTextureSampleType_UnfilterableFloat;
        rEntries[2].texture.viewDimension = WGPUTextureViewDimension_2D;
        rEntries[3].binding = 3;
        rEntries[3].visibility = WGPUShaderStage_Fragment;
        rEntries[3].buffer.type = WGPUBufferBindingType_Uniform;
        rEntries[3].buffer.minBindingSize = sizeof(GpuSsr);
        WGPUBindGroupLayoutDescriptor rblDesc = {};
        rblDesc.entryCount = 4;
        rblDesc.entries = rEntries;
        ssrLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &rblDesc);

        WGPUPipelineLayoutDescriptor rplDesc = {};
        rplDesc.bindGroupLayoutCount = 1;
        rplDesc.bindGroupLayouts = &ssrLayout_;
        WGPUPipelineLayout rPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &rplDesc);
        WGPUColorTargetState rColor = {};
        rColor.format = kHdrFormat;
        rColor.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState rFrag = {};
        rFrag.module = rModule; rFrag.entryPoint = sv("fs_ssr");
        rFrag.targetCount = 1; rFrag.targets = &rColor;
        WGPURenderPipelineDescriptor rDesc = {};
        rDesc.layout = rPipeLayout;
        rDesc.vertex.module = rModule; rDesc.vertex.entryPoint = sv("vs_ssr");
        rDesc.vertex.bufferCount = 0;
        rDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        rDesc.primitive.frontFace = WGPUFrontFace_CCW;
        rDesc.primitive.cullMode = WGPUCullMode_None;
        rDesc.depthStencil = nullptr;
        rDesc.fragment = &rFrag;
        rDesc.multisample.count = 1; rDesc.multisample.mask = 0xFFFFFFFF;
        ssrPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &rDesc);
        wgpuPipelineLayoutRelease(rPipeLayout);
        wgpuShaderModuleRelease(rModule);

        // Atmosphere pipeline (procedural-planet-plan P3): { uniform } -> HDR, an
        // additive fullscreen glow. No scene texture — the blend (One, One) adds
        // the in-scatter over the HDR scene, so it shares the HDR target it writes.
        WGPUShaderSourceWGSL atWgsl = {};
        atWgsl.chain.sType = WGPUSType_ShaderSourceWGSL;
        atWgsl.code = sv(kAtmosphereWgsl);
        WGPUShaderModuleDescriptor atModDesc = {};
        atModDesc.nextInChain = &atWgsl.chain;
        WGPUShaderModule atModule = wgpuDeviceCreateShaderModule(device_, &atModDesc);

        WGPUBindGroupLayoutEntry atEntry = {};
        atEntry.binding = 0;
        atEntry.visibility = WGPUShaderStage_Fragment;
        atEntry.buffer.type = WGPUBufferBindingType_Uniform;
        atEntry.buffer.minBindingSize = sizeof(GpuAtmosphere);
        WGPUBindGroupLayoutDescriptor atblDesc = {};
        atblDesc.entryCount = 1;
        atblDesc.entries = &atEntry;
        atmosphereLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &atblDesc);

        WGPUPipelineLayoutDescriptor atplDesc = {};
        atplDesc.bindGroupLayoutCount = 1;
        atplDesc.bindGroupLayouts = &atmosphereLayout_;
        WGPUPipelineLayout atPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &atplDesc);
        // Additive blend: dst stays, src (the glow) is added on top.
        WGPUBlendState atBlend = {};
        atBlend.color.operation = WGPUBlendOperation_Add;
        atBlend.color.srcFactor = WGPUBlendFactor_One;
        atBlend.color.dstFactor = WGPUBlendFactor_One;
        atBlend.alpha.operation = WGPUBlendOperation_Add;
        atBlend.alpha.srcFactor = WGPUBlendFactor_One;
        atBlend.alpha.dstFactor = WGPUBlendFactor_One;
        WGPUColorTargetState atColor = {};
        atColor.format = kHdrFormat;
        atColor.blend = &atBlend;
        atColor.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState atFrag = {};
        atFrag.module = atModule; atFrag.entryPoint = sv("fs_main");
        atFrag.targetCount = 1; atFrag.targets = &atColor;
        WGPURenderPipelineDescriptor atDesc = {};
        atDesc.layout = atPipeLayout;
        atDesc.vertex.module = atModule; atDesc.vertex.entryPoint = sv("vs_main");
        atDesc.vertex.bufferCount = 0;
        atDesc.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        atDesc.primitive.frontFace = WGPUFrontFace_CCW;
        atDesc.primitive.cullMode = WGPUCullMode_None;
        atDesc.depthStencil = nullptr;
        atDesc.fragment = &atFrag;
        atDesc.multisample.count = 1; atDesc.multisample.mask = 0xFFFFFFFF;
        atmospherePipeline_ = wgpuDeviceCreateRenderPipeline(device_, &atDesc);
        wgpuPipelineLayoutRelease(atPipeLayout);
        wgpuShaderModuleRelease(atModule);

        // Atmosphere uniform buffer + bind group (only references the buffer, so it
        // survives resizes — no rebuild needed).
        WGPUBufferDescriptor atBufDesc = {};
        atBufDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        atBufDesc.size = sizeof(GpuAtmosphere);
        atmosphereUbo_ = wgpuDeviceCreateBuffer(device_, &atBufDesc);
        WGPUBindGroupEntry atGrpEntry = {};
        atGrpEntry.binding = 0;
        atGrpEntry.buffer = atmosphereUbo_;
        atGrpEntry.offset = 0;
        atGrpEntry.size = sizeof(GpuAtmosphere);
        WGPUBindGroupDescriptor atGrpDesc = {};
        atGrpDesc.layout = atmosphereLayout_;
        atGrpDesc.entryCount = 1;
        atGrpDesc.entries = &atGrpEntry;
        atmosphereGroup_ = wgpuDeviceCreateBindGroup(device_, &atGrpDesc);

        if (!pipeline_ || !shadowPipeline_ || !skyPipeline_ || !compositePipeline_
            || !bloomBrightPipeline_ || !bloomBlurPipeline_ || !ssaoPipeline_
            || !instancedPipeline_ || !instancedShadowPipeline_
            || !terrainPipeline_ || !terrainShadowPipeline_ || !ssrPipeline_) {
            LOG_ERROR("WebGPU: render pipeline creation failed");
            return false;
        }
        // The atmosphere pass is an OPTIONAL effect — a null pipeline must not brick
        // the whole renderer (recordAtmosphere() skips when it is null). Warn only.
        if (!atmospherePipeline_) {
            LOG_WARN("WebGPU: atmosphere pipeline unavailable — glow pass disabled");
        }
        return true;
    }

    void createShadowResources() {
        // A depth array with one layer per cascade: per-layer views drive the
        // shadow passes, the array view feeds the comparison sampler in the lit
        // pass (texture_depth_2d_array).
        WGPUTextureDescriptor td = {};
        td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
        td.dimension = WGPUTextureDimension_2D;
        td.size = {static_cast<uint32_t>(kShadowMapSize), static_cast<uint32_t>(kShadowMapSize),
                   static_cast<uint32_t>(kMaxCascades)};
        td.format = kShadowFormat;
        td.mipLevelCount = 1;
        td.sampleCount = 1;
        shadowTexture_ = wgpuDeviceCreateTexture(device_, &td);

        WGPUTextureViewDescriptor avd = {};
        avd.format = kShadowFormat;
        avd.dimension = WGPUTextureViewDimension_2DArray;
        avd.baseArrayLayer = 0;
        avd.arrayLayerCount = static_cast<uint32_t>(kMaxCascades);
        avd.mipLevelCount = 1;
        shadowArrayView_ = wgpuTextureCreateView(shadowTexture_, &avd);
        for (int c = 0; c < kMaxCascades; ++c) {
            WGPUTextureViewDescriptor lvd = {};
            lvd.format = kShadowFormat;
            lvd.dimension = WGPUTextureViewDimension_2D;
            lvd.baseArrayLayer = static_cast<uint32_t>(c);
            lvd.arrayLayerCount = 1;
            lvd.mipLevelCount = 1;
            shadowLayerViews_[c] = wgpuTextureCreateView(shadowTexture_, &lvd);
        }

        WGPUSamplerDescriptor sd = {};
        sd.compare = WGPUCompareFunction_LessEqual;   // a comparison (depth) sampler
        sd.magFilter = WGPUFilterMode_Linear;         // hardware 2x2 PCF
        sd.minFilter = WGPUFilterMode_Linear;
        sd.addressModeU = WGPUAddressMode_ClampToEdge;
        sd.addressModeV = WGPUAddressMode_ClampToEdge;
        sd.addressModeW = WGPUAddressMode_ClampToEdge;
        sd.maxAnisotropy = 1;
        shadowSampler_ = wgpuDeviceCreateSampler(device_, &sd);

        WGPUBindGroupEntry entries[2] = {};
        entries[0].binding = 0;
        entries[0].textureView = shadowArrayView_;
        entries[1].binding = 1;
        entries[1].sampler = shadowSampler_;
        WGPUBindGroupDescriptor bgDesc = {};
        bgDesc.layout = shadowSampleLayout_;
        bgDesc.entryCount = 2;
        bgDesc.entries = entries;
        shadowSampleGroup_ = wgpuDeviceCreateBindGroup(device_, &bgDesc);

        // Per-cascade index buffer (one 256-byte dynamic slot per cascade), read
        // by vs_shadow to pick its cascadeVP. Bound on group 1, binding 2.
        WGPUBufferDescriptor sidesc = {};
        sidesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        sidesc.size = static_cast<uint64_t>(kMaxCascades) * kDrawStride;
        shadowIdxBuf_ = wgpuDeviceCreateBuffer(device_, &sidesc);
        for (int c = 0; c < kMaxCascades; ++c) {
            int32_t idx[4] = {c, 0, 0, 0};
            wgpuQueueWriteBuffer(queue_, shadowIdxBuf_, static_cast<uint64_t>(c) * kDrawStride,
                                 idx, sizeof(idx));
        }
        WGPUBindGroupEntry sie = {};
        sie.binding = 2;
        sie.buffer = shadowIdxBuf_;
        sie.offset = 0;
        sie.size = sizeof(int32_t) * 4;
        WGPUBindGroupDescriptor sigDesc = {};
        sigDesc.layout = shadowVsLayout_;
        sigDesc.entryCount = 1;
        sigDesc.entries = &sie;
        shadowIdxGroup_ = wgpuDeviceCreateBindGroup(device_, &sigDesc);
    }

    void createMaterialDefaults() {
        WGPUSamplerDescriptor sd = {};
        sd.magFilter = WGPUFilterMode_Linear;
        sd.minFilter = WGPUFilterMode_Linear;
        sd.mipmapFilter = WGPUMipmapFilterMode_Linear;
        sd.addressModeU = WGPUAddressMode_Repeat;   // textures tile
        sd.addressModeV = WGPUAddressMode_Repeat;
        sd.addressModeW = WGPUAddressMode_Repeat;
        sd.maxAnisotropy = 1;
        materialSampler_ = wgpuDeviceCreateSampler(device_, &sd);

        const uint8_t white[4] = {255, 255, 255, 255};
        whiteTex_ = createTexture2D(1, 1, WGPUTextureFormat_RGBA8Unorm, white, 4);
        whiteView_ = wgpuTextureCreateView(whiteTex_, nullptr);
        const uint8_t flat[4] = {128, 128, 255, 255};   // tangent-space (0,0,1)
        flatNormalTex_ = createTexture2D(1, 1, WGPUTextureFormat_RGBA8Unorm, flat, 4);
        flatNormalView_ = wgpuTextureCreateView(flatNormalTex_, nullptr);

        // Mipmap-downsample pipeline (core WebGPU has no generateMipmap).
        WGPUShaderSourceWGSL bw = {};
        bw.chain.sType = WGPUSType_ShaderSourceWGSL;
        bw.code = sv(kBlitWgsl);
        WGPUShaderModuleDescriptor bmd = {};
        bmd.nextInChain = &bw.chain;
        WGPUShaderModule bmod = wgpuDeviceCreateShaderModule(device_, &bmd);
        WGPUBindGroupLayoutEntry be[2] = {};
        be[0].binding = 0; be[0].visibility = WGPUShaderStage_Fragment;
        be[0].texture.sampleType = WGPUTextureSampleType_Float;
        be[0].texture.viewDimension = WGPUTextureViewDimension_2D;
        be[1].binding = 1; be[1].visibility = WGPUShaderStage_Fragment;
        be[1].sampler.type = WGPUSamplerBindingType_Filtering;
        WGPUBindGroupLayoutDescriptor bld = {};
        bld.entryCount = 2; bld.entries = be;
        blitLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &bld);
        WGPUPipelineLayoutDescriptor bpl = {};
        bpl.bindGroupLayoutCount = 1; bpl.bindGroupLayouts = &blitLayout_;
        WGPUPipelineLayout blitPipeLayout = wgpuDeviceCreatePipelineLayout(device_, &bpl);
        WGPUColorTargetState bct = {};
        bct.format = WGPUTextureFormat_RGBA8Unorm; bct.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState bfs = {};
        bfs.module = bmod; bfs.entryPoint = sv("fs_blit"); bfs.targetCount = 1; bfs.targets = &bct;
        WGPURenderPipelineDescriptor bpd = {};
        bpd.layout = blitPipeLayout;
        bpd.vertex.module = bmod; bpd.vertex.entryPoint = sv("vs_blit");
        bpd.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        bpd.primitive.frontFace = WGPUFrontFace_CCW;
        bpd.primitive.cullMode = WGPUCullMode_None;
        bpd.fragment = &bfs;
        bpd.multisample.count = 1; bpd.multisample.mask = 0xFFFFFFFF;
        blitPipeline_ = wgpuDeviceCreateRenderPipeline(device_, &bpd);
        wgpuPipelineLayoutRelease(blitPipeLayout);
        wgpuShaderModuleRelease(bmod);
    }

    // Fill a texture's mip chain by successive downsample blits (base already up).
    void generateMips(WGPUTexture tex, int levels) {
        WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(device_, nullptr);
        for (int i = 1; i < levels; ++i) {
            WGPUTextureViewDescriptor svd = {};
            svd.format = WGPUTextureFormat_RGBA8Unorm;
            svd.dimension = WGPUTextureViewDimension_2D;
            svd.baseMipLevel = static_cast<uint32_t>(i - 1); svd.mipLevelCount = 1;
            svd.baseArrayLayer = 0; svd.arrayLayerCount = 1;
            WGPUTextureView src = wgpuTextureCreateView(tex, &svd);
            WGPUTextureViewDescriptor dvd = svd;
            dvd.baseMipLevel = static_cast<uint32_t>(i);
            WGPUTextureView dst = wgpuTextureCreateView(tex, &dvd);

            WGPUBindGroupEntry e[2] = {};
            e[0].binding = 0; e[0].textureView = src;
            // Clamp-to-edge for the downsample: the tiling material sampler
            // (Repeat) would bleed opposite edges into every mip.
            e[1].binding = 1; e[1].sampler = linearSampler_;
            WGPUBindGroupDescriptor bgd = {};
            bgd.layout = blitLayout_; bgd.entryCount = 2; bgd.entries = e;
            WGPUBindGroup bg = wgpuDeviceCreateBindGroup(device_, &bgd);

            WGPURenderPassColorAttachment a = {};
            a.view = dst; a.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
            a.loadOp = WGPULoadOp_Clear; a.storeOp = WGPUStoreOp_Store;
            a.clearValue = {0.0, 0.0, 0.0, 1.0};
            WGPURenderPassDescriptor pd = {};
            pd.colorAttachmentCount = 1; pd.colorAttachments = &a;
            WGPURenderPassEncoder rp = wgpuCommandEncoderBeginRenderPass(enc, &pd);
            wgpuRenderPassEncoderSetPipeline(rp, blitPipeline_);
            wgpuRenderPassEncoderSetBindGroup(rp, 0, bg, 0, nullptr);
            wgpuRenderPassEncoderDraw(rp, 3, 1, 0, 0);
            wgpuRenderPassEncoderEnd(rp);
            wgpuRenderPassEncoderRelease(rp);
            wgpuBindGroupRelease(bg);
            wgpuTextureViewRelease(src);
            wgpuTextureViewRelease(dst);
        }
        WGPUCommandBuffer cb = wgpuCommandEncoderFinish(enc, nullptr);
        wgpuQueueSubmit(queue_, 1, &cb);
        wgpuCommandBufferRelease(cb);
        wgpuCommandEncoderRelease(enc);
    }

    WGPUTextureView texViewOr(uint32_t index, WGPUTextureView def) {
        if (index == 0 || index > textures_.size()) return def;
        WGPUTextureView v = textures_[index - 1].view;
        return v ? v : def;
    }

    // Get-or-build the group-2 bind group for a material's map set (cached).
    WGPUBindGroup materialGroupFor(const MaterialKey& k) {
        auto it = materialGroups_.find(k);
        if (it != materialGroups_.end()) return it->second;
        WGPUBindGroupEntry e[6] = {};
        e[0].binding = 0; e[0].textureView = texViewOr(k.albedo, whiteView_);
        e[1].binding = 1; e[1].textureView = texViewOr(k.normal, flatNormalView_);
        e[2].binding = 2; e[2].textureView = texViewOr(k.mr, whiteView_);
        e[3].binding = 3; e[3].textureView = texViewOr(k.emissive, whiteView_);
        e[4].binding = 4; e[4].textureView = texViewOr(k.ao, whiteView_);
        e[5].binding = 5; e[5].sampler = materialSampler_;
        WGPUBindGroupDescriptor d = {};
        d.layout = materialLayout_; d.entryCount = 6; d.entries = e;
        WGPUBindGroup g = wgpuDeviceCreateBindGroup(device_, &d);
        materialGroups_.emplace(k, g);
        return g;
    }

    void createUniformResources() {
        WGPUBufferDescriptor gDesc = {};
        gDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        gDesc.size = sizeof(GpuGlobals);
        globalBuf_ = wgpuDeviceCreateBuffer(device_, &gDesc);

        createEnvResources();   // env sampler + 1x1 default + baked BRDF LUT

        drawCapacity_ = 256;  // initial per-draw slot count; grows as needed
        allocDrawBuffer(drawCapacity_);
        rebuildBindGroup();
        rebuildSkyBindGroup();

        // Composite: a small post-uniform buffer + a bind group over the HDR view.
        WGPUBufferDescriptor pDesc = {};
        pDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        pDesc.size = sizeof(GpuPost);
        postBuf_ = wgpuDeviceCreateBuffer(device_, &pDesc);

        // Bloom uniforms: one per pass (bright + the two blur directions).
        WGPUBufferDescriptor bDesc = {};
        bDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        bDesc.size = sizeof(GpuBloom);
        bloomUboBright_ = wgpuDeviceCreateBuffer(device_, &bDesc);
        bloomUboH_ = wgpuDeviceCreateBuffer(device_, &bDesc);
        bloomUboV_ = wgpuDeviceCreateBuffer(device_, &bDesc);

        WGPUBufferDescriptor aDesc = {};
        aDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        aDesc.size = sizeof(GpuSsao);
        ssaoUbo_ = wgpuDeviceCreateBuffer(device_, &aDesc);

        WGPUBufferDescriptor rDesc = {};
        rDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        rDesc.size = sizeof(GpuSsr);
        ssrUbo_ = wgpuDeviceCreateBuffer(device_, &rDesc);

        rebuildCompositeBindGroup();
        rebuildBloomGroups();
        rebuildSsaoGroup();
        rebuildSsrGroup();
    }

    // SSR bind group references hdr/depth/gbuffer views (recreated on resize).
    void rebuildSsrGroup() {
        if (ssrGroup_) { wgpuBindGroupRelease(ssrGroup_); ssrGroup_ = nullptr; }
        WGPUBindGroupEntry e[4] = {};
        e[0].binding = 0; e[0].textureView = hdrView_;
        e[1].binding = 1; e[1].textureView = depthView_;
        e[2].binding = 2; e[2].textureView = gbufView_;
        e[3].binding = 3; e[3].buffer = ssrUbo_; e[3].offset = 0; e[3].size = sizeof(GpuSsr);
        WGPUBindGroupDescriptor d = {};
        d.layout = ssrLayout_; d.entryCount = 4; d.entries = e;
        ssrGroup_ = wgpuDeviceCreateBindGroup(device_, &d);
    }

    // SSAO bind group references depthView_ (recreated on resize).
    void rebuildSsaoGroup() {
        if (ssaoGroup_) { wgpuBindGroupRelease(ssaoGroup_); ssaoGroup_ = nullptr; }
        WGPUBindGroupEntry e[3] = {};
        e[0].binding = 0; e[0].textureView = depthView_;
        e[1].binding = 1; e[1].buffer = ssaoUbo_; e[1].offset = 0; e[1].size = sizeof(GpuSsao);
        e[2].binding = 2; e[2].textureView = gbufView_;
        WGPUBindGroupDescriptor d = {};
        d.layout = ssaoLayout_; d.entryCount = 3; d.entries = e;
        ssaoGroup_ = wgpuDeviceCreateBindGroup(device_, &d);
    }

    // The composite bind group references hdrView_ + bloomViewA_, both recreated
    // on resize, so it must be rebuilt whenever those targets change.
    void rebuildCompositeBindGroup() {
        if (compositeBindGroup_) { wgpuBindGroupRelease(compositeBindGroup_); compositeBindGroup_ = nullptr; }
        WGPUBindGroupEntry ce[6] = {};
        ce[0].binding = 0;
        ce[0].textureView = hdrView_;
        ce[1].binding = 1;
        ce[1].buffer = postBuf_;
        ce[1].offset = 0;
        ce[1].size = sizeof(GpuPost);
        ce[2].binding = 2;
        ce[2].textureView = bloomViewA_;       // final blurred bloom
        ce[3].binding = 3;
        ce[3].sampler = linearSampler_;
        ce[4].binding = 4;
        ce[4].textureView = ssaoView_;
        ce[5].binding = 5;
        ce[5].textureView = ssrView_;
        WGPUBindGroupDescriptor cbgDesc = {};
        cbgDesc.layout = compositeLayout_;
        cbgDesc.entryCount = 6;
        cbgDesc.entries = ce;
        compositeBindGroup_ = wgpuDeviceCreateBindGroup(device_, &cbgDesc);
    }

    // Bloom bind groups reference the HDR + bloom views (recreated on resize).
    void rebuildBloomGroups() {
        auto make = [&](WGPUBindGroup& g, WGPUTextureView src, WGPUBuffer ubo) {
            if (g) { wgpuBindGroupRelease(g); g = nullptr; }
            WGPUBindGroupEntry e[3] = {};
            e[0].binding = 0; e[0].textureView = src;
            e[1].binding = 1; e[1].sampler = linearSampler_;
            e[2].binding = 2; e[2].buffer = ubo; e[2].offset = 0; e[2].size = sizeof(GpuBloom);
            WGPUBindGroupDescriptor d = {};
            d.layout = bloomLayout_; d.entryCount = 3; d.entries = e;
            g = wgpuDeviceCreateBindGroup(device_, &d);
        };
        make(bloomBrightGroup_, hdrView_, bloomUboBright_);   // HDR -> bloomA
        make(bloomBlurHGroup_, bloomViewA_, bloomUboH_);      // bloomA -> bloomB
        make(bloomBlurVGroup_, bloomViewB_, bloomUboV_);      // bloomB -> bloomA
    }

    void allocDrawBuffer(size_t slots) {
        if (drawBuf_) { wgpuBufferRelease(drawBuf_); drawBuf_ = nullptr; }
        WGPUBufferDescriptor dDesc = {};
        dDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        dDesc.size = slots * kDrawStride;
        drawBuf_ = wgpuDeviceCreateBuffer(device_, &dDesc);
    }

    void rebuildBindGroup() {
        if (bindGroup_) { wgpuBindGroupRelease(bindGroup_); bindGroup_ = nullptr; }
        WGPUBindGroupEntry entries[5] = {};
        entries[0].binding = 0;
        entries[0].buffer = globalBuf_;
        entries[0].offset = 0;
        entries[0].size = sizeof(GpuGlobals);
        entries[1].binding = 1;
        entries[1].buffer = drawBuf_;
        entries[1].offset = 0;
        entries[1].size = sizeof(GpuDraw);  // dynamic offset is applied per draw
        entries[2].binding = 2;
        entries[2].textureView = envCurrentView_;
        entries[3].binding = 3;
        entries[3].sampler = envSampler_;
        entries[4].binding = 4;
        entries[4].textureView = brdfLutView_;
        WGPUBindGroupDescriptor desc = {};
        desc.layout = bindLayout_;
        desc.entryCount = 5;
        desc.entries = entries;
        bindGroup_ = wgpuDeviceCreateBindGroup(device_, &desc);
    }

    // Sky bind group: globals + the env equirect + its sampler (own layout).
    void rebuildSkyBindGroup() {
        if (skyBindGroup_) { wgpuBindGroupRelease(skyBindGroup_); skyBindGroup_ = nullptr; }
        WGPUBindGroupEntry e[3] = {};
        e[0].binding = 0;
        e[0].buffer = globalBuf_;
        e[0].offset = 0;
        e[0].size = sizeof(GpuGlobals);
        e[1].binding = 2;
        e[1].textureView = envCurrentView_;
        e[2].binding = 3;
        e[2].sampler = envSampler_;
        WGPUBindGroupDescriptor d = {};
        d.layout = skyLayout_;
        d.entryCount = 3;
        d.entries = e;
        skyBindGroup_ = wgpuDeviceCreateBindGroup(device_, &d);
    }

    // Env sampler, a 1x1 default equirect (used when no HDR is bound), and the
    // baked split-sum BRDF LUT. Created before the group-0 bind groups.
    void createEnvResources() {
        WGPUSamplerDescriptor sd = {};
        sd.magFilter = WGPUFilterMode_Linear;
        sd.minFilter = WGPUFilterMode_Linear;
        sd.mipmapFilter = WGPUMipmapFilterMode_Linear;
        sd.addressModeU = WGPUAddressMode_Repeat;       // equirect wraps in longitude
        sd.addressModeV = WGPUAddressMode_ClampToEdge;  // ...clamps at the poles
        sd.addressModeW = WGPUAddressMode_ClampToEdge;
        sd.maxAnisotropy = 1;
        envSampler_ = wgpuDeviceCreateSampler(device_, &sd);

        const uint8_t gray[4] = {40, 44, 52, 255};   // dim neutral (never sampled at envMode 0)
        envDefaultTex_ = createTexture2D(1, 1, WGPUTextureFormat_RGBA8Unorm, gray, 4);
        envDefaultView_ = wgpuTextureCreateView(envDefaultTex_, nullptr);
        envCurrentView_ = envDefaultView_;

        bakeBrdfLut();
    }

    // Render the split-sum BRDF integration LUT once (RG16Float, 256x256).
    void bakeBrdfLut() {
        const int kRes = 256;
        WGPUTextureDescriptor td = {};
        td.usage = WGPUTextureUsage_TextureBinding | WGPUTextureUsage_RenderAttachment;
        td.dimension = WGPUTextureDimension_2D;
        td.size = {static_cast<uint32_t>(kRes), static_cast<uint32_t>(kRes), 1};
        td.format = WGPUTextureFormat_RG16Float;
        td.mipLevelCount = 1;
        td.sampleCount = 1;
        brdfLutTex_ = wgpuDeviceCreateTexture(device_, &td);
        brdfLutView_ = wgpuTextureCreateView(brdfLutTex_, nullptr);

        WGPUShaderSourceWGSL src = {};
        src.chain.sType = WGPUSType_ShaderSourceWGSL;
        src.code = sv(kBrdfWgsl);
        WGPUShaderModuleDescriptor smd = {};
        smd.nextInChain = &src.chain;
        WGPUShaderModule mod = wgpuDeviceCreateShaderModule(device_, &smd);

        WGPUPipelineLayoutDescriptor pld = {};   // no bindings — pure numeric bake
        pld.bindGroupLayoutCount = 0;
        WGPUPipelineLayout pl = wgpuDeviceCreatePipelineLayout(device_, &pld);
        WGPUColorTargetState ct = {};
        ct.format = WGPUTextureFormat_RG16Float;
        ct.writeMask = WGPUColorWriteMask_All;
        WGPUFragmentState fs = {};
        fs.module = mod; fs.entryPoint = sv("fs"); fs.targetCount = 1; fs.targets = &ct;
        WGPURenderPipelineDescriptor pd = {};
        pd.layout = pl;
        pd.vertex.module = mod; pd.vertex.entryPoint = sv("vs");
        pd.primitive.topology = WGPUPrimitiveTopology_TriangleList;
        pd.primitive.frontFace = WGPUFrontFace_CCW;
        pd.primitive.cullMode = WGPUCullMode_None;
        pd.fragment = &fs;
        pd.multisample.count = 1; pd.multisample.mask = 0xFFFFFFFF;
        WGPURenderPipeline pipe = wgpuDeviceCreateRenderPipeline(device_, &pd);

        WGPUCommandEncoder enc = wgpuDeviceCreateCommandEncoder(device_, nullptr);
        WGPURenderPassColorAttachment a = {};
        a.view = brdfLutView_; a.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        a.loadOp = WGPULoadOp_Clear; a.storeOp = WGPUStoreOp_Store;
        a.clearValue = {0.0, 0.0, 0.0, 1.0};
        WGPURenderPassDescriptor rpd = {};
        rpd.colorAttachmentCount = 1; rpd.colorAttachments = &a;
        WGPURenderPassEncoder rp = wgpuCommandEncoderBeginRenderPass(enc, &rpd);
        wgpuRenderPassEncoderSetPipeline(rp, pipe);
        wgpuRenderPassEncoderDraw(rp, 3, 1, 0, 0);
        wgpuRenderPassEncoderEnd(rp);
        wgpuRenderPassEncoderRelease(rp);
        WGPUCommandBuffer cb = wgpuCommandEncoderFinish(enc, nullptr);
        wgpuQueueSubmit(queue_, 1, &cb);
        wgpuCommandBufferRelease(cb);
        wgpuCommandEncoderRelease(enc);
        wgpuRenderPipelineRelease(pipe);
        wgpuPipelineLayoutRelease(pl);
        wgpuShaderModuleRelease(mod);
    }

    void ensureDrawCapacity(size_t needed) {
        if (needed <= drawCapacity_) return;
        while (drawCapacity_ < needed) drawCapacity_ *= 2;
        allocDrawBuffer(drawCapacity_);
        rebuildBindGroup();
    }

    // ---- state -------------------------------------------------------------

    int width_ = 1, height_ = 1;

    WGPUInstance instance_ = nullptr;
    WGPUAdapter adapter_ = nullptr;
    WGPUDevice device_ = nullptr;
    WGPUQueue queue_ = nullptr;
    WGPUSurface surface_ = nullptr;
    bool adapterDone_ = false;
    bool deviceDone_ = false;

    WGPUTexture depthTexture_ = nullptr;
    WGPUTextureView depthView_ = nullptr;
    WGPUTexture hdrTexture_ = nullptr;            // linear HDR scene target
    WGPUTextureView hdrView_ = nullptr;
    WGPUTexture gbufTexture_ = nullptr;           // material G-buffer (normal, roughness)
    WGPUTextureView gbufView_ = nullptr;
    WGPUTexture bloomTexA_ = nullptr, bloomTexB_ = nullptr;     // half-res ping-pong
    WGPUTextureView bloomViewA_ = nullptr, bloomViewB_ = nullptr;
    int bloomW_ = 1, bloomH_ = 1;
    int fxW_ = 1, fxH_ = 1;           // SSAO/SSR buffer size (postEffectScale)
    float appliedFxScale_ = 0.0f;     // last-applied scale; 0 forces first build
    WGPUSampler linearSampler_ = nullptr;
    WGPUBindGroupLayout bloomLayout_ = nullptr;
    WGPURenderPipeline bloomBrightPipeline_ = nullptr, bloomBlurPipeline_ = nullptr;
    WGPUBuffer bloomUboBright_ = nullptr, bloomUboH_ = nullptr, bloomUboV_ = nullptr;
    WGPUBindGroup bloomBrightGroup_ = nullptr, bloomBlurHGroup_ = nullptr, bloomBlurVGroup_ = nullptr;
    WGPUTexture ssaoTex_ = nullptr;
    WGPUTextureView ssaoView_ = nullptr;
    WGPUBindGroupLayout ssaoLayout_ = nullptr;
    WGPURenderPipeline ssaoPipeline_ = nullptr;
    WGPUBuffer ssaoUbo_ = nullptr;
    WGPUBindGroup ssaoGroup_ = nullptr;
    WGPUTexture ssrTexture_ = nullptr;
    WGPUTextureView ssrView_ = nullptr;
    WGPUBindGroupLayout ssrLayout_ = nullptr;
    WGPURenderPipeline ssrPipeline_ = nullptr;
    WGPUBuffer ssrUbo_ = nullptr;
    WGPUBindGroup ssrGroup_ = nullptr;

    // Planetary atmosphere glow (procedural-planet-plan P3): additive HDR pass.
    WGPUBindGroupLayout atmosphereLayout_ = nullptr;
    WGPURenderPipeline atmospherePipeline_ = nullptr;
    WGPUBuffer atmosphereUbo_ = nullptr;
    WGPUBindGroup atmosphereGroup_ = nullptr;
    AtmosphereRenderParams atmosphere_;   // enabled=false by default -> pass skipped

    WGPUBindGroupLayout bindLayout_ = nullptr;
    WGPURenderPipeline pipeline_ = nullptr;
    WGPURenderPipeline overlayPipeline_ = nullptr;   // FLAG_OVERLAY: depth Always, no write, drawn last
    WGPURenderPipeline instancedOverlayPipeline_ = nullptr;   // FLAG_OVERLAY InstanceGroups (citysim gizmos)
    WGPUBindGroupLayout skyLayout_ = nullptr;     // group 0 (globals only)
    WGPURenderPipeline skyPipeline_ = nullptr;    // fullscreen procedural sky
    WGPUBindGroup skyBindGroup_ = nullptr;
    WGPUBindGroupLayout compositeLayout_ = nullptr;  // HDR -> swapchain
    WGPURenderPipeline compositePipeline_ = nullptr;
    WGPUBindGroup compositeBindGroup_ = nullptr;
    WGPUBuffer postBuf_ = nullptr;
    WGPUBuffer globalBuf_ = nullptr;
    WGPUBuffer drawBuf_ = nullptr;
    WGPUBindGroup bindGroup_ = nullptr;
    size_t drawCapacity_ = 0;
    WGPURenderPipeline instancedPipeline_ = nullptr, instancedShadowPipeline_ = nullptr;
    WGPURenderPipeline terrainPipeline_ = nullptr, terrainShadowPipeline_ = nullptr;
    WGPUBuffer instanceBuf_ = nullptr;
    size_t instanceCapacity_ = 0;   // in instances (64 bytes each)
    std::vector<InstancedGroup> instancedGroups_;
    std::vector<float> instanceStaging_;
    std::vector<QueuedDraw> terrainDraws_;

    // Sun shadow (cascaded, up to 4 layers): a depth array + its own pipeline; the main
    // pipeline samples it via group 1.
    static constexpr int kShadowMapSize = 2048;
    static constexpr int kMaxCascades = 4;
    WGPUTexture shadowTexture_ = nullptr;
    WGPUTextureView shadowArrayView_ = nullptr;          // array view for sampling
    WGPUTextureView shadowLayerViews_[kMaxCascades] = {};// per-cascade render targets
    WGPUSampler shadowSampler_ = nullptr;
    WGPUBindGroupLayout shadowSampleLayout_ = nullptr;  // group 1 (array tex + comparison sampler)
    WGPUBindGroup shadowSampleGroup_ = nullptr;
    WGPUBindGroupLayout shadowVsLayout_ = nullptr;       // group 1 (shadow pass cascade index)
    WGPUBuffer shadowIdxBuf_ = nullptr;
    WGPUBindGroup shadowIdxGroup_ = nullptr;
    WGPURenderPipeline shadowPipeline_ = nullptr;
    Vec3 cameraEye_;
    Mat4 camVP_;                            // camera view-projection (cascade fit)
    Real camNear_ = 0.1, camFar_ = 1000.0;
    Vec3 sunDir_{0, 1, 0};
    bool shadowOn_ = false;
    int shadowCascadeCount_ = 3, activeCascades_ = 0;
    float windTime_ = 0.0f;
    double windLastMs_ = 0.0;             // wall clock of the last wind tick
    std::vector<uint8_t> drawStaging_;    // reused per-frame draw-uniform scratch
    float shadowDistance_ = 150.0f, shadowDepthBias_ = 0.0015f,
          shadowNormalBias_ = 0.04f, shadowPcf_ = 1.0f;

    std::vector<GpuMesh> meshes_;
    std::vector<uint32_t> freeSlots_;
    uint32_t generationCounter_ = 0;
    uint32_t textureCounter_ = 0;

    // Material textures (group 2): a slab of GpuTextures + a bind-group cache
    // keyed by the 5-map set, plus 1x1 defaults for missing maps.
    std::vector<GpuTexture> textures_;
    std::vector<uint32_t> freeTextures_;
    std::unordered_map<MaterialKey, WGPUBindGroup, MaterialKeyHash> materialGroups_;
    WGPUBindGroupLayout materialLayout_ = nullptr;
    WGPUSampler materialSampler_ = nullptr;
    WGPUTexture whiteTex_ = nullptr;      WGPUTextureView whiteView_ = nullptr;
    WGPUTexture flatNormalTex_ = nullptr; WGPUTextureView flatNormalView_ = nullptr;
    WGPUBindGroupLayout blitLayout_ = nullptr;   // mipmap downsample
    WGPURenderPipeline blitPipeline_ = nullptr;

    // Scene environment / IBL (group 0). envCurrentView_ is the bound HDR equirect
    // or a 1x1 default; envMode_ mirrors it into skySunColor.w for the shaders.
    WGPUSampler envSampler_ = nullptr;
    WGPUTexture envDefaultTex_ = nullptr; WGPUTextureView envDefaultView_ = nullptr;
    WGPUTextureView envCurrentView_ = nullptr;
    WGPUTexture brdfLutTex_ = nullptr;    WGPUTextureView brdfLutView_ = nullptr;
    bool envMode_ = false;

    std::vector<QueuedDraw> draws_;
    bool frameDiagLogged_ = false;
    bool bindDiagLogged_ = false;   // one-shot draw/bind-count diagnostic
    GpuGlobals globals_ = {};
    WGPUColor clearColor_ = {0.5, 0.7, 0.9, 1.0};
    RenderStats stats_;
};

}  // namespace

// The web build links exactly this backend, so it provides the factory (the
// Metal/Vulkan/Null TUs provide it for their platforms — link only one).
std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<WebGpuRenderer>();
}

}  // namespace engine
