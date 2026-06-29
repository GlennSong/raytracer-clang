// WebGPU renderer backend (ADR-0058) — the web target's implementation of the
// `Renderer` seam (../../renderer/renderer.h), compiled under Emscripten and
// driving the browser's WebGPU device. Phases 0+1: device/surface bring-up,
// a cleared swapchain with a depth buffer, and a forward, single-directional-
// light Cook-Torrance pass over uploaded meshes (no shadows, textures, post,
// instancing, or terrain morph yet — those are later phases, mirroring the
// Vulkan backend's phasing in docs/webgpu-renderer-plan.md).
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
#include "../../log.h"

#include <webgpu/webgpu.h>
#include <emscripten/emscripten.h>

#include <cmath>
#include <cstring>
#include <vector>

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

// std140-compatible scene globals (group 0, binding 0). Mirrors `Globals` in the
// WGSL. 128 bytes.
struct GpuGlobals {
    float viewProjection[16];  // column-major (GPU layout)
    float cameraPosition[4];   // xyz, w unused
    float sunDirection[4];     // xyz toward the sun, w intensity
    float sunColor[4];         // rgb, w unused
    float ambient[4];          // rgb ambient term (tint * multiplier), w unused
};

// Per-draw uniforms (group 0, binding 1, dynamic). 96 bytes, written into a
// kDrawStride slot.
struct GpuDraw {
    float model[16];          // column-major
    float albedoMetallic[4];  // rgb albedo, a metallic
    float emissionRough[4];   // rgb emission, a roughness
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

// The forward mesh shader. Embedded as a string (the web build has no offline
// shader-compile step; this matches the Metal backend's runtime string source).
// Phase 1: one directional light + flat ambient, per-vertex tint, scene-linear
// shading with a manual sRGB encode at the end.
const char* kMeshWgsl = R"WGSL(
struct Globals {
  viewProjection : mat4x4<f32>,
  cameraPosition : vec4<f32>,
  sunDirection   : vec4<f32>,   // xyz toward sun, w intensity
  sunColor       : vec4<f32>,
  ambient        : vec4<f32>,
};
struct DrawData {
  model          : mat4x4<f32>,
  albedoMetallic : vec4<f32>,   // rgb albedo, a metallic
  emissionRough  : vec4<f32>,   // rgb emission, a roughness
};

@group(0) @binding(0) var<uniform> g : Globals;
@group(0) @binding(1) var<uniform> d : DrawData;

struct VSOut {
  @builtin(position) clip : vec4<f32>,
  @location(0) worldPos    : vec3<f32>,
  @location(1) worldNormal : vec3<f32>,
  @location(2) color       : vec3<f32>,
};

@vertex
fn vs_main(
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) tangent  : vec3<f32>,
  @location(3) texcoord : vec2<f32>,
  @location(4) color    : vec3<f32>,
) -> VSOut {
  var out : VSOut;
  let world = d.model * vec4<f32>(position, 1.0);
  out.worldPos = world.xyz;
  // Phase 1 uses the model's upper 3x3 directly (correct for rigid / uniform
  // scale). A proper inverse-transpose normal matrix arrives with non-uniform
  // scaling support in a later phase (WGSL has no inverse() builtin).
  out.worldNormal = normalize((d.model * vec4<f32>(normal, 0.0)).xyz);
  out.color = color;
  out.clip = g.viewProjection * world;
  return out;
}

const PI : f32 = 3.14159265359;

fn distributionGGX(NdotH : f32, a2 : f32) -> f32 {
  let dd = NdotH * NdotH * (a2 - 1.0) + 1.0;
  return a2 / max(PI * dd * dd, 1e-6);
}
fn visibilitySmith(NdotV : f32, NdotL : f32, a2 : f32) -> f32 {
  let gv = NdotL * sqrt(NdotV * NdotV * (1.0 - a2) + a2);
  let gl = NdotV * sqrt(NdotL * NdotL * (1.0 - a2) + a2);
  return 0.5 / max(gv + gl, 1e-5);
}
fn fresnelSchlick(cosT : f32, f0 : vec3<f32>) -> vec3<f32> {
  return f0 + (vec3<f32>(1.0) - f0) * pow(clamp(1.0 - cosT, 0.0, 1.0), 5.0);
}

@fragment
fn fs_main(in : VSOut) -> @location(0) vec4<f32> {
  let albedo    = d.albedoMetallic.rgb * in.color;
  let metallic  = clamp(d.albedoMetallic.a, 0.0, 1.0);
  let roughness = clamp(d.emissionRough.a, 0.04, 1.0);
  let emission  = d.emissionRough.rgb;

  let N = normalize(in.worldNormal);
  let V = normalize(g.cameraPosition.xyz - in.worldPos);
  let L = normalize(g.sunDirection.xyz);
  let f0 = mix(vec3<f32>(0.04), albedo, metallic);
  let a  = max(roughness * roughness, 0.002);
  let a2 = a * a;
  let NdotV = max(dot(N, V), 1e-4);
  let NdotL = max(dot(N, L), 0.0);

  var direct = vec3<f32>(0.0);
  if (NdotL > 0.0) {
    let H = normalize(L + V);
    let NdotH = max(dot(N, H), 0.0);
    let VdotH = max(dot(V, H), 0.0);
    let D   = distributionGGX(NdotH, a2);
    let Vis = visibilitySmith(NdotV, NdotL, a2);
    let F   = fresnelSchlick(VdotH, f0);
    let spec = D * Vis * F;
    let diff = (vec3<f32>(1.0) - F) * (1.0 - metallic) * albedo / PI;
    direct = (diff + spec) * g.sunColor.rgb * (g.sunDirection.w * NdotL);
  }

  // Flat ambient stand-in for IBL (a later phase brings the procedural-sky /
  // baked-cube path over from the Metal/Vulkan backends).
  let ambient = albedo * g.ambient.rgb;
  var color = direct + ambient + emission;
  // The swapchain is non-sRGB (kSwapFormat), so encode here.
  color = pow(color, vec3<f32>(1.0 / 2.2));
  return vec4<f32>(color, 1.0);
}
)WGSL";

class WebGpuRenderer final : public Renderer {
public:
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

        LOG_INFO("WebGPU backend initialized (%dx%d)", width_, height_);
        return true;
    }

    void shutdown() override {
        releaseDepthTarget();
        if (bindGroup_) { wgpuBindGroupRelease(bindGroup_); bindGroup_ = nullptr; }
        if (globalBuf_) { wgpuBufferRelease(globalBuf_); globalBuf_ = nullptr; }
        if (drawBuf_)   { wgpuBufferRelease(drawBuf_);   drawBuf_ = nullptr; }
        if (pipeline_)  { wgpuRenderPipelineRelease(pipeline_); pipeline_ = nullptr; }
        if (bindLayout_) { wgpuBindGroupLayoutRelease(bindLayout_); bindLayout_ = nullptr; }
        for (auto& m : meshes_) freeMesh(m);
        meshes_.clear();
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

    TextureHandle uploadTexture(int, int, int, const uint8_t*) override {
        // Phase 2: real GPU textures. Return a valid-looking handle so material
        // setup that stores a texture handle doesn't trip; sampling is a no-op
        // until the texture path lands.
        TextureHandle h;
        h.index = ++textureCounter_;
        h.generation = 1;
        return h;
    }
    void removeTexture(TextureHandle) override {}

    RenderStats getRenderStats() const override { return stats_; }

    void beginFrame() override {
        draws_.clear();
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
        globals_.cameraPosition[0] = (float)camera.position.x;
        globals_.cameraPosition[1] = (float)camera.position.y;
        globals_.cameraPosition[2] = (float)camera.position.z;
        globals_.cameraPosition[3] = 1.0f;
    }

    void setLights(const SceneLighting& lighting) override {
        const DirectionalLight& sun = lighting.sun;
        Vec3 dir = normalize(sun.direction);
        globals_.sunDirection[0] = (float)dir.x;
        globals_.sunDirection[1] = (float)dir.y;
        globals_.sunDirection[2] = (float)dir.z;
        globals_.sunDirection[3] = sun.intensity;
        globals_.sunColor[0] = (float)sun.color.x;
        globals_.sunColor[1] = (float)sun.color.y;
        globals_.sunColor[2] = (float)sun.color.z;
        globals_.sunColor[3] = 0.0f;
        float amb = lighting.ambientMultiplier;
        globals_.ambient[0] = (float)lighting.ambientTint.x * amb;
        globals_.ambient[1] = (float)lighting.ambientTint.y * amb;
        globals_.ambient[2] = (float)lighting.ambientTint.z * amb;
        globals_.ambient[3] = 0.0f;

        // Clear color: the procedural sky's horizon tint, so empty regions read
        // as sky rather than a flat fill (gamma-encoded to match the shader).
        const Vec3& h = lighting.sky.horizonColor;
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
        draws_.push_back(qd);
    }

    void endFrame() override {
        if (!device_ || !surface_) return;

        // Upload scene globals.
        wgpuQueueWriteBuffer(queue_, globalBuf_, 0, &globals_, sizeof(GpuGlobals));

        // Grow the per-draw uniform buffer (and rebuild the bind group bound to
        // it) if this frame needs more slots than the current capacity.
        ensureDrawCapacity(draws_.size());

        // Pack each draw into its 256-byte slot and upload in one write.
        if (!draws_.empty()) {
            std::vector<uint8_t> staging(draws_.size() * kDrawStride, 0);
            for (size_t i = 0; i < draws_.size(); ++i)
                std::memcpy(staging.data() + i * kDrawStride, &draws_[i].data, sizeof(GpuDraw));
            wgpuQueueWriteBuffer(queue_, drawBuf_, 0, staging.data(), staging.size());
        }

        WGPUSurfaceTexture surfaceTexture = {};
        wgpuSurfaceGetCurrentTexture(surface_, &surfaceTexture);
        if (!surfaceTexture.texture) {
            LOG_WARN("WebGPU: no current surface texture this frame");
            return;
        }
        WGPUTextureView backbuffer = wgpuTextureCreateView(surfaceTexture.texture, nullptr);

        WGPUCommandEncoder encoder = wgpuDeviceCreateCommandEncoder(device_, nullptr);

        WGPURenderPassColorAttachment color = {};
        color.view = backbuffer;
        color.depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        color.loadOp = WGPULoadOp_Clear;
        color.storeOp = WGPUStoreOp_Store;
        color.clearValue = clearColor_;

        WGPURenderPassDepthStencilAttachment depth = {};
        depth.view = depthView_;
        depth.depthLoadOp = WGPULoadOp_Clear;
        depth.depthStoreOp = WGPUStoreOp_Store;
        depth.depthClearValue = 1.0f;

        WGPURenderPassDescriptor passDesc = {};
        passDesc.colorAttachmentCount = 1;
        passDesc.colorAttachments = &color;
        passDesc.depthStencilAttachment = &depth;

        WGPURenderPassEncoder pass = wgpuCommandEncoderBeginRenderPass(encoder, &passDesc);
        wgpuRenderPassEncoderSetPipeline(pass, pipeline_);

        for (size_t i = 0; i < draws_.size(); ++i) {
            const GpuMesh& m = meshes_[draws_[i].mesh];
            uint32_t dynOffset = static_cast<uint32_t>(i * kDrawStride);
            wgpuRenderPassEncoderSetBindGroup(pass, 0, bindGroup_, 1, &dynOffset);
            wgpuRenderPassEncoderSetVertexBuffer(pass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
            wgpuRenderPassEncoderSetIndexBuffer(pass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                0, WGPU_WHOLE_SIZE);
            wgpuRenderPassEncoderDrawIndexed(pass, m.indexCount, 1, 0, 0, 0);
            stats_.drawCalls++;
            stats_.trianglesDrawn += m.indexCount / 3;
        }

        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);

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
        desc.usage = WGPUTextureUsage_RenderAttachment;
        desc.dimension = WGPUTextureDimension_2D;
        desc.size = {static_cast<uint32_t>(width_), static_cast<uint32_t>(height_), 1};
        desc.format = kDepthFormat;
        desc.mipLevelCount = 1;
        desc.sampleCount = 1;
        depthTexture_ = wgpuDeviceCreateTexture(device_, &desc);
        depthView_ = wgpuTextureCreateView(depthTexture_, nullptr);
    }

    void releaseDepthTarget() {
        if (depthView_) { wgpuTextureViewRelease(depthView_); depthView_ = nullptr; }
        if (depthTexture_) { wgpuTextureRelease(depthTexture_); depthTexture_ = nullptr; }
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

        // Bind group layout: globals (uniform) + per-draw (dynamic uniform).
        WGPUBindGroupLayoutEntry entries[2] = {};
        entries[0].binding = 0;
        entries[0].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
        entries[0].buffer.type = WGPUBufferBindingType_Uniform;
        entries[0].buffer.minBindingSize = sizeof(GpuGlobals);
        entries[1].binding = 1;
        entries[1].visibility = WGPUShaderStage_Vertex | WGPUShaderStage_Fragment;
        entries[1].buffer.type = WGPUBufferBindingType_Uniform;
        entries[1].buffer.hasDynamicOffset = true;
        entries[1].buffer.minBindingSize = sizeof(GpuDraw);

        WGPUBindGroupLayoutDescriptor blDesc = {};
        blDesc.entryCount = 2;
        blDesc.entries = entries;
        bindLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &blDesc);

        WGPUPipelineLayoutDescriptor plDesc = {};
        plDesc.bindGroupLayoutCount = 1;
        plDesc.bindGroupLayouts = &bindLayout_;
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

        WGPUColorTargetState colorTarget = {};
        colorTarget.format = kSwapFormat;
        colorTarget.writeMask = WGPUColorWriteMask_All;

        WGPUFragmentState fragment = {};
        fragment.module = module;
        fragment.entryPoint = sv("fs_main");
        fragment.targetCount = 1;
        fragment.targets = &colorTarget;

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
        // Phase 1: no back-face culling until winding is confirmed on device
        // (matches the Vulkan Phase-1 choice); a later phase turns it on.
        desc.primitive.cullMode = WGPUCullMode_None;
        desc.depthStencil = &depthState;
        desc.fragment = &fragment;
        desc.multisample.count = 1;
        desc.multisample.mask = 0xFFFFFFFF;

        pipeline_ = wgpuDeviceCreateRenderPipeline(device_, &desc);

        wgpuPipelineLayoutRelease(pipelineLayout);
        wgpuShaderModuleRelease(module);
        if (!pipeline_) {
            LOG_ERROR("WebGPU: render pipeline creation failed");
            return false;
        }
        return true;
    }

    void createUniformResources() {
        WGPUBufferDescriptor gDesc = {};
        gDesc.usage = WGPUBufferUsage_Uniform | WGPUBufferUsage_CopyDst;
        gDesc.size = sizeof(GpuGlobals);
        globalBuf_ = wgpuDeviceCreateBuffer(device_, &gDesc);

        drawCapacity_ = 256;  // initial per-draw slot count; grows as needed
        allocDrawBuffer(drawCapacity_);
        rebuildBindGroup();
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
        WGPUBindGroupEntry entries[2] = {};
        entries[0].binding = 0;
        entries[0].buffer = globalBuf_;
        entries[0].offset = 0;
        entries[0].size = sizeof(GpuGlobals);
        entries[1].binding = 1;
        entries[1].buffer = drawBuf_;
        entries[1].offset = 0;
        entries[1].size = sizeof(GpuDraw);  // dynamic offset is applied per draw
        WGPUBindGroupDescriptor desc = {};
        desc.layout = bindLayout_;
        desc.entryCount = 2;
        desc.entries = entries;
        bindGroup_ = wgpuDeviceCreateBindGroup(device_, &desc);
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

    WGPUBindGroupLayout bindLayout_ = nullptr;
    WGPURenderPipeline pipeline_ = nullptr;
    WGPUBuffer globalBuf_ = nullptr;
    WGPUBuffer drawBuf_ = nullptr;
    WGPUBindGroup bindGroup_ = nullptr;
    size_t drawCapacity_ = 0;

    std::vector<GpuMesh> meshes_;
    std::vector<uint32_t> freeSlots_;
    uint32_t generationCounter_ = 0;
    uint32_t textureCounter_ = 0;

    std::vector<QueuedDraw> draws_;
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
