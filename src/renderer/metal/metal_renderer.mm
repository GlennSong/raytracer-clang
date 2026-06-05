#ifdef __APPLE__

#import "metal_renderer.h"
#import <Metal/Metal.h>
#import <QuartzCore/CAMetalLayer.h>
#import <AppKit/AppKit.h>
#import <simd/simd.h>
#include "../../slot_map.h"
#include <vector>
#include <algorithm>

#ifdef RT_ENABLE_IMGUI
#include "imgui.h"
#include "backends/imgui_impl_metal.h"
#endif

// Uniform structs must match Metal shader layout exactly.
// simd_float3 is 16 bytes on Apple — no manual padding between float3 fields.

struct CameraUniforms {
    simd_float4x4 viewProjection;
    simd_float4x4 view;
    simd_float3 cameraPosition;
};

struct ModelUniforms {
    simd_float4x4 model;
    simd_float4x4 normalMatrix;
};

struct MaterialUniforms {
    simd_float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    simd_float3 emission;
};

struct LightData {
    simd_float3 position;
    simd_float3 color;
    float intensity;
};

struct LightUniforms {
    LightData lights[8];
    int lightCount;
    float exposure;
};

struct GPUMesh {
    id<MTLBuffer> vertexBuffer;
    id<MTLBuffer> indexBuffer;
    uint32_t indexCount;
    int materialIndex;
};

static simd_float4x4 toSimd(const Mat4& m) {
    simd_float4x4 result;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            result.columns[j][i] = static_cast<float>(m.m[i][j]);
    return result;
}

static simd_float4x4 inverseTranspose(simd_float4x4 m) {
    return simd_transpose(simd_inverse(m));
}

struct MetalRenderer::Impl {
    id<MTLDevice> device;
    id<MTLCommandQueue> commandQueue;
    id<MTLRenderPipelineState> opaquePipeline;
    id<MTLRenderPipelineState> transparentPipeline;
    id<MTLDepthStencilState> depthStateOpaque;
    id<MTLDepthStencilState> depthStateTransparent;
    CAMetalLayer* metalLayer;
    NSWindow* nsWindow;
    id<MTLTexture> depthTexture;

    // Generation-checked GPU mesh storage (ADR-0007): hands out MeshHandles and
    // detects use of a freed/reused handle, replacing the old uint32 counter.
    SlotMap<GPUMesh, MeshTag> meshes;

    CameraUniforms cameraUniforms;
    LightUniforms lightUniforms;

    id<CAMetalDrawable> currentDrawable;
    id<MTLCommandBuffer> currentCommandBuffer;
    id<MTLRenderCommandEncoder> currentEncoder;
    MTLRenderPassDescriptor* currentPassDesc;   // built in beginFrame, used in endFrame
    bool imguiInitialized = false;

    int framebufferWidth = 0;
    int framebufferHeight = 0;

    struct DrawCall {
        MeshHandle meshHandle;
        Mat4 transform;
        RenderMaterial material;
        float distanceToCamera;
    };

    std::vector<DrawCall> opaqueDrawCalls;
    std::vector<DrawCall> transparentDrawCalls;
    Vec3 currentCameraPos;
};

MetalRenderer::MetalRenderer() : impl(std::make_unique<Impl>()) {}
MetalRenderer::~MetalRenderer() { shutdown(); }

bool MetalRenderer::initialize(void* windowHandle, int width, int height) {
    NSWindow* nsWindow = (__bridge NSWindow*)windowHandle;

    impl->device = MTLCreateSystemDefaultDevice();
    if (!impl->device) return false;

    impl->commandQueue = [impl->device newCommandQueue];

    impl->metalLayer = [CAMetalLayer layer];
    impl->metalLayer.device = impl->device;
    impl->metalLayer.pixelFormat = MTLPixelFormatBGRA8Unorm;
    impl->metalLayer.framebufferOnly = YES;
    impl->metalLayer.contentsScale = nsWindow.backingScaleFactor;

    nsWindow.contentView.wantsLayer = YES;
    nsWindow.contentView.layer = impl->metalLayer;
    impl->nsWindow = nsWindow;

    // Load shaders
    NSError* error = nil;
    NSString* shaderPath = @"shaders/metal/phong.metal";
    NSString* shaderSource = [NSString stringWithContentsOfFile:shaderPath
                                                      encoding:NSUTF8StringEncoding
                                                         error:&error];
    if (!shaderSource) {
        NSLog(@"Failed to load shader: %@", error);
        return false;
    }

    MTLCompileOptions* options = [[MTLCompileOptions alloc] init];
    id<MTLLibrary> library = [impl->device newLibraryWithSource:shaderSource
                                                        options:options
                                                          error:&error];
    if (!library) {
        NSLog(@"Shader compile error: %@", error);
        return false;
    }

    id<MTLFunction> vertexFunc = [library newFunctionWithName:@"vertexMain"];
    id<MTLFunction> fragmentFunc = [library newFunctionWithName:@"fragmentMain"];

    // Opaque pipeline
    MTLRenderPipelineDescriptor* pipelineDesc = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineDesc.vertexFunction = vertexFunc;
    pipelineDesc.fragmentFunction = fragmentFunc;
    pipelineDesc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
    pipelineDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;

    impl->opaquePipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                        error:&error];
    if (!impl->opaquePipeline) {
        NSLog(@"Pipeline error: %@", error);
        return false;
    }

    // Transparent pipeline (alpha blending)
    pipelineDesc.colorAttachments[0].blendingEnabled = YES;
    pipelineDesc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    pipelineDesc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    pipelineDesc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
    pipelineDesc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;

    impl->transparentPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                             error:&error];

    // Depth states
    MTLDepthStencilDescriptor* depthDesc = [[MTLDepthStencilDescriptor alloc] init];
    depthDesc.depthCompareFunction = MTLCompareFunctionLess;
    depthDesc.depthWriteEnabled = YES;
    impl->depthStateOpaque = [impl->device newDepthStencilStateWithDescriptor:depthDesc];

    depthDesc.depthWriteEnabled = NO;
    impl->depthStateTransparent = [impl->device newDepthStencilStateWithDescriptor:depthDesc];

    resize(width, height);
    return true;
}

void MetalRenderer::shutdown() {
    impl->meshes.clear();
    impl->device = nil;
}

void MetalRenderer::resize(int width, int height) {
    impl->framebufferWidth = width;
    impl->framebufferHeight = height;
    impl->metalLayer.contentsScale = impl->nsWindow.backingScaleFactor;
    impl->metalLayer.drawableSize = CGSizeMake(width, height);

    MTLTextureDescriptor* depthDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    depthDesc.usage = MTLTextureUsageRenderTarget;
    depthDesc.storageMode = MTLStorageModePrivate;
    impl->depthTexture = [impl->device newTextureWithDescriptor:depthDesc];
}

MeshHandle MetalRenderer::uploadMesh(const RenderMesh& mesh) {
    GPUMesh gpuMesh;
    gpuMesh.materialIndex = mesh.materialIndex;

    struct GPUVertex {
        float position[3];
        float normal[3];
        float texcoord[2];
    };

    std::vector<GPUVertex> gpuVertices(mesh.vertices.size());
    for (size_t i = 0; i < mesh.vertices.size(); i++) {
        gpuVertices[i] = {
            {static_cast<float>(mesh.vertices[i].position.x),
             static_cast<float>(mesh.vertices[i].position.y),
             static_cast<float>(mesh.vertices[i].position.z)},
            {static_cast<float>(mesh.vertices[i].normal.x),
             static_cast<float>(mesh.vertices[i].normal.y),
             static_cast<float>(mesh.vertices[i].normal.z)},
            {mesh.vertices[i].u, mesh.vertices[i].v}
        };
    }

    gpuMesh.vertexBuffer = [impl->device newBufferWithBytes:gpuVertices.data()
                                                     length:gpuVertices.size() * sizeof(GPUVertex)
                                                    options:MTLResourceStorageModeShared];

    gpuMesh.indexBuffer = [impl->device newBufferWithBytes:mesh.indices.data()
                                                    length:mesh.indices.size() * sizeof(uint32_t)
                                                   options:MTLResourceStorageModeShared];

    gpuMesh.indexCount = static_cast<uint32_t>(mesh.indices.size());

    return impl->meshes.insert(gpuMesh);
}

void MetalRenderer::removeMesh(MeshHandle handle) {
    impl->meshes.erase(handle);
}

void MetalRenderer::beginFrame() {
    impl->opaqueDrawCalls.clear();
    impl->transparentDrawCalls.clear();

    // Acquire the drawable and build the pass descriptor up front so the debug
    // UI's new-frame (which needs the descriptor's formats) can run before
    // systems emit ImGui widgets in their render() hooks. endFrame() reuses it.
    impl->currentDrawable = [impl->metalLayer nextDrawable];
    impl->currentPassDesc = nil;
    if (impl->currentDrawable) {
        MTLRenderPassDescriptor* passDesc = [MTLRenderPassDescriptor renderPassDescriptor];
        passDesc.colorAttachments[0].texture = impl->currentDrawable.texture;
        passDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
        passDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
        passDesc.colorAttachments[0].clearColor = MTLClearColorMake(0.1, 0.1, 0.12, 1.0);
        passDesc.depthAttachment.texture = impl->depthTexture;
        passDesc.depthAttachment.loadAction = MTLLoadActionClear;
        passDesc.depthAttachment.storeAction = MTLStoreActionDontCare;
        passDesc.depthAttachment.clearDepth = 1.0;
        impl->currentPassDesc = passDesc;
    }

#ifdef RT_ENABLE_IMGUI
    if (impl->imguiInitialized && impl->currentPassDesc) {
        ImGui_ImplMetal_NewFrame(impl->currentPassDesc);
        // ImGui_ImplGlfw_NewFrame() is run by Window in pollEvents.
        ImGui::NewFrame();
    }
#endif
}

void MetalRenderer::setCamera(const CameraState& camera) {
    float fovRad = static_cast<float>(degreesToRadians(camera.fovDegrees));

    // View and projection are both built engine-side (Mat4) and transposed into
    // Metal's column-major layout here; no matrix math lives in the backend.
    simd_float4x4 view = toSimd(Mat4::lookAt(camera.position, camera.target, camera.up));
    Mat4 projMat = (camera.projection == CameraProjection::Orthographic)
        ? Mat4::orthographic(camera.orthoHeight, camera.aspectRatio,
                             camera.nearPlane, camera.farPlane)
        : Mat4::perspective(fovRad, camera.aspectRatio,
                            camera.nearPlane, camera.farPlane);
    simd_float4x4 proj = toSimd(projMat);

    impl->cameraUniforms.viewProjection = simd_mul(proj, view);
    impl->cameraUniforms.view = view;
    impl->cameraUniforms.cameraPosition = {static_cast<float>(camera.position.x),
                                           static_cast<float>(camera.position.y),
                                           static_cast<float>(camera.position.z)};
    impl->currentCameraPos = camera.position;
}

void MetalRenderer::setLights(const std::vector<PointLight>& lights, float exposure) {
    impl->lightUniforms.lightCount = std::min(static_cast<int>(lights.size()), 8);
    impl->lightUniforms.exposure = exposure;
    for (int i = 0; i < impl->lightUniforms.lightCount; i++) {
        impl->lightUniforms.lights[i].position = {
            static_cast<float>(lights[i].position.x),
            static_cast<float>(lights[i].position.y),
            static_cast<float>(lights[i].position.z)
        };
        impl->lightUniforms.lights[i].color = {
            static_cast<float>(lights[i].color.x),
            static_cast<float>(lights[i].color.y),
            static_cast<float>(lights[i].color.z)
        };
        impl->lightUniforms.lights[i].intensity = lights[i].intensity;
    }
}

void MetalRenderer::drawMesh(MeshHandle handle, const Mat4& transform,
                              const RenderMaterial& material) {
    Vec3 meshCenter = transform.transformPoint(Vec3(0, 0, 0));
    Vec3 diff = meshCenter - impl->currentCameraPos;
    float dist = static_cast<float>(diff.lengthSquared());

    Impl::DrawCall dc;
    dc.meshHandle = handle;
    dc.transform = transform;
    dc.material = material;
    dc.distanceToCamera = dist;

    if (material.opacity < 1.0f) {
        impl->transparentDrawCalls.push_back(dc);
    } else {
        impl->opaqueDrawCalls.push_back(dc);
    }
}

void MetalRenderer::endFrame() {
    // Drawable + pass descriptor were acquired in beginFrame(). If there was no
    // drawable, beginFrame() also skipped ImGui::NewFrame(), so the NewFrame/
    // Render pairing stays balanced by doing nothing here.
    if (!impl->currentDrawable || !impl->currentPassDesc) return;

    impl->currentCommandBuffer = [impl->commandQueue commandBuffer];
    impl->currentEncoder = [impl->currentCommandBuffer
        renderCommandEncoderWithDescriptor:impl->currentPassDesc];

    [impl->currentEncoder setFrontFacingWinding:MTLWindingClockwise];
    [impl->currentEncoder setCullMode:MTLCullModeBack];

    auto issueDraw = [&](const Impl::DrawCall& dc) {
        const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
        if (!mesh) return;   // stale/removed handle — skip

        ModelUniforms modelUniforms;
        modelUniforms.model = toSimd(dc.transform);
        modelUniforms.normalMatrix = inverseTranspose(modelUniforms.model);

        MaterialUniforms matUniforms;
        matUniforms.albedo = {static_cast<float>(dc.material.albedo.x),
                              static_cast<float>(dc.material.albedo.y),
                              static_cast<float>(dc.material.albedo.z)};
        matUniforms.metallic = dc.material.metallic;
        matUniforms.roughness = dc.material.roughness;
        matUniforms.opacity = dc.material.opacity;
        matUniforms.emission = {static_cast<float>(dc.material.emission.x),
                                static_cast<float>(dc.material.emission.y),
                                static_cast<float>(dc.material.emission.z)};

        [impl->currentEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
        [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                      length:sizeof(CameraUniforms) atIndex:1];
        [impl->currentEncoder setVertexBytes:&modelUniforms
                                      length:sizeof(ModelUniforms) atIndex:2];
        [impl->currentEncoder setFragmentBytes:&impl->cameraUniforms
                                        length:sizeof(CameraUniforms) atIndex:1];
        [impl->currentEncoder setFragmentBytes:&matUniforms
                                        length:sizeof(MaterialUniforms) atIndex:3];
        [impl->currentEncoder setFragmentBytes:&impl->lightUniforms
                                        length:sizeof(LightUniforms) atIndex:4];

        [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                         indexCount:mesh->indexCount
                                          indexType:MTLIndexTypeUInt32
                                        indexBuffer:mesh->indexBuffer
                                  indexBufferOffset:0];
    };

    // Opaque pass: front-to-back, depth write on
    std::sort(impl->opaqueDrawCalls.begin(), impl->opaqueDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera < b.distanceToCamera;
              });

    [impl->currentEncoder setRenderPipelineState:impl->opaquePipeline];
    [impl->currentEncoder setDepthStencilState:impl->depthStateOpaque];
    for (const auto& dc : impl->opaqueDrawCalls) {
        issueDraw(dc);
    }

    // Transparent pass: back-to-front, depth write off
    std::sort(impl->transparentDrawCalls.begin(), impl->transparentDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera > b.distanceToCamera;
              });

    [impl->currentEncoder setRenderPipelineState:impl->transparentPipeline];
    [impl->currentEncoder setDepthStencilState:impl->depthStateTransparent];
    for (const auto& dc : impl->transparentDrawCalls) {
        issueDraw(dc);
    }

    // Debug UI (ADR-0011) records last, over the scene, into the same encoder.
#ifdef RT_ENABLE_IMGUI
    if (impl->imguiInitialized) {
        ImGui::Render();
        ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(),
                                       impl->currentCommandBuffer,
                                       impl->currentEncoder);
    }
#endif

    [impl->currentEncoder endEncoding];
    [impl->currentCommandBuffer presentDrawable:impl->currentDrawable];
    [impl->currentCommandBuffer commit];
}

void MetalRenderer::initDebugUi(void* /*windowHandle*/) {
#ifdef RT_ENABLE_IMGUI
    // Create the ImGui context and the Metal backend. Runs before
    // Window::initDebugUi, which attaches the GLFW backend to this context.
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::GetIO().IniFilename = nullptr;   // don't write imgui.ini
    ImGui::StyleColorsDark();
    ImGui_ImplMetal_Init(impl->device);
    impl->imguiInitialized = true;
#endif
}

void MetalRenderer::shutdownDebugUi() {
#ifdef RT_ENABLE_IMGUI
    if (!impl->imguiInitialized) return;
    ImGui_ImplMetal_Shutdown();
    ImGui::DestroyContext();   // Window::shutdownDebugUi (GLFW) ran first
    impl->imguiInitialized = false;
#endif
}

std::unique_ptr<Renderer> Renderer::create() {
    return std::make_unique<MetalRenderer>();
}

#endif // __APPLE__
