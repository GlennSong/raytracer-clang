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

namespace engine {

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

enum GPULightType : int32_t {
    LightType_Point       = 0,
    LightType_Directional = 1,
    LightType_Spot        = 2,
};

struct alignas(16) GPULight {
    simd_float3   position;       // point/spot
    float         intensity;
    simd_float3   direction;      // directional/spot
    float         innerCosAngle;  // spot (cos of inner cone)
    simd_float3   color;
    float         outerCosAngle;  // spot (cos of outer cone)
    simd_float4x4 lightViewProjection; // shadow matrix
    int32_t       type;           // GPULightType
    int32_t       shadowMapIndex; // -1 = no shadow
    float         _pad[2];
};

struct LightUniforms {
    GPULight lights[32];
    int32_t  lightCount;
    float    exposure;
    float    _pad[2];
};

struct GPUMesh {
    id<MTLBuffer> vertexBuffer;
    id<MTLBuffer> indexBuffer;
    uint32_t indexCount;
    int materialIndex;
    BoundingSphere bounds;
};

struct alignas(16) GPUInstanceData {
    simd_float4x4 model;
    simd_float4x4 normalMatrix;
    simd_float4 albedo;     // w unused
    float metallic;
    float roughness;
    float opacity;
    float _pad0;
    simd_float4 emission;   // w unused
};

static constexpr uint32_t MAX_INSTANCES = 4096;

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
    id<MTLRenderPipelineState> opaqueInstancedPipeline;
    id<MTLRenderPipelineState> transparentInstancedPipeline;
    id<MTLDepthStencilState> depthStateOpaque;
    id<MTLDepthStencilState> depthStateTransparent;
    id<MTLBuffer> instanceBuffer;
    CAMetalLayer* metalLayer;
    NSWindow* nsWindow;
    id<MTLTexture> depthTexture;

    // Generation-checked GPU mesh storage (ADR-0007): hands out MeshHandles and
    // detects use of a freed/reused handle, replacing the old uint32 counter.
    SlotMap<GPUMesh, MeshTag> meshes;

    CameraUniforms cameraUniforms;
    LightUniforms lightUniforms;
    id<MTLBuffer> lightBuffer;

    // Shadow mapping
    id<MTLTexture> shadowMap;
    id<MTLRenderPipelineState> shadowPipeline;
    id<MTLRenderPipelineState> shadowInstancedPipeline;
    id<MTLDepthStencilState> shadowDepthState;
    id<MTLSamplerState> shadowSampler;
    int shadowMapSize = 2048;
    bool shadowEnabled = false;
    CameraUniforms shadowCameraUniforms;  // light VP for shadow pass

    struct ShadowUniforms {
        float shadowBias;
        float normalBias;
        float pcfRadius;
        int32_t shadowMapSize;
    };
    ShadowUniforms shadowUniforms;

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
    RenderStats lastStats;
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
    id<MTLFunction> vertexInstancedFunc = [library newFunctionWithName:@"vertexMainInstanced"];
    id<MTLFunction> fragmentInstancedFunc = [library newFunctionWithName:@"fragmentMainInstanced"];

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

    // Opaque instanced pipeline
    pipelineDesc.vertexFunction = vertexInstancedFunc;
    pipelineDesc.fragmentFunction = fragmentInstancedFunc;
    impl->opaqueInstancedPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                                 error:&error];
    if (!impl->opaqueInstancedPipeline) {
        NSLog(@"Instanced pipeline error: %@", error);
        return false;
    }

    // Transparent pipeline (alpha blending)
    pipelineDesc.vertexFunction = vertexFunc;
    pipelineDesc.fragmentFunction = fragmentFunc;
    pipelineDesc.colorAttachments[0].blendingEnabled = YES;
    pipelineDesc.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    pipelineDesc.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    pipelineDesc.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
    pipelineDesc.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;

    impl->transparentPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                             error:&error];

    // Transparent instanced pipeline
    pipelineDesc.vertexFunction = vertexInstancedFunc;
    pipelineDesc.fragmentFunction = fragmentInstancedFunc;
    impl->transparentInstancedPipeline = [impl->device newRenderPipelineStateWithDescriptor:pipelineDesc
                                                                                      error:&error];

    // Instance data buffer
    impl->instanceBuffer = [impl->device newBufferWithLength:MAX_INSTANCES * sizeof(GPUInstanceData)
                                                     options:MTLResourceStorageModeShared];

    // Light uniform buffer (exceeds setBytes 4KB limit with 32 lights)
    impl->lightBuffer = [impl->device newBufferWithLength:sizeof(LightUniforms)
                                                  options:MTLResourceStorageModeShared];

    // Shadow map texture
    {
        MTLTextureDescriptor* shadowDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                         width:impl->shadowMapSize
                                        height:impl->shadowMapSize
                                     mipmapped:NO];
        shadowDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        shadowDesc.storageMode = MTLStorageModePrivate;
        impl->shadowMap = [impl->device newTextureWithDescriptor:shadowDesc];
    }

    // Shadow depth-only pipelines
    {
        id<MTLFunction> shadowVertFunc = [library newFunctionWithName:@"vertexShadow"];
        id<MTLFunction> shadowVertInstFunc = [library newFunctionWithName:@"vertexShadowInstanced"];

        MTLRenderPipelineDescriptor* shadowPipeDesc = [[MTLRenderPipelineDescriptor alloc] init];
        shadowPipeDesc.vertexFunction = shadowVertFunc;
        shadowPipeDesc.fragmentFunction = nil;
        shadowPipeDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
        // No color attachment for shadow pass

        impl->shadowPipeline = [impl->device newRenderPipelineStateWithDescriptor:shadowPipeDesc
                                                                            error:&error];
        if (!impl->shadowPipeline) {
            NSLog(@"Shadow pipeline error: %@", error);
        }

        shadowPipeDesc.vertexFunction = shadowVertInstFunc;
        impl->shadowInstancedPipeline = [impl->device newRenderPipelineStateWithDescriptor:shadowPipeDesc
                                                                                     error:&error];
        if (!impl->shadowInstancedPipeline) {
            NSLog(@"Shadow instanced pipeline error: %@", error);
        }
    }

    // Shadow depth state
    {
        MTLDepthStencilDescriptor* shadowDepthDesc = [[MTLDepthStencilDescriptor alloc] init];
        shadowDepthDesc.depthCompareFunction = MTLCompareFunctionLess;
        shadowDepthDesc.depthWriteEnabled = YES;
        impl->shadowDepthState = [impl->device newDepthStencilStateWithDescriptor:shadowDepthDesc];
    }

    // Shadow comparison sampler
    {
        MTLSamplerDescriptor* samplerDesc = [[MTLSamplerDescriptor alloc] init];
        samplerDesc.compareFunction = MTLCompareFunctionLessEqual;
        samplerDesc.minFilter = MTLSamplerMinMagFilterLinear;
        samplerDesc.magFilter = MTLSamplerMinMagFilterLinear;
        samplerDesc.mipFilter = MTLSamplerMipFilterNotMipmapped;
        samplerDesc.sAddressMode = MTLSamplerAddressModeClampToEdge;
        samplerDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->shadowSampler = [impl->device newSamplerStateWithDescriptor:samplerDesc];
    }

    impl->shadowUniforms = {0.005f, 0.02f, 1.0f, impl->shadowMapSize};

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
    gpuMesh.bounds = computeBoundingSphere(mesh.vertices.data(), mesh.vertices.size());

    return impl->meshes.insert(gpuMesh);
}

void MetalRenderer::removeMesh(MeshHandle handle) {
    impl->meshes.erase(handle);
}

BoundingSphere MetalRenderer::getMeshBounds(MeshHandle handle) const {
    const GPUMesh* mesh = impl->meshes.get(handle);
    if (!mesh) return {};
    return mesh->bounds;
}

RenderStats MetalRenderer::getRenderStats() const {
    return impl->lastStats;
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

static simd_float3 toSimd3(const Vec3& v) {
    return {static_cast<float>(v.x), static_cast<float>(v.y),
            static_cast<float>(v.z)};
}

void MetalRenderer::setLights(const SceneLighting& lighting) {
    auto& lu = impl->lightUniforms;
    int idx = 0;
    constexpr int MAX_LIGHTS = 32;

    // Directional light (sun)
    impl->shadowEnabled = false;
    if (idx < MAX_LIGHTS) {
        auto& g = lu.lights[idx];
        g.position = {};
        g.intensity = lighting.sun.intensity;
        Vec3 sunDir = normalize(lighting.sun.direction);
        g.direction = toSimd3(sunDir);
        g.innerCosAngle = 0;
        g.color = toSimd3(lighting.sun.color);
        g.outerCosAngle = 0;

        if (lighting.sun.castsShadow && lighting.shadow.enabled) {
            // Compute light VP for shadow mapping
            Real sceneBound = 30.0;
            Vec3 sceneCenter(0, 0, 0);
            Vec3 lightPos = sceneCenter + sunDir * sceneBound;
            // Avoid degenerate up vector when light is nearly vertical
            Vec3 up = (std::abs(sunDir.y) > 0.99) ? Vec3(0, 0, 1) : Vec3(0, 1, 0);
            Mat4 lightView = Mat4::lookAt(lightPos, sceneCenter, up);
            Mat4 lightProj = Mat4::orthographic(sceneBound * 2.0, 1.0, 0.1, sceneBound * 2.0);
            Mat4 lightVP = lightProj * lightView;
            g.lightViewProjection = toSimd(lightVP);

            // Store for shadow pass encoding
            impl->shadowCameraUniforms.viewProjection = g.lightViewProjection;
            impl->shadowCameraUniforms.view = toSimd(lightView);
            impl->shadowCameraUniforms.cameraPosition = toSimd3(lightPos);
            impl->shadowEnabled = true;

            g.shadowMapIndex = 0;
            impl->shadowUniforms.shadowBias = lighting.shadow.bias;
            impl->shadowUniforms.normalBias = lighting.shadow.normalBias;
            impl->shadowUniforms.shadowMapSize = impl->shadowMapSize;
        } else {
            g.lightViewProjection = simd_matrix_from_rows(
                simd_make_float4(1,0,0,0), simd_make_float4(0,1,0,0),
                simd_make_float4(0,0,1,0), simd_make_float4(0,0,0,1));
            g.shadowMapIndex = -1;
        }

        g.type = LightType_Directional;
        g._pad[0] = g._pad[1] = 0;
        idx++;
    }

    // Point lights
    for (size_t i = 0; i < lighting.pointLights.size() && idx < MAX_LIGHTS; i++, idx++) {
        auto& g = lu.lights[idx];
        const auto& pl = lighting.pointLights[i];
        g.position = toSimd3(pl.position);
        g.intensity = pl.intensity;
        g.direction = {};
        g.innerCosAngle = 0;
        g.color = toSimd3(pl.color);
        g.outerCosAngle = 0;
        g.lightViewProjection = simd_matrix_from_rows(
            simd_make_float4(1,0,0,0), simd_make_float4(0,1,0,0),
            simd_make_float4(0,0,1,0), simd_make_float4(0,0,0,1));
        g.type = LightType_Point;
        g.shadowMapIndex = -1;
        g._pad[0] = g._pad[1] = 0;
    }

    // Spot lights
    for (size_t i = 0; i < lighting.spotLights.size() && idx < MAX_LIGHTS; i++, idx++) {
        auto& g = lu.lights[idx];
        const auto& sl = lighting.spotLights[i];
        g.position = toSimd3(sl.position);
        g.intensity = sl.intensity;
        g.direction = toSimd3(normalize(sl.direction));
        g.innerCosAngle = std::cos(sl.innerConeAngle);
        g.color = toSimd3(sl.color);
        g.outerCosAngle = std::cos(sl.outerConeAngle);
        g.lightViewProjection = simd_matrix_from_rows(
            simd_make_float4(1,0,0,0), simd_make_float4(0,1,0,0),
            simd_make_float4(0,0,1,0), simd_make_float4(0,0,0,1));
        g.type = LightType_Spot;
        g.shadowMapIndex = -1;
        g._pad[0] = g._pad[1] = 0;
    }

    lu.lightCount = idx;
    lu.exposure = lighting.exposure;
    lu._pad[0] = lu._pad[1] = 0;

    memcpy([impl->lightBuffer contents], &lu, sizeof(LightUniforms));
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
    if (!impl->currentDrawable || !impl->currentPassDesc) return;

    impl->currentCommandBuffer = [impl->commandQueue commandBuffer];

    // --- Shadow pass ---
    if (impl->shadowEnabled && impl->shadowPipeline) {
        MTLRenderPassDescriptor* shadowPassDesc = [MTLRenderPassDescriptor renderPassDescriptor];
        shadowPassDesc.depthAttachment.texture = impl->shadowMap;
        shadowPassDesc.depthAttachment.loadAction = MTLLoadActionClear;
        shadowPassDesc.depthAttachment.storeAction = MTLStoreActionStore;
        shadowPassDesc.depthAttachment.clearDepth = 1.0;

        id<MTLRenderCommandEncoder> shadowEncoder = [impl->currentCommandBuffer
            renderCommandEncoderWithDescriptor:shadowPassDesc];
        [shadowEncoder setFrontFacingWinding:MTLWindingClockwise];
        [shadowEncoder setCullMode:MTLCullModeBack];
        [shadowEncoder setDepthStencilState:impl->shadowDepthState];
        [shadowEncoder setDepthBias:0.005 slopeScale:1.5 clamp:0.01];

        // Render all opaque draw calls from light's perspective
        for (auto& dc : impl->opaqueDrawCalls) {
            const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
            if (!mesh) continue;

            ModelUniforms modelU;
            modelU.model = toSimd(dc.transform);
            modelU.normalMatrix = inverseTranspose(modelU.model);

            [shadowEncoder setRenderPipelineState:impl->shadowPipeline];
            [shadowEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
            [shadowEncoder setVertexBytes:&impl->shadowCameraUniforms
                                   length:sizeof(CameraUniforms) atIndex:1];
            [shadowEncoder setVertexBytes:&modelU
                                   length:sizeof(ModelUniforms) atIndex:2];
            [shadowEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                      indexCount:mesh->indexCount
                                       indexType:MTLIndexTypeUInt32
                                     indexBuffer:mesh->indexBuffer
                               indexBufferOffset:0];
        }

        [shadowEncoder endEncoding];
    }

    // --- Main color pass ---
    impl->currentEncoder = [impl->currentCommandBuffer
        renderCommandEncoderWithDescriptor:impl->currentPassDesc];

    [impl->currentEncoder setFrontFacingWinding:MTLWindingClockwise];
    [impl->currentEncoder setCullMode:MTLCullModeBack];

    // Bind shadow resources for the entire main pass
    if (impl->shadowEnabled) {
        [impl->currentEncoder setFragmentTexture:impl->shadowMap atIndex:0];
        [impl->currentEncoder setFragmentSamplerState:impl->shadowSampler atIndex:0];
        [impl->currentEncoder setFragmentBytes:&impl->shadowUniforms
                                        length:sizeof(Impl::ShadowUniforms) atIndex:5];
    } else {
        // Still need to bind something so the shader doesn't crash
        [impl->currentEncoder setFragmentTexture:impl->shadowMap atIndex:0];
        [impl->currentEncoder setFragmentSamplerState:impl->shadowSampler atIndex:0];
        Impl::ShadowUniforms noShadow = {0, 0, 0, impl->shadowMapSize};
        [impl->currentEncoder setFragmentBytes:&noShadow
                                        length:sizeof(Impl::ShadowUniforms) atIndex:5];
    }

    RenderStats stats;

    auto fillInstanceData = [&](const Impl::DrawCall& dc) -> GPUInstanceData {
        GPUInstanceData inst;
        inst.model = toSimd(dc.transform);
        inst.normalMatrix = inverseTranspose(inst.model);
        inst.albedo = {static_cast<float>(dc.material.albedo.x),
                       static_cast<float>(dc.material.albedo.y),
                       static_cast<float>(dc.material.albedo.z), 0};
        inst.metallic = dc.material.metallic;
        inst.roughness = dc.material.roughness;
        inst.opacity = dc.material.opacity;
        inst._pad0 = 0;
        inst.emission = {static_cast<float>(dc.material.emission.x),
                         static_cast<float>(dc.material.emission.y),
                         static_cast<float>(dc.material.emission.z), 0};
        return inst;
    };

    auto issueSingleDraw = [&](const Impl::DrawCall& dc) {
        const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
        if (!mesh) return;

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
        [impl->currentEncoder setFragmentBuffer:impl->lightBuffer
                                        offset:0 atIndex:4];

        [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                         indexCount:mesh->indexCount
                                          indexType:MTLIndexTypeUInt32
                                        indexBuffer:mesh->indexBuffer
                                  indexBufferOffset:0];
        stats.drawCalls++;
        stats.totalInstances++;
    };

    auto issuePass = [&](std::vector<Impl::DrawCall>& drawCalls,
                         id<MTLRenderPipelineState> singlePipeline,
                         id<MTLRenderPipelineState> instancedPipeline,
                         id<MTLDepthStencilState> depthState) {
        if (drawCalls.empty()) return;

        [impl->currentEncoder setDepthStencilState:depthState];

        // Stable sort by mesh handle to group identical meshes while preserving
        // depth order within each group.
        std::stable_sort(drawCalls.begin(), drawCalls.end(),
                         [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                             return a.meshHandle < b.meshHandle;
                         });

        GPUInstanceData* instanceBuf =
            static_cast<GPUInstanceData*>([impl->instanceBuffer contents]);
        uint32_t instanceOffset = 0;

        size_t i = 0;
        while (i < drawCalls.size()) {
            size_t batchStart = i;
            MeshHandle batchMesh = drawCalls[i].meshHandle;
            while (i < drawCalls.size() && drawCalls[i].meshHandle == batchMesh)
                i++;
            uint32_t batchSize = static_cast<uint32_t>(i - batchStart);

            const GPUMesh* mesh = impl->meshes.get(batchMesh);
            if (!mesh) continue;

            if (batchSize == 1) {
                [impl->currentEncoder setRenderPipelineState:singlePipeline];
                issueSingleDraw(drawCalls[batchStart]);
                continue;
            }

            if (instanceOffset + batchSize > MAX_INSTANCES) {
                [impl->currentEncoder setRenderPipelineState:singlePipeline];
                for (size_t j = batchStart; j < batchStart + batchSize; j++)
                    issueSingleDraw(drawCalls[j]);
                continue;
            }

            for (size_t j = batchStart; j < batchStart + batchSize; j++)
                instanceBuf[instanceOffset + (j - batchStart)] =
                    fillInstanceData(drawCalls[j]);

            [impl->currentEncoder setRenderPipelineState:instancedPipeline];
            [impl->currentEncoder setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
            [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                          length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setVertexBuffer:impl->instanceBuffer
                                           offset:instanceOffset * sizeof(GPUInstanceData)
                                          atIndex:2];
            [impl->currentEncoder setFragmentBytes:&impl->cameraUniforms
                                            length:sizeof(CameraUniforms) atIndex:1];
            [impl->currentEncoder setFragmentBytes:&impl->lightUniforms
                                            length:sizeof(LightUniforms) atIndex:4];

            [impl->currentEncoder drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                             indexCount:mesh->indexCount
                                              indexType:MTLIndexTypeUInt32
                                            indexBuffer:mesh->indexBuffer
                                      indexBufferOffset:0
                                          instanceCount:batchSize];
            stats.drawCalls++;
            stats.instancedDrawCalls++;
            stats.totalInstances += batchSize;
            instanceOffset += batchSize;
        }
    };

    stats.entitiesSubmitted = static_cast<uint32_t>(
        impl->opaqueDrawCalls.size() + impl->transparentDrawCalls.size());

    // Sort each pass by distance first, then issuePass stable-sorts by mesh.
    std::sort(impl->opaqueDrawCalls.begin(), impl->opaqueDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera < b.distanceToCamera;
              });

    issuePass(impl->opaqueDrawCalls, impl->opaquePipeline,
              impl->opaqueInstancedPipeline, impl->depthStateOpaque);

    std::sort(impl->transparentDrawCalls.begin(), impl->transparentDrawCalls.end(),
              [](const Impl::DrawCall& a, const Impl::DrawCall& b) {
                  return a.distanceToCamera > b.distanceToCamera;
              });

    issuePass(impl->transparentDrawCalls, impl->transparentPipeline,
              impl->transparentInstancedPipeline, impl->depthStateTransparent);

    impl->lastStats = stats;

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

}  // namespace engine

#endif // __APPLE__
