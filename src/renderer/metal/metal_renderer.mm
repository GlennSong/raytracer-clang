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
    float _camPad0;
    simd_float4x4 invViewProjection;
    simd_float4x4 projection;
    simd_float4x4 invProjection;
    simd_float2 screenSize;
    float nearPlane;
    float farPlane;
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
    float flags;
    uint32_t textureFlags;
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
    float    ambientMultiplier;
    float    _pad[1];
    // Procedural sky (ADR-0016, day/night). Mirrors `LightUniforms` in
    // phong.metal — each simd_float3 packs with its trailing scalar into 16
    // bytes, matching MSL's float3 layout (as GPULight above already does).
    simd_float3 skySunDir;   float skySunIntensity;
    simd_float3 skySunColor; float _skp0;
    simd_float3 skyZenith;   float _skp1;
    simd_float3 skyHorizon;  float _skp2;
    simd_float3 skyGround;   float _skp3;
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
    float flags;
    simd_float4 emission;   // w unused
    uint32_t textureFlags;
    float _instPad[3];
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

    SlotMap<GPUMesh, MeshTag> meshes;
    SlotMap<id<MTLTexture>, TextureTag> textures;
    id<MTLSamplerState> linearWrapSampler;
    id<MTLTexture> defaultWhiteTexture;

    CameraUniforms cameraUniforms;
    LightUniforms lightUniforms;
    id<MTLBuffer> lightBuffer;

    // Skybox
    id<MTLRenderPipelineState> skyboxPipeline;
    id<MTLDepthStencilState> skyboxDepthState;

    // Environment (ADR-0016): an equirectangular HDR map drives the skybox and,
    // via the probe bake, IBL. nil => procedural sky. Mirrors `EnvUniforms` in
    // the shader (mode 0 = procedural, 1 = HDR equirect).
    struct EnvUniforms {
        int32_t mode;
        float   _pad[3];
    };
    id<MTLTexture> environmentTexture;     // equirect RGBA16Float, nil if procedural
    id<MTLSamplerState> equirectSampler;   // linear, wrap-U / clamp-V

    // Post-processing: offscreen HDR target + G-buffer normals + composite pass
    id<MTLTexture> sceneColorTexture;
    id<MTLTexture> viewNormalTexture;
    id<MTLRenderPipelineState> compositePipeline;
    id<MTLSamplerState> linearClampSampler;

    // Screen-space reflections (SSR)
    id<MTLTexture> ssrTexture;         // RGBA16Float — SSR result (rgb=color, a=confidence)
    id<MTLTexture> ssrBlurTemp;        // RGBA16Float — ping-pong for bilateral blur
    id<MTLComputePipelineState> ssrPipeline;
    id<MTLComputePipelineState> ssrBlurHPipeline;
    id<MTLComputePipelineState> ssrBlurVPipeline;

    // Bloom
    static constexpr int BLOOM_MIP_COUNT = 5;
    id<MTLTexture> bloomMips[BLOOM_MIP_COUNT];     // RGBA16Float downsample chain
    id<MTLTexture> bloomUpsampleMips[BLOOM_MIP_COUNT]; // upsample chain (reuses mip 0 slot for final)
    id<MTLComputePipelineState> bloomDownsamplePipeline;
    id<MTLComputePipelineState> bloomUpsamplePipeline;

    // Screen-space ambient occlusion (SSAO/GTAO)
    id<MTLTexture> aoTexture;          // R16Float — full-res AO result
    id<MTLTexture> aoBlurTemp;         // R16Float — full-res blur ping-pong
    id<MTLComputePipelineState> aoPipeline;
    id<MTLComputePipelineState> aoBlurHPipeline;
    id<MTLComputePipelineState> aoBlurVPipeline;

    // Reflection probes (cubemap-based IBL)
    struct alignas(16) GPUReflectionProbe {
        simd_float3 position;
        float influenceRadius;
        simd_float3 boxMin;
        float _pad0;
        simd_float3 boxMax;
        int32_t probeIndex;
    };
    struct ProbeUniforms {
        int32_t probeCount;
        int32_t maxMipLevel;
        float _pad[2];
    };
    id<MTLTexture> probeCubemapArray;      // texturecube_array, RGBA16Float
    id<MTLTexture> brdfLUT;                // 256×256 RG16Float
    id<MTLBuffer> probeBuffer;             // GPUReflectionProbe[]
    ProbeUniforms probeUniforms = {};
    id<MTLComputePipelineState> brdfLUTPipeline;
    id<MTLComputePipelineState> prefilterPipeline;
    static constexpr int PROBE_CUBEMAP_SIZE = 256;
    static constexpr int PROBE_MIP_LEVELS = 6;  // mip 0..5
    static constexpr int MAX_PROBES = 8;
    int probeCount = 0;
    bool probesBaked = false;
    bool probesPendingBake = false;
    std::vector<ReflectionProbe> pendingProbes;

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
    MTLRenderPassDescriptor* currentPassDesc;   // scene pass (HDR offscreen)
    MTLRenderPassDescriptor* compositePassDesc; // composite pass (LDR drawable)
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

    static void bakeProbes(Impl* impl, const std::vector<ReflectionProbe>& probes);
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

    // Opaque pipeline (MRT: color + view-space normals)
    MTLRenderPipelineDescriptor* pipelineDesc = [[MTLRenderPipelineDescriptor alloc] init];
    pipelineDesc.vertexFunction = vertexFunc;
    pipelineDesc.fragmentFunction = fragmentFunc;
    pipelineDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;
    pipelineDesc.colorAttachments[1].pixelFormat = MTLPixelFormatRGBA8Unorm;
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

    // Skybox pipeline
    {
        id<MTLFunction> skyVert = [library newFunctionWithName:@"vertexSkybox"];
        id<MTLFunction> skyFrag = [library newFunctionWithName:@"fragmentSkybox"];
        MTLRenderPipelineDescriptor* skyDesc = [[MTLRenderPipelineDescriptor alloc] init];
        skyDesc.vertexFunction = skyVert;
        skyDesc.fragmentFunction = skyFrag;
        skyDesc.colorAttachments[0].pixelFormat = MTLPixelFormatRGBA16Float;
        skyDesc.colorAttachments[1].pixelFormat = MTLPixelFormatRGBA8Unorm;
        skyDesc.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
        impl->skyboxPipeline = [impl->device newRenderPipelineStateWithDescriptor:skyDesc
                                                                            error:&error];
        if (!impl->skyboxPipeline) NSLog(@"Skybox pipeline error: %@", error);

        MTLDepthStencilDescriptor* skyDepthDesc = [[MTLDepthStencilDescriptor alloc] init];
        skyDepthDesc.depthCompareFunction = MTLCompareFunctionAlways;
        skyDepthDesc.depthWriteEnabled = NO;
        impl->skyboxDepthState = [impl->device newDepthStencilStateWithDescriptor:skyDepthDesc];
    }

    // Composite pipeline (fullscreen triangle, reads HDR scene texture, writes to drawable)
    {
        id<MTLFunction> compVert = [library newFunctionWithName:@"vertexComposite"];
        id<MTLFunction> compFrag = [library newFunctionWithName:@"fragmentComposite"];
        MTLRenderPipelineDescriptor* compDesc = [[MTLRenderPipelineDescriptor alloc] init];
        compDesc.vertexFunction = compVert;
        compDesc.fragmentFunction = compFrag;
        compDesc.colorAttachments[0].pixelFormat = MTLPixelFormatBGRA8Unorm;
        // No depth attachment for the composite pass
        impl->compositePipeline = [impl->device newRenderPipelineStateWithDescriptor:compDesc
                                                                               error:&error];
        if (!impl->compositePipeline) NSLog(@"Composite pipeline error: %@", error);
    }

    // Linear clamp sampler (for post-processing texture reads)
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeClampToEdge;
        sampDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->linearClampSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // Linear wrap sampler (for material textures)
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.mipFilter = MTLSamplerMipFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeRepeat;
        sampDesc.tAddressMode = MTLSamplerAddressModeRepeat;
        impl->linearWrapSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // Equirectangular environment sampler: wrap horizontally (longitude is
    // periodic), clamp vertically (poles), linear filtering (ADR-0016).
    {
        MTLSamplerDescriptor* sampDesc = [[MTLSamplerDescriptor alloc] init];
        sampDesc.minFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.magFilter = MTLSamplerMinMagFilterLinear;
        sampDesc.sAddressMode = MTLSamplerAddressModeRepeat;
        sampDesc.tAddressMode = MTLSamplerAddressModeClampToEdge;
        impl->equirectSampler = [impl->device newSamplerStateWithDescriptor:sampDesc];
    }

    // 1x1 white default texture (bound when no material texture is set)
    {
        MTLTextureDescriptor* desc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                         width:1 height:1 mipmapped:NO];
        desc.usage = MTLTextureUsageShaderRead;
        impl->defaultWhiteTexture = [impl->device newTextureWithDescriptor:desc];
        uint8_t white[] = {255, 255, 255, 255};
        [impl->defaultWhiteTexture replaceRegion:MTLRegionMake2D(0, 0, 1, 1)
                                     mipmapLevel:0
                                       withBytes:white
                                     bytesPerRow:4];
    }

    // Compute pipelines for IBL (reflection probes)
    {
        id<MTLFunction> brdfFunc = [library newFunctionWithName:@"integrateBRDF"];
        if (brdfFunc) {
            impl->brdfLUTPipeline = [impl->device newComputePipelineStateWithFunction:brdfFunc
                                                                                error:&error];
            if (!impl->brdfLUTPipeline) NSLog(@"BRDF LUT pipeline error: %@", error);
        }
        id<MTLFunction> prefilterFunc = [library newFunctionWithName:@"prefilterEnvMap"];
        if (prefilterFunc) {
            impl->prefilterPipeline = [impl->device newComputePipelineStateWithFunction:prefilterFunc
                                                                                  error:&error];
            if (!impl->prefilterPipeline) NSLog(@"Prefilter pipeline error: %@", error);
        }
    }

    // SSR compute pipelines
    {
        id<MTLFunction> ssrFunc = [library newFunctionWithName:@"ssrRayMarch"];
        if (ssrFunc) {
            impl->ssrPipeline = [impl->device newComputePipelineStateWithFunction:ssrFunc
                                                                            error:&error];
            if (!impl->ssrPipeline) NSLog(@"SSR pipeline error: %@", error);
        }
        id<MTLFunction> blurHFunc = [library newFunctionWithName:@"ssrBlurH"];
        if (blurHFunc) {
            impl->ssrBlurHPipeline = [impl->device newComputePipelineStateWithFunction:blurHFunc
                                                                                 error:&error];
            if (!impl->ssrBlurHPipeline) NSLog(@"SSR blur H pipeline error: %@", error);
        }
        id<MTLFunction> blurVFunc = [library newFunctionWithName:@"ssrBlurV"];
        if (blurVFunc) {
            impl->ssrBlurVPipeline = [impl->device newComputePipelineStateWithFunction:blurVFunc
                                                                                 error:&error];
            if (!impl->ssrBlurVPipeline) NSLog(@"SSR blur V pipeline error: %@", error);
        }
    }

    // SSAO compute pipelines
    {
        id<MTLFunction> aoFunc = [library newFunctionWithName:@"gtaoCompute"];
        if (aoFunc) {
            impl->aoPipeline = [impl->device newComputePipelineStateWithFunction:aoFunc
                                                                           error:&error];
            if (!impl->aoPipeline) NSLog(@"GTAO pipeline error: %@", error);
        }
        id<MTLFunction> aoBlurHFunc = [library newFunctionWithName:@"aoBlurH"];
        if (aoBlurHFunc) {
            impl->aoBlurHPipeline = [impl->device newComputePipelineStateWithFunction:aoBlurHFunc
                                                                                error:&error];
            if (!impl->aoBlurHPipeline) NSLog(@"AO blur H pipeline error: %@", error);
        }
        id<MTLFunction> aoBlurVFunc = [library newFunctionWithName:@"aoBlurV"];
        if (aoBlurVFunc) {
            impl->aoBlurVPipeline = [impl->device newComputePipelineStateWithFunction:aoBlurVFunc
                                                                                error:&error];
            if (!impl->aoBlurVPipeline) NSLog(@"AO blur V pipeline error: %@", error);
        }
    }

    // Bloom compute pipelines
    {
        id<MTLFunction> downFunc = [library newFunctionWithName:@"bloomDownsample"];
        if (downFunc) {
            impl->bloomDownsamplePipeline = [impl->device newComputePipelineStateWithFunction:downFunc
                                                                                         error:&error];
            if (!impl->bloomDownsamplePipeline) NSLog(@"Bloom downsample pipeline error: %@", error);
        }
        id<MTLFunction> upFunc = [library newFunctionWithName:@"bloomUpsample"];
        if (upFunc) {
            impl->bloomUpsamplePipeline = [impl->device newComputePipelineStateWithFunction:upFunc
                                                                                       error:&error];
            if (!impl->bloomUpsamplePipeline) NSLog(@"Bloom upsample pipeline error: %@", error);
        }
    }

    // Generate BRDF integration LUT (one-time, 256×256)
    {
        MTLTextureDescriptor* brdfDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRG16Float
                                         width:256
                                        height:256
                                     mipmapped:NO];
        brdfDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
        brdfDesc.storageMode = MTLStorageModePrivate;
        impl->brdfLUT = [impl->device newTextureWithDescriptor:brdfDesc];

        if (impl->brdfLUTPipeline) {
            id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];
            id<MTLComputeCommandEncoder> enc = [cmdBuf computeCommandEncoder];
            [enc setComputePipelineState:impl->brdfLUTPipeline];
            [enc setTexture:impl->brdfLUT atIndex:0];
            MTLSize grid = MTLSizeMake(256, 256, 1);
            MTLSize group = MTLSizeMake(16, 16, 1);
            [enc dispatchThreads:grid threadsPerThreadgroup:group];
            [enc endEncoding];
            [cmdBuf commit];
            [cmdBuf waitUntilCompleted];
        }
    }

    // Probe buffer (GPU-side probe metadata)
    impl->probeBuffer = [impl->device newBufferWithLength:Impl::MAX_PROBES * sizeof(Impl::GPUReflectionProbe)
                                                  options:MTLResourceStorageModeShared];

    // Dummy 1×1 cubemap array for when no probes are baked (shader requires valid binding)
    {
        MTLTextureDescriptor* dummyCubeDesc = [[MTLTextureDescriptor alloc] init];
        dummyCubeDesc.textureType = MTLTextureTypeCubeArray;
        dummyCubeDesc.pixelFormat = MTLPixelFormatRGBA16Float;
        dummyCubeDesc.width = 1;
        dummyCubeDesc.height = 1;
        dummyCubeDesc.arrayLength = 1;
        dummyCubeDesc.mipmapLevelCount = 1;
        dummyCubeDesc.usage = MTLTextureUsageShaderRead;
        dummyCubeDesc.storageMode = MTLStorageModePrivate;
        impl->probeCubemapArray = [impl->device newTextureWithDescriptor:dummyCubeDesc];
    }

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
    depthDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    depthDesc.storageMode = MTLStorageModePrivate;
    impl->depthTexture = [impl->device newTextureWithDescriptor:depthDesc];

    // Offscreen HDR scene color target (Phase 0C)
    MTLTextureDescriptor* sceneColorDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    sceneColorDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    sceneColorDesc.storageMode = MTLStorageModePrivate;
    impl->sceneColorTexture = [impl->device newTextureWithDescriptor:sceneColorDesc];

    // View-space normal G-buffer for SSR
    MTLTextureDescriptor* normalDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:width
                                    height:height
                                 mipmapped:NO];
    normalDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    normalDesc.storageMode = MTLStorageModePrivate;
    impl->viewNormalTexture = [impl->device newTextureWithDescriptor:normalDesc];

    // SSR textures (half resolution for performance)
    int halfW = std::max(width / 2, 1);
    int halfH = std::max(height / 2, 1);
    MTLTextureDescriptor* ssrDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:halfW
                                    height:halfH
                                 mipmapped:NO];
    ssrDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
    ssrDesc.storageMode = MTLStorageModePrivate;
    impl->ssrTexture = [impl->device newTextureWithDescriptor:ssrDesc];
    impl->ssrBlurTemp = [impl->device newTextureWithDescriptor:ssrDesc];

    // AO textures (full resolution — fewer samples per pixel instead of half-res)
    MTLTextureDescriptor* aoDesc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatR16Float
                                     width:width
                                    height:height
                                 mipmapped:NO];
    aoDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
    aoDesc.storageMode = MTLStorageModePrivate;
    impl->aoTexture = [impl->device newTextureWithDescriptor:aoDesc];
    impl->aoBlurTemp = [impl->device newTextureWithDescriptor:aoDesc];

    // Bloom mip chain (progressive half-res)
    {
        int mipW = halfW;
        int mipH = halfH;
        for (int m = 0; m < MetalRenderer::Impl::BLOOM_MIP_COUNT; m++) {
            MTLTextureDescriptor* bloomDesc = [MTLTextureDescriptor
                texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                             width:std::max(mipW, 1)
                                            height:std::max(mipH, 1)
                                         mipmapped:NO];
            bloomDesc.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
            bloomDesc.storageMode = MTLStorageModePrivate;
            impl->bloomMips[m] = [impl->device newTextureWithDescriptor:bloomDesc];
            impl->bloomUpsampleMips[m] = [impl->device newTextureWithDescriptor:bloomDesc];
            mipW = std::max(mipW / 2, 1);
            mipH = std::max(mipH / 2, 1);
        }
    }
}

MeshHandle MetalRenderer::uploadMesh(const RenderMesh& mesh) {
    GPUMesh gpuMesh;
    gpuMesh.materialIndex = mesh.materialIndex;

    struct GPUVertex {
        float position[3];
        float normal[3];
        float tangent[3];
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
            {static_cast<float>(mesh.vertices[i].tangent.x),
             static_cast<float>(mesh.vertices[i].tangent.y),
             static_cast<float>(mesh.vertices[i].tangent.z)},
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

TextureHandle MetalRenderer::uploadTexture(int width, int height, int channels,
                                            const uint8_t* data) {
    MTLTextureDescriptor* desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                     width:width height:height mipmapped:YES];
    desc.usage = MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModeShared;

    id<MTLTexture> texture = [impl->device newTextureWithDescriptor:desc];

    if (channels == 4) {
        [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
                   mipmapLevel:0
                     withBytes:data
                   bytesPerRow:width * 4];
    } else {
        std::vector<uint8_t> rgba(width * height * 4);
        for (int i = 0; i < width * height; i++) {
            rgba[i * 4 + 0] = (channels > 0) ? data[i * channels + 0] : 255;
            rgba[i * 4 + 1] = (channels > 1) ? data[i * channels + 1] : 255;
            rgba[i * 4 + 2] = (channels > 2) ? data[i * channels + 2] : 255;
            rgba[i * 4 + 3] = (channels > 3) ? data[i * channels + 3] : 255;
        }
        [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
                   mipmapLevel:0
                     withBytes:rgba.data()
                   bytesPerRow:width * 4];
    }

    // Generate mipmaps
    id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
    [blit generateMipmapsForTexture:texture];
    [blit endEncoding];
    [cmdBuf commit];
    [cmdBuf waitUntilCompleted];

    return impl->textures.insert(texture);
}

TextureHandle MetalRenderer::uploadTextureHDR(int width, int height, int channels,
                                              const float* data) {
    if (!data || width <= 0 || height <= 0) return TextureHandle{};

    MTLTextureDescriptor* desc = [MTLTextureDescriptor
        texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                     width:width height:height mipmapped:NO];
    desc.usage = MTLTextureUsageShaderRead;
    desc.storageMode = MTLStorageModeShared;
    id<MTLTexture> texture = [impl->device newTextureWithDescriptor:desc];

    // Expand to RGBA half-float; Metal has no 3-channel float texture format.
    const int n = width * height;
    std::vector<__fp16> rgba(static_cast<size_t>(n) * 4);
    for (int i = 0; i < n; i++) {
        const float* src = data + static_cast<size_t>(i) * channels;
        rgba[i * 4 + 0] = static_cast<__fp16>(channels > 0 ? src[0] : 0.0f);
        rgba[i * 4 + 1] = static_cast<__fp16>(channels > 1 ? src[1] : 0.0f);
        rgba[i * 4 + 2] = static_cast<__fp16>(channels > 2 ? src[2] : 0.0f);
        rgba[i * 4 + 3] = static_cast<__fp16>(channels > 3 ? src[3] : 1.0f);
    }
    [texture replaceRegion:MTLRegionMake2D(0, 0, width, height)
               mipmapLevel:0
                 withBytes:rgba.data()
               bytesPerRow:static_cast<NSUInteger>(width) * 4 * sizeof(__fp16)];

    return impl->textures.insert(texture);
}

void MetalRenderer::removeTexture(TextureHandle handle) {
    impl->textures.erase(handle);
}

void MetalRenderer::setEnvironmentMap(TextureHandle equirect) {
    auto* tex = impl->textures.get(equirect);
    impl->environmentTexture = tex ? *tex : nil;

    // If probes are already baked against the old environment, re-bake so IBL
    // tracks the new map (ADR-0016: the bake renders the skybox into the cubes).
    if (!impl->pendingProbes.empty()) {
        impl->probesBaked = false;
        impl->probesPendingBake = true;
    }
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
    impl->compositePassDesc = nil;
    if (impl->currentDrawable) {
        // Main scene pass renders to offscreen HDR texture (not the drawable).
        // Tone mapping + gamma happens in a separate composite pass.
        MTLRenderPassDescriptor* passDesc = [MTLRenderPassDescriptor renderPassDescriptor];
        passDesc.colorAttachments[0].texture = impl->sceneColorTexture;
        passDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
        passDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
        passDesc.colorAttachments[0].clearColor = MTLClearColorMake(0.0, 0.0, 0.0, 1.0);
        passDesc.colorAttachments[1].texture = impl->viewNormalTexture;
        passDesc.colorAttachments[1].loadAction = MTLLoadActionClear;
        passDesc.colorAttachments[1].storeAction = MTLStoreActionStore;
        passDesc.colorAttachments[1].clearColor = MTLClearColorMake(0.5, 0.5, 1.0, 0.0);
        passDesc.depthAttachment.texture = impl->depthTexture;
        passDesc.depthAttachment.loadAction = MTLLoadActionClear;
        passDesc.depthAttachment.storeAction = MTLStoreActionStore;
        passDesc.depthAttachment.clearDepth = 1.0;
        impl->currentPassDesc = passDesc;

        // Composite pass renders to the drawable (BGRA8Unorm, no depth).
        MTLRenderPassDescriptor* compDesc = [MTLRenderPassDescriptor renderPassDescriptor];
        compDesc.colorAttachments[0].texture = impl->currentDrawable.texture;
        compDesc.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        compDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
        impl->compositePassDesc = compDesc;
    }

#ifdef RT_ENABLE_IMGUI
    if (impl->imguiInitialized && impl->compositePassDesc) {
        ImGui_ImplMetal_NewFrame(impl->compositePassDesc);
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

    simd_float4x4 vp = simd_mul(proj, view);
    impl->cameraUniforms.viewProjection = vp;
    impl->cameraUniforms.view = view;
    impl->cameraUniforms.cameraPosition = {static_cast<float>(camera.position.x),
                                           static_cast<float>(camera.position.y),
                                           static_cast<float>(camera.position.z)};
    impl->cameraUniforms._camPad0 = 0;

    // Inverse matrices for screen-space effects (SSR, SSAO)
    Mat4 viewMat = Mat4::lookAt(camera.position, camera.target, camera.up);
    Mat4 vpMat = projMat * viewMat;
    impl->cameraUniforms.invViewProjection = toSimd(vpMat.inverse());
    impl->cameraUniforms.projection = proj;
    impl->cameraUniforms.invProjection = toSimd(projMat.inverse());
    impl->cameraUniforms.screenSize = {static_cast<float>(impl->framebufferWidth),
                                       static_cast<float>(impl->framebufferHeight)};
    impl->cameraUniforms.nearPlane = static_cast<float>(camera.nearPlane);
    impl->cameraUniforms.farPlane = static_cast<float>(camera.farPlane);
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
    lu.ambientMultiplier = lighting.ambientMultiplier;
    lu._pad[0] = 0;

    // Procedural sky (ADR-0016): the analytic skybox and IBL fallback read these.
    const ProceduralSky& sky = lighting.sky;
    lu.skySunDir = toSimd3(normalize(sky.sunDirection));
    lu.skySunIntensity = sky.sunDiscIntensity;
    lu.skySunColor = toSimd3(sky.sunColor);
    lu.skyZenith = toSimd3(sky.zenithColor);
    lu.skyHorizon = toSimd3(sky.horizonColor);
    lu.skyGround = toSimd3(sky.groundColor);
    lu._skp0 = lu._skp1 = lu._skp2 = lu._skp3 = 0;

    memcpy([impl->lightBuffer contents], &lu, sizeof(LightUniforms));
}

void MetalRenderer::setReflectionProbes(const std::vector<ReflectionProbe>& probes) {
    if (probes.empty()) {
        impl->probeUniforms.probeCount = 0;
        impl->probeCount = 0;
        return;
    }

    // Store probes and mark for baking on the next endFrame() when draw calls exist
    impl->pendingProbes = probes;
    impl->probesPendingBake = true;
    impl->probesBaked = false;
}

// Internal: actually bake probes (called from endFrame when draw calls are available).
void MetalRenderer::Impl::bakeProbes(Impl* impl, const std::vector<ReflectionProbe>& probes) {
    int count = std::min(static_cast<int>(probes.size()), static_cast<int>(8));
    impl->probeCount = count;

    // Create cubemap array if needed
    if (!impl->probeCubemapArray) {
        int size = Impl::PROBE_CUBEMAP_SIZE;
        MTLTextureDescriptor* cubeDesc = [[MTLTextureDescriptor alloc] init];
        cubeDesc.textureType = MTLTextureTypeCubeArray;
        cubeDesc.pixelFormat = MTLPixelFormatRGBA16Float;
        cubeDesc.width = size;
        cubeDesc.height = size;
        cubeDesc.arrayLength = count;
        cubeDesc.mipmapLevelCount = Impl::PROBE_MIP_LEVELS;
        cubeDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
        cubeDesc.storageMode = MTLStorageModePrivate;
        impl->probeCubemapArray = [impl->device newTextureWithDescriptor:cubeDesc];
    }

    // Upload probe metadata
    auto* gpuProbes = static_cast<Impl::GPUReflectionProbe*>([impl->probeBuffer contents]);
    for (int i = 0; i < count; i++) {
        gpuProbes[i].position = toSimd3(probes[i].position);
        gpuProbes[i].influenceRadius = probes[i].influenceRadius;
        gpuProbes[i].boxMin = toSimd3(probes[i].boxMin);
        gpuProbes[i]._pad0 = 0;
        gpuProbes[i].boxMax = toSimd3(probes[i].boxMax);
        gpuProbes[i].probeIndex = i;
    }

    impl->probeUniforms.probeCount = count;
    impl->probeUniforms.maxMipLevel = Impl::PROBE_MIP_LEVELS - 1;
    impl->probeUniforms._pad[0] = impl->probeUniforms._pad[1] = 0;

    // Bake cubemaps: render 6 faces per probe using existing scene geometry
    if (!impl->probesBaked) {
        int size = Impl::PROBE_CUBEMAP_SIZE;

        // Temporary per-face render target for baking (we'll blit to the array)
        MTLTextureDescriptor* faceDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Float
                                         width:size
                                        height:size
                                     mipmapped:NO];
        faceDesc.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        faceDesc.storageMode = MTLStorageModePrivate;

        MTLTextureDescriptor* depthFaceDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                         width:size
                                        height:size
                                     mipmapped:NO];
        depthFaceDesc.usage = MTLTextureUsageRenderTarget;
        depthFaceDesc.storageMode = MTLStorageModePrivate;
        id<MTLTexture> faceDepth = [impl->device newTextureWithDescriptor:depthFaceDesc];

        // Dummy normal attachment (pipelines now output MRT)
        MTLTextureDescriptor* faceNormalDesc = [MTLTextureDescriptor
            texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                         width:size
                                        height:size
                                     mipmapped:NO];
        faceNormalDesc.usage = MTLTextureUsageRenderTarget;
        faceNormalDesc.storageMode = MTLStorageModePrivate;
        id<MTLTexture> faceNormal = [impl->device newTextureWithDescriptor:faceNormalDesc];

        // Face directions for cubemap rendering
        struct CubeFace {
            Vec3 target;
            Vec3 up;
        };
        CubeFace faces[6] = {
            {{1, 0, 0},  {0, -1, 0}},  // +X
            {{-1, 0, 0}, {0, -1, 0}},  // -X
            {{0, 1, 0},  {0, 0, 1}},   // +Y
            {{0, -1, 0}, {0, 0, -1}},  // -Y
            {{0, 0, 1},  {0, -1, 0}},  // +Z
            {{0, 0, -1}, {0, -1, 0}},  // -Z
        };

        Real fovRad = degreesToRadians(90.0);
        Mat4 projMat = Mat4::perspective(fovRad, 1.0, 0.1, 200.0);
        simd_float4x4 proj = toSimd(projMat);

        for (int pi = 0; pi < count; pi++) {
            id<MTLTexture> faceColor = [impl->device newTextureWithDescriptor:faceDesc];

            for (int face = 0; face < 6; face++) {
                Vec3 eye = probes[pi].position;
                Vec3 target = eye + faces[face].target;
                Mat4 viewMat = Mat4::lookAt(eye, target, faces[face].up);
                simd_float4x4 view = toSimd(viewMat);

                CameraUniforms faceCam = {};
                faceCam.viewProjection = simd_mul(proj, view);
                faceCam.view = view;
                faceCam.cameraPosition = toSimd3(eye);

                // Render scene to face
                id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];

                MTLRenderPassDescriptor* rpDesc = [MTLRenderPassDescriptor renderPassDescriptor];
                rpDesc.colorAttachments[0].texture = faceColor;
                rpDesc.colorAttachments[0].loadAction = MTLLoadActionClear;
                rpDesc.colorAttachments[0].storeAction = MTLStoreActionStore;
                rpDesc.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
                rpDesc.colorAttachments[1].texture = faceNormal;
                rpDesc.colorAttachments[1].loadAction = MTLLoadActionDontCare;
                rpDesc.colorAttachments[1].storeAction = MTLStoreActionDontCare;
                rpDesc.depthAttachment.texture = faceDepth;
                rpDesc.depthAttachment.loadAction = MTLLoadActionClear;
                rpDesc.depthAttachment.storeAction = MTLStoreActionDontCare;
                rpDesc.depthAttachment.clearDepth = 1.0;

                id<MTLRenderCommandEncoder> enc = [cmdBuf renderCommandEncoderWithDescriptor:rpDesc];
                [enc setFrontFacingWinding:MTLWindingClockwise];
                [enc setCullMode:MTLCullModeBack];
                [enc setDepthStencilState:impl->depthStateOpaque];

                // Bind shadow resources (lights won't have shadows in probes, but shader expects bindings)
                [enc setFragmentTexture:impl->shadowMap atIndex:0];
                [enc setFragmentSamplerState:impl->shadowSampler atIndex:0];
                Impl::ShadowUniforms noShadow = {0, 0, 0, impl->shadowMapSize};
                [enc setFragmentBytes:&noShadow length:sizeof(Impl::ShadowUniforms) atIndex:5];

                // Bind empty probe data (no recursion)
                Impl::ProbeUniforms noProbes = {0, 0, {0, 0}};
                [enc setFragmentBytes:&noProbes length:sizeof(Impl::ProbeUniforms) atIndex:6];
                [enc setFragmentBuffer:impl->probeBuffer offset:0 atIndex:7];
                // Bind dummy textures for probe slots (shader expects them)
                [enc setFragmentTexture:impl->brdfLUT atIndex:2];
                [enc setFragmentSamplerState:impl->linearClampSampler atIndex:1];

                // Draw skybox (HDR equirect or procedural — see ADR-0016). Baking
                // it into the cube faces is what makes IBL track the environment.
                if (impl->skyboxPipeline) {
                    [enc setRenderPipelineState:impl->skyboxPipeline];
                    [enc setDepthStencilState:impl->skyboxDepthState];
                    [enc setVertexBytes:&faceCam length:sizeof(CameraUniforms) atIndex:1];
                    [enc setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
                    Impl::EnvUniforms envU = {impl->environmentTexture ? 1 : 0, {0, 0, 0}};
                    [enc setFragmentBytes:&envU length:sizeof(envU) atIndex:5];
                    [enc setFragmentTexture:(impl->environmentTexture ? impl->environmentTexture
                                                                       : impl->defaultWhiteTexture)
                                    atIndex:0];
                    [enc setFragmentSamplerState:impl->equirectSampler atIndex:0];
                    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
                }

                // Draw opaque scene geometry
                [enc setDepthStencilState:impl->depthStateOpaque];
                for (auto& dc : impl->opaqueDrawCalls) {
                    const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
                    if (!mesh) continue;

                    ModelUniforms modelU;
                    modelU.model = toSimd(dc.transform);
                    modelU.normalMatrix = inverseTranspose(modelU.model);

                    MaterialUniforms matU;
                    matU.albedo = {static_cast<float>(dc.material.albedo.x),
                                   static_cast<float>(dc.material.albedo.y),
                                   static_cast<float>(dc.material.albedo.z)};
                    matU.metallic = dc.material.metallic;
                    matU.roughness = dc.material.roughness;
                    matU.opacity = dc.material.opacity;
                    matU.flags = static_cast<float>(dc.material.flags);
                    matU.textureFlags = 0;
                    matU.emission = {static_cast<float>(dc.material.emission.x),
                                     static_cast<float>(dc.material.emission.y),
                                     static_cast<float>(dc.material.emission.z)};

                    [enc setRenderPipelineState:impl->opaquePipeline];
                    [enc setVertexBuffer:mesh->vertexBuffer offset:0 atIndex:0];
                    [enc setVertexBytes:&faceCam length:sizeof(CameraUniforms) atIndex:1];
                    [enc setVertexBytes:&modelU length:sizeof(ModelUniforms) atIndex:2];
                    [enc setFragmentBytes:&faceCam length:sizeof(CameraUniforms) atIndex:1];
                    [enc setFragmentBytes:&matU length:sizeof(MaterialUniforms) atIndex:3];
                    [enc setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:3];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:4];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:5];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:6];
                    [enc setFragmentTexture:impl->defaultWhiteTexture atIndex:7];
                    [enc setFragmentSamplerState:impl->linearWrapSampler atIndex:2];
                    [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                    indexCount:mesh->indexCount
                                     indexType:MTLIndexTypeUInt32
                                   indexBuffer:mesh->indexBuffer
                             indexBufferOffset:0];
                }

                [enc endEncoding];

                // Blit face to cubemap array (mip 0)
                id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
                [blit copyFromTexture:faceColor
                          sourceSlice:0 sourceLevel:0
                         sourceOrigin:MTLOriginMake(0, 0, 0)
                           sourceSize:MTLSizeMake(size, size, 1)
                            toTexture:impl->probeCubemapArray
                     destinationSlice:pi * 6 + face destinationLevel:0
                    destinationOrigin:MTLOriginMake(0, 0, 0)];
                [blit endEncoding];

                [cmdBuf commit];
                [cmdBuf waitUntilCompleted];
            }

            // Generate mipmaps for roughness blur (box filter for now — GGX prefilter
            // upgrade is a TODO; this gives reasonable roughness blur)
            {
                id<MTLCommandBuffer> cmdBuf = [impl->commandQueue commandBuffer];
                id<MTLBlitCommandEncoder> blit = [cmdBuf blitCommandEncoder];
                // Generate mipmaps for this probe's slices in the cubemap array
                // Metal's generateMipmaps works on the entire texture
                [blit generateMipmapsForTexture:impl->probeCubemapArray];
                [blit endEncoding];
                [cmdBuf commit];
                [cmdBuf waitUntilCompleted];
            }
        }

        impl->probesBaked = true;
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
    if (!impl->currentDrawable || !impl->currentPassDesc) return;

    // Bake reflection probes on first frame when draw calls exist
    if (impl->probesPendingBake && !impl->opaqueDrawCalls.empty()) {
        Impl::bakeProbes(impl.get(), impl->pendingProbes);
        impl->probesPendingBake = false;
        impl->pendingProbes.clear();
    }

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

    // Bind reflection probe resources for the entire main pass
    Impl::ProbeUniforms activeProbeUniforms = impl->probeUniforms;
    if (!reflectionProbesEnabled) {
        activeProbeUniforms.probeCount = 0;
    }
    [impl->currentEncoder setFragmentBytes:&activeProbeUniforms
                                    length:sizeof(Impl::ProbeUniforms) atIndex:6];
    [impl->currentEncoder setFragmentBuffer:impl->probeBuffer offset:0 atIndex:7];
    if (impl->probeCubemapArray) {
        [impl->currentEncoder setFragmentTexture:impl->probeCubemapArray atIndex:1];
    }
    [impl->currentEncoder setFragmentTexture:impl->brdfLUT atIndex:2];
    [impl->currentEncoder setFragmentSamplerState:impl->linearClampSampler atIndex:1];

    // Draw skybox first (behind everything, no depth write)
    if (impl->skyboxPipeline) {
        [impl->currentEncoder setRenderPipelineState:impl->skyboxPipeline];
        [impl->currentEncoder setDepthStencilState:impl->skyboxDepthState];
        [impl->currentEncoder setVertexBytes:&impl->cameraUniforms
                                      length:sizeof(CameraUniforms) atIndex:1];
        [impl->currentEncoder setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
        Impl::EnvUniforms envU = {impl->environmentTexture ? 1 : 0, {0, 0, 0}};
        [impl->currentEncoder setFragmentBytes:&envU length:sizeof(envU) atIndex:5];
        [impl->currentEncoder setFragmentTexture:(impl->environmentTexture ? impl->environmentTexture
                                                                            : impl->defaultWhiteTexture)
                                         atIndex:0];
        [impl->currentEncoder setFragmentSamplerState:impl->equirectSampler atIndex:0];
        [impl->currentEncoder drawPrimitives:MTLPrimitiveTypeTriangle
                                 vertexStart:0 vertexCount:3];
    }

    RenderStats stats;

    auto computeTextureFlags = [&](const RenderMaterial& mat) -> uint32_t {
        uint32_t tf = 0;
        if (mat.albedoMap.valid())            tf |= 1u;
        if (mat.metallicRoughnessMap.valid()) tf |= 2u;
        if (mat.normalMap.valid())            tf |= 4u;
        if (mat.aoMap.valid())                tf |= 8u;
        if (mat.emissiveMap.valid())          tf |= 16u;
        return tf;
    };

    auto bindMaterialTextures = [&](const RenderMaterial& mat, uint32_t) {
        auto resolve = [&](TextureHandle h) -> id<MTLTexture> {
            if (!h.valid()) return impl->defaultWhiteTexture;
            auto* t = impl->textures.get(h);
            return t ? *t : impl->defaultWhiteTexture;
        };
        [impl->currentEncoder setFragmentTexture:resolve(mat.albedoMap) atIndex:3];
        [impl->currentEncoder setFragmentTexture:resolve(mat.metallicRoughnessMap) atIndex:4];
        [impl->currentEncoder setFragmentTexture:resolve(mat.normalMap) atIndex:5];
        [impl->currentEncoder setFragmentTexture:resolve(mat.aoMap) atIndex:6];
        [impl->currentEncoder setFragmentTexture:resolve(mat.emissiveMap) atIndex:7];
        [impl->currentEncoder setFragmentSamplerState:impl->linearWrapSampler atIndex:2];
    };

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
        inst.flags = static_cast<float>(dc.material.flags);
        inst.emission = {static_cast<float>(dc.material.emission.x),
                         static_cast<float>(dc.material.emission.y),
                         static_cast<float>(dc.material.emission.z), 0};
        inst.textureFlags = computeTextureFlags(dc.material);
        inst._instPad[0] = inst._instPad[1] = inst._instPad[2] = 0;
        return inst;
    };

    auto issueSingleDraw = [&](const Impl::DrawCall& dc) {
        const GPUMesh* mesh = impl->meshes.get(dc.meshHandle);
        if (!mesh) return;

        ModelUniforms modelUniforms;
        modelUniforms.model = toSimd(dc.transform);
        modelUniforms.normalMatrix = inverseTranspose(modelUniforms.model);

        uint32_t tf = computeTextureFlags(dc.material);
        MaterialUniforms matUniforms;
        matUniforms.albedo = {static_cast<float>(dc.material.albedo.x),
                              static_cast<float>(dc.material.albedo.y),
                              static_cast<float>(dc.material.albedo.z)};
        matUniforms.metallic = dc.material.metallic;
        matUniforms.roughness = dc.material.roughness;
        matUniforms.opacity = dc.material.opacity;
        matUniforms.flags = static_cast<float>(dc.material.flags);
        matUniforms.textureFlags = tf;
        matUniforms.emission = {static_cast<float>(dc.material.emission.x),
                                static_cast<float>(dc.material.emission.y),
                                static_cast<float>(dc.material.emission.z)};

        bindMaterialTextures(dc.material, tf);

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

            bindMaterialTextures(drawCalls[batchStart].material,
                                computeTextureFlags(drawCalls[batchStart].material));

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

    [impl->currentEncoder endEncoding];

    // --- Post-processing compute passes (single encoder) ---
    // Batching SSAO, SSR, and bloom into one compute encoder eliminates
    // per-encoder CPU overhead (~15 encoder create/destroy → 1).
    bool needsCompute = (impl->aoPipeline && impl->aoTexture && ssaoEnabled)
                     || (impl->ssrPipeline && impl->ssrTexture && ssrEnabled)
                     || (bloomEnabled && impl->bloomDownsamplePipeline
                         && impl->bloomUpsamplePipeline && impl->bloomMips[0]);

    if (needsCompute) {
        id<MTLComputeCommandEncoder> enc = [impl->currentCommandBuffer computeCommandEncoder];
        MTLSize group = MTLSizeMake(8, 8, 1);

        // --- SSAO ---
        if (impl->aoPipeline && impl->aoTexture && ssaoEnabled) {
            int fullW = impl->framebufferWidth;
            int fullH = impl->framebufferHeight;
            MTLSize aoGrid = MTLSizeMake(fullW, fullH, 1);

            [enc setComputePipelineState:impl->aoPipeline];
            [enc setTexture:impl->depthTexture atIndex:0];
            [enc setTexture:impl->aoTexture atIndex:1];
            [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
            struct { float radius, intensity, bias; int dirs, steps; float _p[3]; } aoP = {
                ssaoParams.radius, ssaoParams.intensity, ssaoParams.bias,
                ssaoParams.directions, ssaoParams.steps, {}
            };
            [enc setBytes:&aoP length:sizeof(aoP) atIndex:1];
            [enc dispatchThreads:aoGrid threadsPerThreadgroup:group];

            if (impl->aoBlurHPipeline && impl->aoBlurVPipeline && impl->aoBlurTemp) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->aoBlurHPipeline];
                [enc setTexture:impl->aoTexture atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->aoBlurTemp atIndex:2];
                [enc dispatchThreads:aoGrid threadsPerThreadgroup:group];

                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->aoBlurVPipeline];
                [enc setTexture:impl->aoBlurTemp atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->aoTexture atIndex:2];
                [enc dispatchThreads:aoGrid threadsPerThreadgroup:group];
            }
        }

        // --- SSR ---
        if (impl->ssrPipeline && impl->ssrTexture && ssrEnabled) {
            int halfW = std::max(impl->framebufferWidth / 2, 1);
            int halfH = std::max(impl->framebufferHeight / 2, 1);
            MTLSize ssrGrid = MTLSizeMake(halfW, halfH, 1);

            [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
            [enc setComputePipelineState:impl->ssrPipeline];
            [enc setTexture:impl->sceneColorTexture atIndex:0];
            [enc setTexture:impl->depthTexture atIndex:1];
            [enc setTexture:impl->ssrTexture atIndex:2];
            [enc setTexture:impl->viewNormalTexture atIndex:3];
            [enc setBytes:&impl->cameraUniforms length:sizeof(CameraUniforms) atIndex:0];
            struct { float maxRayDist, thickness, thicknessFar, stride, blendStrength; float _p[3]; } ssrP = {
                ssrParams.maxRayDist, ssrParams.thickness, ssrParams.thicknessFar,
                ssrParams.stride, ssrParams.blendStrength, {}
            };
            [enc setBytes:&ssrP length:sizeof(ssrP) atIndex:1];
            [enc dispatchThreads:ssrGrid threadsPerThreadgroup:group];

            if (impl->ssrBlurHPipeline && impl->ssrBlurVPipeline && impl->ssrBlurTemp) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->ssrBlurHPipeline];
                [enc setTexture:impl->ssrTexture atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->ssrBlurTemp atIndex:2];
                [enc dispatchThreads:ssrGrid threadsPerThreadgroup:group];

                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                [enc setComputePipelineState:impl->ssrBlurVPipeline];
                [enc setTexture:impl->ssrBlurTemp atIndex:0];
                [enc setTexture:impl->depthTexture atIndex:1];
                [enc setTexture:impl->ssrTexture atIndex:2];
                [enc dispatchThreads:ssrGrid threadsPerThreadgroup:group];
            }
        }

        // --- Bloom ---
        if (bloomEnabled && impl->bloomDownsamplePipeline && impl->bloomUpsamplePipeline
            && impl->bloomMips[0]) {
            [enc memoryBarrierWithScope:MTLBarrierScopeTextures];

            // Downsample chain: scene → mip0 → mip1 → ... → mip[N-1]
            for (int m = 0; m < MetalRenderer::Impl::BLOOM_MIP_COUNT; m++) {
                if (m > 0) [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                id<MTLTexture> src = (m == 0) ? impl->sceneColorTexture : impl->bloomMips[m - 1];
                id<MTLTexture> dst = impl->bloomMips[m];

                struct { float threshold, knee, intensity; int srcW, srcH; float _p[3]; } bp = {
                    bloomParams.threshold, bloomParams.knee, bloomParams.intensity,
                    static_cast<int>(src.width), static_cast<int>(src.height), {}
                };

                [enc setComputePipelineState:impl->bloomDownsamplePipeline];
                [enc setTexture:src atIndex:0];
                [enc setTexture:dst atIndex:1];
                [enc setBytes:&bp length:sizeof(bp) atIndex:0];
                MTLSize grid = MTLSizeMake(dst.width, dst.height, 1);
                [enc dispatchThreads:grid threadsPerThreadgroup:group];
            }

            // Upsample chain: read directly from downsample mips (no blit copy needed)
            int last = MetalRenderer::Impl::BLOOM_MIP_COUNT - 1;
            for (int m = last; m >= 0; m--) {
                [enc memoryBarrierWithScope:MTLBarrierScopeTextures];
                id<MTLTexture> src = (m == last)
                    ? impl->bloomMips[last]           // smallest downsample mip as seed
                    : impl->bloomUpsampleMips[m + 1]; // previously upsampled result
                id<MTLTexture> higher = impl->bloomMips[m];
                id<MTLTexture> dst = impl->bloomUpsampleMips[m];

                struct { float threshold, knee, intensity; int srcW, srcH; float _p[3]; } bp = {
                    bloomParams.threshold, bloomParams.knee, bloomParams.intensity,
                    static_cast<int>(src.width), static_cast<int>(src.height), {}
                };

                [enc setComputePipelineState:impl->bloomUpsamplePipeline];
                [enc setTexture:src atIndex:0];
                [enc setTexture:higher atIndex:1];
                [enc setTexture:dst atIndex:2];
                [enc setBytes:&bp length:sizeof(bp) atIndex:0];
                MTLSize grid = MTLSizeMake(dst.width, dst.height, 1);
                [enc dispatchThreads:grid threadsPerThreadgroup:group];
            }
        }

        [enc endEncoding];
    }

    // --- Composite pass: tone map HDR scene to LDR drawable ---
    if (impl->compositePipeline && impl->compositePassDesc) {
        id<MTLRenderCommandEncoder> compEncoder = [impl->currentCommandBuffer
            renderCommandEncoderWithDescriptor:impl->compositePassDesc];
        [compEncoder setRenderPipelineState:impl->compositePipeline];
        [compEncoder setFragmentTexture:impl->sceneColorTexture atIndex:0];
        [compEncoder setFragmentTexture:impl->ssrTexture atIndex:1];
        [compEncoder setFragmentTexture:impl->aoTexture atIndex:2];
        [compEncoder setFragmentTexture:impl->depthTexture atIndex:3];
        [compEncoder setFragmentTexture:impl->viewNormalTexture atIndex:4];
        if (bloomEnabled && impl->bloomUpsampleMips[0]) {
            [compEncoder setFragmentTexture:impl->bloomUpsampleMips[0] atIndex:5];
        }
        [compEncoder setFragmentBytes:&impl->cameraUniforms
                               length:sizeof(CameraUniforms) atIndex:0];
        struct { int32_t ssaoEnabled; int32_t ssrEnabled; int32_t debugView;
                 float ssrBlendStrength; int32_t bloomEnabled; float bloomIntensity;
                 float _pad[2]; } compositeParams;
        compositeParams.ssaoEnabled = ssaoEnabled ? 1 : 0;
        compositeParams.ssrEnabled = ssrEnabled ? 1 : 0;
        compositeParams.debugView = debugView;
        compositeParams.ssrBlendStrength = ssrParams.blendStrength;
        compositeParams.bloomEnabled = bloomEnabled ? 1 : 0;
        compositeParams.bloomIntensity = bloomParams.intensity;
        compositeParams._pad[0] = 0; compositeParams._pad[1] = 0;
        [compEncoder setFragmentBytes:&compositeParams
                               length:sizeof(compositeParams) atIndex:1];
        // Day/night procedural sky for sky pixels rendered in the composite pass.
        [compEncoder setFragmentBuffer:impl->lightBuffer offset:0 atIndex:4];
        [compEncoder setFragmentSamplerState:impl->linearClampSampler atIndex:0];
        [compEncoder drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];

        // Debug UI (ADR-0011) renders on top of the tone-mapped image.
#ifdef RT_ENABLE_IMGUI
        if (impl->imguiInitialized) {
            ImGui::Render();
            ImGui_ImplMetal_RenderDrawData(ImGui::GetDrawData(),
                                           impl->currentCommandBuffer,
                                           compEncoder);
        }
#endif

        [compEncoder endEncoding];
    }

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
