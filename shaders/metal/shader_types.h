// GPU-shared uniform/struct layouts, included by BOTH C++ (metal_renderer.mm)
// and MSL (prepended first by the shader loader — see loadShaderSource). One
// definition replaces the hand-mirrored copies that previously had to agree on
// padding by careful comments (ADR-0017 Phase 0).
//
// Layout rules both languages agree on: a 3-component vector is 16 bytes with
// 16-byte alignment (simd_float3 in C++, float3 in MSL), so a float3 followed
// by a scalar occupies 32 bytes, and explicit _pad fields exist only where a
// struct's total size must round up to a 16-byte multiple.
#ifndef RAYTRACER_SHADER_TYPES_H
#define RAYTRACER_SHADER_TYPES_H

#ifdef __METAL_VERSION__
#include <metal_stdlib>
using namespace metal;
typedef float2   simd_float2;
typedef float3   simd_float3;
typedef float4   simd_float4;
typedef float4x4 simd_float4x4;
typedef int      int32_t;
typedef uint     uint32_t;
#else
#include <simd/simd.h>
#include <cstdint>
namespace engine {
#endif

#define RT_MAX_LIGHTS 32

struct CameraUniforms {
    simd_float4x4 viewProjection;
    simd_float4x4 view;
    simd_float3   cameraPosition;
    float         _camPad0;
    simd_float4x4 invViewProjection;
    simd_float4x4 projection;
    simd_float4x4 invProjection;
    simd_float2   screenSize;
    float         nearPlane;
    float         farPlane;
};

struct ModelUniforms {
    simd_float4x4 model;
    simd_float4x4 normalMatrix;
};

struct MaterialUniforms {
    simd_float3 albedo;
    float       metallic;
    float       roughness;
    float       opacity;
    float       flags;
    uint32_t    textureFlags;
    simd_float3 emission;
};

// Per-instance variant of ModelUniforms + MaterialUniforms for batched draws.
struct GPUInstanceData {
    simd_float4x4 model;
    simd_float4x4 normalMatrix;
    simd_float4   albedo;      // w unused
    float         metallic;
    float         roughness;
    float         opacity;
    float         flags;
    simd_float4   emission;    // w unused
    uint32_t      textureFlags;
    float         _instPad[3];
};

enum GPULightType : int {
    LightType_Point       = 0,
    LightType_Directional = 1,
    LightType_Spot        = 2,
};

struct GPULight {
    simd_float3   position;            // point/spot
    float         intensity;
    simd_float3   direction;           // directional/spot
    float         innerCosAngle;       // spot (cos of inner cone)
    simd_float3   color;
    float         outerCosAngle;       // spot (cos of outer cone)
    simd_float4x4 lightViewProjection; // shadow matrix
    int32_t       type;                // GPULightType
    int32_t       shadowMapIndex;      // -1 = no shadow
    float         range;               // point/spot falloff window radius
    float         _pad[1];
};

struct LightUniforms {
    GPULight lights[RT_MAX_LIGHTS];
    int32_t  lightCount;
    float    exposure;
    float    ambientMultiplier;
    float    _pad[1];
    // Procedural sky (ADR-0016, day/night). Written from `SceneLighting::sky`
    // in setLights(); the skybox, composite, and IBL-fallback paths read it.
    simd_float3 skySunDir;   float skySunIntensity;  // disc brightness, 0 at night
    simd_float3 skySunColor; float _skp0;
    simd_float3 skyZenith;   float _skp1;
    simd_float3 skyHorizon;  float _skp2;
    simd_float3 skyGround;   float _skp3;
    // Procedural clouds (ADR-0016 step 3). time = drift phase in seconds.
    float skyCloudCoverage; float skyCloudDensity;
    float skyCloudScale;    float skyCloudTime;
    // Grading (ADR-0017 Phase 3): tints the ambient/irradiance term only.
    simd_float3 ambientTint; float _grad0;
};

// Per-frame shadow sampling parameters. The rasterization depth bias is NOT
// here — it is applied on the shadow encoder (setDepthBias) from
// ShadowConfig::bias; these are the lookup-side and artistic controls.
struct ShadowUniforms {
    float   normalBias;      // world-space offset along the surface normal
    float   pcfRadius;       // PCF tap spread, in shadow-map texels
    int32_t shadowMapSize;
    int32_t debugShadow;     // 5 = lit shaders output the sun shadow factor as grayscale
    // Artistic response (ADR-0017 Phase 2): occluded regions lerp toward tint.
    simd_float3 shadowTint;
    float   shadowStrength;  // 0 = shadows off, 1 = full occlusion of direct light
    float   ambientStrength; // how much shadow also occludes the ambient/IBL terms
    float   _pad[2];
};

// Environment selection (ADR-0016). mode 0 = procedural sky, 1 = HDR cube.
// cloudsEnabled gates the procedural cloud overlay so the reflection-probe bake
// (which reuses the skybox shader) can render a clouds-free sky — animated
// clouds are a screen/SSR visual only, never baked into probes. envMaxMip is
// the top mip of the GGX-prefiltered environment cube (ADR-0017 Phase 3).
struct EnvUniforms {
    int32_t mode;
    int32_t cloudsEnabled;
    int32_t envMaxMip;
    float   _pad[1];
};

struct GPUReflectionProbe {
    simd_float3 position;
    float       influenceRadius;
    simd_float3 boxMin;
    float       _pad0;
    simd_float3 boxMax;
    int32_t     probeIndex;   // index into cubemap array
};

struct ProbeUniforms {
    int32_t probeCount;
    int32_t maxMipLevel;   // number of roughness mip levels in cubemap
    float   _pad[2];
};

struct SSRUniforms {
    float maxRayDist;
    float thickness;
    float thicknessFar;
    float stride;
    float blendStrength;
    float _pad[3];
};

struct SSAOUniforms {
    float   radius;
    float   intensity;
    float   bias;
    int32_t directions;
    int32_t steps;
    float   _pad[3];
};

struct BloomUniforms {
    float   threshold;
    float   knee;
    float   intensity;
    int32_t srcWidth;
    int32_t srcHeight;
    float   _pad[3];
};

struct CompositeUniforms {
    int32_t ssaoEnabled;
    int32_t ssrEnabled;
    int32_t debugView;      // 0=normal, 1=AO only, 2=SSR only, 3=depth, 4=normals,
                            // 5=shadow factor, 6=albedo (5/6 written by the lit pass)
    float   ssrBlendStrength;
    int32_t bloomEnabled;
    float   bloomIntensity;
    int32_t envMode;        // 0=procedural sky (+clouds), 1=HDR equirect
    float   aoFloor;        // darkest the AO multiply can go
};

#ifndef __METAL_VERSION__
}  // namespace engine
#endif

#endif  // RAYTRACER_SHADER_TYPES_H
