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
#define RT_MAX_CASCADES 4

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
    // Wind (FLAG_WIND): foliage sway evaluated in the vertex shader. windDir.xz
    // is the world-space sway direction; windTime is seconds.
    simd_float3   windDir;
    float         windTime;
    float         windAmplitude;
    float         windFrequency;
    float         windHeight;     // height over which the sway weight ramps 0->1
    float         _windPad;
    // Wireframe line colour override: rgb is the line colour, w>0.5 enables it (the
    // lit fragment then returns this flat colour instead of shading). Set per pass.
    simd_float4   wireColor;
};

struct ModelUniforms {
    simd_float4x4 model;
    simd_float4x4 normalMatrix;
};

// Planetary atmosphere pass (procedural-planet-plan P3). Filled each frame from the
// camera + sun + Renderer::AtmosphereParams; consumed by fragmentAtmosphereGlow.
struct AtmosphereUniforms {
    simd_float4x4 invViewProjection;
    simd_float4   cameraPosition;   // xyz
    simd_float4   sunDirection;     // xyz toward the sun
    simd_float4   planetCenter;     // xyz
    simd_float4   sunColor;         // rgb, w = intensity
    simd_float4   rayleighCoeff;    // rgb per length
    simd_float4   radii;            // x planetRadius, y atmosphereRadius, z rayleighH, w mieH
    simd_float4   mie;              // x mieCoeff, y mieG, z viewSamples, w lightSamples
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
    // Aerial-perspective fog: lit color lerps toward fogColor with
    // 1-exp(-fogDensity*dist). Mirrors the offline tracer's Scene::fog so both
    // paths fade distant terrain into atmosphere and hide the far clip / LOD
    // seams. fogDensity 0 = off.
    simd_float3 fogColor; float fogDensity;
    // Exponential HEIGHT fog: when fogHeightFalloff > 0 the density decays with
    // altitude (density(y) = fogDensity * exp(-falloff * y)), integrated in
    // closed form along the view ray — ground vistas keep their haze while a
    // camera high above looks down through thin air instead of a white wash.
    // 0 = the old uniform fog.
    float fogHeightFalloff;
    // --- Scattering sky (cinematic-sky phase). skyModel 1 = the Hillaire-style
    // LUT sky is active: the skybox samples the sky-view LUT, the lit pass
    // replaces the exp fog with physically-based aerial perspective (per-channel
    // Rayleigh+Mie transmittance fading toward the sky radiance in the view
    // direction), and IBL rides the sky-baked environment cubes. 0 = legacy
    // gradient sky; every field below is then ignored.
    float skyModel;
    float skyMieG;             // Mie phase asymmetry (sun-ward haze glow)
    float skyAerial;           // aerial-perspective density multiplier (1 = physical)
    simd_float3 skyRayleighBeta; float skyRayleighH;   // scatter /m, scale height m
    simd_float3 skyMieBeta;      float skyMieH;        // EXTINCTION /m, scale height m
    float skyPlanetRadius;     // ground radius (m) — sky-view LUT uv mapping
    float skyAtmosRadius;      // atmosphere top radius (m)
    float skyCamHeight;        // camera height above ground the LUT was built at (m)
    float skySunDiscCos;       // cos of the sun's angular radius (disc cutoff)
    // The moon (the month): a phase-lit sphere disc. skyMoonSun is the TRUE
    // sun direction the disc is lit from (skySunDir carries the active light
    // — the moon itself at night); skyMoonIntensity 0 = below the horizon.
    simd_float3 skyMoonDir;    float skyMoonIntensity;
    simd_float3 skyMoonSun;    float skyMoonIllum;
    // The stars: the celestial frame in local space (see DayNightState),
    // the field's visibility (0 by day) and the Milky Way strength.
    simd_float3 skyCelX;       float skyStarGate;
    simd_float3 skyCelY;       float skyMilkyWay;
    simd_float3 skyCelZ;       float _skp4;
};

// Sky LUT bake (cinematic-sky phase): parameters for the transmittance +
// sky-view compute kernels in atmosphere.metal. Filled from
// SkyScatteringParams + the scene sun whenever the cache key changes.
struct SkyLUTUniforms {
    simd_float4 sunDirection;   // xyz toward the sun (world), w = camera height (m)
    simd_float4 sunColor;       // rgb, w = intensity (sun illuminance scale)
    simd_float4 rayleigh;       // xyz scatter /m, w = scale height (m)
    simd_float4 mie;            // x scatter /m, y extinction /m, z scale height, w = phase g
    simd_float4 planet;         // x ground radius, y atmosphere top radius, z brightness, w = multiScatter
    simd_float4 ground;         // rgb ground albedo, w unused
};

// Volumetric clouds (cinematic-sky phase): one shared uniform block for the
// half-res cloud march (fragmentClouds) and its full-res composite.
struct CloudUniforms {
    simd_float4x4 invViewProjection;
    simd_float4   cameraPosition;
    simd_float4   sunDirection;
    simd_float4   sunColor;        // rgb, w intensity
    simd_float4   skyAmbient;      // rgb ambient, w time
    simd_float4   planetCenter;
    simd_float4   layer;           // x bottom, y top, z planetRadius, w domainMode
    simd_float4   params;          // x coverage, y densityScale, z noiseScale, w windSpeed
    simd_float4   march;           // x viewSteps, y lightSteps, z phaseG, w farDistance
    simd_float4   detail;          // x detailStrength (edge-erosion depth), yzw unused
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
    // Cascaded shadow maps (sun): per-cascade light view-projection (into the
    // matching slice of the shadow-map array) and the view-space far distance of
    // each cascade for selection. cascadeCount active cascades, 0..RT_MAX_CASCADES.
    simd_float4x4 cascadeViewProjection[RT_MAX_CASCADES];
    simd_float4   cascadeSplit;   // view-space far depth of cascades 0..3 in x..w
    int32_t       cascadeCount;
    float         _pad2[3];
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
    // Scattering sky (cinematic-sky): 1 = mode-0 sky pixels sample the sky-view
    // LUT (+ analytic sun disc) instead of the gradient. Ignored when mode == 1.
    int32_t skyModel;
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
    float maxRoughness;   // SSR fades to 0 by this roughness (rough = no mirror)
    float debug;          // !=0: write a color-coded outcome per pixel instead of
                          // the reflected color, so the SSR debug view (2) shows
                          // *where* SSR drops out. See ssrRayMarch for the legend.
    float _pad;
};

// CDLOD terrain (ADR-0036): the per-node morph band. The terrain vertex shader
// lerps a vertex toward its baked morph target (in Vertex::tangent) by morphK,
// which ramps 0->1 as the camera distance crosses [morphStart, morphEnd].
struct TerrainUniforms {
    float morphStart;
    float morphEnd;
    float _pad[2];
};

struct SSAOUniforms {
    float   radius;
    float   intensity;
    float   bias;
    int32_t directions;
    int32_t steps;
    float   frameRotation;   // per-frame rotation offset (rad); temporal resolve
                             // averages it, so few directions look banding-free
    float   _pad[2];
};

// Temporal AO reprojection (kills foliage flicker from G-buffer aliasing).
// prevViewProjection reprojects this frame's world position into last frame's
// screen to fetch the history AO; alpha is the history blend weight (0 on the
// first frame / after a resize, when no valid history exists yet).
struct AOTemporalUniforms {
    simd_float4x4 prevViewProjection;
    float         alpha;
    float         _pad[3];
};

struct BloomUniforms {
    float   threshold;
    float   knee;
    float   intensity;
    int32_t srcWidth;
    int32_t srcHeight;
    // Is this the FIRST downsample (scene -> mip 0)? Only that pass applies the
    // bright-pass threshold. This used to be inferred as `srcWidth > dstWidth*3`
    // — which is never true, because every pass halves the width, so src is
    // always ~2x dst. The threshold therefore never ran once: bloom was the
    // whole image blurred and added back, which is what "it blows everything
    // out" was. Do not re-derive this from sizes.
    int32_t firstPass;
    float   _pad[2];
};

struct CompositeUniforms {
    // Does the render target apply the sRGB transfer function in HARDWARE on
    // write? Set from the presentation surface's pixel format, so the display
    // encode happens exactly once: in-shader for a linear-storage target
    // (macOS BGRA8Unorm, Vulkan's preferred UNORM swapchain), in hardware for
    // an sRGB one (visionOS — CompositorServices allows nothing else).
    int32_t targetEncodesSRGB;
    int32_t ssaoEnabled;
    int32_t ssrEnabled;
    int32_t debugView;      // 0=normal, 1=AO only, 2=SSR only, 3=depth, 4=normals,
                            // 5=shadow factor, 6=albedo (5/6 written by the lit pass)
    float   ssrBlendStrength;
    int32_t bloomEnabled;
    float   bloomIntensity;
    float   aoFloor;        // darkest the AO multiply can go
    int32_t tonemapOp;      // 0=ACES, 1=AgX (the "view transform" / film curve)
    float   gradeContrast;  // log-space contrast around middle grey (1 = neutral)
    float   gradeSaturation;// saturation around luma (1 = neutral)
    // (No cloudMode. The volumetric overlay is composited onto the SCENE target
    // by fragmentCloudComposite, a fullscreen pass — so sky pixels carry it too,
    // and this pass just passes them through. It only needed a flag back when
    // the composite re-derived the sky itself.)
};

// Final lens-warp pass (virtual-camera plan Phase 4): Brown radial distortion,
// lateral chromatic aberration, and vignette in one resample of the composited
// image. All zero = exact passthrough, in which case the pass is skipped.
struct LensPostUniforms {
    float k1;                   // Brown radial terms (LensParams::distortionK1/K2)
    float k2;
    float chromaticAberration;  // ~0.01 = clearly visible fringe
    float vignette;             // fraction of darkening reached at the corners
    float aspect;               // width / height — keeps the warp radius circular
    float _pad[3];
};

// Depth-of-field gather (virtual-camera plan Phase 4). Distances in meters;
// cocScale converts sensor-plane CoC (meters) to pixels, maxCocPixels clamps
// the gather radius to the tap budget.
struct DOFUniforms {
    float focusDistance;
    float focalLength;      // meters (LensParams stores mm)
    float aperture;         // diameter, meters; 0 = pinhole (pass skipped)
    float cocScale;
    float maxCocPixels;
    float _pad[3];
};

#ifndef __METAL_VERSION__
}  // namespace engine
#endif

#endif  // RAYTRACER_SHADER_TYPES_H
