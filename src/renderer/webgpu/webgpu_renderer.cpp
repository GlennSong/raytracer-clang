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
    float params[4];          // x maxDist, y thickness, z steps, w maxRoughness
    float texel[4];           // xy = 1/resolution
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

// The forward mesh shader. Ported from the Vulkan shaders/vulkan/
// mesh.{vert,frag} — multi-light Cook-Torrance, the analytic procedural surface
// library (brick/concrete/asphalt/...), texture/material maps, cascaded shadows,
// IBL (procedural-sky or a bound HDR equirect) with a baked split-sum BRDF LUT,
// fog, checkerboard, and debug views. Embedded as a string (matches the Metal
// backend; the web has no offline compile). Scene-linear with sRGB on store.
const char* kMeshWgsl = R"WGSL(
struct Light {
  positionIntensity : vec4<f32>,   // xyz pos (point/spot), w intensity
  directionInner    : vec4<f32>,   // xyz dir (dir/spot), w innerCos
  colorOuter        : vec4<f32>,   // rgb color, w outerCos
  typeRange         : vec4<f32>,   // x type (0 point,1 dir,2 spot), y range
};
struct Globals {
  viewProjection : mat4x4<f32>,
  view           : mat4x4<f32>,
  invViewProjection : mat4x4<f32>, // reconstruct world rays (sky background)
  cameraPosition : vec4<f32>,
  ambient        : vec4<f32>,      // rgb = ambient tint * multiplier (IBL strength)
  skySunDir      : vec4<f32>,      // xyz dir, w disc intensity
  skySunColor    : vec4<f32>,
  skyZenith      : vec4<f32>,
  skyHorizon     : vec4<f32>,
  skyGround      : vec4<f32>,
  fog            : vec4<f32>,      // rgb color, w density (0 = off)
  counts         : vec4<i32>,      // x lightCount, y debugView, z shadowMapSize, w cascadeCount
  cascadeVP      : array<mat4x4<f32>, 4>,  // per-cascade sun shadow matrices (CSM)
  cascadeSplit   : vec4<f32>,      // far view-space depth of cascades 0..3
  shadowParams   : vec4<f32>,      // x enabled, y depthBias, z normalBias, w pcfTexels
  postParams     : vec4<f32>,      // x exposure, y tonemapOp, z contrast, w saturation
  lights         : array<Light, 32>,
};
struct DrawData {
  model          : mat4x4<f32>,
  albedoMetallic : vec4<f32>,      // rgb albedo, a metallic
  emissionRough  : vec4<f32>,      // rgb emission, a roughness
  surfaceFlags   : vec4<u32>,      // x surfaceId, y rawFlags, z mapBits
  terrainMorph   : vec4<f32>,      // x morphStart, y morphEnd
};

@group(0) @binding(0) var<uniform> g : Globals;
@group(0) @binding(1) var<uniform> d : DrawData;
// Scene environment (group 0): an equirectangular HDR when one is bound
// (envMode = skySunColor.w > 0.5), the split-sum BRDF integration LUT, and a
// shared sampler. envMode 0 ignores envTex and uses the analytic procedural sky
// (sampleEnvironment), matching the Vulkan backend's counts.z env-mode switch.
@group(0) @binding(2) var envTex  : texture_2d<f32>;
@group(0) @binding(3) var envSamp : sampler;
@group(0) @binding(4) var brdfLut : texture_2d<f32>;
// Sun cascaded shadow map (group 1, main pass only — the shadow pass writes it).
@group(1) @binding(0) var shadowMap  : texture_depth_2d_array;
@group(1) @binding(1) var shadowSamp : sampler_comparison;
// Shadow pass only: which cascade layer this depth pass is rendering (group 1,
// a separate binding so it doesn't collide with the main pass's shadow texture).
struct ShadowIdx { idx : vec4<i32> };
@group(1) @binding(2) var<uniform> scidx : ShadowIdx;
// Material maps (group 2). Missing maps bind a 1x1 default (white, or flat
// normal), so sampling is always valid. d.surfaceFlags.z (mapBits) is only
// consulted for the normal map (bit 1); the rest rely on the neutral defaults.
@group(2) @binding(0) var albedoTex   : texture_2d<f32>;
@group(2) @binding(1) var normalTex   : texture_2d<f32>;
@group(2) @binding(2) var mrTex       : texture_2d<f32>;
@group(2) @binding(3) var emissiveTex : texture_2d<f32>;
@group(2) @binding(4) var aoTex       : texture_2d<f32>;
@group(2) @binding(5) var matSamp     : sampler;

struct VSOut {
  @builtin(position) clip : vec4<f32>,
  @location(0) worldPos     : vec3<f32>,
  @location(1) worldNormal  : vec3<f32>,
  @location(2) color        : vec3<f32>,
  @location(3) uv           : vec2<f32>,
  @location(4) worldTangent : vec3<f32>,
};

// Wind sway (FLAG_WIND): displace the tips of a mesh by a height-weighted
// oscillation (base planted). g.ambient.w carries the wind time.
fn applyWind(world : vec4<f32>, localHeight : f32, flags : u32) -> vec4<f32> {
  if ((flags & 4u) == 0u) { return world; }
  let t = g.ambient.w;
  let h = max(localHeight, 0.0);
  var w = world;
  w.x += sin(t * 1.5 + world.z * 0.5) * 0.12 * h;
  w.z += cos(t * 1.1 + world.x * 0.4) * 0.10 * h;
  return w;
}

@vertex
fn vs_main(
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) tangent  : vec3<f32>,
  @location(3) texcoord : vec2<f32>,
  @location(4) color    : vec3<f32>,
) -> VSOut {
  var out : VSOut;
  var world = d.model * vec4<f32>(position, 1.0);
  world = applyWind(world, position.y, d.surfaceFlags.y);
  out.worldPos = world.xyz;
  // Upper 3x3 (correct for rigid / uniform scale); inverse-transpose arrives
  // with the texture/normal-map phase.
  out.worldNormal = normalize((d.model * vec4<f32>(normal, 0.0)).xyz);
  out.worldTangent = (d.model * vec4<f32>(tangent, 0.0)).xyz;
  out.color = color;
  out.uv = texcoord;
  out.clip = g.viewProjection * world;
  return out;
}

// Depth-only shadow pass: transform by the current cascade's view-projection
// (the cascade index rides scidx, set per cascade via a dynamic offset).
@vertex
fn vs_shadow(@location(0) position : vec3<f32>) -> @builtin(position) vec4<f32> {
  return g.cascadeVP[scidx.idx.x] * d.model * vec4<f32>(position, 1.0);
}

// Instanced variants: the per-instance model rides a second vertex buffer
// (locations 5-8 = the four columns), so one draw covers a whole InstanceGroup.
@vertex
fn vs_instanced(
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) tangent  : vec3<f32>,
  @location(3) texcoord : vec2<f32>,
  @location(4) color    : vec3<f32>,
  @location(5) m0 : vec4<f32>, @location(6) m1 : vec4<f32>,
  @location(7) m2 : vec4<f32>, @location(8) m3 : vec4<f32>,
) -> VSOut {
  let model = mat4x4<f32>(m0, m1, m2, m3);
  var out : VSOut;
  var world = model * vec4<f32>(position, 1.0);
  world = applyWind(world, position.y, d.surfaceFlags.y);
  out.worldPos = world.xyz;
  out.worldNormal = normalize((model * vec4<f32>(normal, 0.0)).xyz);
  out.worldTangent = (model * vec4<f32>(tangent, 0.0)).xyz;
  out.color = color;
  out.uv = texcoord;
  out.clip = g.viewProjection * world;
  return out;
}
@vertex
fn vs_shadow_instanced(
  @location(0) position : vec3<f32>,
  @location(5) m0 : vec4<f32>, @location(6) m1 : vec4<f32>,
  @location(7) m2 : vec4<f32>, @location(8) m3 : vec4<f32>,
) -> @builtin(position) vec4<f32> {
  let model = mat4x4<f32>(m0, m1, m2, m3);
  return g.cascadeVP[scidx.idx.x] * model * vec4<f32>(position, 1.0);
}

// CDLOD terrain (ADR-0036): identity model, world-space verts. `tangent` holds
// the morph-target position (where this vertex collapses on the next-coarser
// grid); mix toward it by camera distance so the node matches the neighbour LOD
// before it switches, killing the popping seam. (Ported from lighting.metal.)
fn terrainMorphPos(position : vec3<f32>, morphTarget : vec3<f32>) -> vec3<f32> {
  let dist = distance(position, g.cameraPosition.xyz);
  let k = clamp((dist - d.terrainMorph.x) / max(d.terrainMorph.y - d.terrainMorph.x, 1e-3),
                0.0, 1.0);
  return mix(position, morphTarget, k);
}
@vertex
fn vs_terrain(
  @location(0) position : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) tangent  : vec3<f32>,
  @location(3) texcoord : vec2<f32>,
  @location(4) color    : vec3<f32>,
) -> VSOut {
  var out : VSOut;
  let world = vec4<f32>(terrainMorphPos(position, tangent), 1.0);
  out.worldPos = world.xyz;
  out.worldNormal = normalize(normal);
  out.worldTangent = vec3<f32>(0.0);   // terrain never normal-maps (tangent is morph)
  out.color = color;
  out.uv = texcoord;
  out.clip = g.viewProjection * world;
  return out;
}
@vertex
fn vs_shadow_terrain(
  @location(0) position : vec3<f32>,
  @location(2) tangent  : vec3<f32>,
) -> @builtin(position) vec4<f32> {
  // Same morph as the receiver so the caster matches (no peter-panning seam).
  return g.cascadeVP[scidx.idx.x] * vec4<f32>(terrainMorphPos(position, tangent), 1.0);
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
fn fresnelRoughness(cosT : f32, f0 : vec3<f32>, rough : f32) -> vec3<f32> {
  let r = max(vec3<f32>(1.0 - rough), f0);
  return f0 + (r - f0) * pow(clamp(1.0 - cosT, 0.0, 1.0), 5.0);
}
fn distanceAttenuation(dist : f32, range : f32) -> f32 {
  let ratio2 = (dist * dist) / max(range * range, 1e-4);
  let window = clamp(1.0 - ratio2 * ratio2, 0.0, 1.0);
  return window * window / max(dist * dist, 1e-4);
}
// Procedural-sky environment (no clouds — never baked into IBL). Ported from
// environment.metal sampleEnvironment; an analytic stand-in for baked irradiance.
fn sampleEnvironment(dir : vec3<f32>) -> vec3<f32> {
  let skyBlend = clamp(dir.y, 0.0, 1.0);
  let sky = mix(g.skyHorizon.rgb, g.skyZenith.rgb, pow(skyBlend, 0.5));
  let lowerHaze = mix(g.skyHorizon.rgb, g.skyGround.rgb, 1.0 - smoothstep(-0.4, 0.0, dir.y));
  let horizonBlend = smoothstep(-0.05, 0.05, dir.y);
  var col = mix(lowerHaze, sky, horizonBlend);
  let disc = g.skySunDir.w;
  let sc = g.skySunColor.rgb;
  let sunDot = max(dot(dir, g.skySunDir.xyz), 0.0);
  col += sc * pow(sunDot, 256.0) * 8.0 * disc;
  col += sc * pow(sunDot, 32.0) * 1.0 * disc;
  col += sc * pow(sunDot, 4.0) * 0.15 * disc;
  return col;
}

// Equirectangular HDR environment lookup (matches shaders/vulkan/sky.frag's
// sampleEquirect). Level-sampled (no mip chain), so it stays valid in the
// non-uniform control flow of the debug-view branches.
fn sampleEquirect(dir : vec3<f32>) -> vec3<f32> {
  let u = atan2(dir.z, dir.x) * (0.5 / PI) + 0.5;
  let v = acos(clamp(dir.y, -1.0, 1.0)) * (1.0 / PI);
  return textureSampleLevel(envTex, envSamp, vec2<f32>(u, v), 0.0).rgb;
}

// ---- procedural surface library (ported from common.metal / mesh.frag) -----
fn hash21(a : f32, b : f32) -> f32 { return fract(sin(a * 12.9898 + b * 78.233) * 43758.5453); }
fn mod2(x : f32) -> f32 { return x - 2.0 * floor(x / 2.0); }
fn vnoise2(x : f32, y : f32) -> f32 {
  let xi = floor(x); let yi = floor(y); let xf = x - xi; let yf = y - yi;
  let a = hash21(xi, yi); let b = hash21(xi + 1.0, yi);
  let c = hash21(xi, yi + 1.0); let dd = hash21(xi + 1.0, yi + 1.0);
  let ux = xf * xf * (3.0 - 2.0 * xf); let uy = yf * yf * (3.0 - 2.0 * yf);
  return a * (1.0 - ux) * (1.0 - uy) + b * ux * (1.0 - uy) + c * (1.0 - ux) * uy + dd * ux * uy;
}
fn fbm2(x : f32, y : f32) -> f32 {
  var v = 0.0; var amp = 0.5; var f = 1.0;
  for (var i = 0; i < 4; i = i + 1) { v += amp * vnoise2(x * f, y * f); f *= 2.0; amp *= 0.5; }
  return v;
}
fn tile1(x : f32, m : f32) -> f32 { return x - m * floor(x / m); }
fn surfUV(p : vec3<f32>, n : vec3<f32>) -> vec2<f32> {
  if (abs(n.y) > 0.5) { return vec2<f32>(p.x, p.z); }
  var t = vec2<f32>(n.z, -n.x);
  let tl = length(t);
  if (tl < 1e-6) { t = vec2<f32>(1.0, 0.0); } else { t = t / tl; }
  return vec2<f32>(p.x * t.x + p.z * t.y, p.y);
}
fn surfBrick(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let courseH = 0.075; let brickL = 0.20; let mortar = 0.011;
  let row = floor(v / courseH);
  var off = 0.0; if (mod2(abs(row)) >= 1.0) { off = brickL * 0.5; }
  let uu = u + off; let col = floor(uu / brickL);
  let fy = v - row * courseH; let fx = uu - col * brickL;
  let joint = min(min(fy, courseH - fy), min(fx, brickL - fx));
  let h = hash21(col, row); let h2 = hash21(col * 1.7 + 3.1, row * 0.9 + 5.7);
  var shade = 0.74 + 0.46 * h;
  if (h2 < 0.12) { shade *= 0.6; }
  shade *= 0.94 + 0.12 * (fx / brickL);
  let t = clamp((joint - mortar) / 0.004, 0.0, 1.0);
  return mix(vec3<f32>(0.30, 0.29, 0.27), base * shade, t);
}
fn surfConcrete(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let n = fbm2(u * 0.6, v * 0.6); let fine = vnoise2(u * 9.0, v * 9.0);
  let shade = 0.84 + 0.22 * n + 0.06 * (fine - 0.5);
  var gu = tile1(u, 3.0); gu = min(gu, 3.0 - gu);
  var gv = tile1(v, 3.0); gv = min(gv, 3.0 - gv);
  let jt = smoothstep(0.015, 0.04, min(gu, gv));
  return base * shade * (0.74 + 0.26 * jt);
}
fn surfStucco(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let n = fbm2(u * 3.0, v * 3.0); let fine = vnoise2(u * 22.0, v * 22.0);
  return base * (0.90 + 0.12 * (n - 0.5) + 0.10 * (fine - 0.5));
}
fn surfRoofTile(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let tileW = 0.18; let rowH = 0.32;
  let row = floor(v / rowH);
  var off = 0.0; if (mod2(abs(row)) >= 1.0) { off = tileW * 0.5; }
  let uu = u + off; let col = floor(uu / tileW);
  let fx = uu - col * tileW; let fy = v - row * rowH;
  let curve = sin(PI * (fx / tileW));
  let valley = smoothstep(0.0, 0.02, min(fx, tileW - fx));
  let lap = smoothstep(0.0, 0.05, fy);
  let h = hash21(col, row);
  return base * ((0.55 + 0.5 * curve) * (0.85 + 0.30 * h) * valley * (0.6 + 0.4 * lap));
}
fn surfShingle(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let tabW = 0.30; let rowH = 0.14;
  let row = floor(v / rowH);
  var off = 0.0; if (mod2(abs(row)) >= 1.0) { off = tabW * 0.5; }
  let uu = u + off; let col = floor(uu / tabW);
  let fx = uu - col * tabW; let fy = v - row * rowH;
  let h = hash21(col, row);
  let key = smoothstep(0.0, 0.012, min(fx, tabW - fx));
  let shadow = smoothstep(0.0, 0.03, fy);
  return base * (0.82 + 0.32 * h) * (0.55 + 0.45 * key) * (0.5 + 0.5 * shadow);
}
fn surfCorrugated(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let ribW = 0.12;
  let rib = cos(2.0 * PI * u / ribW);
  let shade = 0.72 + 0.28 * rib;
  let rust = fbm2(u * 1.5, v * 0.6);
  let rmask = clamp((rust - 0.62) / 0.18, 0.0, 1.0) * 0.45;
  return base * shade * (1.0 - rmask) + vec3<f32>(0.40, 0.22, 0.12) * rmask;
}
fn surfAsphalt(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let spk = vnoise2(u * 30.0, v * 30.0); let blotch = fbm2(u * 0.4, v * 0.4);
  let shade = clamp(0.92 + 0.46 * (spk - 0.5) + 0.12 * (blotch - 0.5), 0.5, 1.4);
  return base * shade;
}
fn surfPavement(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let slab = 1.2;
  let su = tile1(u, slab); let sv = tile1(v, slab);
  let joint = min(min(su, slab - su), min(sv, slab - sv));
  let h = hash21(floor(u / slab), floor(v / slab));
  let spk = vnoise2(u * 26.0, v * 26.0);
  let shade = 0.90 + 0.12 * (h - 0.5) + 0.06 * (spk - 0.5);
  let jt = smoothstep(0.02, 0.05, joint);
  return base * shade * (0.6 + 0.4 * jt);
}
fn surfCobble(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let cell = 0.18;
  let cu = u / cell; let cv = v / cell; let iu = floor(cu); let iv = floor(cv);
  var best = 1e9; var bh = 0.0;
  for (var dj = -1; dj <= 1; dj = dj + 1) {
    for (var di = -1; di <= 1; di = di + 1) {
      let ci = iu + f32(di); let cj = iv + f32(dj);
      let jx = hash21(ci, cj); let jy = hash21(ci + 5.2, cj + 1.7);
      let px = ci + 0.5 + (jx - 0.5) * 0.7; let py = cj + 0.5 + (jy - 0.5) * 0.7;
      let dx = cu - px; let dy = cv - py; let ddv = dx * dx + dy * dy;
      if (ddv < best) { best = ddv; bh = hash21(ci + 9.1, cj + 4.3); }
    }
  }
  let stone = smoothstep(0.0, 0.12, 0.62 - sqrt(best));
  return mix(vec3<f32>(0.32, 0.30, 0.27), base * (0.7 + 0.6 * bh), stone);
}
fn surfWood(base : vec3<f32>, u : f32, v : f32) -> vec3<f32> {
  let boardH = 0.18;
  let row = floor(v / boardH); let fy = v - row * boardH;
  let h = hash21(row, 3.0); let grain = vnoise2(u * 40.0, row * 9.0 + v * 2.0);
  let shadow = smoothstep(0.0, 0.02, fy);
  return base * (0.85 + 0.20 * h + 0.12 * (grain - 0.5)) * (0.55 + 0.45 * shadow);
}
fn surfRoadMarkings(base : vec3<f32>, mu : f32, mv : f32) -> vec3<f32> {
  if (mu < 0.5) { return base; }
  let lat = mu - 2.0;
  let yL = 1.0 - smoothstep(0.013, 0.019, abs(lat - 0.030));
  let yR = 1.0 - smoothstep(0.013, 0.019, abs(lat + 0.030));
  let y = max(yL, yR);
  let wL = 1.0 - smoothstep(0.016, 0.022, abs(lat - 0.86));
  let wR = 1.0 - smoothstep(0.016, 0.022, abs(lat + 0.86));
  let w = max(wL, wR);
  var c = mix(base, vec3<f32>(0.82, 0.68, 0.13), y);
  c = mix(c, vec3<f32>(0.86, 0.86, 0.83), w);
  return c;
}
fn applySurface(id : u32, base : vec3<f32>, worldPos : vec3<f32>, n : vec3<f32>, meshUV : vec2<f32>) -> vec3<f32> {
  let uv = surfUV(worldPos, n);
  var c = base;
  if      (id == 1u)  { c = surfBrick(base, uv.x, uv.y); }
  else if (id == 2u)  { c = surfConcrete(base, uv.x, uv.y); }
  else if (id == 3u)  { c = surfStucco(base, uv.x, uv.y); }
  else if (id == 4u)  { c = surfRoofTile(base, uv.x, uv.y); }
  else if (id == 5u)  { c = surfShingle(base, uv.x, uv.y); }
  else if (id == 6u)  { c = surfCorrugated(base, uv.x, uv.y); }
  else if (id == 7u)  { c = surfAsphalt(base, uv.x, uv.y); }
  else if (id == 8u)  { c = surfPavement(base, uv.x, uv.y); }
  else if (id == 9u)  { c = surfCobble(base, uv.x, uv.y); }
  else if (id == 10u) { c = surfWood(base, uv.x, uv.y); }
  else if (id == 11u) { c = surfRoadMarkings(base, meshUV.x, meshUV.y); }
  else { return base; }
  return clamp(c, vec3<f32>(0.0), vec3<f32>(1.0));
}
fn applyCheckerboard(albedo : vec3<f32>, worldPos : vec3<f32>) -> vec3<f32> {
  let cx = i32(floor(worldPos.x));
  let cz = i32(floor(worldPos.z));
  if (((cx + cz) & 1) != 0) { return albedo * 0.3; }
  return albedo;
}

// Pick the tightest cascade whose far split still contains this fragment's
// view-space depth. Returns a uniform-per-fragment index; clamped to the active
// cascade count. (counts.w = cascade count.)
fn pickCascade(worldPos : vec3<f32>) -> i32 {
  let viewZ = -(g.view * vec4<f32>(worldPos, 1.0)).z;
  let cc = g.counts.w;
  var ci = cc - 1;
  for (var c = 0; c < cc; c = c + 1) {
    if (viewZ <= g.cascadeSplit[c]) { ci = c; break; }
  }
  return clamp(ci, 0, cc - 1);
}

// Cascaded sun shadow with hardware PCF (a comparison sampler does the 2x2
// filtering). Returns 1 = lit, 0 = shadowed. textureSampleCompare must be in
// uniform control flow, so we gather the per-fragment cascade/coords first and
// apply the out-of-bounds test via select() *after* the sample.
fn computeShadow(worldPos : vec3<f32>, N : vec3<f32>) -> f32 {
  if (g.shadowParams.x < 0.5) { return 1.0; }
  let ci = pickCascade(worldPos);
  let lp = g.cascadeVP[ci] * vec4<f32>(worldPos + N * g.shadowParams.z, 1.0);
  let ndc = lp.xyz / lp.w;
  let uv = ndc.xy * vec2<f32>(0.5, -0.5) + vec2<f32>(0.5, 0.5);
  let inb = uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0 && ndc.z >= 0.0 && ndc.z <= 1.0;
  let cuv = clamp(uv, vec2<f32>(0.0), vec2<f32>(1.0));
  let s = textureSampleCompare(shadowMap, shadowSamp, cuv, ci, ndc.z - g.shadowParams.y);
  return select(1.0, s, inb);
}

fn evaluateLighting(worldPos : vec3<f32>, N : vec3<f32>, V : vec3<f32>,
                    albedo : vec3<f32>, metallic : f32, roughness : f32,
                    f0 : vec3<f32>, sunShadow : f32) -> vec3<f32> {
  var directLight = vec3<f32>(0.0);
  let a = max(roughness * roughness, 0.002);
  let a2 = a * a;
  let NdotV = max(dot(N, V), 1e-4);
  let count = min(g.counts.x, 32);
  for (var i = 0; i < count; i = i + 1) {
    let light = g.lights[i];
    let ltype = i32(light.typeRange.x);
    var L : vec3<f32>;
    var attenuation : f32;
    if (ltype == 1) {
      L = normalize(light.directionInner.xyz);
      attenuation = light.positionIntensity.w;
    } else {
      var Lv = light.positionIntensity.xyz - worldPos;
      let dist = length(Lv);
      L = normalize(Lv);
      attenuation = light.positionIntensity.w * distanceAttenuation(dist, light.typeRange.y);
      if (ltype == 2) {
        let theta = dot(-L, normalize(light.directionInner.xyz));
        attenuation = attenuation * smoothstep(light.colorOuter.w, light.directionInner.w, theta);
      }
    }
    let NdotL = max(dot(N, L), 0.0);
    if (NdotL <= 0.0 || attenuation <= 0.0) { continue; }
    let H = normalize(L + V);
    let NdotH = max(dot(N, H), 0.0);
    let VdotH = max(dot(V, H), 0.0);
    let D = distributionGGX(NdotH, a2);
    let Vis = visibilitySmith(NdotV, NdotL, a2);
    let F = fresnelSchlick(VdotH, f0);
    let spec = D * Vis * F;
    let diff = (vec3<f32>(1.0) - F) * (1.0 - metallic) * albedo / PI;
    var sh = 1.0;
    if (ltype == 1) { sh = sunShadow; }   // only the sun casts shadows
    directLight += (diff + spec) * light.colorOuter.rgb * (attenuation * NdotL * sh);
  }
  return directLight;
}

// (The view transform — grade + ACES/AgX — lives in kCompositeWgsl only; the
// mesh pass writes scene-linear HDR and never tone-maps.)

// Main pass writes two targets: linear HDR color + a material G-buffer
// (world normal in xyz, roughness in w) that SSAO and SSR read.
struct FsOut {
  @location(0) color : vec4<f32>,
  @location(1) gbuf  : vec4<f32>,
};
@fragment
fn fs_main(in : VSOut) -> FsOut {
  // Sample every material map up front (uniform control flow for textureSample);
  // missing maps read a neutral 1x1 default. d.surfaceFlags.z = which are real.
  let albedoSample = textureSample(albedoTex, matSamp, in.uv);
  let mrSample     = textureSample(mrTex, matSamp, in.uv);
  let emSample     = textureSample(emissiveTex, matSamp, in.uv).rgb;
  let aoSample     = textureSample(aoTex, matSamp, in.uv).r;
  let nmap         = textureSample(normalTex, matSamp, in.uv).xyz * 2.0 - 1.0;

  let rawFlags = d.surfaceFlags.y;
  let maps     = d.surfaceFlags.z;
  // Alpha-cut foliage (FLAG_ALPHA_TEST): drop fully-transparent texels.
  if ((rawFlags & 2u) != 0u && albedoSample.a < 0.5) { discard; }

  var albedo    = d.albedoMetallic.rgb * in.color * albedoSample.rgb;
  let metallic  = clamp(d.albedoMetallic.a * mrSample.b, 0.0, 1.0);   // glTF: B=metal
  let roughness = clamp(d.emissionRough.a * mrSample.g, 0.04, 1.0);   //       G=rough
  let emission  = d.emissionRough.rgb * emSample;

  var N = normalize(in.worldNormal);
  // Normal map (tangent-space -> world via TBN), only with a map + a real tangent.
  if ((maps & 2u) != 0u && length(in.worldTangent) > 0.001) {
    let T = normalize(in.worldTangent - N * dot(in.worldTangent, N));
    let B = cross(N, T);
    N = normalize(T * nmap.x + B * nmap.y + N * nmap.z);
  }
  let V = normalize(g.cameraPosition.xyz - in.worldPos);
  let gbufOut = vec4<f32>(N, roughness);   // material G-buffer (SSAO / SSR)

  if ((rawFlags & 1u) != 0u) { albedo = applyCheckerboard(albedo, in.worldPos); }
  let surfaceId = d.surfaceFlags.x;
  if (surfaceId != 0u) { albedo = applySurface(surfaceId, albedo, in.worldPos, N, in.uv); }

  let f0 = mix(vec3<f32>(0.04), albedo, metallic);
  let NdotV = max(dot(N, V), 1e-4);

  let sunShadow = computeShadow(in.worldPos, N);
  let direct = evaluateLighting(in.worldPos, N, V, albedo, metallic, roughness, f0, sunShadow);

  // Image-based lighting: an equirectangular HDR when bound (skySunColor.w =
  // envMode), else the analytic procedural sky. Split-sum specular weights the
  // prefiltered radiance by the baked BRDF LUT — parity with Vulkan/Metal
  // (envSpecular = prefiltered * (F0 * scale + bias)).
  let R = reflect(-V, N);
  var irradiance : vec3<f32>;
  var prefiltered : vec3<f32>;
  if (g.skySunColor.w > 0.5) {
    irradiance = sampleEquirect(N);
    prefiltered = mix(sampleEquirect(R), irradiance, roughness);   // crude roughness blur
  } else {
    irradiance = sampleEnvironment(N);
    prefiltered = mix(sampleEnvironment(R), irradiance, roughness);
  }
  let Famb = fresnelRoughness(NdotV, f0, roughness);
  let kd = (vec3<f32>(1.0) - Famb) * (1.0 - metallic);
  let envDiffuse = kd * albedo * irradiance;
  let brdf = textureSampleLevel(brdfLut, envSamp, vec2<f32>(NdotV, roughness), 0.0).xy;
  let envSpecular = prefiltered * (f0 * brdf.x + brdf.y);
  let ambient = (envDiffuse + envSpecular) * g.ambient.rgb * aoSample;   // baked AO map

  // Debug views (Renderer::debugView via counts.y) write display-ready values.
  let dbg = g.counts.y;
  if (dbg == 5) { return FsOut(vec4<f32>(vec3<f32>(sunShadow), 1.0), gbufOut); }
  if (dbg == 4) { return FsOut(vec4<f32>(N * 0.5 + 0.5, 1.0), gbufOut); }
  if (dbg == 6) { return FsOut(vec4<f32>(pow(albedo, vec3<f32>(1.0 / 2.2)), 1.0), gbufOut); }
  if (dbg == 7) {
    let ndv = dot(N, V);
    if (ndv >= 0.0) { return FsOut(vec4<f32>(0.0, ndv, 0.0, 1.0), gbufOut); }
    return FsOut(vec4<f32>(-ndv, 0.0, 0.0, 1.0), gbufOut);
  }
  if (dbg == 3) {
    let vd = -(g.view * vec4<f32>(in.worldPos, 1.0)).z;
    let lin = clamp(1.0 - vd / 200.0, 0.0, 1.0);
    return FsOut(vec4<f32>(vec3<f32>(lin), 1.0), gbufOut);
  }
  if (dbg == 8) {                                  // shadow cascade tint
    if (g.shadowParams.x < 0.5) { return FsOut(vec4<f32>(0.0, 0.0, 0.0, 1.0), gbufOut); }
    let ci = pickCascade(in.worldPos);
    var tint = vec3<f32>(1.0, 1.0, 0.4);
    if (ci == 0) { tint = vec3<f32>(1.0, 0.4, 0.4); }
    else if (ci == 1) { tint = vec3<f32>(0.4, 1.0, 0.4); }
    else if (ci == 2) { tint = vec3<f32>(0.4, 0.4, 1.0); }
    return FsOut(vec4<f32>(tint * (direct + ambient), 1.0), gbufOut);
  }

  var color = direct + ambient + emission;
  // Aerial-perspective fog (lerp toward fog color by 1-exp(-density*dist)).
  if (g.fog.w > 0.0) {
    let dist = length(in.worldPos - g.cameraPosition.xyz);
    let f = 1.0 - exp(-g.fog.w * dist);
    color = mix(color, g.fog.rgb, f);
  }
  // Scene-linear into the HDR target; the composite pass owns the view
  // transform (exposure/grade/tonemap) so post effects operate on linear HDR.
  return FsOut(vec4<f32>(color, 1.0), gbufOut);
}

// --- Procedural sky background -------------------------------------------
// A fullscreen triangle whose fragments reconstruct a world-space view ray
// (via invViewProjection) and shade the same procedural sky used for IBL, so
// the background is a real gradient + sun disc instead of a flat clear color.
// Drawn into the main pass with depth-compare Always + no depth write, so the
// meshes (which write depth) paint over it.
struct SkyOut {
  @builtin(position) clip : vec4<f32>,
  @location(0) ray : vec3<f32>,
};
@vertex
fn vs_sky(@builtin(vertex_index) vid : u32) -> SkyOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var out : SkyOut;
  let pos = p[vid];
  out.clip = vec4<f32>(pos, 1.0, 1.0);                  // z = w -> far plane
  let world = g.invViewProjection * vec4<f32>(pos, 1.0, 1.0);
  out.ray = world.xyz / world.w - g.cameraPosition.xyz;
  return out;
}
@fragment
fn fs_sky(in : SkyOut) -> FsOut {
  let dir = normalize(in.ray);
  // Scene-linear into the HDR target; the composite pass tone-maps it with the
  // scene, so the sky tone-matches automatically. The sky writes a sentinel
  // G-buffer (SSAO/SSR skip it anyway — its depth is the far plane). The
  // background follows the bound HDR equirect (envMode) or the procedural sky.
  var col : vec3<f32>;
  if (g.skySunColor.w > 0.5) { col = sampleEquirect(dir); }
  else { col = sampleEnvironment(dir); }
  return FsOut(vec4<f32>(col, 1.0), vec4<f32>(0.0, 0.0, 0.0, 1.0));
}
)WGSL";

// BRDF integration LUT bake (ADR-0057). Ports shaders/vulkan/brdf_lut.frag /
// environment.metal integrateBRDF: for each (NdotV, roughness) texel, GGX
// importance-sample the split-sum specular BRDF into (scale, bias). Rendered
// once at init into an RG16Float texture; the mesh shader samples it for the
// envSpecular term. No inputs — pure numeric integration.
static const char* kBrdfWgsl = R"WGSL(
@vertex
fn vs(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vid], 0.0, 1.0);
}
const PI = 3.14159265359;
const RES = 256.0;
fn radicalInverse(bitsIn : u32) -> f32 {
  var bits = bitsIn;
  bits = (bits << 16u) | (bits >> 16u);
  bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
  bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
  bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
  bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
  return f32(bits) * 2.3283064365386963e-10;
}
fn hammersley(i : u32, n : u32) -> vec2<f32> {
  return vec2<f32>(f32(i) / f32(n), radicalInverse(i));
}
fn importanceSampleGGX(xi : vec2<f32>, N : vec3<f32>, rough : f32) -> vec3<f32> {
  let a = rough * rough;
  let phi = 2.0 * PI * xi.x;
  let cosT = sqrt((1.0 - xi.y) / (1.0 + (a * a - 1.0) * xi.y));
  let sinT = sqrt(1.0 - cosT * cosT);
  let H = vec3<f32>(cos(phi) * sinT, sin(phi) * sinT, cosT);
  var up = vec3<f32>(0.0, 0.0, 1.0);
  if (abs(N.z) >= 0.999) { up = vec3<f32>(1.0, 0.0, 0.0); }
  let tangent = normalize(cross(up, N));
  let bitangent = cross(N, tangent);
  return normalize(tangent * H.x + bitangent * H.y + N * H.z);
}
@fragment
fn fs(@builtin(position) fragCoord : vec4<f32>) -> @location(0) vec2<f32> {
  let NdotV = max(fragCoord.x / RES, 0.001);   // texel x -> NdotV
  let roughness = fragCoord.y / RES;           // texel y -> roughness
  let V = vec3<f32>(sqrt(1.0 - NdotV * NdotV), 0.0, NdotV);
  let N = vec3<f32>(0.0, 0.0, 1.0);
  var A = 0.0;
  var B = 0.0;
  let SAMPLES = 1024u;
  for (var i = 0u; i < SAMPLES; i = i + 1u) {
    let xi = hammersley(i, SAMPLES);
    let H = importanceSampleGGX(xi, N, roughness);
    let L = normalize(2.0 * dot(V, H) * H - V);
    let NdotL = max(L.z, 0.0);
    let NdotH = max(H.z, 0.0);
    let VdotH = max(dot(V, H), 0.0);
    if (NdotL > 0.0) {
      let a2 = roughness * roughness * roughness * roughness;
      let G_V = NdotL * (NdotV * (1.0 - sqrt(a2)) + sqrt(a2));
      let G_L = NdotV * (NdotL * (1.0 - sqrt(a2)) + sqrt(a2));
      let G = 0.5 / max(G_V + G_L, 0.001);
      let G_Vis = (G * VdotH * NdotL) / max(NdotH, 0.001);
      let Fc = pow(1.0 - VdotH, 5.0);
      A = A + (1.0 - Fc) * G_Vis;
      B = B + Fc * G_Vis;
    }
  }
  return vec2<f32>(A / f32(SAMPLES), B / f32(SAMPLES));
}
)WGSL";

// Composite shader: HDR offscreen target -> swapchain. A fullscreen triangle
// reads the linear HDR scene texel-for-texel, applies any screen-space post
// (SSAO/SSR/bloom land here later), then the view transform (exposure -> grade
// -> ACES/AgX). Debug views are already display-ready, so they bypass the tone
// map (counts.y != 0). Its own module + group(0) layout, independent of the
// mesh pipeline.
const char* kCompositeWgsl = R"WGSL(
struct Post {
  postParams : vec4<f32>,   // x exposure, y tonemapOp, z contrast, w saturation
  debugView  : vec4<i32>,   // x = debug view (0 = normal)
  effects    : vec4<f32>,   // x bloom, y ssao, z ssr (0 = off)
};
@group(0) @binding(0) var hdrTex : texture_2d<f32>;
@group(0) @binding(1) var<uniform> p : Post;
@group(0) @binding(2) var bloomTex : texture_2d<f32>;
@group(0) @binding(3) var bloomSamp : sampler;
@group(0) @binding(4) var ssaoTex : texture_2d<f32>;
@group(0) @binding(5) var ssrTex : texture_2d<f32>;

fn applyGrade(x0 : vec3<f32>, contrast : f32, saturation : f32) -> vec3<f32> {
  var x = x0;
  let luma = dot(x, vec3<f32>(0.2126, 0.7152, 0.0722));
  x = max(vec3<f32>(luma) + saturation * (x - vec3<f32>(luma)), vec3<f32>(0.0));
  let grey = 0.18;
  let lg = log2(grey);
  var lx = log2(max(x, vec3<f32>(1e-5)));
  lx = (lx - vec3<f32>(lg)) * contrast + vec3<f32>(lg);
  return exp2(lx);
}
fn tonemapACES(x : vec3<f32>) -> vec3<f32> {
  let ci = vec3<f32>(dot(vec3<f32>(0.59719, 0.35458, 0.04823), x),
                     dot(vec3<f32>(0.07600, 0.90834, 0.01566), x),
                     dot(vec3<f32>(0.02840, 0.13383, 0.83777), x));
  let cf = (ci * (ci + 0.0245786) - 0.000090537) /
           (ci * (0.983729 * ci + 0.432951) + 0.238081);
  let c = vec3<f32>(dot(vec3<f32>( 1.60475, -0.53108, -0.07367), cf),
                    dot(vec3<f32>(-0.10208,  1.10813, -0.00605), cf),
                    dot(vec3<f32>(-0.00327, -0.07276,  1.07602), cf));
  return pow(clamp(c, vec3<f32>(0.0), vec3<f32>(1.0)), vec3<f32>(1.0 / 2.2));
}
fn agxContrastApprox(x : vec3<f32>) -> vec3<f32> {
  let x2 = x * x;
  let x4 = x2 * x2;
  return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4
       - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - vec3<f32>(0.00232);
}
fn tonemapAgX(val0 : vec3<f32>) -> vec3<f32> {
  let agxMat = mat3x3<f32>(
    vec3<f32>(0.842479062253094, 0.0423282422610123, 0.0423756549057051),
    vec3<f32>(0.0784335999999992, 0.878468636469772, 0.0784336),
    vec3<f32>(0.0792237451477643, 0.0791661274605434, 0.879142973793104));
  let agxMatInv = mat3x3<f32>(
    vec3<f32>(1.19687900512017, -0.0528968517574562, -0.0529716355144438),
    vec3<f32>(-0.0980208811401368, 1.15190312990417, -0.0980434501171241),
    vec3<f32>(-0.0990297440797205, -0.0989611768448433, 1.15107367264116));
  let minEv = -12.47393;
  let maxEv = 4.026069;
  var val = agxMat * val0;
  val = clamp(log2(max(val, vec3<f32>(1e-10))), vec3<f32>(minEv), vec3<f32>(maxEv));
  val = (val - vec3<f32>(minEv)) / (maxEv - minEv);
  val = agxContrastApprox(val);
  val = agxMatInv * clamp(val, vec3<f32>(0.0), vec3<f32>(1.0));
  return clamp(val, vec3<f32>(0.0), vec3<f32>(1.0));
}

@vertex
fn vs_composite(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var pos = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(pos[vid], 0.0, 1.0);
}
@fragment
fn fs_composite(@builtin(position) fragCoord : vec4<f32>) -> @location(0) vec4<f32> {
  let px = vec2<i32>(fragCoord.xy);
  var color = textureLoad(hdrTex, px, 0).rgb;
  // The AO target is half-res; linear-sample it (reusing bloomSamp) for a smooth
  // upscale instead of a blocky nearest textureLoad.
  let aoUv = fragCoord.xy / vec2<f32>(textureDimensions(hdrTex));
  if (p.debugView.x == 1) {                                   // AO-only debug view
    let ao = textureSampleLevel(ssaoTex, bloomSamp, aoUv, 0.0).r;
    return vec4<f32>(vec3<f32>(ao), 1.0);
  }
  if (p.debugView.x != 0) { return vec4<f32>(color, 1.0); }   // other debug views, as-is
  if (p.effects.y > 0.0) {                                     // SSAO (darkens crevices)
    let ao = textureSampleLevel(ssaoTex, bloomSamp, aoUv, 0.0).r;
    color = color * mix(1.0, ao, p.effects.y);
  }
  if (p.effects.z > 0.0) {                                     // SSR (add reflection)
    let ssr = textureSampleLevel(ssrTex, bloomSamp, aoUv, 0.0);  // half-res, filtered upscale
    color += ssr.rgb * p.effects.z;                           // rgb premultiplied by confidence
  }
  if (p.effects.x > 0.0) {                                     // additive bloom
    let dim = vec2<f32>(textureDimensions(hdrTex));
    let uv = (fragCoord.xy) / dim;
    color += textureSampleLevel(bloomTex, bloomSamp, uv, 0.0).rgb * p.effects.x;
  }
  color = color * p.postParams.x;                             // exposure
  color = applyGrade(color, p.postParams.z, p.postParams.w);
  if (p.postParams.y > 0.5) { color = tonemapAgX(color); }
  else                      { color = tonemapACES(color); }
  return vec4<f32>(color, 1.0);
}
)WGSL";

// Bloom: a half-res bright-pass (soft-knee threshold, ported from post.metal's
// bloomDownsample) + a separable Gaussian blur, added back in the composite.
// One bind-group layout { src texture, sampler, uniform } shared by both
// fragment entry points; the blur direction rides the uniform.
const char* kBloomWgsl = R"WGSL(
struct BloomU {
  params : vec4<f32>,   // x threshold, y knee, z intensity, w mode (0 bright, 1 blur)
  texel  : vec4<f32>,   // xy = blur step in uv (texelSize * direction)
};
@group(0) @binding(0) var srcTex : texture_2d<f32>;
@group(0) @binding(1) var srcSamp : sampler;
@group(0) @binding(2) var<uniform> u : BloomU;

@vertex
fn vs_bloom(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vid], 0.0, 1.0);
}

// Bright-pass: src is full-res, dst is half-res. Soft-knee threshold.
@fragment
fn fs_bright(@builtin(position) fc : vec4<f32>) -> @location(0) vec4<f32> {
  let srcDim = vec2<f32>(textureDimensions(srcTex));
  let uv = (fc.xy * 2.0) / srcDim;          // half-res frag -> full-res uv
  let c = textureSampleLevel(srcTex, srcSamp, uv, 0.0).rgb;
  let brightness = max(c.r, max(c.g, c.b));
  let knee = max(u.params.y, 1e-4);
  var soft = brightness - u.params.x + knee;
  soft = clamp(soft, 0.0, 2.0 * knee);
  soft = soft * soft / (4.0 * knee + 1e-5);
  let contribution = max(soft, brightness - u.params.x) / max(brightness, 1e-5);
  return vec4<f32>(c * contribution, 1.0);
}

// Separable 9-tap Gaussian along u.texel; src and dst are the same (half) size.
@fragment
fn fs_blur(@builtin(position) fc : vec4<f32>) -> @location(0) vec4<f32> {
  let dim = vec2<f32>(textureDimensions(srcTex));
  let uv = fc.xy / dim;
  let w = array<f32, 5>(0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);
  var sum = textureSampleLevel(srcTex, srcSamp, uv, 0.0).rgb * w[0];
  for (var i = 1; i < 5; i = i + 1) {
    let o = u.texel.xy * f32(i);
    sum += textureSampleLevel(srcTex, srcSamp, uv + o, 0.0).rgb * w[i];
    sum += textureSampleLevel(srcTex, srcSamp, uv - o, 0.0).rgb * w[i];
  }
  return vec4<f32>(sum, 1.0);
}
)WGSL";

// SSAO: reconstruct world position + a geometric normal (from depth derivatives)
// in a fullscreen pass, sample a hemisphere kernel oriented by the normal, and
// accumulate occlusion against the scene depth. Output is a single-channel AO
// the composite multiplies in. Depth-based (no G-buffer normal) for simplicity.
const char* kSsaoWgsl = R"WGSL(
struct SsaoU {
  invVP : mat4x4<f32>,
  viewProj : mat4x4<f32>,
  camPos : vec4<f32>,
  params : vec4<f32>,   // x radius, y bias, z intensity, w aoFloor
  texel  : vec4<f32>,   // xy = 1/halfRes (AO target), zw = full resolution
};
@group(0) @binding(0) var depthTex : texture_depth_2d;
@group(0) @binding(1) var<uniform> s : SsaoU;
@group(0) @binding(2) var gbufTex : texture_2d<f32>;   // world normal (xyz), roughness (w)

@vertex
fn vs_ssao(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vid], 0.0, 1.0);
}

fn reconWorld(uv : vec2<f32>, depth : f32) -> vec3<f32> {
  let ndc = vec3<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, depth);
  let w = s.invVP * vec4<f32>(ndc, 1.0);
  return w.xyz / w.w;
}
fn hash12(p : vec2<f32>) -> f32 {
  var p3 = fract(vec3<f32>(p.xyx) * 0.1031);
  p3 += dot(p3, p3.yzx + 33.33);
  return fract((p3.x + p3.y) * p3.z);
}

// 12 fixed hemisphere directions (tangent space, z up), varying length.
const KERNEL : array<vec3<f32>, 12> = array<vec3<f32>, 12>(
  vec3<f32>( 0.21, 0.32, 0.92), vec3<f32>(-0.46, 0.11, 0.88), vec3<f32>( 0.33,-0.41, 0.85),
  vec3<f32>(-0.25,-0.55, 0.80), vec3<f32>( 0.62, 0.22, 0.75), vec3<f32>(-0.58, 0.48, 0.66),
  vec3<f32>( 0.10, 0.74, 0.66), vec3<f32>( 0.48,-0.62, 0.62), vec3<f32>(-0.71,-0.30, 0.64),
  vec3<f32>( 0.80, 0.05, 0.60), vec3<f32>(-0.12, 0.88, 0.46), vec3<f32>( 0.35, 0.55, 0.76));

@fragment
fn fs_ssao(@builtin(position) fc : vec4<f32>) -> @location(0) f32 {
  // fc is in half-res AO space; texel.xy = 1/halfRes -> uv, texel.zw = full res
  // to index the full-res depth/G-buffer at this AO texel.
  let uv = fc.xy * s.texel.xy;
  let ipx = vec2<i32>(uv * s.texel.zw);
  let depth = textureLoad(depthTex, ipx, 0);
  if (depth >= 1.0) { return 1.0; }          // sky: no occlusion
  let P = reconWorld(uv, depth);
  // Shading normal straight from the G-buffer (cleaner than depth derivatives).
  let N = normalize(textureLoad(gbufTex, ipx, 0).xyz);
  // Random tangent basis (per-pixel rotation breaks up banding).
  let rnd = hash12(fc.xy) * 6.2831853;
  let randVec = normalize(vec3<f32>(cos(rnd), sin(rnd), 0.0));
  let T = normalize(randVec - N * dot(randVec, N));
  let B = cross(N, T);
  let radius = s.params.x;
  let bias = s.params.y;
  var occ = 0.0;
  for (var i = 0; i < 12; i = i + 1) {
    let k = KERNEL[i];
    let dir = T * k.x + B * k.y + N * k.z;     // tangent -> world, hemisphere around N
    let Sp = P + dir * radius;
    let clip = s.viewProj * vec4<f32>(Sp, 1.0);
    if (clip.w <= 0.0) { continue; }
    let sndc = clip.xyz / clip.w;
    let suv = vec2<f32>(sndc.x * 0.5 + 0.5, 0.5 - sndc.y * 0.5);
    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) { continue; }
    let sd = textureLoad(depthTex, vec2<i32>(suv * s.texel.zw), 0);
    let sceneP = reconWorld(suv, sd);
    // Occluded if the scene surface is closer to the camera than the sample,
    // and within the radius (range check kills halos at silhouettes).
    let distScene = length(s.camPos.xyz - sceneP);
    let distSample = length(s.camPos.xyz - Sp);
    let rangeCheck = smoothstep(0.0, 1.0, radius / max(length(P - sceneP), 1e-3));
    if (distScene < distSample - bias) { occ += rangeCheck; }
  }
  let ao = 1.0 - (occ / 12.0) * s.params.z;
  return clamp(ao, s.params.w, 1.0);
}
)WGSL";

// SSR: screen-space reflections. Reconstruct position/normal/roughness from
// depth + G-buffer, march the reflection ray, and on a depth hit sample the HDR
// color. Confidence folds in fresnel, roughness, and a screen-edge fade. The
// composite adds it. All textureLoad (no uniform-control-flow constraint).
const char* kSsrWgsl = R"WGSL(
struct SsrU {
  invVP : mat4x4<f32>,
  viewProj : mat4x4<f32>,
  camPos : vec4<f32>,
  params : vec4<f32>,   // x maxDist, y thickness, z steps, w maxRoughness
  texel  : vec4<f32>,   // xy = 1/effectRes (SSR target), zw = full resolution
};
@group(0) @binding(0) var hdrTex   : texture_2d<f32>;
@group(0) @binding(1) var depthTex : texture_depth_2d;
@group(0) @binding(2) var gbufTex  : texture_2d<f32>;
@group(0) @binding(3) var<uniform> s : SsrU;

fn reconW(uv : vec2<f32>, depth : f32) -> vec3<f32> {
  let ndc = vec3<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, depth);
  let w = s.invVP * vec4<f32>(ndc, 1.0);
  return w.xyz / w.w;
}
@vertex
fn vs_ssr(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vid], 0.0, 1.0);
}
@fragment
fn fs_ssr(@builtin(position) fc : vec4<f32>) -> @location(0) vec4<f32> {
  // fc is in the scaled SSR space; texel.xy = 1/effectRes -> uv, texel.zw =
  // full res to index the full-res depth/G-buffer/HDR at this SSR texel.
  let uv = fc.xy * s.texel.xy;
  let ipx = vec2<i32>(uv * s.texel.zw);
  let depth = textureLoad(depthTex, ipx, 0);
  if (depth >= 1.0) { return vec4<f32>(0.0); }
  let gb = textureLoad(gbufTex, ipx, 0);
  let roughness = gb.w;
  if (roughness > s.params.w) { return vec4<f32>(0.0); }   // too rough to reflect
  let N = normalize(gb.xyz);
  let P = reconW(uv, depth);
  let V = normalize(s.camPos.xyz - P);
  let R = reflect(-V, N);
  let steps = i32(s.params.z);
  let stepLen = s.params.x / max(f32(steps), 1.0);
  var pos = P + N * 0.05;
  var hitUV = vec2<f32>(0.0);
  var hit = false;
  for (var i = 0; i < steps; i = i + 1) {
    pos = pos + R * stepLen;
    let clip = s.viewProj * vec4<f32>(pos, 1.0);
    if (clip.w <= 0.0) { break; }
    let ndc = clip.xyz / clip.w;
    let suv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) { break; }
    let sd = textureLoad(depthTex, vec2<i32>(suv * s.texel.zw), 0);
    if (sd >= 1.0) { continue; }
    let sP = reconW(suv, sd);
    let rayDist = length(s.camPos.xyz - pos);
    let sceneDist = length(s.camPos.xyz - sP);
    if (rayDist > sceneDist && (rayDist - sceneDist) < s.params.y) { hit = true; hitUV = suv; break; }
  }
  if (!hit) { return vec4<f32>(0.0); }
  let refl = textureLoad(hdrTex, vec2<i32>(hitUV * s.texel.zw), 0).rgb;
  let fres = pow(1.0 - max(dot(N, V), 0.0), 4.0);
  let edge = smoothstep(0.0, 0.15, min(min(hitUV.x, 1.0 - hitUV.x), min(hitUV.y, 1.0 - hitUV.y)));
  let conf = (1.0 - roughness / max(s.params.w, 1e-3)) * (0.2 + 0.8 * fres) * edge;
  return vec4<f32>(refl * conf, conf);
}
)WGSL";

// Mipmap downsample: a fullscreen triangle that linearly samples the previous
// (larger) mip level; run once per level to fill a texture's mip chain.
static const char* kBlitWgsl = R"WGSL(
@group(0) @binding(0) var srcTex : texture_2d<f32>;
@group(0) @binding(1) var srcSamp : sampler;
struct VOut { @builtin(position) pos : vec4<f32>, @location(0) uv : vec2<f32> };
@vertex
fn vs_blit(@builtin(vertex_index) vid : u32) -> VOut {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  var o : VOut;
  o.pos = vec4<f32>(p[vid], 0.0, 1.0);
  o.uv = vec2<f32>(p[vid].x * 0.5 + 0.5, 0.5 - p[vid].y * 0.5);
  return o;
}
@fragment
fn fs_blit(in : VOut) -> @location(0) vec4<f32> {
  return textureSampleLevel(srcTex, srcSamp, in.uv, 0.0);
}
)WGSL";

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
        if (instancedPipeline_) { wgpuRenderPipelineRelease(instancedPipeline_); instancedPipeline_ = nullptr; }
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

        // Cascaded sun shadow: split the view frustum into N depth slices and fit
        // a texel-snapped, camera-centered orthographic box to each (port of the
        // Vulkan/Metal cascade fit, adjusted for WebGPU standard-Z: NDC near z=0,
        // far z=1). Written into globals before the upload below.
        activeCascades_ = 0;
        if (shadowOn_) {
            int cc = std::max(1, std::min(shadowCascadeCount_, 4));
            Real lambda = shadowParams.splitLambda;
            Real camNear = camNear_, camFar = camFar_;
            Real shadowDist = std::max(std::min(static_cast<Real>(shadowDistance_), camFar), camNear * 2.0);

            Vec3 up = std::abs(sunDir_.y) > 0.99 ? Vec3(0, 0, 1) : Vec3(0, 1, 0);
            Mat4 invVP = camVP_.inverse();
            // WebGPU standard-Z: near plane at NDC z=0, far at z=1.
            auto corner = [&](Real x, Real y, Real z) { return invVP.transformPoint(Vec3(x, y, z)); };
            Vec3 nearC[4] = {corner(-1,-1,0), corner(1,-1,0), corner(1,1,0), corner(-1,1,0)};
            Vec3 farC[4]  = {corner(-1,-1,1), corner(1,-1,1), corner(1,1,1), corner(-1,1,1)};

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
                Vec3 center = cameraEye_;   // camera-centered fit: stable when turning
                Real radius = 0.01;
                for (const Vec3& p : corners) radius = std::max(radius, (p - center).length());
                radius = std::ceil(radius * 16.0) / 16.0;

                Real pullback = radius + 50.0;
                Real texelWorld = (radius * 2.0) / static_cast<Real>(kShadowMapSize);
                Mat4 lightView = Mat4::lookAt(center + sunDir_ * pullback, center, up);
                Vec3 centerLS = lightView.transformPoint(center);
                centerLS.x = std::round(centerLS.x / texelWorld) * texelWorld;
                centerLS.y = std::round(centerLS.y / texelWorld) * texelWorld;
                Vec3 snapped = lightView.inverse().transformPoint(centerLS);

                lightView = Mat4::lookAt(snapped + sunDir_ * pullback, snapped, up);
                Mat4 lightProj = Mat4::orthographic(radius * 2.0, 1.0, 0.1, pullback + radius);
                packMat4(lightProj * lightView, globals_.cascadeVP[c]);
                globals_.cascadeSplit[c] = static_cast<float>(zFar);
            }
            activeCascades_ = cc;
            globals_.counts[3] = cc;
            globals_.shadowParams[0] = 1.0f;
            globals_.shadowParams[1] = shadowDepthBias_;
            globals_.shadowParams[2] = shadowNormalBias_;
            globals_.shadowParams[3] = shadowPcf_;
        } else {
            globals_.counts[3] = 0;
            globals_.shadowParams[0] = 0.0f;
        }

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
        const size_t terrainBase = draws_.size() + instancedGroups_.size();
        size_t totalSlots = terrainBase + terrainDraws_.size();
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
                std::memcpy(staging + (terrainBase + i) * kDrawStride,
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

        // Cascaded shadow depth passes (sun): one depth pass per cascade layer,
        // before the main pass. The cascade index rides shadowIdxGroup_ via a
        // dynamic offset; vs_shadow uses it to pick cascadeVP[c].
        if (shadowOn_ && (!draws_.empty() || !instancedGroups_.empty() || !terrainDraws_.empty())) {
            for (int c = 0; c < activeCascades_; ++c) {
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
                // Instanced groups cast shadows too.
                if (!instancedGroups_.empty()) {
                    wgpuRenderPassEncoderSetPipeline(spass, instancedShadowPipeline_);
                    for (size_t i = 0; i < instancedGroups_.size(); ++i) {
                        const InstancedGroup& g = instancedGroups_[i];
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
                if (!terrainDraws_.empty()) {
                    wgpuRenderPassEncoderSetPipeline(spass, terrainShadowPipeline_);
                    for (size_t i = 0; i < terrainDraws_.size(); ++i) {
                        const GpuMesh& m = meshes_[terrainDraws_[i].mesh];
                        if (!m.vertexBuffer) continue;   // removed mid-frame
                        uint32_t dynOffset = static_cast<uint32_t>((terrainBase + i) * kDrawStride);
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

        // Which post passes actually run this frame — decided up front so the
        // main pass can skip storing attachments nothing will read. Zero-strength
        // effects are treated as off (their passes would render into targets the
        // composite multiplies by 0).
        bool ssaoOn = ssaoEnabled && globals_.counts[1] == 0 && ssaoParams.intensity > 0.0f;
        bool ssaoForDebug = globals_.counts[1] == 1;   // AO debug view shows AO even if disabled
        bool ssrOn = ssrEnabled && globals_.counts[1] == 0 && ssrParams.blendStrength > 0.0f;
        bool bloomOn = bloomEnabled && globals_.counts[1] == 0 && bloomParams.intensity > 0.0f;
        // Depth + G-buffer are only ever sampled by SSAO/SSR; when neither runs,
        // Discard skips the attachment writeback (a real win on tiler GPUs).
        bool needGbuf = ssaoOn || ssaoForDebug || ssrOn;

        WGPURenderPassColorAttachment colorAtt[2] = {};
        colorAtt[0].view = hdrView_;   // scene -> HDR target; composite -> swapchain
        colorAtt[0].depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        colorAtt[0].loadOp = WGPULoadOp_Clear;
        colorAtt[0].storeOp = WGPUStoreOp_Store;
        colorAtt[0].clearValue = clearColor_;
        colorAtt[1].view = gbufView_;  // material G-buffer (normal, roughness)
        colorAtt[1].depthSlice = WGPU_DEPTH_SLICE_UNDEFINED;
        colorAtt[1].loadOp = WGPULoadOp_Clear;
        colorAtt[1].storeOp = needGbuf ? WGPUStoreOp_Store : WGPUStoreOp_Discard;
        colorAtt[1].clearValue = {0.0, 0.0, 0.0, 1.0};

        WGPURenderPassDepthStencilAttachment depth = {};
        depth.view = depthView_;
        depth.depthLoadOp = WGPULoadOp_Clear;
        depth.depthStoreOp = needGbuf ? WGPUStoreOp_Store : WGPUStoreOp_Discard;
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

        for (size_t i = 0; i < draws_.size(); ++i) {
            const GpuMesh& m = meshes_[draws_[i].mesh];
            if (!m.vertexBuffer) continue;   // removed mid-frame
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
                uint32_t dynOffset = static_cast<uint32_t>((terrainBase + i) * kDrawStride);
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

        wgpuRenderPassEncoderEnd(pass);
        wgpuRenderPassEncoderRelease(pass);

        if (!bindDiagLogged_ && !draws_.empty()) {
            bindDiagLogged_ = true;
            LOG_INFO("WebGPU binds: %zu draws -> %zu material binds, %zu mesh binds",
                     draws_.size(), matBinds, meshBinds);
        }

        // SSAO: reconstruct from the just-written depth into the AO target.
        if (ssaoOn || ssaoForDebug) {
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
        if (ssrOn) {
            GpuSsr ur = {};
            std::memcpy(ur.invViewProjection, globals_.invViewProjection, sizeof(ur.invViewProjection));
            std::memcpy(ur.viewProjection, globals_.viewProjection, sizeof(ur.viewProjection));
            std::memcpy(ur.cameraPosition, globals_.cameraPosition, sizeof(ur.cameraPosition));
            ur.params[0] = ssrParams.maxRayDist;
            ur.params[1] = ssrParams.thickness;
            ur.params[2] = 32.0f;                    // march steps
            ur.params[3] = ssrParams.maxRoughness;
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
        if (bloomOn) {
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

        // Composite the HDR target to the swapchain (view transform / post).
        GpuPost post = {};
        post.postParams[0] = globals_.postParams[0];
        post.postParams[1] = globals_.postParams[1];
        post.postParams[2] = globals_.postParams[2];
        post.postParams[3] = globals_.postParams[3];
        post.debugView[0] = globals_.counts[1];
        post.effects[0] = bloomOn ? bloomParams.intensity : 0.0f;
        post.effects[1] = ssaoOn ? 1.0f : 0.0f;   // SSAO strength (0..1)
        post.effects[2] = ssrOn ? ssrParams.blendStrength : 0.0f;
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
        MaterialKey mat;
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

        if (!pipeline_ || !shadowPipeline_ || !skyPipeline_ || !compositePipeline_
            || !bloomBrightPipeline_ || !bloomBlurPipeline_ || !ssaoPipeline_
            || !instancedPipeline_ || !instancedShadowPipeline_
            || !terrainPipeline_ || !terrainShadowPipeline_ || !ssrPipeline_) {
            LOG_ERROR("WebGPU: render pipeline creation failed");
            return false;
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

    WGPUBindGroupLayout bindLayout_ = nullptr;
    WGPURenderPipeline pipeline_ = nullptr;
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
