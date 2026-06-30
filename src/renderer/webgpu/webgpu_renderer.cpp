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
    float    cameraPosition[4];
    float    ambient[4];          // rgb ambient term (tint * multiplier)
    float    skySunDir[4];        // xyz dir, w disc intensity
    float    skySunColor[4];
    float    skyZenith[4];
    float    skyHorizon[4];
    float    skyGround[4];
    float    fog[4];              // rgb color, w density (0 = off)
    int32_t  counts[4];           // x lightCount, y debugView, z shadowMapSize
    float    lightViewProj[16];   // single-cascade sun shadow matrix
    float    shadowParams[4];     // x enabled, y depthBias, z normalBias, w pcfTexels
    GpuLight lights[32];
};

// Per-draw uniforms (group 0, binding 1, dynamic). Matches the WGSL `DrawData`.
struct GpuDraw {
    float    model[16];          // column-major
    float    albedoMetallic[4];  // rgb albedo, a metallic
    float    emissionRough[4];   // rgb emission, a roughness
    uint32_t surfaceFlags[4];    // x surfaceId, y rawFlags (checkerboard/alpha/wind)
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

// The forward mesh shader (Phase 2a). Ported from the Vulkan shaders/vulkan/
// mesh.{vert,frag} — multi-light Cook-Torrance, the analytic procedural surface
// library (brick/concrete/asphalt/...), procedural-sky IBL, fog, checkerboard,
// and debug views. Still missing vs. Vulkan (later phases): texture maps,
// cascaded shadows, the split-sum BRDF LUT (an analytic env-BRDF stands in).
// Embedded as a string (matches the Metal backend; the web has no offline
// compile). Scene-linear with a manual sRGB encode at the end.
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
  cameraPosition : vec4<f32>,
  ambient        : vec4<f32>,      // rgb = ambient tint * multiplier (IBL strength)
  skySunDir      : vec4<f32>,      // xyz dir, w disc intensity
  skySunColor    : vec4<f32>,
  skyZenith      : vec4<f32>,
  skyHorizon     : vec4<f32>,
  skyGround      : vec4<f32>,
  fog            : vec4<f32>,      // rgb color, w density (0 = off)
  counts         : vec4<i32>,      // x lightCount, y debugView, z shadowMapSize
  lightViewProj  : mat4x4<f32>,    // single-cascade sun shadow matrix
  shadowParams   : vec4<f32>,      // x enabled, y depthBias, z normalBias, w pcfTexels
  lights         : array<Light, 32>,
};
struct DrawData {
  model          : mat4x4<f32>,
  albedoMetallic : vec4<f32>,      // rgb albedo, a metallic
  emissionRough  : vec4<f32>,      // rgb emission, a roughness
  surfaceFlags   : vec4<u32>,      // x surfaceId, y rawFlags
};

@group(0) @binding(0) var<uniform> g : Globals;
@group(0) @binding(1) var<uniform> d : DrawData;
// Sun shadow map (group 1, main pass only — the shadow pass writes it).
@group(1) @binding(0) var shadowMap  : texture_depth_2d;
@group(1) @binding(1) var shadowSamp : sampler_comparison;

struct VSOut {
  @builtin(position) clip : vec4<f32>,
  @location(0) worldPos    : vec3<f32>,
  @location(1) worldNormal : vec3<f32>,
  @location(2) color       : vec3<f32>,
  @location(3) uv          : vec2<f32>,
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
  // Upper 3x3 (correct for rigid / uniform scale); inverse-transpose arrives
  // with the texture/normal-map phase.
  out.worldNormal = normalize((d.model * vec4<f32>(normal, 0.0)).xyz);
  out.color = color;
  out.uv = texcoord;
  out.clip = g.viewProjection * world;
  return out;
}

// Depth-only shadow pass: transform by the light's view-projection.
@vertex
fn vs_shadow(@location(0) position : vec3<f32>) -> @builtin(position) vec4<f32> {
  return g.lightViewProj * d.model * vec4<f32>(position, 1.0);
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
// Analytic split-sum env BRDF (Karis mobile approximation) — stands in for the
// baked BRDF LUT the Vulkan/Metal backends sample.
fn envBRDFApprox(f0 : vec3<f32>, rough : f32, NoV : f32) -> vec3<f32> {
  let c0 = vec4<f32>(-1.0, -0.0275, -0.572, 0.022);
  let c1 = vec4<f32>(1.0, 0.0425, 1.04, -0.04);
  let r = rough * c0 + c1;
  let a004 = min(r.x * r.x, exp2(-9.28 * NoV)) * r.x + r.y;
  let ab = vec2<f32>(-1.04, 1.04) * a004 + r.zw;
  return f0 * ab.x + ab.y;
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

// Single-cascade sun shadow with hardware PCF (a comparison sampler does the
// 2x2 filtering). Returns 1 = lit, 0 = shadowed. textureSampleCompare must be in
// uniform control flow — the only branch above it is on the uniform shadowParams,
// and the out-of-bounds test is applied via select() *after* the sample.
fn computeShadow(worldPos : vec3<f32>, N : vec3<f32>) -> f32 {
  if (g.shadowParams.x < 0.5) { return 1.0; }
  let lp = g.lightViewProj * vec4<f32>(worldPos + N * g.shadowParams.z, 1.0);
  let ndc = lp.xyz / lp.w;
  let uv = ndc.xy * vec2<f32>(0.5, -0.5) + vec2<f32>(0.5, 0.5);
  let inb = uv.x >= 0.0 && uv.x <= 1.0 && uv.y >= 0.0 && uv.y <= 1.0 && ndc.z >= 0.0 && ndc.z <= 1.0;
  let cuv = clamp(uv, vec2<f32>(0.0), vec2<f32>(1.0));
  let s = textureSampleCompare(shadowMap, shadowSamp, cuv, ndc.z - g.shadowParams.y);
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

@fragment
fn fs_main(in : VSOut) -> @location(0) vec4<f32> {
  var albedo    = d.albedoMetallic.rgb * in.color;
  let metallic  = clamp(d.albedoMetallic.a, 0.0, 1.0);
  let roughness = clamp(d.emissionRough.a, 0.04, 1.0);
  let emission  = d.emissionRough.rgb;

  let N = normalize(in.worldNormal);
  let V = normalize(g.cameraPosition.xyz - in.worldPos);

  let rawFlags = d.surfaceFlags.y;
  if ((rawFlags & 1u) != 0u) { albedo = applyCheckerboard(albedo, in.worldPos); }
  let surfaceId = d.surfaceFlags.x;
  if (surfaceId != 0u) { albedo = applySurface(surfaceId, albedo, in.worldPos, N, in.uv); }

  let f0 = mix(vec3<f32>(0.04), albedo, metallic);
  let NdotV = max(dot(N, V), 1e-4);

  let sunShadow = computeShadow(in.worldPos, N);
  let direct = evaluateLighting(in.worldPos, N, V, albedo, metallic, roughness, f0, sunShadow);

  // IBL from the procedural sky (analytic split-sum).
  let R = reflect(-V, N);
  let irradiance = sampleEnvironment(N);
  let prefiltered = mix(sampleEnvironment(R), irradiance, roughness);
  let Famb = fresnelRoughness(NdotV, f0, roughness);
  let kd = (vec3<f32>(1.0) - Famb) * (1.0 - metallic);
  let envDiffuse = kd * albedo * irradiance;
  let envSpecular = prefiltered * envBRDFApprox(f0, roughness, NdotV);
  let ambient = (envDiffuse + envSpecular) * g.ambient.rgb;

  // Debug views (Renderer::debugView via counts.y) write display-ready values.
  let dbg = g.counts.y;
  if (dbg == 5) { return vec4<f32>(vec3<f32>(sunShadow), 1.0); }   // white=lit, black=shadowed
  if (dbg == 4) { return vec4<f32>(N * 0.5 + 0.5, 1.0); }
  if (dbg == 6) { return vec4<f32>(pow(albedo, vec3<f32>(1.0 / 2.2)), 1.0); }
  if (dbg == 7) {
    let ndv = dot(N, V);
    if (ndv >= 0.0) { return vec4<f32>(0.0, ndv, 0.0, 1.0); }
    return vec4<f32>(-ndv, 0.0, 0.0, 1.0);
  }
  if (dbg == 3) {
    let vd = -(g.view * vec4<f32>(in.worldPos, 1.0)).z;
    let lin = clamp(1.0 - vd / 200.0, 0.0, 1.0);
    return vec4<f32>(vec3<f32>(lin), 1.0);
  }

  var color = direct + ambient + emission;
  // Aerial-perspective fog (lerp toward fog color by 1-exp(-density*dist)).
  if (g.fog.w > 0.0) {
    let dist = length(in.worldPos - g.cameraPosition.xyz);
    let f = 1.0 - exp(-g.fog.w * dist);
    color = mix(color, g.fog.rgb, f);
  }
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
        createShadowResources();

        LOG_INFO("WebGPU backend initialized (%dx%d)", width_, height_);
        return true;
    }

    void shutdown() override {
        releaseDepthTarget();
        if (bindGroup_) { wgpuBindGroupRelease(bindGroup_); bindGroup_ = nullptr; }
        if (globalBuf_) { wgpuBufferRelease(globalBuf_); globalBuf_ = nullptr; }
        if (drawBuf_)   { wgpuBufferRelease(drawBuf_);   drawBuf_ = nullptr; }
        if (pipeline_)  { wgpuRenderPipelineRelease(pipeline_); pipeline_ = nullptr; }
        if (shadowPipeline_) { wgpuRenderPipelineRelease(shadowPipeline_); shadowPipeline_ = nullptr; }
        if (shadowSampleGroup_) { wgpuBindGroupRelease(shadowSampleGroup_); shadowSampleGroup_ = nullptr; }
        if (shadowSampler_) { wgpuSamplerRelease(shadowSampler_); shadowSampler_ = nullptr; }
        if (shadowView_) { wgpuTextureViewRelease(shadowView_); shadowView_ = nullptr; }
        if (shadowTexture_) { wgpuTextureRelease(shadowTexture_); shadowTexture_ = nullptr; }
        if (shadowSampleLayout_) { wgpuBindGroupLayoutRelease(shadowSampleLayout_); shadowSampleLayout_ = nullptr; }
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
        packMat4(view, globals_.view);
        cameraEye_ = camera.position;
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
        set4(globals_.skySunColor, (float)sky.sunColor.x, (float)sky.sunColor.y, (float)sky.sunColor.z, 0.0f);
        set4(globals_.skyZenith, (float)sky.zenithColor.x, (float)sky.zenithColor.y, (float)sky.zenithColor.z, 0.0f);
        set4(globals_.skyHorizon, (float)sky.horizonColor.x, (float)sky.horizonColor.y, (float)sky.horizonColor.z, 0.0f);
        set4(globals_.skyGround, (float)sky.groundColor.x, (float)sky.groundColor.y, (float)sky.groundColor.z, 0.0f);

        // Aerial-perspective fog.
        const FogParams& fog = lighting.fog;
        float density = fog.enabled ? fog.density : 0.0f;
        set4(globals_.fog, (float)fog.color.x, (float)fog.color.y, (float)fog.color.z, density);

        // Lights: sun first (directional), then point, then spot, up to 32.
        // Sun shadow config (single cascade). Driven off the level's ShadowConfig
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
        qd.data.surfaceFlags[2] = 0;
        qd.data.surfaceFlags[3] = 0;
        draws_.push_back(qd);
    }

    void endFrame() override {
        if (!device_ || !surface_) return;

        // Single-cascade sun shadow: a camera-centered orthographic box fit along
        // the sun direction (a simplified one-cascade version of the Metal/Vulkan
        // cascade fit). Written into globals before the upload below.
        if (shadowOn_) {
            Real radius = shadowDistance_ * 0.5;
            Vec3 up = std::abs(sunDir_.y) > 0.99 ? Vec3(0, 0, 1) : Vec3(0, 1, 0);
            Real pullback = radius + 50.0;
            Mat4 lightView = Mat4::lookAt(cameraEye_ + sunDir_ * pullback, cameraEye_, up);
            Mat4 lightProj = Mat4::orthographic(radius * 2.0, 1.0, 0.1, pullback + radius);
            packMat4(lightProj * lightView, globals_.lightViewProj);
            globals_.shadowParams[0] = 1.0f;
            globals_.shadowParams[1] = shadowDepthBias_;
            globals_.shadowParams[2] = shadowNormalBias_;
            globals_.shadowParams[3] = shadowPcf_;
        } else {
            globals_.shadowParams[0] = 0.0f;
        }

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

        // Shadow depth pass (sun) into the shadow map, before the main pass.
        if (shadowOn_ && !draws_.empty()) {
            WGPURenderPassDepthStencilAttachment sdepth = {};
            sdepth.view = shadowView_;
            sdepth.depthLoadOp = WGPULoadOp_Clear;
            sdepth.depthStoreOp = WGPUStoreOp_Store;
            sdepth.depthClearValue = 1.0f;
            WGPURenderPassDescriptor sPass = {};
            sPass.colorAttachmentCount = 0;
            sPass.depthStencilAttachment = &sdepth;
            WGPURenderPassEncoder spass = wgpuCommandEncoderBeginRenderPass(encoder, &sPass);
            wgpuRenderPassEncoderSetPipeline(spass, shadowPipeline_);
            for (size_t i = 0; i < draws_.size(); ++i) {
                const GpuMesh& m = meshes_[draws_[i].mesh];
                uint32_t dynOffset = static_cast<uint32_t>(i * kDrawStride);
                wgpuRenderPassEncoderSetBindGroup(spass, 0, bindGroup_, 1, &dynOffset);
                wgpuRenderPassEncoderSetVertexBuffer(spass, 0, m.vertexBuffer, 0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderSetIndexBuffer(spass, m.indexBuffer, WGPUIndexFormat_Uint32,
                                                    0, WGPU_WHOLE_SIZE);
                wgpuRenderPassEncoderDrawIndexed(spass, m.indexCount, 1, 0, 0, 0);
            }
            wgpuRenderPassEncoderEnd(spass);
            wgpuRenderPassEncoderRelease(spass);
        }

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
        wgpuRenderPassEncoderSetBindGroup(pass, 1, shadowSampleGroup_, 0, nullptr);

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

        // Group 1 (main pass only): the sun shadow map + a comparison sampler.
        WGPUBindGroupLayoutEntry shEntries[2] = {};
        shEntries[0].binding = 0;
        shEntries[0].visibility = WGPUShaderStage_Fragment;
        shEntries[0].texture.sampleType = WGPUTextureSampleType_Depth;
        shEntries[0].texture.viewDimension = WGPUTextureViewDimension_2D;
        shEntries[1].binding = 1;
        shEntries[1].visibility = WGPUShaderStage_Fragment;
        shEntries[1].sampler.type = WGPUSamplerBindingType_Comparison;
        WGPUBindGroupLayoutDescriptor shblDesc = {};
        shblDesc.entryCount = 2;
        shblDesc.entries = shEntries;
        shadowSampleLayout_ = wgpuDeviceCreateBindGroupLayout(device_, &shblDesc);

        WGPUBindGroupLayout mainGroups[2] = {bindLayout_, shadowSampleLayout_};
        WGPUPipelineLayoutDescriptor plDesc = {};
        plDesc.bindGroupLayoutCount = 2;
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

        // Shadow pipeline: depth-only (no fragment), vs_shadow, group 0 only.
        WGPUPipelineLayoutDescriptor splDesc = {};
        splDesc.bindGroupLayoutCount = 1;
        splDesc.bindGroupLayouts = &bindLayout_;
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

        wgpuShaderModuleRelease(module);
        if (!pipeline_ || !shadowPipeline_) {
            LOG_ERROR("WebGPU: render pipeline creation failed");
            return false;
        }
        return true;
    }

    void createShadowResources() {
        WGPUTextureDescriptor td = {};
        td.usage = WGPUTextureUsage_RenderAttachment | WGPUTextureUsage_TextureBinding;
        td.dimension = WGPUTextureDimension_2D;
        td.size = {static_cast<uint32_t>(kShadowMapSize), static_cast<uint32_t>(kShadowMapSize), 1};
        td.format = kShadowFormat;
        td.mipLevelCount = 1;
        td.sampleCount = 1;
        shadowTexture_ = wgpuDeviceCreateTexture(device_, &td);
        shadowView_ = wgpuTextureCreateView(shadowTexture_, nullptr);

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
        entries[0].textureView = shadowView_;
        entries[1].binding = 1;
        entries[1].sampler = shadowSampler_;
        WGPUBindGroupDescriptor bgDesc = {};
        bgDesc.layout = shadowSampleLayout_;
        bgDesc.entryCount = 2;
        bgDesc.entries = entries;
        shadowSampleGroup_ = wgpuDeviceCreateBindGroup(device_, &bgDesc);
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

    // Sun shadow (single cascade): a depth map + its own pipeline; the main
    // pipeline samples it via group 1.
    static constexpr int kShadowMapSize = 2048;
    WGPUTexture shadowTexture_ = nullptr;
    WGPUTextureView shadowView_ = nullptr;
    WGPUSampler shadowSampler_ = nullptr;
    WGPUBindGroupLayout shadowSampleLayout_ = nullptr;  // group 1 (tex + comparison sampler)
    WGPUBindGroup shadowSampleGroup_ = nullptr;
    WGPURenderPipeline shadowPipeline_ = nullptr;
    Vec3 cameraEye_;
    Vec3 sunDir_{0, 1, 0};
    bool shadowOn_ = false;
    float shadowDistance_ = 150.0f, shadowDepthBias_ = 0.0015f,
          shadowNormalBias_ = 0.04f, shadowPcf_ = 1.0f;

    std::vector<GpuMesh> meshes_;
    std::vector<uint32_t> freeSlots_;
    uint32_t generationCounter_ = 0;
    uint32_t textureCounter_ = 0;

    std::vector<QueuedDraw> draws_;
    bool frameDiagLogged_ = false;
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
