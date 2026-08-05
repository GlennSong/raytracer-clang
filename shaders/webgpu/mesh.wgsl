// The forward mesh shader. Ported from the Vulkan shaders/vulkan/
// mesh.{vert,frag} — multi-light Cook-Torrance, the analytic procedural surface
// library (brick/concrete/asphalt/...), texture/material maps, cascaded shadows,
// IBL (procedural-sky or a bound HDR equirect) with a baked split-sum BRDF LUT,
// fog, checkerboard, and debug views. Embedded as a string (matches the Metal
// backend; the web has no offline compile). Scene-linear with sRGB on store.
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

// ---- procedural surface library (ported from surfaces_*.metal / mesh.frag) --
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
  let shade = clamp(0.86 + 0.20 * (spk - 0.5) + 0.10 * (blotch - 0.5), 0.6, 1.02);
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
fn surfRoadMarkings(base : vec3<f32>, mu : f32, mv : f32, wu : f32, wv : f32) -> vec3<f32> {
  // One surface id covers the whole welded road, split by road-local mu: the
  // sidewalk/curb band (mu in [0,1]) wears concrete pavement, the carriageway
  // (mu in [1,3]) asphalt grain under the lane paint. wu/wv = the world-planar
  // UV the other surfaces tile by. Mirrors scene.cpp / Metal / Vulkan.
  if (mu < 0.98) {
    // Sidewalk band: concrete grain + scoring joints that FOLLOW the curb (slab
    // tops bake u = -(metres along the kerb loop)). Curb faces (u = 0) stay
    // plain. Mirrors scene.cpp / Metal / Vulkan.
    let spk = vnoise2(wu * 26.0, wv * 26.0);
    var c = base * (0.92 + 0.10 * (spk - 0.5));
    let su = -mu;
    if (su > 0.02) {
      let t = tile1(su, 1.5);
      let jd = min(t, 1.5 - t);
      c = c * (0.68 + 0.32 * smoothstep(0.02, 0.06, jd));
    }
    return c;
  }
  let deck = surfAsphalt(base, wu, wv);
  // Dashed lane DIVIDER strip (u = 4, v = raw arc-length): one thin strip per
  // internal same-direction lane boundary on multilane roads; 3 m of white
  // paint every 7.5 m. Mirrors scene.cpp / Metal / Vulkan.
  if (mu > 3.5) {
    if (fract(mv / 7.5) < 0.4) { return vec3<f32>(0.86, 0.86, 0.83); }
    return deck;
  }
  let lat = mu - 2.0;
  let yL = 1.0 - smoothstep(0.013, 0.019, abs(lat - 0.030));
  let yR = 1.0 - smoothstep(0.013, 0.019, abs(lat + 0.030));
  var y = max(yL, yR);
  let wL = 1.0 - smoothstep(0.016, 0.022, abs(lat - 0.86));
  let wR = 1.0 - smoothstep(0.016, 0.022, abs(lat + 0.86));
  let w = max(wL, wR);
  // The centreline ENDS before a crosswalk (device: the double yellow cut
  // through the zebra bars): the band ends at mv ~3.6, so the yellow fades in
  // just past it. Without crosswalks mv is a large sentinel (full-length line).
  y = y * smoothstep(4.0, 4.8, mv);
  var c = mix(deck, vec3<f32>(0.82, 0.68, 0.13), y);
  c = mix(c, vec3<f32>(0.86, 0.86, 0.83), w);
  // Zebra crosswalk painted into the road texture (ADR-0062): mv = metres PAST
  // the junction mouth (baked by the road mesher), so the band sits set back on
  // the approach, not in the intersection. Bars run across the carriageway (mu).
  // Mirror of Metal (shaders/metal/surface_road.metal) and Vulkan (mesh.frag) for
  // backend parity. Gate on mu > 1.05: the raised curb shares this surface with
  // a 0..1 UV, and the band must never paint onto it.
  let cwEdge = smoothstep(0.5, 0.8, mv) * (1.0 - smoothstep(3.3, 3.6, mv));
  let bars = step(0.5, fract((mu - 1.0) * 4.0));
  c = mix(c, vec3<f32>(0.90, 0.90, 0.88), cwEdge * bars * step(1.05, mu));
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
  else if (id == 11u) { c = surfRoadMarkings(base, meshUV.x, meshUV.y, uv.x, uv.y); }
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
  var roughness = clamp(d.emissionRough.a * mrSample.g, 0.04, 1.0);   //       G=rough
  let emission  = d.emissionRough.rgb * emSample;

  var N = normalize(in.worldNormal);
  // Normal map (tangent-space -> world via TBN), only with a map + a real tangent.
  if ((maps & 2u) != 0u && length(in.worldTangent) > 0.001) {
    let T = normalize(in.worldTangent - N * dot(in.worldTangent, N));
    let B = cross(N, T);
    N = normalize(T * nmap.x + B * nmap.y + N * nmap.z);
  }
  // Road micro-relief (device: roads "don't look like a PBR texture"): the road
  // carries no baked normal/roughness maps (its mesh UV is road-local paint
  // space), so perturb the normal and vary the roughness procedurally from the
  // same world-planar noise the asphalt albedo tiles by. Subtle undulation +
  // sparkle-scale roughness break the uniform specular sheet the flat deck had.
  // Mirrors Metal (surface_road.metal, surfaceReliefRoad) / Vulkan (mesh.frag).
  if (d.surfaceFlags.x == 11u) {
    let rx = in.worldPos.x; let rz = in.worldPos.z;
    // Three octaves: metre-scale undulation, decimetre patching, and a
    // near-aggregate grain — summed finite differences tilt the normal hard
    // enough to read as bumpy asphalt (device: "no sense of bumpiness").
    let b0 = vnoise2(rx * 2.6, rz * 2.6) + 0.5 * vnoise2(rx * 11.0, rz * 11.0)
           + 0.3 * vnoise2(rx * 37.0, rz * 37.0);
    let bx = vnoise2(rx * 2.6 + 0.4, rz * 2.6) + 0.5 * vnoise2(rx * 11.0 + 1.7, rz * 11.0)
           + 0.3 * vnoise2(rx * 37.0 + 2.3, rz * 37.0) - b0;
    let bz = vnoise2(rx * 2.6, rz * 2.6 + 0.4) + 0.5 * vnoise2(rx * 11.0, rz * 11.0 + 1.7)
           + 0.3 * vnoise2(rx * 37.0, rz * 37.0 + 2.3) - b0;
    N = normalize(N + vec3<f32>(-bx, 0.0, -bz) * 0.6);
    let spk = vnoise2(rx * 23.0, rz * 23.0);
    roughness = clamp(roughness + (spk - 0.5) * 0.25, 0.55, 1.0);
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
