// Water surface (procedural-planet-plan P2) — WebGPU port of water.vert + water.frag.
// Animated ripple normals, a GGX sun glint, Schlick-Fresnel sky reflection, and
// depth-tinted transmission (uv.x = depth, uv.y = shore distance, the engine's
// Surface::Water convention) — so it serves a planet ocean AND city/coast water.
// Mirrors the SPIR-V-verified GLSL; the vertex-input + uniform layout must match the
// engine's mesh pipeline when wired on device (WGSL has no validator in CI).

struct Globals {
  viewProjection    : mat4x4<f32>,
  invViewProjection : mat4x4<f32>,
  cameraPosition    : vec4<f32>,
  sunDir            : vec4<f32>,   // xyz toward sun, w disc intensity
  sunColor          : vec4<f32>,
  skyZenith         : vec4<f32>,
  skyHorizon        : vec4<f32>,
  windTime          : vec4<f32>,   // x time
};
struct Draw {
  model        : mat4x4<f32>,
  deepColor    : vec4<f32>,   // rgb
  roughness    : vec4<f32>,   // x
};
@group(0) @binding(0) var<uniform> g : Globals;
@group(0) @binding(1) var<uniform> d : Draw;

const PI : f32 = 3.14159265359;

fn hash21(p0 : vec2<f32>) -> f32 {
  var p = fract(p0 * vec2<f32>(0.1031, 0.1030));
  p = p + dot(p, p.yx + 33.33);
  return fract((p.x + p.y) * p.x);
}
fn valueNoise2(p : vec2<f32>) -> f32 {
  let i = floor(p);
  var f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  let a = hash21(i);
  let b = hash21(i + vec2<f32>(1.0, 0.0));
  let c = hash21(i + vec2<f32>(0.0, 1.0));
  let e = hash21(i + vec2<f32>(1.0, 1.0));
  return mix(mix(a, b, f.x), mix(c, e, f.x), f.y);
}
fn rippleHeight(p : vec2<f32>, t : f32) -> f32 {
  let w = vec2<f32>(t * 0.6, t * 0.35);
  return valueNoise2(p * 1.7 + w) + 0.5 * valueNoise2(p * 3.9 - w * 1.3);
}
fn ggx(ndh : f32, rough : f32) -> f32 {
  let a = rough * rough;
  let a2 = a * a;
  let den = (ndh * ndh) * (a2 - 1.0) + 1.0;
  return a2 / max(1e-5, PI * den * den);
}

struct VSOut {
  @builtin(position) pos : vec4<f32>,
  @location(0) worldPos : vec3<f32>,
  @location(1) normal   : vec3<f32>,
  @location(2) uv       : vec2<f32>,
  @location(3) tangent  : vec3<f32>,
};

@vertex
fn vs_main(@location(0) inPos : vec3<f32>, @location(1) inNormal : vec3<f32>,
           @location(2) inTangent : vec3<f32>, @location(3) inUV : vec2<f32>,
           @location(4) inColor : vec3<f32>) -> VSOut {
  var o : VSOut;
  let world = d.model * vec4<f32>(inPos, 1.0);
  let nm = mat3x3<f32>(d.model[0].xyz, d.model[1].xyz, d.model[2].xyz);
  o.worldPos = world.xyz;
  o.normal = normalize(nm * inNormal);
  o.tangent = normalize(nm * inTangent);
  o.uv = inUV;
  o.pos = g.viewProjection * world;
  return o;
}

struct FSOut { @location(0) color : vec4<f32>, @location(1) normal : vec4<f32> };

@fragment
fn fs_main(in : VSOut) -> FSOut {
  let N0 = normalize(in.normal);
  let V = normalize(g.cameraPosition.xyz - in.worldPos);
  let L = normalize(g.sunDir.xyz);

  let T = normalize(in.tangent - N0 * dot(in.tangent, N0));
  let B = cross(N0, T);
  let t = g.windTime.x;
  let uvp = in.worldPos.xz * 0.15;
  let e = 0.35;
  let hL = rippleHeight(uvp - vec2<f32>(e, 0.0), t);
  let hR = rippleHeight(uvp + vec2<f32>(e, 0.0), t);
  let hD = rippleHeight(uvp - vec2<f32>(0.0, e), t);
  let hU = rippleHeight(uvp + vec2<f32>(0.0, e), t);
  let strength = 0.6;
  let N = normalize(N0 - T * (hR - hL) * strength - B * (hU - hD) * strength);

  let ndv = max(dot(N, V), 0.0);
  let fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);

  let depth = clamp(in.uv.x, 0.0, 1.0);
  let deep = d.deepColor.rgb;
  let shallow = mix(vec3<f32>(0.10, 0.45, 0.50), deep, 0.35);
  let body = mix(shallow, deep, depth);

  let R = reflect(-V, N);
  let upBlend = clamp(R.y * 0.5 + 0.5, 0.0, 1.0);
  let sky = mix(g.skyHorizon.rgb, g.skyZenith.rgb, pow(upBlend, 0.5));

  let H = normalize(L + V);
  let ndh = max(dot(N, H), 0.0);
  let ndl = max(dot(N, L), 0.0);
  let rough = max(0.02, d.roughness.x * 0.4);
  let spec = ggx(ndh, rough) * fres * ndl;
  let glint = g.sunColor.rgb * spec * g.sunDir.w;

  let shore = in.uv.y;
  let foamMask = smoothstep(0.12, 0.0, shore) *
                 step(0.5, valueNoise2(in.worldPos.xz * 0.6 + t * 0.5));
  let foam = vec3<f32>(0.9) * foamMask;

  var color = mix(body, sky, fres) + glint + foam;
  color = color * (0.25 + 0.9 * ndl);

  var o : FSOut;
  o.color = vec4<f32>(color, 1.0);
  o.normal = vec4<f32>(N * 0.5 + 0.5, rough);
  return o;
}
