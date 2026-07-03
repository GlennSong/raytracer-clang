// Composite shader: HDR offscreen target -> swapchain. A fullscreen triangle
// reads the linear HDR scene texel-for-texel, applies any screen-space post
// (SSAO/SSR/bloom land here later), then the view transform (exposure -> grade
// -> ACES/AgX). Debug views are already display-ready, so they bypass the tone
// map (counts.y != 0). Its own module + group(0) layout, independent of the
// mesh pipeline.
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
