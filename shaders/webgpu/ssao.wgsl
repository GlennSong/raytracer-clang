// SSAO: reconstruct world position + a geometric normal (from depth derivatives)
// in a fullscreen pass, sample a hemisphere kernel oriented by the normal, and
// accumulate occlusion against the scene depth. Output is a single-channel AO
// the composite multiplies in. Depth-based (no G-buffer normal) for simplicity.
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
