// SSR: screen-space reflections. Reconstruct position/normal/roughness from
// depth + G-buffer, march the reflection ray, and on a depth hit sample the HDR
// color. Confidence folds in fresnel, roughness, and a screen-edge fade. The
// composite adds it. All textureLoad (no uniform-control-flow constraint).
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
  var prev = pos;                    // last position in front of geometry
  var hitUV = vec2<f32>(0.0);
  var hit = false;
  for (var i = 0; i < steps; i = i + 1) {
    prev = pos;
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
    // Accept within thickness + one step: a thinner window misses geometry
    // that falls between coarse samples and reads as alternating hit/miss
    // bands (matches ssr.frag).
    if (rayDist > sceneDist && (rayDist - sceneDist) < s.params.y + stepLen) {
      // Binary-refine the crossing between the last in-front position and
      // this behind position (port of ssr.frag's refine loop). The coarse
      // fixed world-step quantises the hit to step boundaries, which makes a
      // tall reflection look repeated / laddered ("stripes"); a few
      // bisections localise it to the true surface.
      var a = prev;
      var b = pos;
      hitUV = suv;
      for (var j = 0; j < 5; j = j + 1) {
        let mid = 0.5 * (a + b);
        let mc = s.viewProj * vec4<f32>(mid, 1.0);
        if (mc.w <= 0.0) { break; }
        let mndc = mc.xyz / mc.w;
        let muv = vec2<f32>(mndc.x * 0.5 + 0.5, 0.5 - mndc.y * 0.5);
        if (muv.x < 0.0 || muv.x > 1.0 || muv.y < 0.0 || muv.y > 1.0) { break; }
        let md = textureLoad(depthTex, vec2<i32>(muv * s.texel.zw), 0);
        if (md >= 1.0) { a = mid; continue; }        // sky: midpoint is in front
        let mP = reconW(muv, md);
        if (length(s.camPos.xyz - mid) > length(s.camPos.xyz - mP)) {
          b = mid;                                   // behind geometry: pull far end in
          hitUV = muv;
        } else {
          a = mid;                                   // in front: pull near end in
        }
      }
      hit = true;
      break;
    }
  }
  if (!hit) { return vec4<f32>(0.0); }
  let refl = textureLoad(hdrTex, vec2<i32>(hitUV * s.texel.zw), 0).rgb;
  let fres = pow(1.0 - max(dot(N, V), 0.0), 4.0);
  let edge = smoothstep(0.0, 0.15, min(min(hitUV.x, 1.0 - hitUV.x), min(hitUV.y, 1.0 - hitUV.y)));
  let conf = (1.0 - roughness / max(s.params.w, 1e-3)) * (0.2 + 0.8 * fres) * edge;
  return vec4<f32>(refl * conf, conf);
}
