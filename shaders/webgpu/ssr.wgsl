// SSR — port of Metal's ssrRayMarch (shaders/metal/post.metal), the proven
// implementation: a jittered SCREEN-SPACE DDA over the projected reflection
// ray, with perspective-correct depth interpolation, linear-eye-space
// comparisons, distance-aware thickness, and binary refinement.
//
// The previous version (a port of the unverified Vulkan ssr.frag) marched in
// world space with fixed steps: equal world steps project to huge screen jumps
// near the camera and sub-pixel jumps far away, which quantised hits into
// stripes and bound reflections to the wrong surfaces (warped geometry).
//
// Adaptations from the Metal kernel: fragment pass instead of compute; the ray
// is clipped in CLIP space (clip coords are affine in world position, so the
// lerp is exact — Metal clips in view space, which we don't carry); standard-Z
// depth (near 0, far 1) instead of reverse-Z; Y-flipped UVs.
struct SsrU {
  invVP : mat4x4<f32>,
  viewProj : mat4x4<f32>,
  camPos : vec4<f32>,
  params : vec4<f32>,   // x maxDist, y thickness, z thicknessFar, w maxRoughness
  params2 : vec4<f32>,  // x camNear, y camFar, z pixel stride
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

// Standard-Z NDC depth -> positive linear eye distance (near at z=0, far at
// z=1). Comparisons in this space are distance-true, so the thickness window
// is in real world units (the mirror of Metal's linearizeReverseZ).
fn linearizeZ(z : f32, near : f32, far : f32) -> f32 {
  return near * far / (far - z * (far - near));
}

fn fullPx(uv : vec2<f32>) -> vec2<i32> {
  let p = clamp(uv * s.texel.zw, vec2<f32>(0.0), s.texel.zw - 1.0);
  return vec2<i32>(p);
}

@vertex
fn vs_ssr(@builtin(vertex_index) vid : u32) -> @builtin(position) vec4<f32> {
  var p = array<vec2<f32>, 3>(vec2<f32>(-1.0, -1.0), vec2<f32>(3.0, -1.0), vec2<f32>(-1.0, 3.0));
  return vec4<f32>(p[vid], 0.0, 1.0);
}

@fragment
fn fs_ssr(@builtin(position) fc : vec4<f32>) -> @location(0) vec4<f32> {
  // fc is in the scaled SSR space; texel.xy = 1/effectRes -> uv, texel.zw =
  // full res to index the full-res depth/G-buffer/HDR.
  let uv = fc.xy * s.texel.xy;
  let depth = textureLoad(depthTex, fullPx(uv), 0);
  if (depth >= 1.0) { return vec4<f32>(0.0); }   // sky

  let gb = textureLoad(gbufTex, fullPx(uv), 0);
  let roughness = gb.w;
  // Roughness gate: fades to 0 at maxRoughness (rough surfaces scatter
  // reflections to nothing; skips the march for the common rough pixels).
  let reflectivity = 1.0 - smoothstep(0.0, s.params.w, roughness);
  if (reflectivity <= 0.0) { return vec4<f32>(0.0); }

  let N = normalize(gb.xyz);
  let P = reconW(uv, depth);
  let viewDir = normalize(P - s.camPos.xyz);   // camera -> surface
  let R = reflect(viewDir, N);

  // Project the ray segment; keep the far end in front of the camera.
  let startClip = s.viewProj * vec4<f32>(P, 1.0);
  if (startClip.w <= 0.0) { return vec4<f32>(0.0); }
  var maxRayDist = s.params.x;
  var endClip = s.viewProj * vec4<f32>(P + R * maxRayDist, 1.0);
  if (endClip.w <= 0.01) {
    let tClip = (startClip.w - 0.01) / (startClip.w - endClip.w);
    if (tClip <= 0.0) { return vec4<f32>(0.0); }   // straight at the camera
    let tc = tClip * 0.99;
    endClip = mix(startClip, endClip, tc);
    maxRayDist = maxRayDist * tc;
  }
  let startNDC = startClip.xyz / startClip.w;
  let endNDC = endClip.xyz / endClip.w;
  let startUV = vec2<f32>(startNDC.x * 0.5 + 0.5, 0.5 - startNDC.y * 0.5);
  let endUV   = vec2<f32>(endNDC.x * 0.5 + 0.5, 0.5 - endNDC.y * 0.5);

  let deltaUV = endUV - startUV;
  let deltaPixels = deltaUV * s.texel.zw;
  let pixelDist = max(abs(deltaPixels.x), abs(deltaPixels.y));
  if (pixelDist < 1.0) { return vec4<f32>(0.0); }  // too short on screen to march

  let stride = max(s.params2.z, 1.0);
  let stepCount = clamp(i32(pixelDist / stride), 1, 48);

  // Perspective-correct interpolation setup (screen-linear z/w and 1/w).
  let startInvW = 1.0 / startClip.w;
  let endInvW = 1.0 / endClip.w;
  let startZoW = startNDC.z * startInvW;
  let endZoW = endNDC.z * endInvW;
  let near = s.params2.x;
  let far = s.params2.y;

  // Per-pixel jitter: dithers residual step quantisation into noise instead
  // of coherent bands.
  let hash = fract(sin(dot(fc.xy, vec2<f32>(127.1, 311.7))) * 43758.5453);

  var hit = false;
  var hitUV = vec2<f32>(0.0);
  var hitDist = 0.0;
  for (var i = 0; i < 48; i = i + 1) {
    if (i >= stepCount) { break; }
    let t = (f32(i) + 1.0 + hash) / f32(stepCount);
    let sampleUV = startUV + deltaUV * t;
    if (sampleUV.x < 0.0 || sampleUV.x > 1.0 || sampleUV.y < 0.0 || sampleUV.y > 1.0) { break; }

    let sceneDepth = textureLoad(depthTex, fullPx(sampleUV), 0);
    if (sceneDepth >= 1.0) { continue; }   // sky pixel: keep marching

    let invW = mix(startInvW, endInvW, t);
    let rayDepth = mix(startZoW, endZoW, t) / invW;
    let rayLinZ = linearizeZ(rayDepth, near, far);
    let sceneLinZ = linearizeZ(sceneDepth, near, far);
    let depthDiff = rayLinZ - sceneLinZ;   // > 0: the ray passed behind the surface

    // Distance-aware surface thickness, in world/eye units.
    let thicknessWorld = mix(s.params.y, s.params.z, clamp(rayLinZ / 30.0, 0.0, 1.0));

    if (depthDiff > 0.0 && depthDiff < thicknessWorld) {
      // Binary refinement in t — linear-eye comparison, no matrix math.
      var tLo = (f32(i) + hash) / f32(stepCount);
      var tHi = t;
      var bestUV = sampleUV;
      for (var b = 0; b < 6; b = b + 1) {
        let tMid = (tLo + tHi) * 0.5;
        let midUV = clamp(startUV + deltaUV * tMid, vec2<f32>(0.0), vec2<f32>(1.0));
        let midInvW = mix(startInvW, endInvW, tMid);
        let midRayZ = mix(startZoW, endZoW, tMid) / midInvW;
        let midRayLinZ = linearizeZ(midRayZ, near, far);
        let midSceneLinZ = linearizeZ(textureLoad(depthTex, fullPx(midUV), 0), near, far);
        if (midRayLinZ > midSceneLinZ) {   // behind surface -> pull closer
          tHi = tMid;
          bestUV = midUV;
        } else {
          tLo = tMid;
        }
      }
      hit = true;
      hitUV = bestUV;
      let finalT = (tLo + tHi) * 0.5;
      let finalInvW = mix(startInvW, endInvW, finalT);
      let finalRayZ = mix(startZoW, endZoW, finalT) / finalInvW;
      hitDist = abs(linearizeZ(finalRayZ, near, far) - linearizeZ(startNDC.z, near, far));
      break;
    }
  }
  if (!hit) { return vec4<f32>(0.0); }

  let refl = textureLoad(hdrTex, fullPx(hitUV), 0).rgb;

  // Confidence: screen-edge fade x travel-distance fade x grazing-angle
  // emphasis (high floor — SSR has no per-pixel metalness, a raw Schlick term
  // would crush head-on reflections) x the roughness gate. The composite
  // multiplies in blendStrength (effects.z).
  let edge2 = smoothstep(vec2<f32>(0.0), vec2<f32>(0.05), hitUV)
            * smoothstep(vec2<f32>(0.0), vec2<f32>(0.05), vec2<f32>(1.0) - hitUV);
  var conf = edge2.x * edge2.y;
  conf = conf * (1.0 - clamp(hitDist / maxRayDist, 0.0, 1.0));
  let NdotV = clamp(dot(N, -viewDir), 0.0, 1.0);
  let fres = mix(0.7, 1.0, pow(1.0 - NdotV, 5.0));
  conf = conf * fres * reflectivity;
  return vec4<f32>(refl * conf, conf);
}
