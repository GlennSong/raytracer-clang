// BRDF integration LUT bake (ADR-0057). Ports shaders/vulkan/brdf_lut.frag /
// environment.metal integrateBRDF: for each (NdotV, roughness) texel, GGX
// importance-sample the split-sum specular BRDF into (scale, bias). Rendered
// once at init into an RG16Float texture; the mesh shader samples it for the
// envSpecular term. No inputs — pure numeric integration.
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
