// Volumetric clouds — WebGPU port of the Vulkan clouds.frag (procedural-planet-plan
// P4). A shared density + lighting + march core over two domains selected by
// layer.w: 0 = SLAB (the cloudscape over a ground scene — the procgen city/terrain
// sky, depth-occluded) and 1 = SHELL (a planet's cloud deck from space). One shader
// for the city AND the planet; supersedes the flat 2D FBM sky clouds. Density is a
// compact inline hash-fbm (a 3D Perlin-Worley texture set is the on-device upgrade);
// lighting is a sun light-march (Beer + Powder) with a Henyey-Greenstein phase.
// Output is scene-linear HDR; the composite pass tone maps. (Mirrors the SPIR-V-
// verified GLSL; no WGSL validator in CI, so pipeline wiring is the on-device step.)

struct Clouds {
  invViewProjection : mat4x4<f32>,
  cameraPosition    : vec4<f32>,
  sunDirection      : vec4<f32>,
  sunColor          : vec4<f32>,   // rgb, w intensity
  skyAmbient        : vec4<f32>,   // rgb ambient, w time (drift)
  planetCenter      : vec4<f32>,
  layer             : vec4<f32>,   // x bottom, y top, z planetRadius, w domainMode
  params            : vec4<f32>,   // x coverage, y densityScale, z noiseScale, w windSpeed
  march             : vec4<f32>,   // x viewSteps, y lightSteps, z phaseG, w farDistance
};

@group(0) @binding(0) var sceneTex   : texture_2d<f32>;
@group(0) @binding(1) var sceneSamp  : sampler;
@group(0) @binding(2) var depthTex   : texture_2d<f32>;
@group(0) @binding(3) var<uniform> c : Clouds;

const PI : f32 = 3.14159265359;

fn hash13(p0 : vec3<f32>) -> f32 {
  var p = fract(p0 * 0.3183099 + vec3<f32>(0.1, 0.2, 0.3));
  p = p * 17.0;
  return fract(p.x * p.y * p.z * (p.x + p.y + p.z));
}

fn valueNoise(p : vec3<f32>) -> f32 {
  let i = floor(p);
  var f = fract(p);
  f = f * f * (3.0 - 2.0 * f);
  let n000 = hash13(i + vec3<f32>(0.0, 0.0, 0.0));
  let n100 = hash13(i + vec3<f32>(1.0, 0.0, 0.0));
  let n010 = hash13(i + vec3<f32>(0.0, 1.0, 0.0));
  let n110 = hash13(i + vec3<f32>(1.0, 1.0, 0.0));
  let n001 = hash13(i + vec3<f32>(0.0, 0.0, 1.0));
  let n101 = hash13(i + vec3<f32>(1.0, 0.0, 1.0));
  let n011 = hash13(i + vec3<f32>(0.0, 1.0, 1.0));
  let n111 = hash13(i + vec3<f32>(1.0, 1.0, 1.0));
  return mix(mix(mix(n000, n100, f.x), mix(n010, n110, f.x), f.y),
             mix(mix(n001, n101, f.x), mix(n011, n111, f.x), f.y), f.z);
}

fn fbm(p0 : vec3<f32>) -> f32 {
  var p = p0;
  var sum = 0.0;
  var amp = 0.5;
  for (var i = 0; i < 5; i = i + 1) {
    sum = sum + valueNoise(p) * amp;
    p = p * 2.02;
    amp = amp * 0.5;
  }
  return sum;
}

fn layerHeight(p : vec3<f32>) -> f32 {
  var h : f32;
  if (c.layer.w < 0.5) { h = p.y; }
  else { h = length(p - c.planetCenter.xyz) - c.layer.z; }
  return clamp((h - c.layer.x) / max(1e-4, c.layer.y - c.layer.x), 0.0, 1.0);
}

fn cloudDensity(p : vec3<f32>) -> f32 {
  let hf = layerHeight(p);
  if (hf <= 0.0 || hf >= 1.0) { return 0.0; }
  let profile = smoothstep(0.0, 0.15, hf) * smoothstep(1.0, 0.55, hf);
  let wind = vec3<f32>(c.params.w * c.skyAmbient.w, 0.0, 0.0);
  let q = (p + wind) * c.params.z;
  let base = fbm(q);
  var d = clamp((base - (1.0 - c.params.x)) / max(1e-3, c.params.x), 0.0, 1.0);
  let detail = fbm(q * 3.1 + vec3<f32>(4.0));
  d = clamp(d - detail * 0.35 * (1.0 - d), 0.0, 1.0);
  return d * profile * c.params.y;
}

fn phaseHG(mu : f32, g : f32) -> f32 {
  let g2 = g * g;
  return (1.0 - g2) / (4.0 * PI * pow(1.0 + g2 - 2.0 * g * mu, 1.5));
}

struct Interval { hit : bool, t0 : f32, t1 : f32 };

fn layerInterval(origin : vec3<f32>, dir : vec3<f32>) -> Interval {
  var r : Interval;
  r.hit = false;
  if (c.layer.w < 0.5) {
    if (abs(dir.y) < 1e-5) {
      if (origin.y < c.layer.x || origin.y > c.layer.y) { return r; }
      r.t0 = 0.0; r.t1 = c.march.w; r.hit = true; return r;
    }
    let ta = (c.layer.x - origin.y) / dir.y;
    let tb = (c.layer.y - origin.y) / dir.y;
    r.t0 = max(min(ta, tb), 0.0);
    r.t1 = max(ta, tb);
    r.hit = r.t1 > 0.0;
    return r;
  }
  let oc = origin - c.planetCenter.xyz;
  let b = dot(oc, dir);
  let cOut = dot(oc, oc) - c.layer.y * c.layer.y;
  let discOut = b * b - cOut;
  if (discOut < 0.0) { return r; }
  let sOut = sqrt(discOut);
  let outerFar = -b + sOut;
  if (outerFar < 0.0) { return r; }
  r.t0 = max(-b - sOut, 0.0);
  r.t1 = outerFar;
  let cIn = dot(oc, oc) - c.layer.x * c.layer.x;
  let discIn = b * b - cIn;
  if (discIn > 0.0) {
    let innerNear = -b - sqrt(discIn);
    if (innerNear > 0.0) { r.t1 = min(r.t1, innerNear); }
  }
  r.hit = r.t1 > r.t0;
  return r;
}

fn sceneDistance(uv : vec2<f32>, camPos : vec3<f32>, dir : vec3<f32>) -> f32 {
  let depth = textureSampleLevel(depthTex, sceneSamp, uv, 0.0).r;
  if (depth <= 0.0) { return c.march.w; }
  let world = c.invViewProjection * vec4<f32>(uv * 2.0 - 1.0, depth, 1.0);
  return length(world.xyz / world.w - camPos);
}

struct VSOut { @builtin(position) pos : vec4<f32>, @location(0) uv : vec2<f32> };

@vertex
fn vs_main(@builtin(vertex_index) vi : u32) -> VSOut {
  var o : VSOut;
  let uv = vec2<f32>(f32((vi << 1u) & 2u), f32(vi & 2u));
  o.uv = uv;
  o.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
  return o;
}

@fragment
fn fs_main(@location(0) inUV : vec2<f32>) -> @location(0) vec4<f32> {
  let world = c.invViewProjection * vec4<f32>(inUV * 2.0 - 1.0, 1.0, 1.0);
  let camPos = c.cameraPosition.xyz;
  let dir = normalize(world.xyz / world.w - camPos);
  let sunDir = normalize(c.sunDirection.xyz);
  let scene = textureSample(sceneTex, sceneSamp, inUV).rgb;

  let iv = layerInterval(camPos, dir);
  if (!iv.hit) { return vec4<f32>(scene, 1.0); }
  let t0 = iv.t0;
  let t1 = min(iv.t1, sceneDistance(inUV, camPos, dir));
  if (t1 <= t0) { return vec4<f32>(scene, 1.0); }

  let viewSteps = i32(c.march.x);
  let lightSteps = i32(c.march.y);
  let ds = (t1 - t0) / f32(max(2, viewSteps));
  let mu = dot(dir, sunDir);
  let phase = phaseHG(mu, c.march.z);

  var transmittance = 1.0;
  var scattered = vec3<f32>(0.0);
  let sunLight = c.sunColor.rgb * c.sunColor.w;

  for (var i = 0; i < viewSteps; i = i + 1) {
    let p = camPos + dir * (t0 + ds * (f32(i) + 0.5));
    let density = cloudDensity(p);
    if (density > 0.001) {
      var lightOD = 0.0;
      let lds = (c.layer.y - c.layer.x) / f32(max(1, lightSteps));
      for (var j = 0; j < lightSteps; j = j + 1) {
        let lp = p + sunDir * (lds * (f32(j) + 0.5));
        lightOD = lightOD + cloudDensity(lp) * lds;
      }
      let beer = exp(-lightOD);
      let powder = 1.0 - exp(-2.0 * density * ds);
      let sunTerm = sunLight * beer * phase;
      let lum = (sunTerm * powder + c.skyAmbient.rgb) * density;
      let stepT = exp(-density * ds);
      scattered = scattered + transmittance * lum * ds;
      transmittance = transmittance * stepT;
      if (transmittance < 0.01) { break; }
    }
  }

  return vec4<f32>(scene * transmittance + scattered, 1.0);
}
