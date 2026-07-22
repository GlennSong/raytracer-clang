// Star / sun (procedural-planet-plan P6) — WebGPU port of star.frag. The local star
// as a bright HDR disc with limb darkening + a blackbody colour from its temperature,
// plus a bloom-friendly glow, composited over the background (gated on the reverse-Z
// far plane). Mirrors the SPIR-V-verified GLSL; pipeline wiring is the on-device step.

struct Star {
  invViewProjection : mat4x4<f32>,
  cameraPosition    : vec4<f32>,
  starDirection     : vec4<f32>,   // xyz toward the star, w angular radius (rad)
  tune              : vec4<f32>,   // x temperatureK, y intensity, z glowFalloff, w limbDark
};

@group(0) @binding(0) var sceneTex  : texture_2d<f32>;
@group(0) @binding(1) var sceneSamp : sampler;
@group(0) @binding(2) var depthTex  : texture_2d<f32>;
@group(0) @binding(3) var<uniform> s : Star;

fn blackbody(kelvin : f32) -> vec3<f32> {
  let t = clamp(kelvin, 1000.0, 40000.0) / 100.0;
  var r : f32;
  var g : f32;
  var b : f32;
  if (t <= 66.0) { r = 1.0; } else { r = clamp(1.292 * pow(t - 60.0, -0.1332), 0.0, 1.0); }
  if (t <= 66.0) { g = clamp(0.390 * log(t) - 0.631, 0.0, 1.0); }
  else { g = clamp(1.130 * pow(t - 60.0, -0.0755), 0.0, 1.0); }
  if (t >= 66.0) { b = 1.0; }
  else if (t <= 19.0) { b = 0.0; }
  else { b = clamp(0.543 * log(t - 10.0) - 1.196, 0.0, 1.0); }
  return vec3<f32>(r, g, b);
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
  let world = s.invViewProjection * vec4<f32>(inUV * 2.0 - 1.0, 1.0, 1.0);
  let dir = normalize(world.xyz / world.w - s.cameraPosition.xyz);
  let toStar = normalize(s.starDirection.xyz);
  let scene = textureSample(sceneTex, sceneSamp, inUV).rgb;

  let depth = textureSampleLevel(depthTex, sceneSamp, inUV, 0.0).r;
  if (depth > 0.0) { return vec4<f32>(scene, 1.0); }

  let ang = acos(clamp(dot(dir, toStar), -1.0, 1.0));
  let radius = max(1e-4, s.starDirection.w);
  let color = blackbody(s.tune.x);
  let intensity = s.tune.y;

  var add = vec3<f32>(0.0);
  if (ang < radius) {
    let r = ang / radius;
    let limb = mix(1.0, s.tune.w, r * r);
    add = color * intensity * limb;
  } else {
    let d = (ang - radius) / radius;
    add = color * intensity * exp(-d * s.tune.z);
  }
  return vec4<f32>(scene + add, 1.0);
}
