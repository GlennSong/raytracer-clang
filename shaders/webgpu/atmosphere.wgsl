// Planetary atmosphere — WebGPU port of the CPU reference (engine/procgen/
// atmosphere.cpp) and the Vulkan atmosphere.frag, single-scattering Rayleigh + Mie
// (procedural-planet-plan P3). A fullscreen pass: reconstruct the world ray per
// pixel, raymarch the atmosphere shell, composite the in-scattered light over the
// HDR scene by its transmittance. The planet is an analytic sphere so the march
// clips at the surface without a depth fetch. Output is scene-linear HDR; the
// composite pass tone maps. Its own group(0) layout, independent of the mesh
// pipeline. (Verified against the SPIR-V build of atmosphere.frag; no WGSL
// validator in this environment, so the pipeline wiring is the on-device step.)

struct Atmosphere {
  invViewProjection : mat4x4<f32>,
  cameraPosition    : vec4<f32>,   // xyz
  sunDirection      : vec4<f32>,   // xyz toward the sun
  planetCenter      : vec4<f32>,   // xyz
  sunColor          : vec4<f32>,   // rgb, w = intensity
  rayleighCoeff     : vec4<f32>,   // rgb per length
  radii             : vec4<f32>,   // x planetRadius, y atmosphereRadius, z rayleighH, w mieH
  mie               : vec4<f32>,   // x mieCoeff, y mieG, z viewSamples, w lightSamples
};

@group(0) @binding(0) var sceneTex  : texture_2d<f32>;
@group(0) @binding(1) var sceneSamp : sampler;
@group(0) @binding(2) var<uniform> a : Atmosphere;

const PI : f32 = 3.14159265359;

struct RayHit { hit : bool, t0 : f32, t1 : f32 };

// t0<t1 where origin+t*dir meets the sphere of `radius` at the origin.
fn raySphere(origin : vec3<f32>, dir : vec3<f32>, radius : f32) -> RayHit {
  var r : RayHit;
  let b = dot(origin, dir);
  let c = dot(origin, origin) - radius * radius;
  let disc = b * b - c;
  if (disc < 0.0) { r.hit = false; return r; }
  let s = sqrt(disc);
  r.t0 = -b - s;
  r.t1 = -b + s;
  r.hit = r.t1 >= 0.0;
  return r;
}

fn densityAt(q : vec3<f32>, planetRadius : f32, scaleH : f32) -> f32 {
  let h = length(q) - planetRadius;
  return exp(-max(h, 0.0) / scaleH);
}

// Returns (odRayleigh, odMie) along pa->pb.
fn opticalDepth(pa : vec3<f32>, pb : vec3<f32>, steps : i32,
                planetRadius : f32, rH : f32, mH : f32) -> vec2<f32> {
  var od = vec2<f32>(0.0, 0.0);
  let seg = pb - pa;
  let len = length(seg);
  if (len < 1e-6) { return od; }
  let d = seg / len;
  let ds = len / f32(steps);
  for (var i = 0; i < steps; i = i + 1) {
    let q = pa + d * (ds * (f32(i) + 0.5));
    od.x = od.x + densityAt(q, planetRadius, rH) * ds;
    od.y = od.y + densityAt(q, planetRadius, mH) * ds;
  }
  return od;
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
  let world = a.invViewProjection * vec4<f32>(inUV * 2.0 - 1.0, 1.0, 1.0);
  let camPos = a.cameraPosition.xyz;
  let dir = normalize(world.xyz / world.w - camPos);
  let sunDir = normalize(a.sunDirection.xyz);

  let scene = textureSample(sceneTex, sceneSamp, inUV).rgb;

  let planetRadius = a.radii.x;
  let atmosRadius  = a.radii.y;
  let rH = a.radii.z;
  let mH = a.radii.w;
  let mieCoeff = a.mie.x;
  let mieG = a.mie.y;
  let viewSamples = i32(a.mie.z);
  let lightSamples = i32(a.mie.w);
  let rayleighCoeff = a.rayleighCoeff.rgb;

  let origin = camPos - a.planetCenter.xyz;

  let atmos = raySphere(origin, dir, atmosRadius);
  if (!atmos.hit) { return vec4<f32>(scene, 1.0); }

  var tMax = atmos.t1;
  let planet = raySphere(origin, dir, planetRadius);
  if (planet.hit && planet.t0 > 0.0) { tMax = min(tMax, planet.t0); }
  let tEnter = max(0.0, atmos.t0);
  if (tMax <= tEnter) { return vec4<f32>(scene, 1.0); }

  let steps = max(2, viewSamples);
  let ds = (tMax - tEnter) / f32(steps);

  let mu = dot(dir, sunDir);
  let phaseR = 3.0 / (16.0 * PI) * (1.0 + mu * mu);
  let g = mieG;
  let phaseM = 3.0 / (8.0 * PI) * ((1.0 - g * g) * (1.0 + mu * mu)) /
               ((2.0 + g * g) * pow(1.0 + g * g - 2.0 * g * mu, 1.5));

  var odViewR = 0.0;
  var odViewM = 0.0;
  var inscatR = vec3<f32>(0.0);
  var inscatM = vec3<f32>(0.0);

  for (var i = 0; i < steps; i = i + 1) {
    let t = tEnter + ds * (f32(i) + 0.5);
    let q = origin + dir * t;

    let dR = densityAt(q, planetRadius, rH) * ds;
    let dM = densityAt(q, planetRadius, mH) * ds;
    odViewR = odViewR + dR;
    odViewM = odViewM + dM;

    let sun = raySphere(q, sunDir, planetRadius);
    let shadowed = sun.hit && sun.t1 > 0.0 && sun.t0 > 0.0;
    if (!shadowed) {
      let la = raySphere(q, sunDir, atmosRadius);
      let lightExit = q + sunDir * max(0.0, la.t1);
      let odL = opticalDepth(q, lightExit, lightSamples, planetRadius, rH, mH);
      let tau = rayleighCoeff * (odViewR + odL.x) + vec3<f32>(mieCoeff * (odViewM + odL.y));
      let tr = exp(-tau);
      inscatR = inscatR + tr * dR;
      inscatM = inscatM + tr * dM;
    }
  }

  let rayleigh = rayleighCoeff * inscatR * phaseR;
  let mieTerm = inscatM * (mieCoeff * phaseM);
  let inScatter = a.sunColor.rgb * (rayleigh + mieTerm) * a.sunColor.w;

  let tauView = rayleighCoeff * odViewR + vec3<f32>(mieCoeff * odViewM);
  let viewTransmittance = exp(-tauView);

  return vec4<f32>(scene * viewTransmittance + inScatter, 1.0);
}
