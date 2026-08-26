#version 450
// Procedural day/night sky + FBM clouds (ported from environment.metal
// sampleEnvironment + applyClouds). Output is scene-linear; the sRGB swapchain
// encodes on store. Without the Phase 5 tone map the bright sun disc clips to
// white — expected until the composite tonemap lands.

struct Light {
    vec4 positionIntensity;
    vec4 directionInner;
    vec4 colorOuter;
    vec4 typeRange;
};
layout(set = 0, binding = 0) uniform Globals {
    mat4  viewProjection;
    mat4  view;
    mat4  invViewProjection;
    mat4  cascadeVP[4];
    vec4  cameraPosition;
    vec4  ambient;
    vec4  cascadeSplit;
    ivec4 counts;
    vec4  shadowParams;
    vec4  skySunDir;       // xyz dir, w disc intensity
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;        // x coverage, y density, z scale, w time
    Light lights[32];
    vec4  fog;             // (appended after lights — see GlobalsUBO)
    vec4  shadowTint;
    vec4  wind1;
    vec4  wind2;
    vec4  skyMoonDir;      // xyz toward the moon, w disc radiance (0 = down)
    vec4  skyMoonSun;      // xyz TRUE sun direction (lights the disc), w lit fraction
} g;

layout(set = 0, binding = 2) uniform sampler2D envEquirect;   // HDR env (counts.z==1)

layout(location = 0) in vec3 inViewDir;
layout(location = 0) out vec4 outColor;

vec3 sampleEquirect(vec3 dir) {
    const float PI = 3.14159265359;
    float u = atan(dir.z, dir.x) * (0.5 / PI) + 0.5;
    float v = acos(clamp(dir.y, -1.0, 1.0)) * (1.0 / PI);
    return texture(envEquirect, vec2(u, v)).rgb;
}

// THE MOON (the month): a sphere disc lit from the TRUE sun, so the terminator
// falls where the phase says — a thin crescent hugging the sun's side, a
// half at the quarters, a full face opposite the sun. Angular radius is ~2x
// the real 0.26 deg (it reads as a moon rather than a dot at this size), the
// dark side keeps a whisper of earthshine, and a faint glow scales with the
// lit fraction. Zero radiance below the horizon (w = 0 from the CPU).
const float kMoonRadius = 0.026;    // radians (~1.5 deg: ~3x real, so the phase READS)
vec3 moonDisc(vec3 dir) {
    float I = g.skyMoonDir.w;
    if (I <= 0.0) return vec3(0.0);
    vec3 m = normalize(g.skyMoonDir.xyz);
    vec3 sun = normalize(g.skyMoonSun.xyz);
    float illum = g.skyMoonSun.w;
    float cosM = dot(dir, m);
    const vec3 moonTint = vec3(0.86, 0.88, 0.95);
    // Glow: a soft halo that the lit fraction owns (no halo round a new moon).
    vec3 col = moonTint * pow(max(cosM, 0.0), 300.0) * 0.08 * I * illum;
    float cosR = cos(kMoonRadius);
    if (cosM < cosR - 0.0004) return col;
    // Disc-plane frame and the point on the lunar sphere under this pixel.
    vec3 ref = abs(m.y) < 0.98 ? vec3(0.0, 1.0, 0.0) : vec3(1.0, 0.0, 0.0);
    vec3 right = normalize(cross(m, ref));
    vec3 up = cross(right, m);
    vec3 off = dir - m * cosM;
    vec2 uv = vec2(dot(off, right), dot(off, up)) / sin(kMoonRadius);
    float r2 = dot(uv, uv);
    float edge = 1.0 - smoothstep(0.92, 1.02, sqrt(r2));
    float w = sqrt(max(1.0 - r2, 0.0));
    vec3 n = right * uv.x + up * uv.y - m * w;   // faces the viewer at the centre
    float lit = max(dot(n, sun), 0.0);
    float earthshine = 0.012;   // a whisper: the dark limb must not read as a grey disc
    col += moonTint * I * (lit + earthshine) * edge;
    return col;
}

vec3 sampleEnvironment(vec3 dir) {
    float skyBlend = clamp(dir.y, 0.0, 1.0);
    vec3 sky = mix(g.skyHorizon.rgb, g.skyZenith.rgb, pow(skyBlend, 0.5));
    vec3 lowerHaze = mix(g.skyHorizon.rgb, g.skyGround.rgb, smoothstep(0.0, -0.4, dir.y));
    float horizonBlend = smoothstep(-0.05, 0.05, dir.y);
    vec3 col = mix(lowerHaze, sky, horizonBlend);

    float disc = g.skySunDir.w;
    vec3 sc = g.skySunColor.rgb;
    float sunDot = max(dot(dir, g.skySunDir.xyz), 0.0);
    col += sc * pow(sunDot, 256.0) * 8.0 * disc;
    col += sc * pow(sunDot, 32.0) * 1.0 * disc;
    col += sc * pow(sunDot, 4.0) * 0.15 * disc;
    float horizonGlow = pow(1.0 - abs(dir.y), 8.0);
    col += sc * horizonGlow * 0.1 * disc;
    col += moonDisc(dir);
    return col;
}

// Lattice hash over the float BITS (see hash21 in mesh.frag / common.metal):
// the old fract(p*123.34) mangle loses all sub-unit precision once the drift
// term grows the coordinates past ~1e5 — a long session turned the cloud field
// into angular slabs. Inputs here are integral lattice coords, exact in a
// float's 24-bit mantissa far beyond any wrapped drift.
float cloudHash(vec2 p) {
    uint h = floatBitsToUint(p.x) * 0x85EBCA6Bu ^ floatBitsToUint(p.y) * 0xC2B2AE35u;
    h = (h ^ (h >> 13)) * 0x27D4EB2Du;
    h ^= h >> 15;
    return float(h >> 8) * (1.0 / 16777216.0);
}
float cloudNoise(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = cloudHash(i);
    float b = cloudHash(i + vec2(1.0, 0.0));
    float c = cloudHash(i + vec2(0.0, 1.0));
    float d = cloudHash(i + vec2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}
float cloudFbm(vec2 p) {
    float s = 0.0, a = 0.5;
    for (int i = 0; i < 5; ++i) { s += a * cloudNoise(p); p *= 2.02; a *= 0.5; }
    return s;
}
vec3 applyClouds(vec3 baseSky, vec3 dir) {
    if (dir.y <= 0.02) return baseSky;
    vec2 uv = dir.xz / (dir.y + 0.10);
    // Wrapped drift: unbounded time pushes the noise-lattice interpolant into
    // float quantization (the hash is magnitude-proof, fract(p) is not). One
    // pattern jump every ~38 days of drift is the whole cost.
    vec2 wind = mod(vec2(g.skyCloud.w * 0.02, g.skyCloud.w * 0.012), 65536.0);
    float n = cloudFbm(uv * g.skyCloud.z + wind);
    float cov = g.skyCloud.x;
    float mask = smoothstep(cov, cov + 0.20, n) * g.skyCloud.y;
    mask *= smoothstep(0.02, 0.25, dir.y);
    mask = clamp(mask, 0.0, 1.0);
    float sunAmt = max(dot(dir, g.skySunDir.xyz), 0.0);
    vec3 cloudLit = g.skySunColor.rgb;
    vec3 cloudDark = vec3(0.40, 0.42, 0.50);
    vec3 cloudColor = mix(cloudDark, cloudLit, pow(sunAmt, 1.5));
    cloudColor *= (0.25 + 0.75 * g.skySunDir.w);
    return mix(baseSky, cloudColor, mask);
}

void main() {
    vec3 dir = normalize(inViewDir);
    vec3 col;
    if (g.counts.z == 1) {
        col = sampleEquirect(dir);   // HDR environment background (no clouds)
    } else {
        col = sampleEnvironment(dir);
        col = applyClouds(col, dir);
    }
    outColor = vec4(col, 1.0);
}
