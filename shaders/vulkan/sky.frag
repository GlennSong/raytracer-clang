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
    vec4  skyCelX;         // the celestial frame in local space (stars)
    vec4  skyCelY;
    vec4  skyCelZ;
    vec4  skyStars;        // x visibility (0 by day), y Milky Way strength
    vec4  skyCity;         // xy unit XZ toward the city, z light pollution (0..1)
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


// THE STARS: a procedural field on the CELESTIAL sphere. skyCel{X,Y,Z} are
// the equatorial axes in local space, rotated by sidereal time (CPU), so
// dotting a view direction with them gives its (RA, Dec): the field wheels
// about the pole with the hour and slides with the season. Stars live on a
// 3-D LATTICE around the sphere — one candidate per voxel, jittered, kept
// only within the shell the sphere passes through, 27-neighbour lookup —
// which has no seams (the first cut hashed cube-map faces and stars on a
// face edge were cut in half). Power-law brightness, blue-white to warm
// tint; the Milky Way (galactic pole in equatorial coordinates) raises the
// density and adds a faint glow. Gated by skyStars.x (0 by day, washed by a
// bright moon), dimmed toward the horizon. Clouds overlay afterwards.
uint starHash(uvec3 v) {
    uint h = v.x * 0x8da6b343u ^ v.y * 0xd8163841u ^ v.z * 0xcb1ab31fu;
    h ^= h >> 13; h *= 0x9e3779b1u; h ^= h >> 16;
    return h;
}
vec3 starField(vec3 dir) {
    float gate = g.skyStars.x;
    if (gate <= 0.001 || dir.y < -0.02) return vec3(0.0);
    vec3 dc = vec3(dot(dir, g.skyCelX.xyz), dot(dir, g.skyCelY.xyz), dot(dir, g.skyCelZ.xyz));
    const float R = 34.0;                                   // lattice cells ~1.7 deg
    vec3 base = floor(dc * R);
    const vec3 galPole = vec3(-0.8676, -0.1981, 0.4560);   // RA 192.9, Dec 27.1
    // LIGHT POLLUTION: the Milky Way is the first thing a city sky loses.
    float pollution = clamp(g.skyCity.z, 0.0, 1.0);
    float band = exp(-pow(dot(dc, galPole) / 0.16, 2.0)) * g.skyStars.y * (1.0 - pollution);
    vec3 col = vec3(0.0);
    for (int k = -1; k <= 1; ++k)
    for (int j = -1; j <= 1; ++j)
    for (int i = -1; i <= 1; ++i) {
        vec3 c = base + vec3(float(i), float(j), float(k));
        uint h = starHash(uvec3(uint(int(c.x) + 512), uint(int(c.y) + 512), uint(int(c.z) + 512)));
        float r0 = float(h & 0xffffu) / 65535.0;
        float r1 = float(h >> 16) / 65535.0;
        uint h2 = starHash(uvec3(h, 19u, 23u));
        float r2 = float(h2 & 0xffffu) / 65535.0;
        float r3 = float(h2 >> 16) / 65535.0;
        uint h3 = starHash(uvec3(h2, 29u, 31u));
        float r4 = float(h3 & 0xffffu) / 65535.0;
        if (r0 > 0.16 + 0.32 * band) continue;             // no star here
        vec3 q = c + vec3(r1, r2, r4);
        float ql = length(q);
        if (abs(ql - R) > 0.9) continue;                    // off the sphere's shell
        vec3 sd = q / ql;
        float ang = length(cross(dc, sd));                  // ~ the angle
        float mag = pow(r3, 4.0);                           // many faint, very few bright
        float sigma = 0.0007 + 0.0007 * mag;                // sub-pixel to ~1 px
        float I = exp(-ang * ang / (2.0 * sigma * sigma)) * (0.10 + 2.0 * mag);
        // A city sky keeps only its brighter stars: the faint end fades with
        // the pollution (downtown: a few dozen stars; a dark site: thousands).
        I *= mix(1.0, smoothstep(0.03, 0.30, mag), pollution);
        vec3 tint = mix(vec3(0.78, 0.85, 1.0), vec3(1.0, 0.86, 0.70), r1 * r1);
        col += tint * I;
    }
    col += vec3(0.30, 0.34, 0.46) * 0.03 * band;            // the Milky Way glow
    float horizon = smoothstep(-0.02, 0.18, dir.y);         // extinction low down
    return col * gate * horizon;
}

// LIGHT POLLUTION's sky glow: a warm dome hugging the horizon, brightest
// toward the city, only once the sun disc is gone (skySunDir.w is 0 at
// night — the moon in slot 0 carries no disc).
vec3 skyGlow(vec3 dir) {
    float pollution = clamp(g.skyCity.z, 0.0, 1.0);
    float night = 1.0 - g.skySunDir.w;
    if (pollution <= 0.001 || night <= 0.001) return vec3(0.0);
    vec2 h = dir.xz;
    float hl = length(h);
    float toward = hl > 1e-4 ? max(dot(h / hl, g.skyCity.xy), 0.0) : 0.0;
    float low = pow(clamp(1.0 - dir.y, 0.0, 1.0), 10.0);
    return vec3(1.0, 0.72, 0.45) * 0.06 * pollution * night * low * (0.35 + 0.65 * toward);
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
    col += starField(dir);   // under the moon: its glow sits over the field
    col += skyGlow(dir);
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
