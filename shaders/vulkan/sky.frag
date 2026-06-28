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
} g;

layout(location = 0) in vec3 inViewDir;
layout(location = 0) out vec4 outColor;

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
    return col;
}

float cloudHash(vec2 p) {
    p = fract(p * vec2(123.34, 345.45));
    p += dot(p, p + 34.345);
    return fract(p.x * p.y);
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
    vec2 wind = vec2(g.skyCloud.w * 0.02, g.skyCloud.w * 0.012);
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
    vec3 col = sampleEnvironment(dir);
    col = applyClouds(col, dir);
    outColor = vec4(col, 1.0);
}
