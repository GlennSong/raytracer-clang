#version 450
// Composite: tonemap (ACES / AgX) + color grade + exposure of the HDR scene
// target → the swapchain. Ported from post.metal (applyGrade / tonemapACES /
// tonemapAgX). Both tonemappers fold in the ~2.2 display encode, so the output
// is sRGB-encoded and the swapchain is UNORM (no second gamma). Phase 5b adds
// SSAO/SSR/bloom/lens/DOF here.

layout(set = 0, binding = 0) uniform sampler2D hdrTex;
layout(set = 0, binding = 1) uniform sampler2D bloomTex;
layout(set = 0, binding = 2) uniform sampler2D aoTex;
layout(set = 0, binding = 3) uniform sampler2D ssrTex;

layout(push_constant) uniform Push {
    float exposure;
    int   tonemapOp;        // 0 = ACES, 1 = AgX
    float gradeContrast;
    float gradeSaturation;
    int   bloomEnabled;
    float bloomIntensity;
    int   ssaoEnabled;
    float aoFloor;
    int   ssrEnabled;
    float _pad;
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

vec3 applyGrade(vec3 x, float contrast, float saturation) {
    float luma = dot(x, vec3(0.2126, 0.7152, 0.0722));
    x = max(vec3(luma) + saturation * (x - luma), vec3(0.0));
    const float grey = 0.18;
    vec3 lx = log2(max(x, vec3(1e-5)));
    lx = (lx - log2(grey)) * contrast + log2(grey);
    return exp2(lx);
}

vec3 tonemapACES(vec3 x) {
    vec3 ci = vec3(dot(vec3(0.59719, 0.35458, 0.04823), x),
                   dot(vec3(0.07600, 0.90834, 0.01566), x),
                   dot(vec3(0.02840, 0.13383, 0.83777), x));
    vec3 cf = (ci * (ci + 0.0245786) - 0.000090537) /
              (ci * (0.983729 * ci + 0.432951) + 0.238081);
    vec3 c = vec3(dot(vec3( 1.60475, -0.53108, -0.07367), cf),
                  dot(vec3(-0.10208,  1.10813, -0.00605), cf),
                  dot(vec3(-0.00327, -0.07276,  1.07602), cf));
    return pow(clamp(c, 0.0, 1.0), vec3(1.0 / 2.2));
}

vec3 agxContrastApprox(vec3 x) {
    vec3 x2 = x * x;
    vec3 x4 = x2 * x2;
    return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4
         - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

vec3 tonemapAgX(vec3 val) {
    const mat3 agxMat = mat3(
        0.842479062253094, 0.0423282422610123, 0.0423756549057051,
        0.0784335999999992, 0.878468636469772, 0.0784336,
        0.0792237451477643, 0.0791661274605434, 0.879142973793104);
    const mat3 agxMatInv = mat3(
        1.19687900512017, -0.0528968517574562, -0.0529716355144438,
        -0.0980208811401368, 1.15190312990417, -0.0980434501171241,
        -0.0990297440797205, -0.0989611768448433, 1.15107367264116);
    const float minEv = -12.47393;
    const float maxEv = 4.026069;
    val = agxMat * val;
    val = clamp(log2(max(val, vec3(1e-10))), minEv, maxEv);
    val = (val - minEv) / (maxEv - minEv);
    val = agxContrastApprox(val);
    val = agxMatInv * clamp(val, 0.0, 1.0);
    return clamp(val, 0.0, 1.0);
}

void main() {
    vec3 hdr = texture(hdrTex, inUV).rgb;
    // SSAO darkens the scene (approximation of Metal applying it to ambient
    // only), clamped to aoFloor so creases don't go fully black.
    if (pc.ssaoEnabled != 0)
        hdr *= max(texture(aoTex, inUV).r, pc.aoFloor);
    // SSR: rgb = reflected color, a = confidence (already scaled by blendStrength).
    if (pc.ssrEnabled != 0) {
        vec4 ssr = texture(ssrTex, inUV);
        hdr = mix(hdr, ssr.rgb, clamp(ssr.a, 0.0, 1.0));
    }
    if (pc.bloomEnabled != 0)
        hdr += texture(bloomTex, inUV).rgb * pc.bloomIntensity;
    hdr *= pc.exposure;
    hdr = applyGrade(hdr, pc.gradeContrast, pc.gradeSaturation);
    vec3 color = (pc.tonemapOp == 1) ? tonemapAgX(hdr) : tonemapACES(hdr);
    outColor = vec4(color, 1.0);
}
