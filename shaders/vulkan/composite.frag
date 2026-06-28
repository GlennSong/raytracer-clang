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
layout(set = 0, binding = 4) uniform sampler2D depthTex;     // debug views
layout(set = 0, binding = 5) uniform sampler2D normalTex;    // debug views

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
    int   lensEnabled;
    float lensK1;
    float lensK2;
    float lensCA;
    float lensVignette;
    float lensAspect;
    int   debugView;        // 0 normal, 1 AO, 2 SSR, 3 depth, 4 normals
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
    // Debug views bypass lens/tonemap and show a raw buffer (ADR-0057). 5/6/7
    // (shadow/albedo/facing) need lit-pass output not available here yet.
    if (pc.debugView == 1) { outColor = vec4(vec3(texture(aoTex, inUV).r), 1.0); return; }
    if (pc.debugView == 2) { outColor = vec4(texture(ssrTex, inUV).rgb, 1.0); return; }
    if (pc.debugView == 3) { outColor = vec4(vec3(texture(depthTex, inUV).r), 1.0); return; }
    if (pc.debugView == 4) { outColor = vec4(texture(normalTex, inUV).rgb, 1.0); return; }

    // Lens distortion (Brown radial) + lateral CA, around the aspect-corrected
    // center. Neutral params (or lens disabled) => exact passthrough. Ported from
    // post.metal fragmentLensWarp, folded into the composite to avoid a pass.
    vec2 centered = inUV - 0.5;
    float on = pc.lensEnabled != 0 ? 1.0 : 0.0;
    float k1 = pc.lensK1 * on, k2 = pc.lensK2 * on;
    float ca = pc.lensCA * on, vig = pc.lensVignette * on;
    vec2 ap = centered * vec2(pc.lensAspect, 1.0);
    float r2 = dot(ap, ap);
    float distort = 1.0 + k1 * r2 + k2 * r2 * r2;
    vec2 uvG = 0.5 + centered * distort;
    vec2 uvR = 0.5 + centered * distort * (1.0 + ca);
    vec2 uvB = 0.5 + centered * distort * (1.0 - ca);

    // CA splits the HDR channels; the post terms use the (distorted) green UV.
    vec3 hdr = vec3(texture(hdrTex, uvR).r, texture(hdrTex, uvG).g, texture(hdrTex, uvB).b);
    if (pc.ssaoEnabled != 0)
        hdr *= max(texture(aoTex, uvG).r, pc.aoFloor);
    if (pc.ssrEnabled != 0) {
        vec4 ssr = texture(ssrTex, uvG);
        hdr = mix(hdr, ssr.rgb, clamp(ssr.a, 0.0, 1.0));
    }
    if (pc.bloomEnabled != 0)
        hdr += texture(bloomTex, uvG).rgb * pc.bloomIntensity;
    hdr *= pc.exposure;
    hdr = applyGrade(hdr, pc.gradeContrast, pc.gradeSaturation);
    vec3 color = (pc.tonemapOp == 1) ? tonemapAgX(hdr) : tonemapACES(hdr);
    color *= 1.0 - vig * smoothstep(0.0, 1.0, r2);   // vignette
    outColor = vec4(color, 1.0);
}
