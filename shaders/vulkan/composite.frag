#version 450
// Composite: tonemap (ACES / AgX) + color grade + exposure of the HDR scene
// target → the swapchain. Ported from post.metal (applyGrade / tonemapACES /
// tonemapAgX). Both tonemappers fold in the ~2.2 display encode, so the output
// is sRGB-encoded and the swapchain is UNORM (no second gamma). Phase 5b adds
// SSAO/SSR/bloom/lens/DOF here.

layout(set = 1, binding = 0) uniform sampler2D hdrTex;
layout(set = 1, binding = 1) uniform sampler2D bloomTex;
layout(set = 1, binding = 2) uniform sampler2D aoTex;
layout(set = 1, binding = 3) uniform sampler2D ssrTex;
layout(set = 1, binding = 4) uniform sampler2D depthTex;     // debug views
layout(set = 1, binding = 5) uniform sampler2D normalTex;    // debug views
layout(set = 1, binding = 6) uniform sampler2D dofTex;       // DOF-blurred scene

// The frame's globals (set 0, the mesh passes' block): only its head is read here -- the camera, to
// rebuild each pixel's world position, and the sky, to light the water (#58). A prefix of a std140 block
// is layout-compatible with the whole.
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
    vec4  skySunDir;          // xyz toward the sun, w disc intensity
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
} g;

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
    int   debugView;        // 0 normal; see the switch in main()
    int   dofEnabled;       // 1 → sample the DOF-blurred scene instead of HDR
    vec4  underwater;       // x active, y surface y, z visibility (m), w time (s)   (#58)
    vec4  underwaterColor;  // rgb the water's colour
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

// ---- UNDER THE WATER (#58) ------------------------------------------------------------------------
// Moving caustics: light focused by the rippling surface into bright threads on the bed. A classic
// iterated warp (after the well-known "tileable water caustic"); `p` in metres, a ~4 m tile.
float caustic(vec2 p, float t) {
    vec2 q = mod(p * (6.28318 / 4.0), 6.28318) - 250.0;
    vec2 i = q;
    float c = 1.0;
    const float inten = 0.005;
    for (int n = 0; n < 4; ++n) {
        float tt = t * (1.0 - 3.5 / float(n + 1));
        i = q + vec2(cos(tt - i.x) + sin(tt + i.y), sin(tt - i.y) + cos(tt + i.x));
        c += 1.0 / length(vec2(q.x / (sin(i.x + tt) / inten), q.y / (cos(i.y + tt) / inten)));
    }
    c /= 4.0;
    c = 1.17 - pow(c, 1.4);
    return clamp(pow(abs(c), 8.0), 0.0, 3.0);
}

// The scene through water, from a camera under it. Every view ray travels through water until it meets
// the bed (or anything else) or the surface; along that path light is absorbed (red first, blue last)
// and replaced by light the water scatters toward the eye. A ray reaching the surface inside Snell's
// window (~48.6 deg of vertical) sees the world above; outside it, the surface is a mirror of the depths.
vec3 underwaterScene(vec3 hdr, vec2 uv) {
    const vec3 luma = vec3(0.2126, 0.7152, 0.0722);
    float depth = texture(depthTex, uv).r;                 // reverse-Z: 0 = far / sky
    vec3 cam = g.cameraPosition.xyz;
    vec4 nearW = g.invViewProjection * vec4(uv * 2.0 - 1.0, 1.0, 1.0);
    vec3 dir = normalize(nearW.xyz / nearW.w - cam);
    float tScene = 1e9;
    vec3 P = cam;
    if (depth > 0.0) {
        vec4 w = g.invViewProjection * vec4(uv * 2.0 - 1.0, depth, 1.0);
        P = w.xyz / w.w;
        tScene = length(P - cam);
    }
    float surf = pc.underwater.y;
    float tSurf = dir.y > 1e-4 ? (surf - cam.y) / dir.y : 1e9;
    float path = min(tScene, tSurf);
    // light in the water: the sky's brightness, dimming with depth below the surface
    float sky = dot(g.skyZenith.rgb, luma) * 1.2 + dot(g.skyHorizon.rgb, luma) * 0.5;
    float sunUp = max(g.skySunDir.y, 0.0);
    float sunLum = dot(g.skySunColor.rgb, luma) * sunUp * 0.08;
    float camDepth = max(0.0, surf - cam.y);
    float light = (sky + sunLum) * exp(-camDepth / 22.0);
    vec3 water = pc.underwaterColor.rgb;
    vec3 scatter = water * light * 1.6;
    // absorption per channel: red dies in a few metres, blue-green carries
    vec3 k = vec3(2.6, 1.0, 0.75) / pc.underwater.z;
    vec3 T = exp(-k * path);
    vec3 seen = hdr;
    // the water mesh itself, seen from below, is the SURFACE, not the bed: it draws its above-water shading
    // (stripes of wave normals at a grazing angle) and would otherwise be caustic-lit as seabed
    bool hitsSurface = tSurf <= tScene || (depth > 0.0 && P.y > surf - 0.25);
    if (hitsSurface) path = min(path, tSurf);
    if (!hitsSurface) {
        // the bed and anything in the water: caustics where it faces up and the sun reaches
        vec3 n = texture(normalTex, uv).rgb * 2.0 - 1.0;
        float below = max(0.0, surf - P.y);
        float c = caustic(P.xz + vec2(0.0, 0.0), pc.underwater.w * 0.8);
        float reach = exp(-below / 7.0) * smoothstep(0.2, 0.8, n.y) * (0.25 + sunUp);
        seen = hdr * (1.0 + c * 1.4 * reach);
    } else {
        // the surface from below
        float window = smoothstep(0.62, 0.70, dir.y);   // cos(48.6 deg) = 0.661
        // the sky and shore, through the window: the water mesh drawn over them from below reads dark, so
        // lean toward the sky's own light the way a diver sees the bright disc overhead
        vec3 above = mix(g.skyHorizon.rgb, g.skyZenith.rgb, smoothstep(0.66, 1.0, dir.y)) * 1.1;
        // the surface's ripples bend the disc: a little moving brightness across it
        above *= 0.9 + 0.2 * caustic(cam.xz + dir.xz / max(dir.y, 0.2) * (surf - cam.y), pc.underwater.w * 0.6);
        vec3 mirror = scatter * 0.55;                    // total internal reflection: the depths
        float rim = exp(-pow((dir.y - 0.661) / 0.02, 2.0)) * 0.6;   // the window's bright edge
        seen = mix(mirror, above * 1.3, window) + scatter * rim;
    }
    return seen * T + scatter * (1.0 - T);
}

void main() {
    // Debug views bypass lens/tonemap (ADR-0057; parity with post.metal):
    //   1 AO, 2 SSR, 4 normals       — buffer views, read here.
    //   3 depth, 5 shadow, 6 albedo, 7 facing, 8 cascades
    //                                — written display-ready into the HDR target
    //                                  by mesh.frag; shown raw, background guarded
    //                                  by depth (reverse-Z: 0.0 == far/sky).
    if (pc.debugView != 0) {
        float dpt = texture(depthTex, inUV).r;
        bool bg = dpt <= 0.0;
        if (pc.debugView == 1) {                 // ambient occlusion (white = none)
            float ao = bg ? 1.0 : texture(aoTex, inUV).r;
            outColor = vec4(vec3(ao), 1.0); return;
        }
        if (pc.debugView == 2) {                 // SSR (reflected color where it hit)
            vec4 s = texture(ssrTex, inUV);
            outColor = vec4(s.rgb * s.a, 1.0); return;
        }
        if (pc.debugView == 4) {                 // world normals (G-buffer, encoded)
            outColor = bg ? vec4(0.5, 0.5, 1.0, 1.0)
                          : vec4(texture(normalTex, inUV).rgb, 1.0);
            return;
        }
        // 3/5/6/7/8: raw HDR (mesh.frag wrote the debug value); flat background.
        vec3 c = texture(hdrTex, inUV).rgb;
        if (bg) c = (pc.debugView == 5) ? vec3(0.3) : vec3(0.0);
        outColor = vec4(c, 1.0); return;
    }

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

    // CA splits the scene channels; the post terms use the (distorted) green UV.
    // When DOF is on, the blurred scene (dofTex) replaces the sharp HDR.
    vec3 hdr;
    if (pc.dofEnabled != 0)
        hdr = vec3(texture(dofTex, uvR).r, texture(dofTex, uvG).g, texture(dofTex, uvB).b);
    else
        hdr = vec3(texture(hdrTex, uvR).r, texture(hdrTex, uvG).g, texture(hdrTex, uvB).b);
    if (pc.ssaoEnabled != 0)
        hdr *= max(texture(aoTex, uvG).r, pc.aoFloor);
    if (pc.ssrEnabled != 0) {
        vec4 ssr = texture(ssrTex, uvG);
        hdr = mix(hdr, ssr.rgb, clamp(ssr.a, 0.0, 1.0));
    }
    if (pc.underwater.x > 0.5) {
        // the surface ripples what is seen through it: a gentle wobble of the lookup
        vec2 wob = vec2(sin(uvG.y * 40.0 + pc.underwater.w * 1.7), cos(uvG.x * 35.0 + pc.underwater.w * 1.3)) * 0.0015;
        vec3 wobbled = texture(hdrTex, clamp(uvG + wob, 0.0, 1.0)).rgb;
        hdr = underwaterScene(mix(hdr, wobbled, 0.7), uvG);
    }
    if (pc.bloomEnabled != 0)
        hdr += texture(bloomTex, uvG).rgb * pc.bloomIntensity;
    hdr *= pc.exposure;
    hdr = applyGrade(hdr, pc.gradeContrast, pc.gradeSaturation);
    vec3 color = (pc.tonemapOp == 1) ? tonemapAgX(hdr) : tonemapACES(hdr);
    color *= 1.0 - vig * smoothstep(0.0, 1.0, r2);   // vignette
    outColor = vec4(color, 1.0);
}
