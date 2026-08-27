#version 450
// Phase 2 forward fragment stage (ADR-0057). Faithful port of the Metal
// evaluateLighting + the procedural surface library (shaders/metal/lighting.metal
// + common.metal): multi-light Cook-Torrance (GGX + height-correlated Smith,
// windowed inverse-square falloff, spot cones) for directional/point/spot lights,
// plus the analytic city-surface library. Still missing vs Metal (later phases):
// shadows (Phase 3), IBL/probes (Phase 4 — ambient is a flat stand-in here),
// texture maps (Phase 2b), tone mapping (Phase 5). Output is scene-linear; the
// sRGB swapchain encodes on store.

const float PI = 3.14159265359;

struct Light {
    vec4 positionIntensity;   // xyz position (point/spot), w intensity
    vec4 directionInner;      // xyz direction (dir/spot), w innerCosAngle
    vec4 colorOuter;          // rgb color, w outerCosAngle
    vec4 typeRange;           // x type (0 point,1 directional,2 spot), y range
};

layout(set = 0, binding = 0) uniform Globals {
    mat4  viewProjection;
    mat4  view;
    mat4  invViewProjection;
    mat4  cascadeVP[4];
    vec4  cameraPosition;
    vec4  ambient;            // rgb = ambient tint * multiplier (IBL strength)
    vec4  cascadeSplit;       // far view-space depth of cascades 0..3
    ivec4 counts;             // x lightCount, y cascadeCount, z envMode, w debugView
    vec4  shadowParams;       // x normalBias, y pcfRadius, z mapSize, w strength
    vec4  skySunDir;          // xyz dir, w disc intensity
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
    vec4  fog;                // rgb fog color, w density (0 = off)
    vec4  shadowTint;         // rgb artistic shadow tint, w ambientStrength
    vec4  wind1;              // xyz wind dir, w time (used by mesh.vert)
    vec4  wind2;              // x frequency, y height, z amplitude
    vec4  skyMoonDir;         // (sky.frag's members, declared to reach the tail)
    vec4  skyMoonSun;
    vec4  skyCelX;
    vec4  skyCelY;
    vec4  skyCelZ;
    vec4  skyStars;
    vec4  skyCity;
    vec4  terrainHorizon;     // xy world origin, z 1/extent, w enabled
    vec4  terrainHorizon2;    // x encodeLo, y encodeHi - encodeLo
} g;

layout(set = 0, binding = 1) uniform sampler2DArrayShadow shadowMap;
layout(set = 0, binding = 2) uniform sampler2D envEquirect;   // HDR env (counts.z==1)
layout(set = 0, binding = 3) uniform sampler2D brdfLut;       // split-sum BRDF LUT
layout(set = 0, binding = 4) uniform sampler2D terrainHorizonMap;   // mountain shadows

const float ENV_PI = 3.14159265359;
vec3 sampleEquirect(vec3 dir) {
    float u = atan(dir.z, dir.x) * (0.5 / ENV_PI) + 0.5;
    float v = acos(clamp(dir.y, -1.0, 1.0)) * (1.0 / ENV_PI);
    return texture(envEquirect, vec2(u, v)).rgb;
}

layout(push_constant) uniform Push {
    mat4  model;
    vec4  albedoMetallic;     // rgb albedo, a metallic
    vec4  emissionRough;      // rgb emission, a roughness
    uvec4 surfaceFlags;       // x surfaceId, y rawFlags, z textureFlags
} pc;

// Material textures (set 1). textureFlags bits: 0 albedo, 1 metallic-roughness,
// 2 normal (sampled in a later phase — needs TBN), 3 AO, 4 emissive. Absent maps
// are bound to a 1x1 white default, so an unflagged sample is harmless.
layout(set = 1, binding = 0) uniform sampler2D albedoMap;
layout(set = 1, binding = 1) uniform sampler2D metallicRoughnessMap;
layout(set = 1, binding = 2) uniform sampler2D normalMap;
layout(set = 1, binding = 3) uniform sampler2D aoMap;
layout(set = 1, binding = 4) uniform sampler2D emissiveMap;

layout(location = 0) in vec3 inWorldPos;
layout(location = 1) in vec3 inWorldNormal;
layout(location = 2) in vec2 inTexcoord;
layout(location = 3) in vec3 inColor;
layout(location = 4) in vec3 inWorldTangent;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormal;   // world normal *0.5+0.5 (SSAO G-buffer)

// ---- BRDF (ported from lighting.metal) ------------------------------------
float distributionGGX(float NdotH, float a2) {
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / max(PI * d * d, 1e-6);
}
float visibilitySmithGGX(float NdotV, float NdotL, float a2) {
    float gv = NdotL * sqrt(NdotV * NdotV * (1.0 - a2) + a2);
    float gl = NdotV * sqrt(NdotL * NdotL * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}
vec3 fresnelSchlickVec(float cosTheta, vec3 f0) {
    return f0 + (1.0 - f0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}
float distanceAttenuation(float dist, float range) {
    float ratio2 = (dist * dist) / max(range * range, 1e-4);
    float window = clamp(1.0 - ratio2 * ratio2, 0.0, 1.0);
    return window * window / max(dist * dist, 1e-4);
}

// Roughness-aware Fresnel for the ambient/IBL term.
vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness) {
    vec3 r = max(vec3(1.0 - roughness), F0);
    return F0 + (r - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

// Procedural-sky environment (no clouds — they are never baked into IBL),
// ported from environment.metal sampleEnvironment. Used as an analytic stand-in
// for Metal's baked irradiance/prefiltered cubes (Phase 4b adds the real bake).
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
    return col;
}

// ---- Procedural surface library (ported byte-for-byte from common.metal) ---
const float SURF_PI = 3.14159265;
// Lattice hash. The classic fract(sin(x)*43758) collapses once |x| outgrows
// float phase precision (~1e5) — routine here, where world-planar coords feed
// octaves like worldPos*37 before the *12.9898 — turning road grain and brick
// shading into BLOCKS on device. Hash the raw float BITS instead (xxhash-style
// avalanche): exact at any magnitude, and bit-identical across CPU / Metal /
// Vulkan (IEEE-754), so every backend now agrees per-brick. Pattern layouts
// reshuffle once relative to the old sin hash; statistics are unchanged.
float hash21(float a, float b) {
    uint h = floatBitsToUint(a) * 0x85EBCA6Bu ^ floatBitsToUint(b) * 0xC2B2AE35u;
    h = (h ^ (h >> 13)) * 0x27D4EB2Du;
    h ^= h >> 15;
    return float(h >> 8) * (1.0 / 16777216.0);
}
float vnoise2(float x, float y) {
    float xi = floor(x), yi = floor(y), xf = x - xi, yf = y - yi;
    float a = hash21(xi, yi), b = hash21(xi + 1.0, yi);
    float c = hash21(xi, yi + 1.0), d = hash21(xi + 1.0, yi + 1.0);
    float ux = xf * xf * (3.0 - 2.0 * xf), uy = yf * yf * (3.0 - 2.0 * yf);
    return a * (1.0 - ux) * (1.0 - uy) + b * ux * (1.0 - uy) +
           c * (1.0 - ux) * uy + d * ux * uy;
}
float fbm2(float x, float y) {
    float v = 0.0, amp = 0.5, f = 1.0;
    for (int i = 0; i < 4; ++i) { v += amp * vnoise2(x * f, y * f); f *= 2.0; amp *= 0.5; }
    return v;
}
float tile1(float x, float m) { return x - m * floor(x / m); }
vec2 surfUV(vec3 p, vec3 n) {
    if (abs(n.y) > 0.5) return vec2(p.x, p.z);
    vec2 t = vec2(n.z, -n.x);
    float tl = length(t);
    t = tl < 1e-6 ? vec2(1.0, 0.0) : t / tl;
    return vec2(p.x * t.x + p.z * t.y, p.y);
}
vec3 surfBrick(vec3 base, float u, float v) {
    const float courseH = 0.075, brickL = 0.20, mortar = 0.011;
    float row = floor(v / courseH);
    float off = (mod(abs(row), 2.0) < 1.0) ? 0.0 : brickL * 0.5;
    float uu = u + off, col = floor(uu / brickL);
    float fy = v - row * courseH, fx = uu - col * brickL;
    float joint = min(min(fy, courseH - fy), min(fx, brickL - fx));
    float h = hash21(col, row), h2 = hash21(col * 1.7 + 3.1, row * 0.9 + 5.7);
    float shade = 0.74 + 0.46 * h;
    if (h2 < 0.12) shade *= 0.6;
    shade *= 0.94 + 0.12 * (fx / brickL);
    float t = clamp((joint - mortar) / 0.004, 0.0, 1.0);
    return mix(vec3(0.30, 0.29, 0.27), base * shade, t);
}
vec3 surfConcrete(vec3 base, float u, float v) {
    float n = fbm2(u * 0.6, v * 0.6), fine = vnoise2(u * 9.0, v * 9.0);
    float shade = 0.84 + 0.22 * n + 0.06 * (fine - 0.5);
    float gu = tile1(u, 3.0); gu = min(gu, 3.0 - gu);
    float gv = tile1(v, 3.0); gv = min(gv, 3.0 - gv);
    float jt = smoothstep(0.015, 0.04, min(gu, gv));
    return base * shade * (0.74 + 0.26 * jt);
}
vec3 surfStucco(vec3 base, float u, float v) {
    float n = fbm2(u * 3.0, v * 3.0), fine = vnoise2(u * 22.0, v * 22.0);
    return base * (0.90 + 0.12 * (n - 0.5) + 0.10 * (fine - 0.5));
}
vec3 surfRoofTile(vec3 base, float u, float v) {
    const float tileW = 0.18, rowH = 0.32;
    float row = floor(v / rowH);
    float off = (mod(abs(row), 2.0) < 1.0) ? 0.0 : tileW * 0.5;
    float uu = u + off, col = floor(uu / tileW);
    float fx = uu - col * tileW, fy = v - row * rowH;
    float curve = sin(SURF_PI * (fx / tileW));
    float valley = smoothstep(0.0, 0.02, min(fx, tileW - fx));
    float lap = smoothstep(0.0, 0.05, fy);
    float h = hash21(col, row);
    return base * ((0.55 + 0.5 * curve) * (0.85 + 0.30 * h) * valley * (0.6 + 0.4 * lap));
}
vec3 surfShingle(vec3 base, float u, float v) {
    const float tabW = 0.30, rowH = 0.14;
    float row = floor(v / rowH);
    float off = (mod(abs(row), 2.0) < 1.0) ? 0.0 : tabW * 0.5;
    float uu = u + off, col = floor(uu / tabW);
    float fx = uu - col * tabW, fy = v - row * rowH;
    float h = hash21(col, row);
    float key = smoothstep(0.0, 0.012, min(fx, tabW - fx));
    float shadow = smoothstep(0.0, 0.03, fy);
    return base * (0.82 + 0.32 * h) * (0.55 + 0.45 * key) * (0.5 + 0.5 * shadow);
}
vec3 surfCorrugated(vec3 base, float u, float v) {
    const float ribW = 0.12;
    float rib = cos(2.0 * SURF_PI * u / ribW);
    float shade = 0.72 + 0.28 * rib;
    float rust = fbm2(u * 1.5, v * 0.6);
    float rmask = clamp((rust - 0.62) / 0.18, 0.0, 1.0) * 0.45;
    return base * shade * (1.0 - rmask) + vec3(0.40, 0.22, 0.12) * rmask;
}
vec3 surfAsphalt(vec3 base, float u, float v) {
    float spk = vnoise2(u * 30.0, v * 30.0), blotch = fbm2(u * 0.4, v * 0.4);
    float shade = clamp(0.86 + 0.20 * (spk - 0.5) + 0.10 * (blotch - 0.5), 0.6, 1.02);
    return base * shade;
}
vec3 surfPavement(vec3 base, float u, float v) {
    const float slab = 1.2;
    float su = tile1(u, slab), sv = tile1(v, slab);
    float joint = min(min(su, slab - su), min(sv, slab - sv));
    float h = hash21(floor(u / slab), floor(v / slab));
    float spk = vnoise2(u * 26.0, v * 26.0);
    float shade = 0.90 + 0.12 * (h - 0.5) + 0.06 * (spk - 0.5);
    float jt = smoothstep(0.02, 0.05, joint);
    return base * shade * (0.6 + 0.4 * jt);
}
vec3 surfCobble(vec3 base, float u, float v) {
    const float cell = 0.18;
    float cu = u / cell, cv = v / cell, iu = floor(cu), iv = floor(cv);
    float best = 1e9, bh = 0.0;
    for (int dj = -1; dj <= 1; ++dj)
        for (int di = -1; di <= 1; ++di) {
            float ci = iu + float(di), cj = iv + float(dj);
            float jx = hash21(ci, cj), jy = hash21(ci + 5.2, cj + 1.7);
            float px = ci + 0.5 + (jx - 0.5) * 0.7, py = cj + 0.5 + (jy - 0.5) * 0.7;
            float dx = cu - px, dy = cv - py, dd = dx * dx + dy * dy;
            if (dd < best) { best = dd; bh = hash21(ci + 9.1, cj + 4.3); }
        }
    float stone = smoothstep(0.0, 0.12, 0.62 - sqrt(best));
    return mix(vec3(0.32, 0.30, 0.27), base * (0.7 + 0.6 * bh), stone);
}
vec3 surfWood(vec3 base, float u, float v) {
    const float boardH = 0.18;
    float row = floor(v / boardH), fy = v - row * boardH;
    float h = hash21(row, 3.0), grain = vnoise2(u * 40.0, row * 9.0 + v * 2.0);
    float shadow = smoothstep(0.0, 0.02, fy);
    return base * (0.85 + 0.20 * h + 0.12 * (grain - 0.5)) * (0.55 + 0.45 * shadow);
}
vec3 surfRoadMarkings(vec3 base, float mu, float mv, float wu, float wv) {
    // One surface id covers the welded road, split by road-local mu: the
    // sidewalk/curb band (mu in [0,1]) wears concrete pavement, the carriageway
    // (mu in [1,3]) asphalt grain under the lane paint. wu/wv = the world-planar
    // UV the other surfaces tile by. Mirrors scene.cpp / WGSL / Metal.
    if (mu < 0.98) {
        // Sidewalk band: concrete grain + scoring joints that FOLLOW the curb
        // (slab tops bake u = -(metres along the kerb loop)). Curb faces (u = 0)
        // stay plain. Mirrors scene.cpp / WGSL / Metal.
        float spk = vnoise2(wu * 26.0, wv * 26.0);
        vec3 c = base * (0.92 + 0.10 * (spk - 0.5));
        float su = -mu;
        if (su > 0.02) {
            float t = tile1(su, 1.5);
            float jd = min(t, 1.5 - t);
            c = c * (0.68 + 0.32 * smoothstep(0.02, 0.06, jd));
        }
        return c;
    }
    vec3 deck = surfAsphalt(base, wu, wv);              // grained asphalt deck
    // Dashed lane DIVIDER strip (u = 4, v = raw arc-length): one thin strip per
    // internal same-direction lane boundary on multilane roads; 3 m of white
    // paint every 7.5 m. Mirrors scene.cpp / WGSL / Metal.
    if (mu > 3.5)
        return (fract(mv / 7.5) < 0.4) ? vec3(0.86, 0.86, 0.83) : deck;
    float lat = mu - 2.0;
    float yL = 1.0 - smoothstep(0.013, 0.019, abs(lat - 0.030));
    float yR = 1.0 - smoothstep(0.013, 0.019, abs(lat + 0.030));
    float y  = max(yL, yR);
    float wL = 1.0 - smoothstep(0.016, 0.022, abs(lat - 0.86));
    float wR = 1.0 - smoothstep(0.016, 0.022, abs(lat + 0.86));
    float w  = max(wL, wR);
    // The centreline ENDS before a crosswalk (device: the double yellow cut
    // through the zebra bars): the band ends at mv ~3.6, so the yellow fades in
    // just past it. Without crosswalks mv is a large sentinel (full-length line).
    y *= smoothstep(4.0, 4.8, mv);
    vec3 c = mix(deck, vec3(0.82, 0.68, 0.13), y);
    c = mix(c, vec3(0.86, 0.86, 0.83), w);
    // Zebra crosswalk painted into the road texture (ADR-0062): mv = metres PAST
    // the junction mouth (baked by the road mesher), so the band sits set back on
    // the approach, not in the intersection. Bars run across the carriageway (mu).
    // Mirror of the Metal path (shaders/metal/common.metal) for backend parity.
    // Only the carriageway (mu in [~1,3]) — the raised sidewalk/curb shares this
    // surface but carries a 0..1 UV, so gate on mu > 1.05 to keep paint off the curb.
    float cwEdge = smoothstep(0.5, 0.8, mv) * (1.0 - smoothstep(3.3, 3.6, mv));
    float bars = step(0.5, fract((mu - 1.0) * 4.0));
    c = mix(c, vec3(0.90, 0.90, 0.88), cwEdge * bars * step(1.05, mu));
    return c;
}
// Natural ground (Surface::TerrainGround, id 13): the biome colour is baked in
// the vertex colour (grass/rock/sand/snow/sea floor); add fine albedo GRAIN so
// it isn't a flat wash. Normal micro-relief + roughness live in
// surfaceReliefTerrain below, like the road. Ported from surface_terrain.metal.
vec3 surfTerrain(vec3 base, vec3 worldPos) {
    float gr = fbm2(worldPos.x * 0.5, worldPos.z * 0.5) * 0.6 +
               fbm2(worldPos.x * 2.1, worldPos.z * 2.1) * 0.4;   // ~[0,1]
    return base * (0.90 + 0.18 * gr);
}

// Water (Surface::Water, id 12): depth-graded ocean colour + shoreline foam.
// meshUV.x = baked water depth (m), meshUV.y = baked distance to land (m).
// Colour only — the animated wave normals live in surfaceReliefWater below.
// Ported from surface_water.metal; `time` is g.wind1.w (seconds).
vec3 surfWater(vec3 base, float depth, float shore, vec3 worldPos, float time) {
    vec3 deep    = base;
    vec3 mid     = base * 2.0 + vec3(0.010, 0.045, 0.050);   // green-blue
    vec3 shallow = base * 3.0 + vec3(0.030, 0.130, 0.120);   // teal shallows
    float t = clamp(depth / 30.0, 0.0, 1.0);
    vec3 c = t < 0.5 ? mix(shallow, mid, t * 2.0) : mix(mid, deep, (t - 0.5) * 2.0);
    // Two-scale mottling, each drifting at its own rate (slow macro banding +
    // finer chop shading — a fixed single octave read as repetitive texture).
    float m1 = fbm2(worldPos.x * 0.011 + time * 0.006, worldPos.z * 0.011 - time * 0.002);
    float m2 = fbm2(worldPos.x * 0.055 - time * 0.010, worldPos.z * 0.055 + time * 0.004);
    c *= 0.88 + 0.12 * m1 + 0.07 * m2;
    // Sparse whitecap flecks in open water, advecting with the wind.
    float cap = fbm2(worldPos.x * 0.09 + time * 0.05, worldPos.z * 0.09 - time * 0.03) *
                (0.55 + 0.45 * fbm2(worldPos.x * 0.021 - time * 0.013, worldPos.z * 0.021));
    float capMask = smoothstep(0.60, 0.78, cap) * clamp(depth / 8.0, 0.0, 1.0) * 0.30;
    c = mix(c, vec3(0.85, 0.92, 0.95), capMask);
    // Lapping shoreline foam: a narrow washing edge plus patchy streaks.
    float band = 6.0 + 4.5 * fbm2(worldPos.x * 0.05, worldPos.z * 0.05) +
                 2.5 * sin(time * 0.8 + 6.2831853 * fbm2(worldPos.x * 0.02, worldPos.z * 0.02));
    if (shore > 0.001 && shore < band) {
        float u = 1.0 - shore / band;
        float pattern = fbm2(worldPos.x * 0.35 + time * 0.22, worldPos.z * 0.35 - time * 0.13);
        pattern *= pattern;                      // patchy streaks, not a wash
        float f = u * u * (0.20 + 0.60 * pattern);
        f += smoothstep(0.85, 1.0, u) * 0.30;    // bright washing edge
        c = mix(c, vec3(0.92, 0.96, 0.98), clamp(f, 0.0, 1.0));
    }
    return c;
}

vec3 applySurface(uint id, vec3 base, vec3 worldPos, vec3 n, vec2 meshUV, float time) {
    vec2 uv = surfUV(worldPos, n);
    vec3 c;
    switch (id) {
        case 1u:  c = surfBrick(base, uv.x, uv.y); break;
        case 2u:  c = surfConcrete(base, uv.x, uv.y); break;
        case 3u:  c = surfStucco(base, uv.x, uv.y); break;
        case 4u:  c = surfRoofTile(base, uv.x, uv.y); break;
        case 5u:  c = surfShingle(base, uv.x, uv.y); break;
        case 6u:  c = surfCorrugated(base, uv.x, uv.y); break;
        case 7u:  c = surfAsphalt(base, uv.x, uv.y); break;
        case 8u:  c = surfPavement(base, uv.x, uv.y); break;
        case 9u:  c = surfCobble(base, uv.x, uv.y); break;
        case 10u: c = surfWood(base, uv.x, uv.y); break;
        case 11u: c = surfRoadMarkings(base, meshUV.x, meshUV.y, uv.x, uv.y); break;
        case 12u: c = surfWater(base, meshUV.x, meshUV.y, worldPos, time); break;
        case 13u: c = surfTerrain(base, worldPos); break;
        default:  return base;
    }
    return clamp(c, 0.0, 1.0);
}
vec3 applyCheckerboard(vec3 albedo, vec3 worldPos) {
    int cx = int(floor(worldPos.x));
    int cz = int(floor(worldPos.z));
    bool dark = ((cx + cz) & 1) != 0;
    return dark ? albedo * 0.3 : albedo;
}

// ---- procedural surface relief (ported from surfaces.metal) ----------------
// Normal/roughness perturbation for the surfaces that carry no baked normal or
// roughness map (road, terrain, water). Runs AFTER the normal-map sample, so it
// perturbs whatever normal that produced — road and terrain add to it, water
// replaces it outright.

// Road micro-relief. The 0.35 tilt (not 0.6) is deliberate: the harder tilt was
// tuned under the SUN, but street lamps are close, low lights whose direction
// rakes the surface — the old tilt turned their pools into blotchy patches.
// Ported from surface_road.metal (metal c5fe504).
void surfaceReliefRoad(vec3 worldPos, inout vec3 normal, inout float rough) {
    float rx = worldPos.x, rz = worldPos.z;
    float b0 = vnoise2(rx * 2.6, rz * 2.6) + 0.5 * vnoise2(rx * 11.0, rz * 11.0)
             + 0.3 * vnoise2(rx * 37.0, rz * 37.0);
    float bx = vnoise2(rx * 2.6 + 0.4, rz * 2.6) + 0.5 * vnoise2(rx * 11.0 + 1.7, rz * 11.0)
             + 0.3 * vnoise2(rx * 37.0 + 2.3, rz * 37.0) - b0;
    float bz = vnoise2(rx * 2.6, rz * 2.6 + 0.4) + 0.5 * vnoise2(rx * 11.0, rz * 11.0 + 1.7)
             + 0.3 * vnoise2(rx * 37.0, rz * 37.0 + 2.3) - b0;
    normal = normalize(normal + vec3(-bx, 0.0, -bz) * 0.35);
    float spk = vnoise2(rx * 23.0, rz * 23.0);
    rough = clamp(rough + (spk - 0.5) * 0.25, 0.55, 1.0);
}

// Natural ground micro-relief: slope-scaled normal perturbation (steep rock
// tilts hard, flat sand/grass stays gentle) + roughness variation (rock rough,
// high flat snow a touch glossier). Ported from surface_terrain.metal.
void surfaceReliefTerrain(vec3 worldPos, inout vec3 normal, inout float rough) {
    float wx = worldPos.x, wz = worldPos.z;
    float slope = clamp(1.0 - normal.y, 0.0, 1.0);
    float amp = 0.28 + 0.65 * slope;                 // steeper => more relief
    float g0 = vnoise2(wx * 1.7, wz * 1.7) + 0.5 * vnoise2(wx * 5.3, wz * 5.3)
             + 0.3 * vnoise2(wx * 15.0, wz * 15.0);
    float gx = vnoise2(wx * 1.7 + 0.4, wz * 1.7) + 0.5 * vnoise2(wx * 5.3 + 1.7, wz * 5.3)
             + 0.3 * vnoise2(wx * 15.0 + 2.3, wz * 15.0) - g0;
    float gz = vnoise2(wx * 1.7, wz * 1.7 + 0.4) + 0.5 * vnoise2(wx * 5.3, wz * 5.3 + 1.7)
             + 0.3 * vnoise2(wx * 15.0, wz * 15.0 + 2.3) - g0;
    normal = normalize(normal + vec3(-gx, 0.0, -gz) * amp);
    float snowy = clamp((worldPos.y - 80.0) / 30.0, 0.0, 1.0) * (1.0 - slope);
    rough = clamp(mix(0.93, 0.72, snowy) + (vnoise2(wx * 11.0, wz * 11.0) - 0.5) * 0.14,
                  0.55, 1.0);
}

// Ocean waves: an animated multi-scale wave normal — two swells at offset
// headings + two chop trains, analytic slopes, amplitude-modulated by slow
// noise patches, plus a distance-faded micro-ripple. Crest sharpening + a
// roughness lift near crests structure the sun glint. Ported from
// surface_water.metal.
void surfaceReliefWater(vec3 worldPos, vec2 meshUV, float time, vec3 cameraPos,
                        inout vec3 normal, inout float rough) {
    float wx = worldPos.x, wz = worldPos.z;
    float wt = time;
    vec2 P = vec2(wx, wz);
    // Shallows damp the swell so the waterline doesn't wobble the beach.
    float depthFade = 0.15 + 0.85 * clamp(meshUV.x / 6.0, 0.0, 1.0);
    float slopeX = 0.0, slopeZ = 0.0, crest = 0.0;
    const vec3 trainDirLen[4] = vec3[4](   // dir.x, dir.z, wavelength (m)
        vec3( 0.980,  0.199, 58.0),
        vec3( 0.845,  0.535, 47.0),
        vec3( 0.827, -0.562, 21.0),
        vec3( 0.399,  0.917,  8.5));
    const float trainAmp[4]   = float[4](0.115, 0.100, 0.075, 0.080);
    const float trainSpeed[4] = float[4](6.5, 5.8, 3.8, 2.1);
    // Each train fades once its wavelength drops toward the pixel footprint —
    // otherwise the short chop aliases into a corduroy grating at distance.
    float dist = length(cameraPos - worldPos);
    const float trainFade[4] = float[4](0.00035, 0.00045, 0.0012, 0.0035);
    for (int i = 0; i < 4; ++i) {
        float fade = exp(-dist * trainFade[i]);
        if (fade < 0.02) continue;
        vec2 d = trainDirLen[i].xy;
        float k = 6.2831853 / trainDirLen[i].z;
        // slow amplitude patches: wave GROUPS, not an infinite even train
        float ampPatch = 0.45 + 0.55 * vnoise2(wx * 0.013 + float(i) * 7.31,
                                            wz * 0.013 - float(i) * 3.17);
        // low-frequency phase warp bends the crest lines so no train reads as
        // a ruled grating stretching to the horizon (short trains bend more)
        float warpF = 0.008 + 0.014 * float(i);
        float warp = (2.8 + 1.6 * float(i)) * vnoise2(wx * warpF - float(i) * 5.7,
                                                      wz * warpF + float(i) * 2.9);
        float ph = dot(P, d) * k - wt * trainSpeed[i] * k + warp;
        float s = sin(ph), cph = cos(ph);
        float w = trainAmp[i] * ampPatch * fade;
        float slope = w * cph * (0.55 + 0.45 * (s * 0.5 + 0.5));  // sharpened crests
        slopeX += d.x * slope;
        slopeZ += d.y * slope;
        crest += w * (s * 0.5 + 0.5);
    }
    // Micro-ripple: two advected octaves, faded by camera distance so the
    // horizon stays calm instead of shimmering with sub-pixel noise.
    float lodFade = exp(-dist * 0.004);
    if (lodFade > 0.02) {
        float g0 = vnoise2(wx * 1.6 + wt * 0.50, wz * 1.6 + wt * 0.23) +
                   0.5 * vnoise2(wx * 4.7 - wt * 0.40, wz * 4.7 + wt * 0.31);
        float gx = vnoise2(wx * 1.6 + 0.30 + wt * 0.50, wz * 1.6 + wt * 0.23) +
                   0.5 * vnoise2(wx * 4.7 + 0.30 - wt * 0.40, wz * 4.7 + wt * 0.31) - g0;
        float gz = vnoise2(wx * 1.6 + wt * 0.50, wz * 1.6 + 0.30 + wt * 0.23) +
                   0.5 * vnoise2(wx * 4.7 - wt * 0.40, wz * 4.7 + 0.30 + wt * 0.31) - g0;
        slopeX += gx * 0.45 * lodFade;
        slopeZ += gz * 0.45 * lodFade;
    }
    normal = normalize(vec3(-slopeX * depthFade, 1.0, -slopeZ * depthFade));
    // Sea-state roughness: glassy troughs, scuffed crests.
    rough = clamp(rough + smoothstep(0.10, 0.30, crest) * 0.10, 0.02, 0.30);
}

// The three ids are mutually exclusive; mirrors the applySurfaceRelief
// dispatcher in surfaces.metal.
void applySurfaceRelief(uint id, vec3 worldPos, vec2 meshUV, float time,
                        vec3 cameraPos, inout vec3 normal, inout float rough) {
    if (id == 11u)      surfaceReliefRoad(worldPos, normal, rough);
    else if (id == 13u) surfaceReliefTerrain(worldPos, normal, rough);
    else if (id == 12u) surfaceReliefWater(worldPos, meshUV, time, cameraPos,
                                           normal, rough);
}

// ---- cascaded shadows (ported from shadows.metal) --------------------------
// Vulkan: no uv y-flip (the shadow VP is unflipped, so write and read use the
// same NDC->uv mapping). Forward-Z light depth in [0,1].
float sampleCascade(int slice, vec3 worldPos, vec3 N) {
    vec3 biased = worldPos + N * g.shadowParams.x;          // normalBias
    vec4 clip = g.cascadeVP[slice] * vec4(biased, 1.0);
    vec3 ndc = clip.xyz / clip.w;
    vec2 uv = ndc.xy * 0.5 + 0.5;
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) return 1.0;
    if (ndc.z < 0.0 || ndc.z > 1.0) return 1.0;
    float texel = 1.0 / g.shadowParams.z;                   // 1 / mapSize
    float pcf = g.shadowParams.y;                           // pcfRadius
    float s = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x) {
            vec2 off = vec2(float(x), float(y)) * texel * pcf;
            s += texture(shadowMap, vec4(uv + off, float(slice), ndc.z));
        }
    return s / 9.0;
}

// TERRAIN HORIZON: mountain shadows at landscape scale (see
// engine/procgen/terrain_horizon.h). sin(horizon elevation) toward the
// slot-0 light over the world square; the light below it is behind the
// ridge. A soft edge of ~1.4 deg so the shadow line does not shimmer as
// the sun moves; outside the map, nothing is known and nothing shadows.
float terrainShadow(vec3 worldPos, vec3 L) {
    if (g.terrainHorizon.w < 0.5) return 1.0;
    vec2 uv = (worldPos.xz - g.terrainHorizon.xy) * g.terrainHorizon.z;
    if (uv.x < 0.0 || uv.y < 0.0 || uv.x > 1.0 || uv.y > 1.0) return 1.0;
    float h = g.terrainHorizon2.x + g.terrainHorizon2.y * texture(terrainHorizonMap, uv).r;
    return smoothstep(h - 0.012, h + 0.012, L.y);
}

float computeShadow(vec3 worldPos, vec3 N, float viewDepth) {
    int count = g.counts.y;
    if (count <= 0) return 1.0;
    int c = count - 1;
    for (int i = 0; i < count; ++i)
        if (viewDepth < g.cascadeSplit[i]) { c = i; break; }

    float v = sampleCascade(c, worldPos, N);
    if (c + 1 < count) {
        float splitFar = g.cascadeSplit[c];
        float splitNear = (c == 0) ? 0.0 : g.cascadeSplit[c - 1];
        float bandStart = mix(splitFar, splitNear, 0.15);
        if (viewDepth > bandStart) {
            float t = clamp((viewDepth - bandStart) / (splitFar - bandStart), 0.0, 1.0);
            v = mix(v, sampleCascade(c + 1, worldPos, N), t);
        }
    } else {
        float farSplit = g.cascadeSplit[count - 1];
        float fadeStart = farSplit * 0.8;
        if (viewDepth > fadeStart) {
            float t = clamp((viewDepth - fadeStart) / (farSplit - fadeStart), 0.0, 1.0);
            v = mix(v, 1.0, t);
        }
    }
    return v;
}

// ---- main ------------------------------------------------------------------
vec3 evaluateLighting(vec3 worldPos, vec3 N, vec3 V, vec3 albedo,
                      float metallic, float roughness, vec3 f0, vec3 sunShadow) {
    vec3 directLight = vec3(0.0);
    float a = max(roughness * roughness, 0.002);
    float a2 = a * a;
    float NdotV = max(dot(N, V), 1e-4);

    int count = min(g.counts.x, 32);
    for (int i = 0; i < count; ++i) {
        Light light = g.lights[i];
        int type = int(light.typeRange.x);
        vec3 L;
        float attenuation;
        if (type == 1) {                       // directional
            L = normalize(light.directionInner.xyz);
            attenuation = light.positionIntensity.w;
        } else {
            L = light.positionIntensity.xyz - worldPos;
            float dist = length(L);
            L = normalize(L);
            attenuation = light.positionIntensity.w * distanceAttenuation(dist, light.typeRange.y);
            if (type == 2) {                   // spot
                float theta = dot(-L, normalize(light.directionInner.xyz));
                attenuation *= smoothstep(light.colorOuter.w, light.directionInner.w, theta);
            }
        }
        float NdotL = max(dot(N, L), 0.0);
        if (NdotL <= 0.0 || attenuation <= 0.0) continue;

        vec3 H = normalize(L + V);
        float NdotH = max(dot(N, H), 0.0);
        float VdotH = max(dot(V, H), 0.0);
        float D = distributionGGX(NdotH, a2);
        float Vis = visibilitySmithGGX(NdotV, NdotL, a2);
        vec3 F = fresnelSchlickVec(VdotH, f0);
        vec3 specular = D * Vis * F;
        vec3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo / PI;
        vec3 sh = (type == 1) ? sunShadow : vec3(1.0);   // only the sun casts shadows
        directLight += (diffuse + specular) * light.colorOuter.rgb * (attenuation * NdotL) * sh;
    }
    return directLight;
}

void main() {
    vec3 albedo = pc.albedoMetallic.rgb * inColor;
    float metallic = clamp(pc.albedoMetallic.a, 0.0, 1.0);
    float roughness = clamp(pc.emissionRough.a, 0.04, 1.0);
    vec3 emission = pc.emissionRough.rgb;
    uint texFlags = pc.surfaceFlags.z;
    float ao = 1.0;

    // Texture maps (glTF convention: MR = (_, roughness=g, metallic=b)).
    // Alpha-cut foliage (FLAG_ALPHA_TEST = bit 1): drop fragments under the leaf
    // mask (the albedo map's alpha) before any shading. Ports lighting.metal.
    if ((texFlags & 1u) != 0u || (pc.surfaceFlags.y & 2u) != 0u) {
        vec4 albedoTex = texture(albedoMap, inTexcoord);
        if ((texFlags & 1u) != 0u) albedo *= albedoTex.rgb;
        if ((pc.surfaceFlags.y & 2u) != 0u && albedoTex.a < 0.5) discard;
    }
    if ((texFlags & 2u) != 0u) {
        vec2 mr = texture(metallicRoughnessMap, inTexcoord).gb;
        roughness = clamp(roughness * mr.x, 0.04, 1.0);
        metallic = clamp(metallic * mr.y, 0.0, 1.0);
    }
    if ((texFlags & 8u) != 0u) ao = texture(aoMap, inTexcoord).r;
    if ((texFlags & 16u) != 0u) emission *= texture(emissiveMap, inTexcoord).rgb;

    vec3 N = normalize(inWorldNormal);
    // Normal map (bit 2): perturb N in tangent space. Gram-Schmidt the tangent
    // against N, then T,B,N form the TBN basis (ported from lighting.metal).
    if ((texFlags & 4u) != 0u) {
        vec3 T = normalize(inWorldTangent - N * dot(N, inWorldTangent));
        vec3 B = cross(N, T);
        vec3 tsN = texture(normalMap, inTexcoord).xyz * 2.0 - 1.0;
        N = normalize(T * tsN.x + B * tsN.y + N * tsN.z);
    }

    uint rawFlags = pc.surfaceFlags.y;
    // Both the checkerboard and the procedural surface yield to a BAKED albedo
    // map (texFlags bit 0): a textured material keeps its surface id only as
    // save/load provenance, and the texture drives the look. The port had
    // dropped Metal's !(tf & 1u) gate, so baked-facade buildings (brick from
    // surface_maps.cpp) got the ANALYTIC brick multiplied on top — the wormy
    // contour interference seen on device. Mirrors lighting_surface.metal.
    if ((texFlags & 1u) == 0u) {
        if ((rawFlags & 1u) != 0u) albedo = applyCheckerboard(albedo, inWorldPos);
        uint surfaceId = pc.surfaceFlags.x;
        if (surfaceId != 0u)
            albedo = applySurface(surfaceId, albedo, inWorldPos, N, inTexcoord, g.wind1.w);
    }
    // Procedural relief for surfaces with no baked normal/roughness map (road,
    // terrain, water); mirrors the applySurfaceRelief seam in surfaces.metal.
    // (Runs regardless of the albedo gate above, exactly like Metal — the
    // relief ids never carry baked maps.)
    applySurfaceRelief(pc.surfaceFlags.x, inWorldPos, inTexcoord, g.wind1.w,
                       g.cameraPosition.xyz, N, roughness);

    vec3 V = normalize(g.cameraPosition.xyz - inWorldPos);
    vec3 f0 = mix(vec3(0.04), albedo, metallic);

    // Sun shadow (cascaded). viewDepth is the positive view-space depth; camera
    // looks down -z, so -(view * worldPos).z.
    float sunVis = 1.0;
    if (g.counts.y > 0) {
        float viewDepth = -(g.view * vec4(inWorldPos, 1.0)).z;
        sunVis = computeShadow(inWorldPos, N, viewDepth);
    }
    // The mountains' shadow rides the same visibility (light 0 = the sun by
    // day, the moon by night; the horizon map was marched toward it).
    if (g.counts.x > 0)
        sunVis *= terrainShadow(inWorldPos, normalize(g.lights[0].directionInner.xyz));
    float strength = g.shadowParams.w;
    float sunShadow = mix(1.0, sunVis, strength);   // scalar form for the shadow debug view (5)
    // Artistic shadow tint (ADR-0017): occluded direct light + ambient lerp toward
    // the tint; ambientStrength (shadowTint.w) controls how much the same shadow
    // darkens IBL (so a bright sky doesn't fill shadows back in). Tint 0 = plain
    // darkening, identical to the untinted scalar.
    vec3 directShadow  = mix(vec3(1.0), g.shadowTint.rgb, (1.0 - sunVis) * strength);
    vec3 ambientShadow = mix(vec3(1.0), g.shadowTint.rgb, (1.0 - sunVis) * g.shadowTint.w);

    vec3 direct = evaluateLighting(inWorldPos, N, V, albedo, metallic, roughness, f0, directShadow);

    // Image-based lighting from the procedural sky (analytic approximation of
    // Metal's baked irradiance + GGX-prefiltered split-sum; Phase 4b adds the
    // real cubemap bake + BRDF LUT, and HDR-equirect mode). g.ambient carries
    // the ambient tint * multiplier as the overall IBL strength.
    float NdotV = max(dot(N, V), 1e-4);
    vec3 R = reflect(-V, N);
    // IBL source: HDR equirect when bound (counts.z==1), else the procedural sky.
    vec3 irradiance, prefiltered;
    if (g.counts.z == 1) {
        irradiance = sampleEquirect(N);
        prefiltered = mix(sampleEquirect(R), irradiance, roughness);
    } else {
        irradiance = sampleEnvironment(N);
        prefiltered = mix(sampleEnvironment(R), irradiance, roughness);  // crude roughness blur
    }
    vec3 Famb = fresnelSchlickRoughness(NdotV, f0, roughness);
    vec3 kd = (1.0 - Famb) * (1.0 - metallic);
    vec3 envDiffuse = kd * albedo * irradiance * ao;
    // Split-sum specular (ADR-0017): the baked BRDF LUT gives (scale, bias) so
    // envSpecular = prefiltered * (F0 * scale + bias) — matches Metal's IBL.
    vec2 brdf = texture(brdfLut, vec2(NdotV, roughness)).rg;
    vec3 envSpecular = prefiltered * (f0 * brdf.x + brdf.y);
    vec3 ambient = (envDiffuse + envSpecular) * g.ambient.rgb * ambientShadow;

    // Debug views that need lit-pass data write display-ready values into the HDR
    // target; the composite shows them raw (counts.w carries the selector — see
    // Renderer::debugView). Buffer views (AO=1, SSR=2, normals=4) stay
    // composite-side. Parity with post.metal's debug modes.
    int dbg = g.counts.w;
    if (dbg == 3) {
        // Linearized view depth (white = near, black = far). Normalized by the
        // furthest cascade split when shadows exist, else a scene-scale fallback.
        float vd = -(g.view * vec4(inWorldPos, 1.0)).z;
        float farDist = (g.counts.y > 0) ? g.cascadeSplit[g.counts.y - 1] : 200.0;
        float lin = clamp(1.0 - vd / max(farDist, 1e-3), 0.0, 1.0);
        outColor = vec4(vec3(lin), 1.0);
    } else if (dbg == 5) {
        outColor = vec4(vec3(sunShadow), 1.0);              // white = lit, black = shadowed
    } else if (dbg == 6) {
        outColor = vec4(pow(albedo, vec3(1.0 / 2.2)), 1.0); // raw albedo (gamma only)
    } else if (dbg == 7) {
        float ndv = dot(N, V);                              // green = front, red = back
        outColor = ndv >= 0.0 ? vec4(0.0, ndv, 0.0, 1.0) : vec4(-ndv, 0.0, 0.0, 1.0);
    } else if (dbg == 8) {
        // Shadow cascades: red/green/blue/yellow = cascade 0..3.
        vec3 tint = vec3(0.0);
        if (g.counts.y > 0) {
            float vd = -(g.view * vec4(inWorldPos, 1.0)).z;
            int cc = g.counts.y, ci = cc - 1;
            for (int i = 0; i < cc; ++i) if (vd < g.cascadeSplit[i]) { ci = i; break; }
            vec3 tints[4] = vec3[4](vec3(1.0, 0.0, 0.0), vec3(0.0, 1.0, 0.0),
                                    vec3(0.0, 0.0, 1.0), vec3(1.0, 1.0, 0.0));
            tint = tints[clamp(ci, 0, 3)];
        }
        outColor = vec4(tint, 1.0);
    } else {
        vec3 color = direct + ambient + emission;
        // Aerial-perspective fog. The fade target is the SKY the surface
        // occludes, not the authored fog colour — the P5 fog-restoration
        // semantics (metal 6d20e85): on Metal the metro runs the scattering
        // sky, whose aerial path uses fog.rgb only as optical depth and
        // inscatters the live sky, so haze stays colour-correct at sunset and
        // under the moon. Vulkan faded toward the fixed fog.rgb, which turned
        // night distances WHITE. fog.rgb keeps its density-only role here.
        if (g.fog.w > 0.0) {
            vec3 rd = (inWorldPos - g.cameraPosition.xyz);
            float dist = length(rd);
            float f = 1.0 - exp(-g.fog.w * dist);
            color = mix(color, sampleEnvironment(rd / max(dist, 1e-4)), f);
        }
        // Opacity (float bits in the spare push slot) → output alpha for the
        // transparent blend pass; ignored by the opaque pipeline (blend off).
        outColor = vec4(color, uintBitsToFloat(pc.surfaceFlags.w));
    }
    outNormal = vec4(N * 0.5 + 0.5, roughness);   // world normal (SSAO) + roughness (SSR gate)
}
