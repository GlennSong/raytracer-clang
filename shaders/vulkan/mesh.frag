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
} g;

layout(set = 0, binding = 1) uniform sampler2DArrayShadow shadowMap;
layout(set = 0, binding = 2) uniform sampler2D envEquirect;   // HDR env (counts.z==1)

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
float hash21(float a, float b) { return fract(sin(a * 12.9898 + b * 78.233) * 43758.5453); }
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
    float shade = clamp(0.92 + 0.46 * (spk - 0.5) + 0.12 * (blotch - 0.5), 0.5, 1.4);
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
vec3 surfRoadMarkings(vec3 base, float mu, float mv) {
    if (mu < 0.5) return base;
    float lat = mu - 2.0;
    float yL = 1.0 - smoothstep(0.013, 0.019, abs(lat - 0.030));
    float yR = 1.0 - smoothstep(0.013, 0.019, abs(lat + 0.030));
    float y  = max(yL, yR);
    float wL = 1.0 - smoothstep(0.016, 0.022, abs(lat - 0.86));
    float wR = 1.0 - smoothstep(0.016, 0.022, abs(lat + 0.86));
    float w  = max(wL, wR);
    vec3 c = mix(base, vec3(0.82, 0.68, 0.13), y);
    c = mix(c, vec3(0.86, 0.86, 0.83), w);
    return c;
}
vec3 applySurface(uint id, vec3 base, vec3 worldPos, vec3 n, vec2 meshUV) {
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
        case 11u: c = surfRoadMarkings(base, meshUV.x, meshUV.y); break;
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
                      float metallic, float roughness, vec3 f0, float sunShadow) {
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
        float sh = (type == 1) ? sunShadow : 1.0;   // only the sun casts shadows
        directLight += (diffuse + specular) * light.colorOuter.rgb * (attenuation * NdotL * sh);
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
    if ((texFlags & 1u) != 0u) albedo *= texture(albedoMap, inTexcoord).rgb;
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
    if ((rawFlags & 1u) != 0u) albedo = applyCheckerboard(albedo, inWorldPos);
    uint surfaceId = pc.surfaceFlags.x;
    if (surfaceId != 0u) albedo = applySurface(surfaceId, albedo, inWorldPos, N, inTexcoord);

    vec3 V = normalize(g.cameraPosition.xyz - inWorldPos);
    vec3 f0 = mix(vec3(0.04), albedo, metallic);

    // Sun shadow (cascaded). viewDepth is the positive view-space depth; camera
    // looks down -z, so -(view * worldPos).z. strength scales the darkening.
    float sunShadow = 1.0;
    if (g.counts.y > 0) {
        float viewDepth = -(g.view * vec4(inWorldPos, 1.0)).z;
        sunShadow = computeShadow(inWorldPos, N, viewDepth);
        sunShadow = mix(1.0, sunShadow, g.shadowParams.w);
    }

    vec3 direct = evaluateLighting(inWorldPos, N, V, albedo, metallic, roughness, f0, sunShadow);

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
    vec3 envSpecular = prefiltered * Famb;
    vec3 ambient = (envDiffuse + envSpecular) * g.ambient.rgb;

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
        // Aerial-perspective fog (ports lighting.metal): lerp toward fog color by
        // 1-exp(-density*dist) in scene-linear space, before the composite tonemap.
        if (g.fog.w > 0.0) {
            float dist = length(inWorldPos - g.cameraPosition.xyz);
            float f = 1.0 - exp(-g.fog.w * dist);
            color = mix(color, g.fog.rgb, f);
        }
        // Opacity (float bits in the spare push slot) → output alpha for the
        // transparent blend pass; ignored by the opaque pipeline (blend off).
        outColor = vec4(color, uintBitsToFloat(pc.surfaceFlags.w));
    }
    outNormal = vec4(N * 0.5 + 0.5, roughness);   // world normal (SSAO) + roughness (SSR gate)
}
