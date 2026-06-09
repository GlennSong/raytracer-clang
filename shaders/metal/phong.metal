#include <metal_stdlib>
using namespace metal;

struct Vertex {
    packed_float3 position;
    packed_float3 normal;
    packed_float3 tangent;
    packed_float2 texcoord;
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
};

struct CameraUniforms {
    float4x4 viewProjection;
    float4x4 view;
    float3 cameraPosition;
    float _camPad0;
    float4x4 invViewProjection;
    float4x4 projection;
    float4x4 invProjection;
    float2 screenSize;
    float nearPlane;
    float farPlane;
};

struct ModelUniforms {
    float4x4 model;
    float4x4 normalMatrix;
};

struct MaterialUniforms {
    float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    float flags;
    uint textureFlags;
    float3 emission;
};

enum LightType : int {
    LightType_Point       = 0,
    LightType_Directional = 1,
    LightType_Spot        = 2,
};

struct Light {
    float3      position;
    float       intensity;
    float3      direction;
    float       innerCosAngle;
    float3      color;
    float       outerCosAngle;
    float4x4    lightViewProjection;
    int         type;
    int         shadowMapIndex;
    float       _pad[2];
};

struct LightUniforms {
    Light  lights[32];
    int    lightCount;
    float  exposure;
    float  ambientMultiplier;
    float  _pad[1];
    // Procedural sky (ADR-0016, day/night). Mirrors C++ `LightUniforms`; each
    // float3 packs with its trailing scalar into 16 bytes (as the Light struct
    // already relies on). Written from `SceneLighting::sky` in setLights().
    float3 skySunDir;     float skySunIntensity;  // disc brightness, 0 at night
    float3 skySunColor;   float _skp0;
    float3 skyZenith;     float _skp1;
    float3 skyHorizon;    float _skp2;
    float3 skyGround;     float _skp3;
    // Procedural clouds (ADR-0016 step 3). time = drift phase in seconds.
    float skyCloudCoverage; float skyCloudDensity;
    float skyCloudScale;    float skyCloudTime;
};

struct SSRParams {
    float maxRayDist;
    float thickness;
    float thicknessFar;
    float stride;
    float blendStrength;
    float _pad[3];
};

struct SSAOParams {
    float radius;
    float intensity;
    float bias;
    int   directions;
    int   steps;
    float _pad[3];
};

// Environment selection (ADR-0016). mode 0 = procedural sky, 1 = HDR equirect.
// cloudsEnabled gates the procedural cloud overlay so the reflection-probe bake
// (which reuses this shader) can render a clouds-free sky — animated clouds are
// a screen/SSR visual only, never baked into probes.
struct EnvUniforms {
    int   mode;
    int   cloudsEnabled;
    float _pad[2];
};

// Sample an equirectangular (lat-long) environment map by world-space direction.
// u wraps around the horizon (longitude), v runs zenith→nadir (latitude).
float3 sampleEquirect(texture2d<float> envMap, sampler s, float3 dir) {
    float u = atan2(dir.z, dir.x) * (0.5 / M_PI_F) + 0.5;
    float v = acos(clamp(dir.y, -1.0, 1.0)) * (1.0 / M_PI_F);
    return envMap.sample(s, float2(u, v)).rgb;
}

// Procedural sky environment. Colors and the sun arc come from `env` (the day/
// night state baked into LightUniforms), so dawn→day→dusk→night grade smoothly
// and the disc fades out as the sun sets. The shader only interpolates; the
// time-of-day curve is computed engine-side (DayNightCycle).
float3 sampleEnvironment(float3 dir, device const LightUniforms& env) {
    float skyBlend = saturate(dir.y);
    float3 sky = mix(env.skyHorizon, env.skyZenith, pow(skyBlend, 0.5));

    // Soft ground plane below horizon
    float horizonBlend = smoothstep(-0.05, 0.05, dir.y);
    float3 col = mix(env.skyGround, sky, horizonBlend);

    // Sun disc and glow — `skySunIntensity` is 0 at night, so they vanish.
    float disc = env.skySunIntensity;
    float3 sc = env.skySunColor;
    float sunDot = max(dot(dir, env.skySunDir), 0.0);
    col += sc * pow(sunDot, 256.0) * 8.0  * disc;   // bright disc
    col += sc * pow(sunDot, 32.0)  * 1.0  * disc;   // inner glow
    col += sc * pow(sunDot, 4.0)   * 0.15 * disc;   // outer halo

    // Subtle atmospheric scattering near horizon, tinted by the sun
    float horizonGlow = pow(1.0 - abs(dir.y), 8.0);
    col += sc * horizonGlow * 0.1 * disc;

    return col;
}

// --- Procedural clouds (ADR-0016 step 3) ---
// An FBM noise layer painted on the sky dome — not volumetric, and never baked
// into reflection probes (a screen/SSR visual only). Overlaid by the skybox and
// composite passes on top of the day/night sky from sampleEnvironment().

float cloudHash(float2 p) {
    p = fract(p * float2(123.34, 345.45));
    p += dot(p, p + 34.345);
    return fract(p.x * p.y);
}

float cloudNoise(float2 p) {
    float2 i = floor(p);
    float2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);                 // smoothstep interpolation
    float a = cloudHash(i + float2(0.0, 0.0));
    float b = cloudHash(i + float2(1.0, 0.0));
    float c = cloudHash(i + float2(0.0, 1.0));
    float d = cloudHash(i + float2(1.0, 1.0));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

float cloudFbm(float2 p) {
    float sum = 0.0;
    float amp = 0.5;
    for (int i = 0; i < 5; i++) {
        sum += amp * cloudNoise(p);
        p *= 2.02;
        amp *= 0.5;
    }
    return sum;
}

// Overlay clouds onto a base sky color for a viewing direction.
float3 applyClouds(float3 baseSky, float3 dir, device const LightUniforms& env) {
    if (dir.y <= 0.02) return baseSky;           // below horizon: no clouds

    // Planar projection of the dome onto a cloud plane: directions near the
    // horizon stretch (parallax), overhead compresses. Drift the field over time.
    float2 uv = dir.xz / (dir.y + 0.10);
    float2 wind = float2(env.skyCloudTime * 0.02, env.skyCloudTime * 0.012);
    float n = cloudFbm(uv * env.skyCloudScale + wind);

    // Coverage threshold → soft cloud mask, faded near the horizon.
    float cov = env.skyCloudCoverage;
    float mask = smoothstep(cov, cov + 0.20, n) * env.skyCloudDensity;
    mask *= smoothstep(0.02, 0.25, dir.y);
    mask = saturate(mask);

    // Shade: sunlit tops toward the sun, shadowed base away; whole layer tracks
    // day/night brightness (≈0 at night → dark silhouettes) and the sun tint.
    float sunAmt = max(dot(dir, env.skySunDir), 0.0);
    float3 cloudLit  = env.skySunColor;
    float3 cloudDark = float3(0.40, 0.42, 0.50);
    float3 cloudColor = mix(cloudDark, cloudLit, pow(sunAmt, 1.5));
    cloudColor *= (0.25 + 0.75 * env.skySunIntensity);

    return mix(baseSky, cloudColor, mask);
}

// --- Skybox shaders ---
struct SkyboxOut {
    float4 position [[position]];
    float3 viewDir;
};

// Fullscreen triangle: 3 vertices cover the screen without a vertex buffer
vertex SkyboxOut vertexSkybox(
    constant CameraUniforms& camera [[buffer(1)]],
    uint vid [[vertex_id]]
) {
    // Generate fullscreen triangle (oversized, clipped to viewport)
    float2 uv = float2((vid << 1) & 2, vid & 2);
    // z=0.999 — just inside the far plane to avoid far-plane clipping on some GPUs
    float4 clipPos = float4(uv * 2.0 - 1.0, 0.999, 1.0);

    SkyboxOut out;
    out.position = clipPos;

    // Reconstruct world-space view direction via inverse view-projection.
    // This correctly accounts for FOV and aspect ratio so the sky matches
    // the scene perspective.
    float4 worldPos = camera.invViewProjection * float4(clipPos.xy, 1.0, 1.0);
    out.viewDir = normalize(worldPos.xyz / worldPos.w - camera.cameraPosition);
    return out;
}

fragment float4 fragmentSkybox(
    SkyboxOut in [[stage_in]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant EnvUniforms& env [[buffer(5)]],
    texture2d<float> envMap [[texture(0)]],
    sampler envSampler [[sampler(0)]]
) {
    float3 dir = normalize(in.viewDir);
    float3 color = (env.mode == 1) ? sampleEquirect(envMap, envSampler, dir)
                                   : sampleEnvironment(dir, lightData);
    // Clouds overlay the procedural sky only (a captured HDR has its own), and
    // are skipped during the probe bake (env.cloudsEnabled == 0).
    if (env.mode == 0 && env.cloudsEnabled != 0)
        color = applyClouds(color, dir, lightData);
    color *= lightData.exposure;
    return float4(color, 1.0);  // linear HDR — tone mapping in composite pass
}

// Checkerboard pattern based on world position
float3 applyCheckerboard(float3 albedo, float3 worldPos) {
    int cx = int(floor(worldPos.x));
    int cz = int(floor(worldPos.z));
    bool dark = ((cx + cz) & 1) != 0;
    return dark ? albedo * 0.3 : albedo;
}

float fresnelSchlick(float cosTheta, float f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

float3 fresnelSchlickVec(float cosTheta, float3 f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

struct InstanceData {
    float4x4 model;
    float4x4 normalMatrix;
    float4 albedo;      // w unused
    float metallic;
    float roughness;
    float opacity;
    float flags;
    float4 emission;    // w unused
    uint textureFlags;
    float _instPad[3];
};

struct ShadowUniforms {
    float  shadowBias;
    float  normalBias;
    float  pcfRadius;
    int    shadowMapSize;
};

// Shadow depth-only vertex shaders (no fragment needed — Metal writes depth)
vertex float4 vertexShadow(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& lightCamera [[buffer(1)]],
    constant ModelUniforms& model [[buffer(2)]],
    uint vid [[vertex_id]]
) {
    float4 worldPos = model.model * float4(vertices[vid].position, 1.0);
    return lightCamera.viewProjection * worldPos;
}

vertex float4 vertexShadowInstanced(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& lightCamera [[buffer(1)]],
    const device InstanceData* instances [[buffer(2)]],
    uint vid [[vertex_id]],
    uint iid [[instance_id]]
) {
    float4 worldPos = instances[iid].model * float4(vertices[vid].position, 1.0);
    return lightCamera.viewProjection * worldPos;
}

// Shadow sampling
float sampleShadowPCF(depth2d<float> shadowMap, sampler smp,
                       float2 uv, float compareDepth, float texelSize) {
    float shadow = 0.0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            shadow += shadowMap.sample_compare(smp, uv + float2(x, y) * texelSize, compareDepth);
        }
    }
    return shadow / 9.0;
}

float computeShadow(depth2d<float> shadowMap, sampler smp,
                     float4x4 lightVP, float3 worldPos, float texelSize) {
    float4 lightClip = lightVP * float4(worldPos, 1.0);
    float3 ndc = lightClip.xyz / lightClip.w;
    float2 uv = ndc.xy * 0.5 + 0.5;
    uv.y = 1.0 - uv.y;  // Metal texture origin is top-left
    if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) return 1.0;
    return sampleShadowPCF(shadowMap, smp, uv, ndc.z, texelSize);
}

// --- Reflection probe structures ---

struct GPUReflectionProbe {
    float3 position;
    float influenceRadius;
    float3 boxMin;
    float _pad0;
    float3 boxMax;
    int probeIndex;       // index into cubemap array
};

struct ProbeUniforms {
    int probeCount;
    int maxMipLevel;      // number of roughness mip levels in cubemap
    float _pad[2];
};

// Parallax-corrected box projection for cubemap sampling
float3 boxProjectReflection(float3 reflectDir, float3 worldPos,
                             float3 boxMin, float3 boxMax, float3 probePos) {
    float3 firstPlaneIntersect = (boxMax - worldPos) / reflectDir;
    float3 secondPlaneIntersect = (boxMin - worldPos) / reflectDir;
    float3 furthestPlane = max(firstPlaneIntersect, secondPlaneIntersect);
    float dist = min(min(furthestPlane.x, furthestPlane.y), furthestPlane.z);
    float3 intersectPos = worldPos + reflectDir * dist;
    return intersectPos - probePos;
}

// Sample reflection from probes with distance-based blending, fallback to procedural sky
float3 sampleReflectionProbes(float3 worldPos, float3 reflectDir, float roughness,
                               float3 normal, float NdotV, float3 f0,
                               device const GPUReflectionProbe* probes,
                               constant ProbeUniforms& probeParams,
                               texturecube_array<float> cubemapArray,
                               texture2d<float> brdfLUT,
                               sampler envSampler,
                               device const LightUniforms& env) {
    // Perceptual roughness-to-mip mapping — square the roughness for smoother
    // transitions between mip levels (avoids visible banding with box-filter mips)
    float mipLevel = roughness * roughness * float(probeParams.maxMipLevel);

    // Find the two best probes
    int bestIdx[2] = {-1, -1};
    float bestWeight[2] = {0.0, 0.0};

    for (int i = 0; i < probeParams.probeCount && i < 8; i++) {
        float dist = length(worldPos - probes[i].position);
        float radius = probes[i].influenceRadius;
        if (dist >= radius) continue;

        // Smooth falloff from center to edge
        float weight = 1.0 - smoothstep(radius * 0.7, radius, dist);

        if (weight > bestWeight[0]) {
            bestIdx[1] = bestIdx[0];
            bestWeight[1] = bestWeight[0];
            bestIdx[0] = i;
            bestWeight[0] = weight;
        } else if (weight > bestWeight[1]) {
            bestIdx[1] = i;
            bestWeight[1] = weight;
        }
    }

    // Sample BRDF LUT for split-sum
    float2 brdf = brdfLUT.sample(envSampler, float2(NdotV, roughness)).rg;
    float3 fresnel = f0 * brdf.x + brdf.y;

    float totalWeight = bestWeight[0] + bestWeight[1];
    if (totalWeight < 0.001) {
        // Outside all probes — fall back to procedural sky
        float3 envColor = sampleEnvironment(reflectDir, env);
        float mipBlur = roughness * roughness * roughness;  // smooth falloff
        envColor = mix(envColor, sampleEnvironment(normal, env), mipBlur);
        return fresnelSchlickVec(NdotV, f0) * envColor;
    }

    float3 probeColor = float3(0.0);
    for (int j = 0; j < 2; j++) {
        if (bestIdx[j] < 0) continue;
        int pi = bestIdx[j];
        float3 correctedDir = boxProjectReflection(reflectDir, worldPos,
                                                     probes[pi].boxMin,
                                                     probes[pi].boxMax,
                                                     probes[pi].position);
        float3 sample = cubemapArray.sample(envSampler, correctedDir,
                                             probes[pi].probeIndex, level(mipLevel)).rgb;
        probeColor += sample * (bestWeight[j] / totalWeight);
    }

    // Blend remaining weight to procedural sky fallback
    float skyWeight = 1.0 - saturate(totalWeight);
    if (skyWeight > 0.001) {
        float3 skyColor = sampleEnvironment(reflectDir, env);
        float mipBlur = roughness * roughness;
        skyColor = mix(skyColor, sampleEnvironment(normal, env), mipBlur);
        probeColor = mix(probeColor, skyColor, skyWeight);
    }

    return fresnel * probeColor;
}

// --- BRDF integration LUT compute shader (split-sum, run once at startup) ---

// Van der Corput radical inverse for Hammersley sequence
float radicalInverse_VdC(uint bits) {
    bits = (bits << 16u) | (bits >> 16u);
    bits = ((bits & 0x55555555u) << 1u) | ((bits & 0xAAAAAAAAu) >> 1u);
    bits = ((bits & 0x33333333u) << 2u) | ((bits & 0xCCCCCCCCu) >> 2u);
    bits = ((bits & 0x0F0F0F0Fu) << 4u) | ((bits & 0xF0F0F0F0u) >> 4u);
    bits = ((bits & 0x00FF00FFu) << 8u) | ((bits & 0xFF00FF00u) >> 8u);
    return float(bits) * 2.3283064365386963e-10;
}

float2 hammersley(uint i, uint N) {
    return float2(float(i) / float(N), radicalInverse_VdC(i));
}

float3 importanceSampleGGX(float2 Xi, float3 N, float roughness) {
    float a = roughness * roughness;
    float phi = 2.0 * M_PI_F * Xi.x;
    float cosTheta = sqrt((1.0 - Xi.y) / (1.0 + (a * a - 1.0) * Xi.y));
    float sinTheta = sqrt(1.0 - cosTheta * cosTheta);

    float3 H = float3(cos(phi) * sinTheta, sin(phi) * sinTheta, cosTheta);

    // Tangent-space to world-space
    float3 up = abs(N.z) < 0.999 ? float3(0, 0, 1) : float3(1, 0, 0);
    float3 tangent = normalize(cross(up, N));
    float3 bitangent = cross(N, tangent);

    return normalize(tangent * H.x + bitangent * H.y + N * H.z);
}

kernel void integrateBRDF(
    texture2d<float, access::write> lut [[texture(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    float2 texSize = float2(lut.get_width(), lut.get_height());
    if (gid.x >= uint(texSize.x) || gid.y >= uint(texSize.y)) return;

    float NdotV = (float(gid.x) + 0.5) / texSize.x;
    float roughness = (float(gid.y) + 0.5) / texSize.y;
    NdotV = max(NdotV, 0.001);

    float3 V;
    V.x = sqrt(1.0 - NdotV * NdotV);
    V.y = 0.0;
    V.z = NdotV;

    float3 N = float3(0, 0, 1);
    float A = 0.0, B = 0.0;
    const uint SAMPLES = 1024u;

    for (uint i = 0u; i < SAMPLES; i++) {
        float2 Xi = hammersley(i, SAMPLES);
        float3 H = importanceSampleGGX(Xi, N, roughness);
        float3 L = normalize(2.0 * dot(V, H) * H - V);

        float NdotL = max(L.z, 0.0);
        float NdotH = max(H.z, 0.0);
        float VdotH = max(dot(V, H), 0.0);

        if (NdotL > 0.0) {
            float a2 = roughness * roughness * roughness * roughness;
            float G_V = NdotL * (NdotV * (1.0 - sqrt(a2)) + sqrt(a2));
            float G_L = NdotV * (NdotL * (1.0 - sqrt(a2)) + sqrt(a2));
            float G = 0.5 / max(G_V + G_L, 0.001);
            float G_Vis = (G * VdotH * NdotL) / max(NdotH, 0.001);
            float Fc = pow(1.0 - VdotH, 5.0);
            A += (1.0 - Fc) * G_Vis;
            B += Fc * G_Vis;
        }
    }

    A /= float(SAMPLES);
    B /= float(SAMPLES);
    lut.write(float4(A, B, 0, 1), gid);
}

// --- Cubemap pre-filter compute shader (GGX importance sampling per mip level) ---

kernel void prefilterEnvMap(
    texturecube<float> inputCube [[texture(0)]],
    texturecube<float, access::write> outputFace [[texture(1)]],
    constant float& roughness [[buffer(0)]],
    constant int& faceIndex [[buffer(1)]],
    sampler envSampler [[sampler(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint size = outputFace.get_width();
    if (gid.x >= size || gid.y >= size) return;

    // Convert pixel to cubemap direction
    float2 uv = (float2(gid) + 0.5) / float(size) * 2.0 - 1.0;

    float3 dir;
    switch (faceIndex) {
        case 0: dir = float3( 1, -uv.y, -uv.x); break; // +X
        case 1: dir = float3(-1, -uv.y,  uv.x); break; // -X
        case 2: dir = float3( uv.x,  1,  uv.y); break; // +Y
        case 3: dir = float3( uv.x, -1, -uv.y); break; // -Y
        case 4: dir = float3( uv.x, -uv.y,  1); break; // +Z
        case 5: dir = float3(-uv.x, -uv.y, -1); break; // -Z
    }
    float3 N = normalize(dir);

    if (roughness < 0.01) {
        // No filtering needed for mip 0
        outputFace.write(inputCube.sample(envSampler, N), gid, faceIndex);
        return;
    }

    float3 R = N;
    float3 V = R;
    float3 prefilteredColor = float3(0.0);
    float totalWeight = 0.0;
    const uint SAMPLES = 512u;

    for (uint i = 0u; i < SAMPLES; i++) {
        float2 Xi = hammersley(i, SAMPLES);
        float3 H = importanceSampleGGX(Xi, N, roughness);
        float3 L = normalize(2.0 * dot(V, H) * H - V);

        float NdotL = max(dot(N, L), 0.0);
        if (NdotL > 0.0) {
            prefilteredColor += inputCube.sample(envSampler, L).rgb * NdotL;
            totalWeight += NdotL;
        }
    }

    prefilteredColor /= max(totalWeight, 0.001);
    outputFace.write(float4(prefilteredColor, 1.0), gid, faceIndex);
}

struct FragmentData {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
    float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    float flags;
    float3 emission;
    uint textureFlags;
};

vertex FragmentData vertexMainInstanced(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& camera [[buffer(1)]],
    const device InstanceData* instances [[buffer(2)]],
    uint vid [[vertex_id]],
    uint iid [[instance_id]]
) {
    InstanceData inst = instances[iid];
    FragmentData out;
    float4 worldPos = inst.model * float4(vertices[vid].position, 1.0);
    out.position = camera.viewProjection * worldPos;
    out.worldPosition = worldPos.xyz;
    out.worldNormal = normalize((inst.normalMatrix * float4(vertices[vid].normal, 0.0)).xyz);
    out.worldTangent = normalize((inst.model * float4(float3(vertices[vid].tangent), 0.0)).xyz);
    out.texcoord = vertices[vid].texcoord;
    out.albedo = inst.albedo.xyz;
    out.metallic = inst.metallic;
    out.roughness = inst.roughness;
    out.opacity = inst.opacity;
    out.flags = inst.flags;
    out.emission = inst.emission.xyz;
    out.textureFlags = inst.textureFlags;
    return out;
}

// Shared lighting calculation for all fragment shaders
float3 evaluateLighting(float3 worldPos, float3 normal, float3 viewDir,
                         float3 albedo, float metallic, float roughness,
                         float3 fresnel, device const LightUniforms& lightData,
                         depth2d<float> shadowMap, sampler shadowSmp,
                         float shadowTexelSize) {
    float3 directLight = float3(0.0);
    float shininess = mix(16.0, 512.0, 1.0 - roughness);
    float specNorm = (shininess + 8.0) / (8.0 * 3.14159);

    for (int i = 0; i < lightData.lightCount && i < 32; i++) {
        device const Light& light = lightData.lights[i];

        float3 lightDir;
        float attenuation;

        if (light.type == LightType_Directional) {
            lightDir = light.direction;
            attenuation = light.intensity;
        } else {
            lightDir = light.position - worldPos;
            float dist = length(lightDir);
            lightDir = normalize(lightDir);
            attenuation = light.intensity / (1.0 + 0.09 * dist + 0.032 * dist * dist);

            if (light.type == LightType_Spot) {
                float theta = dot(-lightDir, light.direction);
                float spotFactor = smoothstep(light.outerCosAngle,
                                              light.innerCosAngle, theta);
                attenuation *= spotFactor;
            }
        }

        // Shadow
        float shadow = 1.0;
        if (light.shadowMapIndex >= 0) {
            shadow = computeShadow(shadowMap, shadowSmp,
                                    light.lightViewProjection,
                                    worldPos, shadowTexelSize);
        }

        float diff = max(dot(normal, lightDir), 0.0);
        float3 diffuse = albedo * diff * (1.0 - metallic);

        float3 halfVec = normalize(lightDir + viewDir);
        float NdotH = max(dot(normal, halfVec), 0.0);
        float spec = pow(NdotH, shininess);
        float3 specular = fresnel * spec * specNorm;

        directLight += (diffuse + specular) * light.color * attenuation * shadow;
    }
    return directLight;
}

struct GBufferOut {
    float4 color [[color(0)]];
    float4 viewNormal [[color(1)]];
};

fragment GBufferOut fragmentMainInstanced(
    FragmentData in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    depth2d<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    float shadowTexelSize = 1.0 / float(shadowData.shadowMapSize);
    float3 albedo = in.albedo;
    float mtl = in.metallic;
    float rough = in.roughness;
    float3 emit = in.emission;
    float ao = 1.0;
    uint tf = in.textureFlags;
    if (tf & 1u) albedo *= albedoMap.sample(texSampler, in.texcoord).rgb;
    if (tf & 2u) {
        float4 mr = metalRoughMap.sample(texSampler, in.texcoord);
        rough *= mr.g;
        mtl *= mr.b;
    }
    if (tf & 8u) ao = aoMap.sample(texSampler, in.texcoord).r;
    if (tf & 16u) emit *= emissiveMap.sample(texSampler, in.texcoord).rgb;
    if (!(tf & 1u) && (int(in.flags) & 1)) albedo = applyCheckerboard(albedo, in.worldPosition);

    float3 normal = normalize(in.worldNormal);
    if (tf & 4u) {
        float3 T = normalize(in.worldTangent - normal * dot(normal, in.worldTangent));
        float3 B = cross(normal, T);
        float3 tsNormal = normalMap.sample(texSampler, in.texcoord).xyz * 2.0 - 1.0;
        normal = normalize(T * tsNormal.x + B * tsNormal.y + normal * tsNormal.z);
    }
    float3 viewDir = normalize(camera.cameraPosition - in.worldPosition);
    float NdotV = max(dot(normal, viewDir), 0.0);

    float3 reflectDir = reflect(-viewDir, normal);
    float3 f0 = mix(float3(0.04), albedo, mtl);

    float3 envSpecular;
    float3 envReflection;
    if (probeParams.probeCount > 0) {
        envSpecular = sampleReflectionProbes(in.worldPosition, reflectDir,
                                              rough, normal, NdotV, f0,
                                              probes, probeParams,
                                              cubemapArray, brdfLUT, envSampler,
                                              lightData);
        envReflection = cubemapArray.sample(envSampler, reflectDir, 0, level(0.0)).rgb;
    } else {
        envReflection = sampleEnvironment(reflectDir, lightData);
        float3 fresnel = fresnelSchlickVec(NdotV, f0);
        float mipBlur = rough * rough;
        float3 envBlurred = mix(envReflection, sampleEnvironment(normal, lightData), mipBlur);
        envSpecular = fresnel * envBlurred;
    }

    float3 fresnel = fresnelSchlickVec(NdotV, f0);
    float3 directLight = evaluateLighting(in.worldPosition, normal, viewDir,
                                           albedo, mtl, rough,
                                           fresnel, lightData,
                                           shadowMap, shadowSampler,
                                           shadowTexelSize);

    float3 ambientDiffuse = albedo * sampleEnvironment(normal, lightData) * lightData.ambientMultiplier * (1.0 - mtl) * ao;
    float3 color = emit + directLight + ambientDiffuse + envSpecular;

    float alpha = in.opacity;
    if (alpha < 1.0) {
        float fresnelTerm = fresnelSchlick(NdotV, 0.04);
        alpha = mix(alpha, 1.0, fresnelTerm);
        color += envReflection * fresnelTerm * 0.8;
    }

    color *= lightData.exposure;
    float3 viewN = normalize((camera.view * float4(normal, 0.0)).xyz);
    GBufferOut out;
    out.color = float4(color, alpha);
    out.viewNormal = float4(viewN * 0.5 + 0.5, 1.0);
    return out;
}

vertex VertexOut vertexMain(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant ModelUniforms& model [[buffer(2)]],
    uint vid [[vertex_id]]
) {
    VertexOut out;
    float4 worldPos = model.model * float4(vertices[vid].position, 1.0);
    out.position = camera.viewProjection * worldPos;
    out.worldPosition = worldPos.xyz;
    out.worldNormal = normalize((model.normalMatrix * float4(vertices[vid].normal, 0.0)).xyz);
    out.worldTangent = normalize((model.model * float4(float3(vertices[vid].tangent), 0.0)).xyz);
    out.texcoord = vertices[vid].texcoord;
    return out;
}

fragment GBufferOut fragmentMain(
    VertexOut in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant MaterialUniforms& material [[buffer(3)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    depth2d<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    float shadowTexelSize = 1.0 / float(shadowData.shadowMapSize);
    float3 albedo = material.albedo;
    float mtl = material.metallic;
    float rough = material.roughness;
    float3 emit = material.emission;
    float ao = 1.0;
    uint tf = material.textureFlags;
    if (tf & 1u) albedo *= albedoMap.sample(texSampler, in.texcoord).rgb;
    if (tf & 2u) {
        float4 mr = metalRoughMap.sample(texSampler, in.texcoord);
        rough *= mr.g;
        mtl *= mr.b;
    }
    if (tf & 8u) ao = aoMap.sample(texSampler, in.texcoord).r;
    if (tf & 16u) emit *= emissiveMap.sample(texSampler, in.texcoord).rgb;
    if (!(tf & 1u) && (int(material.flags) & 1)) albedo = applyCheckerboard(albedo, in.worldPosition);

    float3 normal = normalize(in.worldNormal);
    if (tf & 4u) {
        float3 T = normalize(in.worldTangent - normal * dot(normal, in.worldTangent));
        float3 B = cross(normal, T);
        float3 tsNormal = normalMap.sample(texSampler, in.texcoord).xyz * 2.0 - 1.0;
        normal = normalize(T * tsNormal.x + B * tsNormal.y + normal * tsNormal.z);
    }
    float3 viewDir = normalize(camera.cameraPosition - in.worldPosition);
    float NdotV = max(dot(normal, viewDir), 0.0);

    float3 reflectDir = reflect(-viewDir, normal);
    float3 f0 = mix(float3(0.04), albedo, mtl);

    float3 envSpecular;
    float3 envReflection;
    if (probeParams.probeCount > 0) {
        envSpecular = sampleReflectionProbes(in.worldPosition, reflectDir,
                                              rough, normal, NdotV, f0,
                                              probes, probeParams,
                                              cubemapArray, brdfLUT, envSampler,
                                              lightData);
        envReflection = cubemapArray.sample(envSampler, reflectDir, 0, level(0.0)).rgb;
    } else {
        envReflection = sampleEnvironment(reflectDir, lightData);
        float3 fresnel = fresnelSchlickVec(NdotV, f0);
        float mipBlur = rough * rough;
        float3 envBlurred = mix(envReflection, sampleEnvironment(normal, lightData), mipBlur);
        envSpecular = fresnel * envBlurred;
    }

    float3 fresnel = fresnelSchlickVec(NdotV, f0);
    float3 directLight = evaluateLighting(in.worldPosition, normal, viewDir,
                                           albedo, mtl, rough, fresnel, lightData,
                                           shadowMap, shadowSampler,
                                           shadowTexelSize);

    float3 ambientDiffuse = albedo * sampleEnvironment(normal, lightData) * lightData.ambientMultiplier * (1.0 - mtl) * ao;
    float3 color = emit + directLight + ambientDiffuse + envSpecular;

    float alpha = material.opacity;
    if (alpha < 1.0) {
        float fresnelTerm = fresnelSchlick(NdotV, 0.04);
        alpha = mix(alpha, 1.0, fresnelTerm);
        color += envReflection * fresnelTerm * 0.8;
    }

    color *= lightData.exposure;
    float3 viewN = normalize((camera.view * float4(normal, 0.0)).xyz);
    GBufferOut gout;
    gout.color = float4(color, alpha);
    gout.viewNormal = float4(viewN * 0.5 + 0.5, 1.0);
    return gout;
}

// --- Screen-Space Reflections (SSR) ---

// Reconstruct view-space position from NDC depth and screen UV
float3 ssrViewPos(float depth, float2 uv, float4x4 invProjection) {
    float4 clip = float4(uv.x * 2.0 - 1.0, -(uv.y * 2.0 - 1.0), depth, 1.0);
    float4 vp = invProjection * clip;
    return vp.xyz / vp.w;
}

kernel void ssrRayMarch(
    texture2d<float, access::read> sceneColor [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> ssrResult [[texture(2)]],
    texture2d<float, access::read> normalTex [[texture(3)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant SSRParams& params [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    // SSR runs at half resolution; depth/scene/normals are full resolution
    uint2 outSize = uint2(ssrResult.get_width(), ssrResult.get_height());
    if (gid.x >= outSize.x || gid.y >= outSize.y) return;

    float2 uv = (float2(gid) + 0.5) / float2(outSize);
    float2 fullSize = camera.screenSize;
    uint2 fullMax = uint2(fullSize) - 1;

    // Read depth at full resolution
    uint2 depthCoord = min(uint2(uv * fullSize), fullMax);
    float depth = depthTex.read(depthCoord).x;

    if (depth >= 0.999) {
        ssrResult.write(float4(0.0), gid);
        return;
    }

    // Reconstruct view-space position from depth, read normal from G-buffer
    float3 viewPos = ssrViewPos(depth, uv, camera.invProjection);
    float3 viewNormal = normalTex.read(depthCoord).xyz * 2.0 - 1.0;
    viewNormal = normalize(viewNormal);

    float3 viewDir = normalize(viewPos);
    float3 reflectDir = reflect(viewDir, viewNormal);

    // Project start and a point along the ray into screen space
    float4 startProj = camera.projection * float4(viewPos, 1.0);
    float3 startNDC = startProj.xyz / startProj.w;

    float maxRayDist = params.maxRayDist;
    float3 rayEnd = viewPos + reflectDir * maxRayDist;
    float4 endProj = camera.projection * float4(rayEnd, 1.0);
    if (endProj.w <= 0.0) {
        // Ray goes behind camera — clip to near plane
        float t = (-viewPos.z - 0.1) / reflectDir.z;
        if (t <= 0.0) { ssrResult.write(float4(0.0), gid); return; }
        rayEnd = viewPos + reflectDir * t * 0.99;
        endProj = camera.projection * float4(rayEnd, 1.0);
        maxRayDist = t * 0.99;
    }
    float3 endNDC = endProj.xyz / endProj.w;

    // Convert NDC to UV
    float2 startScreenUV = float2(startNDC.x * 0.5 + 0.5, -startNDC.y * 0.5 + 0.5);
    float2 endScreenUV   = float2(endNDC.x * 0.5 + 0.5,   -endNDC.y * 0.5 + 0.5);

    float2 deltaUV = endScreenUV - startScreenUV;
    float2 deltaPixels = deltaUV * fullSize;
    float pixelDist = max(abs(deltaPixels.x), abs(deltaPixels.y));
    if (pixelDist < 1.0) { ssrResult.write(float4(0.0), gid); return; }

    const int MAX_STEPS = 48;
    const int BINARY_STEPS = 6;
    float pixelStride = params.stride;
    int stepCount = min(MAX_STEPS, int(pixelDist / pixelStride));
    if (stepCount < 1) stepCount = 1;

    float2 stepUV = deltaUV / float(stepCount);

    // Perspective-correct interpolation: NDC z
    float startInvW = 1.0 / startProj.w;
    float endInvW   = 1.0 / endProj.w;
    float startZoW  = startNDC.z * startInvW;
    float endZoW    = endNDC.z * endInvW;

    // Precompute near/far for NDC→linear conversion
    float near = camera.nearPlane;
    float far  = camera.farPlane;

    // Per-pixel jitter
    float hash = fract(sin(dot(float2(gid), float2(127.1, 311.7))) * 43758.5453);

    bool hit = false;
    float2 hitUV = float2(0.0);
    float hitDist = 0.0;

    for (int i = 0; i < stepCount; i++) {
        float t = (float(i) + 1.0 + hash) / float(stepCount);
        float2 sampleUV = startScreenUV + deltaUV * t;
        if (any(sampleUV < float2(0.0)) || any(sampleUV > float2(1.0))) break;

        uint2 sampleCoord = min(uint2(sampleUV * fullSize), fullMax);
        float sceneDepth = depthTex.read(sampleCoord).x;
        if (sceneDepth >= 0.999) continue;

        // Perspective-correct ray depth in NDC
        float invW = mix(startInvW, endInvW, t);
        float rayDepth = mix(startZoW, endZoW, t) / invW;

        float depthDiff = rayDepth - sceneDepth;

        // Convert ray's linear depth for distance-aware thickness
        // linearZ = near * far / (far - ndcZ * (far - near))
        float rayLinZ = near * far / (far - rayDepth * (far - near));
        float thicknessWorld = mix(params.thickness, params.thicknessFar, saturate(rayLinZ / 30.0));
        // Convert world thickness to NDC at this depth:
        // dNDC/dZ ≈ near * far / (linearZ^2 * (far - near) / (far))
        //         = near * far^2 / (linearZ^2 * (far - near))  ... but simpler:
        float offsetZ = rayLinZ + thicknessWorld;
        float ndcAtOffset = far * (offsetZ - near) / (offsetZ * (far - near));
        float thicknessNDC = ndcAtOffset - rayDepth;

        if (depthDiff > 0.0 && depthDiff < thicknessNDC) {
            // Binary search refinement — pure NDC comparison, no matrix math
            float tLo = (float(i) + hash) / float(stepCount);
            float tHi = t;
            float2 bestUV = sampleUV;
            for (int b = 0; b < BINARY_STEPS; b++) {
                float tMid = (tLo + tHi) * 0.5;
                float2 midUV = startScreenUV + deltaUV * tMid;
                midUV = clamp(midUV, float2(0.0), float2(1.0));
                uint2 midCoord = min(uint2(midUV * fullSize), fullMax);

                float midInvW = mix(startInvW, endInvW, tMid);
                float midRayZ = mix(startZoW, endZoW, tMid) / midInvW;

                if (midRayZ > depthTex.read(midCoord).x) {
                    tHi = tMid; bestUV = midUV;
                } else {
                    tLo = tMid;
                }
            }
            hit = true;
            hitUV = bestUV;
            float finalT = (tLo + tHi) * 0.5;
            float finalInvW = mix(startInvW, endInvW, finalT);
            float finalRayZ = mix(startZoW, endZoW, finalT) / finalInvW;
            float finalLinZ = near * far / (far - finalRayZ * (far - near));
            float startLinZ = near * far / (far - startNDC.z * (far - near));
            hitDist = finalLinZ - startLinZ;
            break;
        }
    }

    if (!hit) {
        ssrResult.write(float4(0.0), gid);
        return;
    }

    uint2 hitCoord = min(uint2(hitUV * fullSize), fullMax);
    float3 hitColor = sceneColor.read(hitCoord).rgb;

    float2 edgeFade = smoothstep(float2(0.0), float2(0.05), hitUV) *
                       smoothstep(float2(0.0), float2(0.05), 1.0 - hitUV);
    float confidence = edgeFade.x * edgeFade.y;
    confidence *= 1.0 - saturate(hitDist / maxRayDist);

    float NdotV = saturate(dot(viewNormal, -viewDir));
    float fresnel = 0.04 + 0.96 * pow(1.0 - NdotV, 5.0);
    confidence *= fresnel;

    ssrResult.write(float4(hitColor, confidence), gid);
}

// Bilateral blur for SSR (horizontal) — half-res input, full-res depth
kernel void ssrBlurH(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = input.get_width();
    uint h = input.get_height();
    if (gid.x >= w || gid.y >= h) return;

    // Map half-res gid to full-res depth coordinate
    uint2 fullSize = uint2(depthTex.get_width(), depthTex.get_height());
    uint2 depthCenter = min(uint2(float2(gid) / float2(w, h) * float2(fullSize)), fullSize - 1);
    float centerDepth = depthTex.read(depthCenter).x;
    float4 centerColor = input.read(gid);

    if (centerColor.a < 0.001) {
        output.write(float4(0.0), gid);
        return;
    }

    float4 total = centerColor;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(clamp(int(gid.x) + i, 0, int(w) - 1), gid.y);
        float4 s = input.read(coord);

        uint2 depthCoord = min(uint2(float2(coord) / float2(w, h) * float2(fullSize)), fullSize - 1);
        float sd = depthTex.read(depthCoord).x;

        float spatial = exp(-float(i * i) / 10.0);
        float depthW = exp(-abs(sd - centerDepth) / 0.003);
        float wt = spatial * depthW;

        total += s * wt;
        weightSum += wt;
    }

    output.write(total / weightSum, gid);
}

// Bilateral blur for SSR (vertical) — half-res input, full-res depth
kernel void ssrBlurV(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = input.get_width();
    uint h = input.get_height();
    if (gid.x >= w || gid.y >= h) return;

    uint2 fullSize = uint2(depthTex.get_width(), depthTex.get_height());
    uint2 depthCenter = min(uint2(float2(gid) / float2(w, h) * float2(fullSize)), fullSize - 1);
    float centerDepth = depthTex.read(depthCenter).x;
    float4 centerColor = input.read(gid);

    if (centerColor.a < 0.001) {
        output.write(float4(0.0), gid);
        return;
    }

    float4 total = centerColor;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(gid.x, clamp(int(gid.y) + i, 0, int(h) - 1));
        float4 s = input.read(coord);

        uint2 depthCoord = min(uint2(float2(coord) / float2(w, h) * float2(fullSize)), fullSize - 1);
        float sd = depthTex.read(depthCoord).x;

        float spatial = exp(-float(i * i) / 10.0);
        float depthW = exp(-abs(sd - centerDepth) / 0.003);
        float wt = spatial * depthW;

        total += s * wt;
        weightSum += wt;
    }

    output.write(total / weightSum, gid);
}

// --- Screen-Space Ambient Occlusion (GTAO) ---
// Full-resolution compute with fewer samples per pixel.

kernel void gtaoCompute(
    texture2d<float, access::read> depthTex [[texture(0)]],
    texture2d<float, access::write> aoResult [[texture(1)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant SSAOParams& aoParams [[buffer(1)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint2 texSize = uint2(aoResult.get_width(), aoResult.get_height());
    if (gid.x >= texSize.x || gid.y >= texSize.y) return;

    float depth = depthTex.read(gid).x;

    // Sky pixels — no occlusion
    if (depth >= 0.999) {
        aoResult.write(float4(1.0), gid);
        return;
    }

    float2 uv = (float2(gid) + 0.5) / float2(texSize);

    // Reconstruct view-space position
    float3 viewPos = ssrViewPos(depth, uv, camera.invProjection);

    // Reconstruct normal from depth neighbors (central differences, pick best pair)
    uint2 maxCoord = texSize - 1;
    float depthL = depthTex.read(uint2(max(int(gid.x) - 1, 0), gid.y)).x;
    float depthR = depthTex.read(uint2(min(gid.x + 1, maxCoord.x), gid.y)).x;
    float depthU = depthTex.read(uint2(gid.x, max(int(gid.y) - 1, 0))).x;
    float depthD = depthTex.read(uint2(gid.x, min(gid.y + 1, maxCoord.y))).x;

    float2 texel = 1.0 / float2(texSize);
    float3 viewPosL = ssrViewPos(depthL, uv - float2(texel.x, 0), camera.invProjection);
    float3 viewPosR = ssrViewPos(depthR, uv + float2(texel.x, 0), camera.invProjection);
    float3 viewPosU = ssrViewPos(depthU, uv - float2(0, texel.y), camera.invProjection);
    float3 viewPosD = ssrViewPos(depthD, uv + float2(0, texel.y), camera.invProjection);

    float3 ddx = (abs(depthR - depth) < abs(depthL - depth))
                 ? (viewPosR - viewPos) : (viewPos - viewPosL);
    float3 ddy = (abs(depthD - depth) < abs(depthU - depth))
                 ? (viewPosD - viewPos) : (viewPos - viewPosU);
    float3 viewNormal = normalize(cross(ddy, ddx));
    if (viewNormal.z < 0.0) viewNormal = -viewNormal;

    const int NUM_DIRECTIONS = aoParams.directions;
    const int NUM_STEPS = aoParams.steps;
    float radius = aoParams.radius;
    float intensity = aoParams.intensity;
    float bias = aoParams.bias;

    // Per-pixel rotation: interleaved gradient noise (Jimenez, SIGGRAPH 2014)
    // Produces well-distributed values in [0, 2π] with no visible tiling
    float rotAngle = fract(52.9829189 * fract(0.06711056 * float(gid.x) + 0.00583715 * float(gid.y))) * 2.0 * M_PI_F;

    // Screen-space pixel radius (adapts to depth)
    float projScale = camera.projection[1][1] * float(texSize.y) * 0.5;
    float screenRadius = clamp(radius * projScale / (-viewPos.z), 3.0, 64.0);

    float occlusion = 0.0;

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        float angle = rotAngle + float(d) * (2.0 * M_PI_F / float(NUM_DIRECTIONS));
        float2 dir = float2(cos(angle), sin(angle));

        float horizonCos = bias;

        for (int s = 1; s <= NUM_STEPS; s++) {
            float t = float(s) / float(NUM_STEPS);
            float2 sampleOffset = dir * t * screenRadius / float2(texSize);
            float2 sampleUV = uv + sampleOffset;

            if (any(sampleUV < float2(0.0)) || any(sampleUV > float2(1.0))) continue;

            uint2 sampleCoord = min(uint2(sampleUV * float2(texSize)), maxCoord);
            float sampleDepth = depthTex.read(sampleCoord).x;
            if (sampleDepth >= 0.999) continue;

            float3 samplePos = ssrViewPos(sampleDepth, sampleUV, camera.invProjection);
            float3 horizonVec = samplePos - viewPos;
            float horizonDist = length(horizonVec);
            if (horizonDist < 0.01) continue;

            float h = dot(normalize(horizonVec), viewNormal);
            // Smooth falloff: gentle fade from 80% of radius to the edge
            float distRatio = horizonDist / radius;
            float distFade = 1.0 - smoothstep(0.3, 1.0, distRatio);
            h *= distFade;

            horizonCos = max(horizonCos, h);
        }

        occlusion += horizonCos;
    }

    occlusion /= float(NUM_DIRECTIONS);
    float ao = 1.0 - saturate(occlusion * intensity);

    aoResult.write(float4(ao, ao, ao, 1.0), gid);
}

// AO bilateral blur (horizontal) — same resolution, depth-aware
kernel void aoBlurH(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = output.get_width();
    uint h = output.get_height();
    if (gid.x >= w || gid.y >= h) return;

    float centerAO = input.read(gid).r;
    float centerDepth = depthTex.read(gid).x;

    float total = centerAO;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(clamp(int(gid.x) + i, 0, int(w) - 1), gid.y);
        float sampleAO = input.read(coord).r;
        float sampleDepth = depthTex.read(coord).x;

        float spatial = exp(-float(i * i) / 8.0);
        float depthDiff = abs(sampleDepth - centerDepth);
        float depthW = exp(-depthDiff * depthDiff * 100000.0);
        float wt = spatial * depthW;

        total += sampleAO * wt;
        weightSum += wt;
    }

    output.write(float4(total / weightSum, 0, 0, 1), gid);
}

// AO bilateral blur (vertical) — same resolution, depth-aware
kernel void aoBlurV(
    texture2d<float, access::read> input [[texture(0)]],
    texture2d<float, access::read> depthTex [[texture(1)]],
    texture2d<float, access::write> output [[texture(2)]],
    uint2 gid [[thread_position_in_grid]]
) {
    uint w = output.get_width();
    uint h = output.get_height();
    if (gid.x >= w || gid.y >= h) return;

    float centerAO = input.read(gid).r;
    float centerDepth = depthTex.read(gid).x;

    float total = centerAO;
    float weightSum = 1.0;
    const int RADIUS = 4;

    for (int i = -RADIUS; i <= RADIUS; i++) {
        if (i == 0) continue;
        uint2 coord = uint2(gid.x, clamp(int(gid.y) + i, 0, int(h) - 1));
        float sampleAO = input.read(coord).r;
        float sampleDepth = depthTex.read(coord).x;

        float spatial = exp(-float(i * i) / 8.0);
        float depthDiff = abs(sampleDepth - centerDepth);
        float depthW = exp(-depthDiff * depthDiff * 100000.0);
        float wt = spatial * depthW;

        total += sampleAO * wt;
        weightSum += wt;
    }

    output.write(float4(total / weightSum, 0, 0, 1), gid);
}

// --- Bloom: threshold extract + progressive downsample/upsample ---

struct BloomParams {
    float threshold;
    float knee;
    float intensity;
    int   srcWidth;
    int   srcHeight;
    float _pad[3];
};

// Extract bright pixels and downsample to first mip (half-res).
// Uses a 13-tap filter (Jimenez 2014 / Call of Duty) for stable downsampling.
kernel void bloomDownsample(
    texture2d<float, access::read>  src  [[texture(0)]],
    texture2d<float, access::write> dst  [[texture(1)]],
    constant BloomParams& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    if (gid.x >= dst.get_width() || gid.y >= dst.get_height()) return;

    // Map output texel to source center (each output pixel = 2x2 source block)
    float2 srcCoord = float2(gid) * 2.0 + 0.5;
    float2 texelSize = 1.0 / float2(params.srcWidth, params.srcHeight);

    // 13-tap downsample: 4 corner samples + 4 edge samples + 1 center + 4 diagonal
    // Weights: center cross = 0.5, corners = 0.125 each (energy preserving)
    float3 a = src.read(uint2(clamp(srcCoord + float2(-1, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 b = src.read(uint2(clamp(srcCoord + float2( 0, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 c = src.read(uint2(clamp(srcCoord + float2( 1, -1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 d = src.read(uint2(clamp(srcCoord + float2(-1,  0), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 e = src.read(uint2(clamp(srcCoord,                  float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 f = src.read(uint2(clamp(srcCoord + float2( 1,  0), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 g = src.read(uint2(clamp(srcCoord + float2(-1,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 h = src.read(uint2(clamp(srcCoord + float2( 0,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;
    float3 i = src.read(uint2(clamp(srcCoord + float2( 1,  1), float2(0), float2(params.srcWidth-1, params.srcHeight-1)))).rgb;

    // Weighted average (box filter with center emphasis)
    float3 color = e * 0.25
                 + (b + d + f + h) * 0.125
                 + (a + c + g + i) * 0.0625;

    // Soft threshold on first pass (srcWidth == full resolution means this is pass 0)
    if (params.srcWidth > int(dst.get_width()) * 3) {
        float brightness = max(color.r, max(color.g, color.b));
        float soft = brightness - params.threshold + params.knee;
        soft = clamp(soft, 0.0, 2.0 * params.knee);
        soft = soft * soft / (4.0 * params.knee + 1e-5);
        float contribution = max(soft, brightness - params.threshold) / max(brightness, 1e-5);
        color *= max(contribution, 0.0);
    }

    dst.write(float4(color, 1.0), gid);
}

// Upsample and additively blend with the next-higher mip.
// Uses a 3x3 tent filter for smooth upsampling.
kernel void bloomUpsample(
    texture2d<float, access::read>  src     [[texture(0)]],  // smaller mip (being upsampled)
    texture2d<float, access::read>  higher  [[texture(1)]],  // larger mip to blend into
    texture2d<float, access::write> dst     [[texture(2)]],  // output (same size as higher)
    constant BloomParams& params [[buffer(0)]],
    uint2 gid [[thread_position_in_grid]]
) {
    int dstW = int(dst.get_width());
    int dstH = int(dst.get_height());
    if (int(gid.x) >= dstW || int(gid.y) >= dstH) return;

    // Bilinear sample from smaller mip using 3x3 tent filter
    float2 srcCoord = (float2(gid) + 0.5) * float2(src.get_width(), src.get_height())
                    / float2(dstW, dstH) - 0.5;

    int srcW = int(src.get_width());
    int srcH = int(src.get_height());

    float3 sum = float3(0);
    // 3x3 tent: weights 1,2,1 / 2,4,2 / 1,2,1 (sum=16)
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            int2 coord = int2(srcCoord) + int2(dx, dy);
            coord = clamp(coord, int2(0), int2(srcW - 1, srcH - 1));
            float w = float((2 - abs(dx)) * (2 - abs(dy)));
            sum += src.read(uint2(coord)).rgb * w;
        }
    }
    sum /= 16.0;

    // Add to higher-resolution mip
    float3 higherColor = higher.read(gid).rgb;
    dst.write(float4(higherColor + sum, 1.0), gid);
}

// --- Composite pass: tone map + gamma from HDR scene texture to LDR drawable ---

struct CompositeParams {
    int ssaoEnabled;
    int ssrEnabled;
    int debugView;      // 0=normal, 1=AO only, 2=SSR only, 3=depth, 4=normals
    float ssrBlendStrength;
    int bloomEnabled;
    float bloomIntensity;
    float _pad[2];
};

struct CompositeOut {
    float4 position [[position]];
    float2 uv;
};

vertex CompositeOut vertexComposite(uint vid [[vertex_id]]) {
    float2 uv = float2((vid << 1) & 2, vid & 2);
    CompositeOut out;
    out.position = float4(uv * 2.0 - 1.0, 0.0, 1.0);
    out.uv = float2(uv.x, 1.0 - uv.y);  // flip Y for Metal texture origin
    return out;
}

fragment float4 fragmentComposite(
    CompositeOut in [[stage_in]],
    texture2d<float> sceneColor [[texture(0)]],
    texture2d<float> ssrTexture [[texture(1)]],
    texture2d<float> aoTexture [[texture(2)]],
    depth2d<float> depthTex [[texture(3)]],
    texture2d<float> normalTexture [[texture(4)]],
    texture2d<float> bloomTexture [[texture(5)]],
    constant CameraUniforms& camera [[buffer(0)]],
    constant CompositeParams& params [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    sampler smp [[sampler(0)]]
) {
    // Read depth at this fragment's pixel position (no sampler needed)
    float depth = depthTex.read(uint2(in.position.xy));

    // --- Debug visualization modes ---
    if (params.debugView == 1) {
        // AO only — white = no occlusion, black = full occlusion
        float ao = aoTexture.sample(smp, in.uv).r;
        if (depth >= 0.999) ao = 1.0;
        return float4(ao, ao, ao, 1.0);
    }
    if (params.debugView == 2) {
        // SSR only — show reflected color where SSR hit, black elsewhere
        float4 ssr = ssrTexture.sample(smp, in.uv);
        return float4(ssr.rgb * ssr.a, 1.0);
    }
    if (params.debugView == 3) {
        // Depth — linearized, white=near black=far
        float lin = camera.nearPlane / (camera.farPlane - depth * (camera.farPlane - camera.nearPlane));
        lin = saturate(lin * camera.farPlane * 0.1);
        return float4(lin, lin, lin, 1.0);
    }
    if (params.debugView == 4) {
        // View-space normals visualization
        float3 n = normalTexture.read(uint2(in.position.xy)).xyz;
        if (depth >= 0.999) return float4(0.5, 0.5, 1.0, 1.0);
        return float4(n, 1.0);
    }

    // --- Normal rendering ---
    float3 hdrColor;
    if (depth >= 0.999) {
        // Sky pixel — render procedural sky directly in composite.
        float2 ndc = float2(in.uv.x * 2.0 - 1.0, -(in.uv.y * 2.0 - 1.0));
        float4 nearWorld = camera.invViewProjection * float4(ndc, 0.0, 1.0);
        float4 farWorld  = camera.invViewProjection * float4(ndc, 1.0, 1.0);
        float3 rayDir = normalize(farWorld.xyz / farWorld.w - nearWorld.xyz / nearWorld.w);
        hdrColor = sampleEnvironment(rayDir, lightData);
        hdrColor = applyClouds(hdrColor, rayDir, lightData);
        hdrColor *= 0.5;  // exposure
    } else {
        float4 hdr = sceneColor.sample(smp, in.uv);

        // Apply ambient occlusion if enabled
        if (params.ssaoEnabled != 0) {
            float ao = aoTexture.sample(smp, in.uv).r;
            hdr.rgb *= max(ao, 0.15);
        }

        // Blend SSR reflections if enabled
        if (params.ssrEnabled != 0) {
            float4 ssr = ssrTexture.sample(smp, in.uv);
            hdr.rgb = mix(hdr.rgb, ssr.rgb, ssr.a * params.ssrBlendStrength);
        }

        hdrColor = hdr.rgb;
    }

    // Add bloom
    if (params.bloomEnabled != 0) {
        float3 bloom = bloomTexture.sample(smp, in.uv).rgb;
        hdrColor += bloom * params.bloomIntensity;
    }

    // ACES filmic tone mapping
    float3 x = hdrColor;
    float3 color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    color = saturate(color);

    // Gamma correction
    color = pow(color, float3(1.0 / 2.2));

    return float4(color, 1.0);
}
