// Lit-surface shading: reflection-probe sampling, direct lighting, and the
// shared shadeSurface() used by both the single-draw and instanced fragment
// entry points (ADR-0017 Phase 0 — previously two ~150-line duplicates).

// --- Unified environment sampling (ADR-0017 Phase 3) ---
// One provider interface for both environments: HDR mode samples the baked
// irradiance / GGX-prefiltered cubes; procedural mode evaluates the analytic
// sky (cheap, low-frequency — baking it per-frame as the day/night cycle
// animates is a deferred optimization, noted in the ADR).

// Cosine-convolved incoming light for the diffuse/ambient term (already
// divided by π — multiplies albedo directly).
float3 envIrradiance(float3 n, constant EnvUniforms& env,
                     texturecube<float> irradianceEnv, sampler s,
                     device const LightUniforms& lightData) {
    return (env.mode == 1) ? irradianceEnv.sample(s, n).rgb
                           : sampleEnvironment(n, lightData);
}

// Prefiltered specular radiance along `dir`; mips are linear in roughness.
// The procedural fallback approximates the blur by lerping toward the
// normal-direction radiance.
float3 envPrefilteredRadiance(float3 dir, float3 n, float rough,
                              constant EnvUniforms& env,
                              texturecube<float> prefilteredEnv, sampler s,
                              device const LightUniforms& lightData) {
    if (env.mode == 1)
        return prefilteredEnv.sample(s, dir, level(rough * float(env.envMaxMip))).rgb;
    float mipBlur = rough * rough;
    return mix(sampleEnvironment(dir, lightData),
               sampleEnvironment(n, lightData), mipBlur);
}

// --- Reflection probes ---

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

// Sample reflection from probes with distance-based blending; falls back to
// the active environment (HDR prefiltered cube or procedural sky).
float3 sampleReflectionProbes(float3 worldPos, float3 reflectDir, float roughness,
                               float3 normal, float NdotV, float3 f0,
                               device const GPUReflectionProbe* probes,
                               constant ProbeUniforms& probeParams,
                               texturecube_array<float> cubemapArray,
                               texture2d<float> brdfLUT,
                               texturecube<float> prefilteredEnv,
                               constant EnvUniforms& envSel,
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
        // Outside all probes — fall back to the active environment, with the
        // same split-sum weighting as everywhere else.
        float3 envColor = envPrefilteredRadiance(reflectDir, normal, roughness,
                                                 envSel, prefilteredEnv,
                                                 envSampler, env);
        return fresnel * envColor;
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

    // Blend remaining weight to the environment fallback
    float skyWeight = 1.0 - saturate(totalWeight);
    if (skyWeight > 0.001) {
        float3 skyColor = envPrefilteredRadiance(reflectDir, normal, roughness,
                                                 envSel, prefilteredEnv,
                                                 envSampler, env);
        probeColor = mix(probeColor, skyColor, skyWeight);
    }

    return fresnel * probeColor;
}
