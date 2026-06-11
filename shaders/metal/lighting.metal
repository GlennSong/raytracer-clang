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

// --- Direct lighting (Cook-Torrance GGX, ADR-0017 Phase 1) ---
//
// Light units, so direct and image-based lighting share one energy scale:
//  - directional: `color * intensity` is the illuminance arriving from the
//    light's direction. A white, albedo-1 surface at normal incidence under
//    intensity π reflects radiance 1.0 (the Lambert 1/π).
//  - point/spot: `color * intensity` is the illuminance at 1 m; falloff is
//    inverse-square, windowed to reach exactly zero at `range`.
// Roughness is perceptual (squared into GGX alpha), matching the split-sum
// prefilter and BRDF LUT used by the environment paths.

float distributionGGX(float NdotH, float a2) {
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / max(M_PI_F * d * d, 1e-6);
}

// Height-correlated Smith visibility — includes the 1/(4 NdotL NdotV) term.
float visibilitySmithGGX(float NdotV, float NdotL, float a2) {
    float gv = NdotL * sqrt(NdotV * NdotV * (1.0 - a2) + a2);
    float gl = NdotV * sqrt(NdotL * NdotL * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}

// Inverse-square falloff windowed to zero at `range` (UE-style smooth window),
// so lights have a finite, artist-controlled reach instead of an infinite tail.
float distanceAttenuation(float dist, float range) {
    float ratio2 = (dist * dist) / max(range * range, 1e-4);
    float window = saturate(1.0 - ratio2 * ratio2);
    return window * window / max(dist * dist, 1e-4);
}

// Shared lighting calculation for all fragment shaders. `directShadow` is the
// sun's visibility-as-color (ADR-0017 Phase 2): white where lit, lerped toward
// the artistic shadow tint where occluded — computed once in shadeSurface and
// applied to every shadow-casting light.
float3 evaluateLighting(float3 worldPos, float3 normal, float3 viewDir,
                         float3 albedo, float metallic, float roughness,
                         float3 f0, device const LightUniforms& lightData,
                         float3 directShadow) {
    float3 directLight = float3(0.0);
    // Alpha floor keeps the GGX highlight finite on perfectly smooth surfaces.
    float a = max(roughness * roughness, 0.002);
    float a2 = a * a;
    float NdotV = max(dot(normal, viewDir), 1e-4);

    for (int i = 0; i < lightData.lightCount && i < RT_MAX_LIGHTS; i++) {
        device const GPULight& light = lightData.lights[i];

        float3 lightDir;
        float attenuation;

        if (light.type == LightType_Directional) {
            lightDir = light.direction;
            attenuation = light.intensity;
        } else {
            lightDir = light.position - worldPos;
            float dist = length(lightDir);
            lightDir = normalize(lightDir);
            attenuation = light.intensity * distanceAttenuation(dist, light.range);

            if (light.type == LightType_Spot) {
                float theta = dot(-lightDir, light.direction);
                float spotFactor = smoothstep(light.outerCosAngle,
                                              light.innerCosAngle, theta);
                attenuation *= spotFactor;
            }
        }

        float NdotL = max(dot(normal, lightDir), 0.0);
        if (NdotL <= 0.0 || attenuation <= 0.0) continue;

        float3 shadow = (light.shadowMapIndex >= 0) ? directShadow : float3(1.0);

        float3 halfVec = normalize(lightDir + viewDir);
        float NdotH = max(dot(normal, halfVec), 0.0);
        float VdotH = max(dot(viewDir, halfVec), 0.0);

        float D = distributionGGX(NdotH, a2);
        float Vis = visibilitySmithGGX(NdotV, NdotL, a2);
        float3 F = fresnelSchlickVec(VdotH, f0);
        float3 specular = D * Vis * F;

        // Energy balance: light reflected specularly (F) doesn't also diffuse.
        float3 diffuse = (1.0 - F) * (1.0 - metallic) * albedo * (1.0 / M_PI_F);

        directLight += (diffuse + specular) * light.color
                       * (attenuation * NdotL) * shadow;
    }
    return directLight;
}

// --- Shared surface shading ---

// Material values after the per-draw uniforms / per-instance data are resolved;
// the only thing that differs between the two fragment entry points.
struct SurfaceMaterial {
    float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    float flags;
    float3 emission;
    uint textureFlags;
};

struct SurfaceGeometry {
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
};

GBufferOut shadeSurface(SurfaceGeometry geom, SurfaceMaterial mat,
                         constant CameraUniforms& camera,
                         device const LightUniforms& lightData,
                         constant ShadowUniforms& shadowData,
                         constant ProbeUniforms& probeParams,
                         device const GPUReflectionProbe* probes,
                         constant EnvUniforms& env,
                         depth2d<float> shadowMap,
                         texturecube_array<float> cubemapArray,
                         texture2d<float> brdfLUT,
                         texture2d<float> albedoMap,
                         texture2d<float> metalRoughMap,
                         texture2d<float> normalMap,
                         texture2d<float> aoMap,
                         texture2d<float> emissiveMap,
                         texturecube<float> prefilteredEnv,
                         texturecube<float> irradianceEnv,
                         sampler shadowSampler,
                         sampler envSampler,
                         sampler texSampler) {
    // Shadow debug view: the sun's shadow factor as grayscale (white = lit,
    // black = shadowed), to inspect the shadow path in isolation.
    if (shadowData.debugShadow == 5) {
        float s = 1.0;
        for (uint i = 0; i < uint(lightData.lightCount); i++) {
            if (lightData.lights[i].type == LightType_Directional &&
                lightData.lights[i].shadowMapIndex >= 0) {
                s = computeShadow(shadowMap, shadowSampler,
                                  lightData.lights[i].lightViewProjection,
                                  geom.worldPosition,
                                  normalize(geom.worldNormal), shadowData);
                break;
            }
        }
        GBufferOut dbg;
        dbg.color = float4(s, s, s, 1.0);
        dbg.viewNormal = float4(0.5, 0.5, 1.0, 1.0);
        return dbg;
    }

    float3 albedo = mat.albedo;
    float mtl = mat.metallic;
    float rough = mat.roughness;
    float3 emit = mat.emission;
    float ao = 1.0;
    uint tf = mat.textureFlags;
    if (tf & 1u) albedo *= albedoMap.sample(texSampler, geom.texcoord).rgb;
    if (tf & 2u) {
        float4 mr = metalRoughMap.sample(texSampler, geom.texcoord);
        rough *= mr.g;
        mtl *= mr.b;
    }
    if (tf & 8u) ao = aoMap.sample(texSampler, geom.texcoord).r;
    if (tf & 16u) emit *= emissiveMap.sample(texSampler, geom.texcoord).rgb;
    if (!(tf & 1u) && (int(mat.flags) & 1)) albedo = applyCheckerboard(albedo, geom.worldPosition);

    float3 normal = normalize(geom.worldNormal);
    if (tf & 4u) {
        float3 T = normalize(geom.worldTangent - normal * dot(normal, geom.worldTangent));
        float3 B = cross(normal, T);
        float3 tsNormal = normalMap.sample(texSampler, geom.texcoord).xyz * 2.0 - 1.0;
        normal = normalize(T * tsNormal.x + B * tsNormal.y + normal * tsNormal.z);
    }
    float3 viewDir = normalize(camera.cameraPosition - geom.worldPosition);
    float NdotV = max(dot(normal, viewDir), 0.0);

    // Albedo debug view: output the final material color (after texture + tint),
    // before any lighting, to isolate material vs lighting issues.
    if (shadowData.debugShadow == 6) {
        GBufferOut dbg;
        dbg.color = float4(albedo, 1.0);
        dbg.viewNormal = float4(0.5, 0.5, 1.0, 1.0);
        return dbg;
    }

    // Sun shadow visibility, evaluated once. Artistic response (ADR-0017
    // Phase 2): occlusion lerps toward the shadow tint; `strength` scales the
    // direct-light occlusion, `ambientStrength` scales how much the same
    // shadow darkens the environment terms — without that, an HDR sky fills
    // shadows right back in through the (otherwise unshadowed) ambient path.
    float sunV = 1.0;
    for (int i = 0; i < lightData.lightCount && i < RT_MAX_LIGHTS; i++) {
        if (lightData.lights[i].shadowMapIndex >= 0) {
            sunV = computeShadow(shadowMap, shadowSampler,
                                 lightData.lights[i].lightViewProjection,
                                 geom.worldPosition, normal, shadowData);
            break;
        }
    }
    float3 directShadow  = mix(float3(1.0), shadowData.shadowTint,
                               (1.0 - sunV) * shadowData.shadowStrength);
    float3 ambientShadow = mix(float3(1.0), shadowData.shadowTint,
                               (1.0 - sunV) * shadowData.ambientStrength);

    float3 reflectDir = reflect(-viewDir, normal);
    float3 f0 = mix(float3(0.04), albedo, mtl);

    // Unified environment path (ADR-0017 Phase 3): irradiance for ambient,
    // GGX-prefiltered radiance weighted by the split-sum BRDF LUT for
    // specular, with local reflection probes blended on top when present.
    float3 envDiffuse = envIrradiance(normal, env, irradianceEnv, envSampler,
                                      lightData);
    float3 envReflection = envPrefilteredRadiance(reflectDir, normal, 0.0, env,
                                                  prefilteredEnv, envSampler,
                                                  lightData);
    float3 envSpecular;
    if (probeParams.probeCount > 0) {
        envSpecular = sampleReflectionProbes(geom.worldPosition, reflectDir,
                                              rough, normal, NdotV, f0,
                                              probes, probeParams,
                                              cubemapArray, brdfLUT,
                                              prefilteredEnv, env, envSampler,
                                              lightData);
    } else {
        float2 brdf = brdfLUT.sample(envSampler, float2(NdotV, rough)).rg;
        float3 prefiltered = envPrefilteredRadiance(reflectDir, normal, rough,
                                                    env, prefilteredEnv,
                                                    envSampler, lightData);
        envSpecular = (f0 * brdf.x + brdf.y) * prefiltered;
    }

    float3 directLight = evaluateLighting(geom.worldPosition, normal, viewDir,
                                           albedo, mtl, rough,
                                           f0, lightData, directShadow);

    float3 ambientDiffuse = albedo * envDiffuse * lightData.ambientTint
                          * lightData.ambientMultiplier * (1.0 - mtl) * ao;
    float3 color = emit + directLight
                 + (ambientDiffuse + envSpecular) * ambientShadow;

    float alpha = mat.opacity;
    if (alpha < 1.0) {
        float fresnelTerm = fresnelSchlick(NdotV, 0.04);
        alpha = mix(alpha, 1.0, fresnelTerm);
        color += envReflection * fresnelTerm * 0.8;
    }

    // Exposure is applied once, uniformly, in the composite pass — sceneColor holds
    // linear scene-referred radiance, so do NOT pre-expose here.
    float3 viewN = normalize((camera.view * float4(normal, 0.0)).xyz);
    GBufferOut out;
    out.color = float4(color, alpha);
    out.viewNormal = float4(viewN * 0.5 + 0.5, 1.0);
    return out;
}

// --- Vertex/fragment entry points ---

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

vertex FragmentData vertexMainInstanced(
    const device Vertex* vertices [[buffer(0)]],
    constant CameraUniforms& camera [[buffer(1)]],
    const device GPUInstanceData* instances [[buffer(2)]],
    uint vid [[vertex_id]],
    uint iid [[instance_id]]
) {
    GPUInstanceData inst = instances[iid];
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

fragment GBufferOut fragmentMain(
    VertexOut in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant MaterialUniforms& material [[buffer(3)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    constant EnvUniforms& env [[buffer(8)]],
    depth2d<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    texturecube<float> prefilteredEnv [[texture(8)]],
    texturecube<float> irradianceEnv [[texture(9)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    SurfaceGeometry geom = {in.worldPosition, in.worldNormal,
                            in.worldTangent, in.texcoord};
    SurfaceMaterial mat = {material.albedo, material.metallic,
                           material.roughness, material.opacity,
                           material.flags, material.emission,
                           material.textureFlags};
    return shadeSurface(geom, mat, camera, lightData, shadowData,
                        probeParams, probes, env,
                        shadowMap, cubemapArray, brdfLUT,
                        albedoMap, metalRoughMap, normalMap, aoMap,
                        emissiveMap, prefilteredEnv, irradianceEnv,
                        shadowSampler, envSampler, texSampler);
}

fragment GBufferOut fragmentMainInstanced(
    FragmentData in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    constant ProbeUniforms& probeParams [[buffer(6)]],
    device const GPUReflectionProbe* probes [[buffer(7)]],
    constant EnvUniforms& env [[buffer(8)]],
    depth2d<float> shadowMap [[texture(0)]],
    texturecube_array<float> cubemapArray [[texture(1)]],
    texture2d<float> brdfLUT [[texture(2)]],
    texture2d<float> albedoMap [[texture(3)]],
    texture2d<float> metalRoughMap [[texture(4)]],
    texture2d<float> normalMap [[texture(5)]],
    texture2d<float> aoMap [[texture(6)]],
    texture2d<float> emissiveMap [[texture(7)]],
    texturecube<float> prefilteredEnv [[texture(8)]],
    texturecube<float> irradianceEnv [[texture(9)]],
    sampler shadowSampler [[sampler(0)]],
    sampler envSampler [[sampler(1)]],
    sampler texSampler [[sampler(2)]]
) {
    SurfaceGeometry geom = {in.worldPosition, in.worldNormal,
                            in.worldTangent, in.texcoord};
    SurfaceMaterial mat = {in.albedo, in.metallic, in.roughness, in.opacity,
                           in.flags, in.emission, in.textureFlags};
    return shadeSurface(geom, mat, camera, lightData, shadowData,
                        probeParams, probes, env,
                        shadowMap, cubemapArray, brdfLUT,
                        albedoMap, metalRoughMap, normalMap, aoMap,
                        emissiveMap, prefilteredEnv, irradianceEnv,
                        shadowSampler, envSampler, texSampler);
}
