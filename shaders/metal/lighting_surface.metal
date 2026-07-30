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
                         depth2d_array<float> shadowMap,
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
        float viewDepth = -(camera.view * float4(geom.worldPosition, 1.0)).z;
        float s = 1.0;
        for (uint i = 0; i < uint(lightData.lightCount); i++) {
            if (lightData.lights[i].type == LightType_Directional &&
                lightData.lights[i].shadowMapIndex >= 0) {
                s = computeShadow(shadowMap, shadowSampler,
                                  geom.worldPosition,
                                  normalize(geom.worldNormal), viewDepth, shadowData);
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
    if (tf & 1u) {
        float4 base = albedoMap.sample(texSampler, geom.texcoord);
        // Alpha-cut foliage (FLAG_ALPHA_TEST): drop fragments under the leaf
        // silhouette so cards stay crisp in the opaque pass.
        if ((int(mat.flags) & 2) && base.a < 0.5) discard_fragment();
        albedo *= base.rgb;
    }
    if (tf & 2u) {
        float4 mr = metalRoughMap.sample(texSampler, geom.texcoord);
        rough *= mr.g;
        mtl *= mr.b;
    }
    if (tf & 8u) ao = aoMap.sample(texSampler, geom.texcoord).r;
    if (tf & 16u) emit *= emissiveMap.sample(texSampler, geom.texcoord).rgb;
    if (!(tf & 1u) && (int(mat.flags) & 1)) albedo = applyCheckerboard(albedo, geom.worldPosition);
    // Procedural surface library: a Surface id packed into flag bits 8..15
    // (brick/concrete/asphalt/...) shades the facade in world space — but only
    // when there's no baked albedo map, so a textured material (which keeps the
    // flag as provenance for save/load) lets its textures drive the look.
    uint surfId = (uint(mat.flags) >> 8) & 0xFFu;
    if (surfId != 0u && !(tf & 1u))
        albedo = applySurface(surfId, albedo, geom.worldPosition, normalize(geom.worldNormal),
                              geom.texcoord, camera.windTime);

    float3 normal = normalize(geom.worldNormal);
    if (tf & 4u) {
        float3 T = normalize(geom.worldTangent - normal * dot(normal, geom.worldTangent));
        float3 B = cross(normal, T);
        float3 tsNormal = normalMap.sample(texSampler, geom.texcoord).xyz * 2.0 - 1.0;
        normal = normalize(T * tsNormal.x + B * tsNormal.y + normal * tsNormal.z);
    }
    // Procedural relief for surfaces with no baked normal/roughness map (road,
    // water, terrain). Lives with each material in surface_*.metal.
    applySurfaceRelief(surfId, geom.worldPosition, geom.texcoord, camera.windTime,
                       camera.cameraPosition, normal, rough);
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

    // Cascade debug view: tint each fragment by which shadow cascade it lands in
    // (red/green/blue/yellow for 0..3) so the cascade fit is visible on-device.
    if (shadowData.debugShadow == 8) {
        float viewDepth = -(camera.view * float4(geom.worldPosition, 1.0)).z;
        int count = max(shadowData.cascadeCount, 1);
        int c = count - 1;
        for (int i = 0; i < count; i++) {
            if (viewDepth < shadowData.cascadeSplit[i]) { c = i; break; }
        }
        float3 tint = (c == 0) ? float3(1.0, 0.4, 0.4)
                    : (c == 1) ? float3(0.4, 1.0, 0.4)
                    : (c == 2) ? float3(0.4, 0.4, 1.0)
                               : float3(1.0, 1.0, 0.4);
        GBufferOut dbg;
        dbg.color = float4(tint, 1.0);
        dbg.viewNormal = float4(0.5, 0.5, 1.0, 1.0);
        return dbg;
    }

    // Sun shadow visibility, evaluated once. Artistic response (ADR-0017
    // Phase 2): occlusion lerps toward the shadow tint; `strength` scales the
    // direct-light occlusion, `ambientStrength` scales how much the same
    // shadow darkens the environment terms — without that, an HDR sky fills
    // shadows right back in through the (otherwise unshadowed) ambient path.
    float sunV = 1.0;
    float viewDepth = -(camera.view * float4(geom.worldPosition, 1.0)).z;
    for (int i = 0; i < lightData.lightCount && i < RT_MAX_LIGHTS; i++) {
        if (lightData.lights[i].shadowMapIndex >= 0) {
            sunV = computeShadow(shadowMap, shadowSampler,
                                 geom.worldPosition, normal, viewDepth, shadowData);
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

    // Aerial-perspective fog (mirrors the offline tracer's Scene::fog): lerp the
    // lit radiance toward the fog color by 1-exp(-density*dist), so distant
    // terrain dissolves into atmosphere and the far clip / LOD seams hide. Done
    // here in scene-referred linear space, before the composite exposure/tonemap.
    // With fogHeightFalloff > 0 the density decays with altitude and the optical
    // depth uses the closed-form integral of exp(-b*y) along the ray — low-lying
    // haze a high camera looks down THROUGH, not a distance white-out.
    if (lightData.fogDensity > 0.0) {
        float dist = length(geom.worldPosition - camera.cameraPosition);
        float f;
        if (lightData.fogHeightFalloff > 0.0) {
            float b = lightData.fogHeightFalloff;
            float3 rd = (geom.worldPosition - camera.cameraPosition) / max(dist, 1e-4);
            float od = lightData.fogDensity * exp(-b * camera.cameraPosition.y) *
                       (fabs(rd.y) > 1e-4
                            ? (1.0 - exp(-b * rd.y * dist)) / (b * rd.y)
                            : dist);
            f = 1.0 - exp(-max(od, 0.0));
        } else {
            f = 1.0 - exp(-lightData.fogDensity * dist);
        }
        color = mix(color, lightData.fogColor, saturate(f));
    }

    // Exposure is applied once, uniformly, in the composite pass — sceneColor holds
    // linear scene-referred radiance, so do NOT pre-expose here.
    float3 viewN = normalize((camera.view * float4(normal, 0.0)).xyz);
    GBufferOut out;
    out.color = float4(color, alpha);
    // Pack perceptual roughness into .w so screen-space passes (SSR) can gate by
    // it — rough surfaces (ground, foliage) must not reflect like mirrors.
    out.viewNormal = float4(viewN * 0.5 + 0.5, rough);
    return out;
}
