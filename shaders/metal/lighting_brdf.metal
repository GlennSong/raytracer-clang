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
