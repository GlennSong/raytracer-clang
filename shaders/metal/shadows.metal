// Shadow mapping: depth-only vertex stages for the shadow pass, and the PCF
// lookup used by the lit pass. The rasterization depth bias is applied on the
// shadow encoder (setDepthBias, driven by ShadowConfig::bias); the lookup-side
// controls (normalBias, pcfRadius) arrive via ShadowUniforms.

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
    const device GPUInstanceData* instances [[buffer(2)]],
    uint vid [[vertex_id]],
    uint iid [[instance_id]]
) {
    float4 worldPos = instances[iid].model * float4(vertices[vid].position, 1.0);
    return lightCamera.viewProjection * worldPos;
}

// 3x3 PCF against one slice of the cascade array; pcfRadius spreads the taps.
float sampleShadowPCF(depth2d_array<float> shadowMap, sampler smp, uint slice,
                       float2 uv, float compareDepth, float texelSize,
                       float pcfRadius) {
    float shadow = 0.0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            float2 offset = float2(x, y) * texelSize * pcfRadius;
            shadow += shadowMap.sample_compare(smp, uv + offset, slice, compareDepth);
        }
    }
    return shadow / 9.0;
}

// Visibility from one cascade: 1 = lit, 0 = shadowed (1 if outside the cascade
// bounds). The lookup point is pushed along the normal (normalBias) so the
// compare samples just off the surface, reducing acne on slopes.
float sampleCascade(depth2d_array<float> shadowMap, sampler smp, uint slice,
                    float4x4 lightVP, float3 worldPos, float3 normal,
                    constant ShadowUniforms& shadowData) {
    float3 biasedPos = worldPos + normal * shadowData.normalBias;
    float4 lightClip = lightVP * float4(biasedPos, 1.0);
    float3 ndc = lightClip.xyz / lightClip.w;
    float2 uv = ndc.xy * 0.5 + 0.5;
    uv.y = 1.0 - uv.y;  // Metal texture origin is top-left
    if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) return 1.0;
    float texelSize = 1.0 / float(shadowData.shadowMapSize);
    return sampleShadowPCF(shadowMap, smp, slice, uv, ndc.z, texelSize,
                           shadowData.pcfRadius);
}

// Cascaded shadow visibility. Selects the cascade by the fragment's view-space
// depth, then cross-fades into the next cascade over a small band before the
// split so the resolution change doesn't pop. 1 = lit, 0 = fully shadowed.
float computeShadow(depth2d_array<float> shadowMap, sampler smp,
                     float3 worldPos, float3 normal, float viewDepth,
                     constant ShadowUniforms& shadowData) {
    int count = shadowData.cascadeCount;
    if (count <= 0) return 1.0;

    int c = count - 1;
    for (int i = 0; i < count; i++) {
        if (viewDepth < shadowData.cascadeSplit[i]) { c = i; break; }
    }

    float v = sampleCascade(shadowMap, smp, uint(c),
                            shadowData.cascadeViewProjection[c],
                            worldPos, normal, shadowData);

    // Blend into the next cascade over the last 15% of this cascade's span.
    if (c + 1 < count) {
        float splitFar  = shadowData.cascadeSplit[c];
        float splitNear = (c == 0) ? 0.0 : shadowData.cascadeSplit[c - 1];
        float bandStart = mix(splitFar, splitNear, 0.15);
        if (viewDepth > bandStart) {
            float t = saturate((viewDepth - bandStart) / (splitFar - bandStart));
            float vNext = sampleCascade(shadowMap, smp, uint(c + 1),
                                        shadowData.cascadeViewProjection[c + 1],
                                        worldPos, normal, shadowData);
            v = mix(v, vNext, t);
        }
    }
    return v;
}
