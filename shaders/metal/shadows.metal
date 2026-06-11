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

// 3x3 PCF; pcfRadius spreads the taps (1.0 = adjacent texels).
float sampleShadowPCF(depth2d<float> shadowMap, sampler smp,
                       float2 uv, float compareDepth, float texelSize,
                       float pcfRadius) {
    float shadow = 0.0;
    for (int y = -1; y <= 1; y++) {
        for (int x = -1; x <= 1; x++) {
            float2 offset = float2(x, y) * texelSize * pcfRadius;
            shadow += shadowMap.sample_compare(smp, uv + offset, compareDepth);
        }
    }
    return shadow / 9.0;
}

// Shadow visibility for a surface point: 1 = lit, 0 = fully shadowed.
// The lookup point is pushed along the surface normal (normalBias) so the
// compare samples just off the surface, reducing acne on slopes.
float computeShadow(depth2d<float> shadowMap, sampler smp,
                     float4x4 lightVP, float3 worldPos, float3 normal,
                     constant ShadowUniforms& shadowData) {
    float3 biasedPos = worldPos + normal * shadowData.normalBias;
    float4 lightClip = lightVP * float4(biasedPos, 1.0);
    float3 ndc = lightClip.xyz / lightClip.w;
    float2 uv = ndc.xy * 0.5 + 0.5;
    uv.y = 1.0 - uv.y;  // Metal texture origin is top-left
    if (uv.x < 0 || uv.x > 1 || uv.y < 0 || uv.y > 1) return 1.0;
    float texelSize = 1.0 / float(shadowData.shadowMapSize);
    return sampleShadowPCF(shadowMap, smp, uv, ndc.z, texelSize,
                           shadowData.pcfRadius);
}
