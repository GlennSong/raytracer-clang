#include <metal_stdlib>
using namespace metal;

struct Vertex {
    packed_float3 position;
    packed_float3 normal;
    float2 texcoord;
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float2 texcoord;
};

struct CameraUniforms {
    float4x4 viewProjection;
    float4x4 view;
    float3 cameraPosition;
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
    float3 emission;
};

struct Light {
    float3 position;
    float3 color;
    float intensity;
};

struct LightUniforms {
    Light lights[8];
    int lightCount;
    float exposure;
};

// Procedural environment — gives metals something to reflect
float3 sampleEnvironment(float3 dir) {
    float3 sky = mix(float3(0.15, 0.15, 0.2), float3(0.4, 0.5, 0.7), saturate(dir.y * 0.5 + 0.5));
    float3 ground = float3(0.1, 0.08, 0.06);
    float blend = smoothstep(-0.1, 0.1, dir.y);
    float3 env = mix(ground, sky, blend);

    // Fake sun highlight
    float3 sunDir = normalize(float3(0.5, 0.7, -0.3));
    float sunDot = max(dot(dir, sunDir), 0.0);
    env += float3(1.0, 0.9, 0.7) * pow(sunDot, 64.0) * 2.0;
    env += float3(1.0, 0.9, 0.8) * pow(sunDot, 8.0) * 0.3;

    return env;
}

float fresnelSchlick(float cosTheta, float f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

float3 fresnelSchlickVec(float cosTheta, float3 f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
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
    out.texcoord = vertices[vid].texcoord;
    return out;
}

fragment float4 fragmentMain(
    VertexOut in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    constant MaterialUniforms& material [[buffer(3)]],
    constant LightUniforms& lightData [[buffer(4)]]
) {
    float3 normal = normalize(in.worldNormal);
    float3 viewDir = normalize(camera.cameraPosition - in.worldPosition);
    float NdotV = max(dot(normal, viewDir), 0.0);

    // Reflection and refraction vectors
    float3 reflectDir = reflect(-viewDir, normal);
    float3 envReflection = sampleEnvironment(reflectDir);

    // Fresnel: F0 for dielectrics is ~0.04, metals use albedo
    float3 f0 = mix(float3(0.04), material.albedo, material.metallic);
    float3 fresnel = fresnelSchlickVec(NdotV, f0);

    // Environment reflection — blur for rough surfaces
    float mipBlur = material.roughness * material.roughness;
    float3 envBlurred = mix(envReflection, sampleEnvironment(normal), mipBlur);

    // Specular environment contribution (metals and glossy dielectrics)
    float3 envSpecular = fresnel * envBlurred;

    // Direct lighting
    float3 directLight = float3(0.0);
    for (int i = 0; i < lightData.lightCount && i < 8; i++) {
        float3 lightDir = lightData.lights[i].position - in.worldPosition;
        float dist = length(lightDir);
        lightDir = normalize(lightDir);

        float attenuation = lightData.lights[i].intensity / (1.0 + 0.09 * dist + 0.032 * dist * dist);

        // Diffuse (non-metals only)
        float diff = max(dot(normal, lightDir), 0.0);
        float3 diffuse = material.albedo * diff * (1.0 - material.metallic);

        // Specular (Blinn-Phong)
        float3 halfVec = normalize(lightDir + viewDir);
        float NdotH = max(dot(normal, halfVec), 0.0);
        float shininess = mix(16.0, 512.0, 1.0 - material.roughness);
        float spec = pow(NdotH, shininess);
        // Normalize specular intensity
        float specNorm = (shininess + 8.0) / (8.0 * 3.14159);
        float3 specular = fresnel * spec * specNorm;

        directLight += (diffuse + specular) * lightData.lights[i].color * attenuation;
    }

    // Ambient diffuse from environment
    float3 ambientDiffuse = material.albedo * sampleEnvironment(normal) * 0.3 * (1.0 - material.metallic);

    // Combine
    float3 color = material.emission + directLight + ambientDiffuse + envSpecular;

    // Glass: boost fresnel reflections, tint transmission
    float alpha = material.opacity;
    if (alpha < 1.0) {
        float fresnelTerm = fresnelSchlick(NdotV, 0.04);
        alpha = mix(alpha, 1.0, fresnelTerm);
        // Add environment reflection on top for glass look
        color += envReflection * fresnelTerm * 0.8;
    }

    // Exposure
    color *= lightData.exposure;

    // ACES-ish tone mapping (better than Reinhard for preserving color)
    float3 x = color;
    color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    color = saturate(color);

    // Gamma correction
    color = pow(color, float3(1.0 / 2.2));

    return float4(color, alpha);
}
