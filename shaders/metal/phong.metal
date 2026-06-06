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
    float  _pad[2];
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

struct InstanceData {
    float4x4 model;
    float4x4 normalMatrix;
    float4 albedo;      // w unused
    float metallic;
    float roughness;
    float opacity;
    float _pad0;
    float4 emission;    // w unused
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

struct FragmentData {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float2 texcoord;
    float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    float3 emission;
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
    out.texcoord = vertices[vid].texcoord;
    out.albedo = inst.albedo.xyz;
    out.metallic = inst.metallic;
    out.roughness = inst.roughness;
    out.opacity = inst.opacity;
    out.emission = inst.emission.xyz;
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

fragment float4 fragmentMainInstanced(
    FragmentData in [[stage_in]],
    constant CameraUniforms& camera [[buffer(1)]],
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    depth2d<float> shadowMap [[texture(0)]],
    sampler shadowSampler [[sampler(0)]]
) {
    float shadowTexelSize = 1.0 / float(shadowData.shadowMapSize);
    float3 normal = normalize(in.worldNormal);
    float3 viewDir = normalize(camera.cameraPosition - in.worldPosition);
    float NdotV = max(dot(normal, viewDir), 0.0);

    float3 reflectDir = reflect(-viewDir, normal);
    float3 envReflection = sampleEnvironment(reflectDir);

    float3 f0 = mix(float3(0.04), in.albedo, in.metallic);
    float3 fresnel = fresnelSchlickVec(NdotV, f0);

    float mipBlur = in.roughness * in.roughness;
    float3 envBlurred = mix(envReflection, sampleEnvironment(normal), mipBlur);
    float3 envSpecular = fresnel * envBlurred;

    float3 directLight = evaluateLighting(in.worldPosition, normal, viewDir,
                                           in.albedo, in.metallic, in.roughness,
                                           fresnel, lightData,
                                           shadowMap, shadowSampler,
                                           shadowTexelSize);

    float3 ambientDiffuse = in.albedo * sampleEnvironment(normal) * 0.3 * (1.0 - in.metallic);
    float3 color = in.emission + directLight + ambientDiffuse + envSpecular;

    float alpha = in.opacity;
    if (alpha < 1.0) {
        float fresnelTerm = fresnelSchlick(NdotV, 0.04);
        alpha = mix(alpha, 1.0, fresnelTerm);
        color += envReflection * fresnelTerm * 0.8;
    }

    color *= lightData.exposure;
    float3 x = color;
    color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    color = saturate(color);
    color = pow(color, float3(1.0 / 2.2));
    return float4(color, alpha);
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
    device const LightUniforms& lightData [[buffer(4)]],
    constant ShadowUniforms& shadowData [[buffer(5)]],
    depth2d<float> shadowMap [[texture(0)]],
    sampler shadowSampler [[sampler(0)]]
) {
    float shadowTexelSize = 1.0 / float(shadowData.shadowMapSize);
    float3 normal = normalize(in.worldNormal);
    float3 viewDir = normalize(camera.cameraPosition - in.worldPosition);
    float NdotV = max(dot(normal, viewDir), 0.0);

    float3 reflectDir = reflect(-viewDir, normal);
    float3 envReflection = sampleEnvironment(reflectDir);

    float3 f0 = mix(float3(0.04), material.albedo, material.metallic);
    float3 fresnel = fresnelSchlickVec(NdotV, f0);

    float mipBlur = material.roughness * material.roughness;
    float3 envBlurred = mix(envReflection, sampleEnvironment(normal), mipBlur);
    float3 envSpecular = fresnel * envBlurred;

    float3 directLight = evaluateLighting(in.worldPosition, normal, viewDir,
                                           material.albedo, material.metallic,
                                           material.roughness, fresnel, lightData,
                                           shadowMap, shadowSampler,
                                           shadowTexelSize);

    float3 ambientDiffuse = material.albedo * sampleEnvironment(normal) * 0.3 * (1.0 - material.metallic);
    float3 color = material.emission + directLight + ambientDiffuse + envSpecular;

    float alpha = material.opacity;
    if (alpha < 1.0) {
        float fresnelTerm = fresnelSchlick(NdotV, 0.04);
        alpha = mix(alpha, 1.0, fresnelTerm);
        color += envReflection * fresnelTerm * 0.8;
    }

    color *= lightData.exposure;
    float3 x = color;
    color = (x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14);
    color = saturate(color);
    color = pow(color, float3(1.0 / 2.2));
    return float4(color, alpha);
}
