#version 450
// Phase 1 forward mesh fragment stage (ADR-0057). A real Cook-Torrance GGX
// direct term for the sun plus a flat ambient — the core of the Metal
// fragmentMain (shaders/metal/lighting.metal), minus shadows / IBL / textures /
// the procedural-surface library, which arrive in Phases 2-4. Output is
// scene-linear; the swapchain is an sRGB format, so the GPU encodes on store.
// (Tone mapping is a Phase 5 composite step.)

layout(set = 0, binding = 0) uniform Globals {
    mat4 viewProjection;
    vec4 cameraPosition;
    vec4 sunDirection;
    vec4 sunColor;
    vec4 ambient;
} g;

layout(push_constant) uniform Push {
    mat4 model;
    vec4 albedoMetallic;
    vec4 emissionRough;
} pc;

layout(location = 0) in vec3 inWorldPos;
layout(location = 1) in vec3 inWorldNormal;
layout(location = 2) in vec2 inTexcoord;
layout(location = 3) in vec3 inColor;

layout(location = 0) out vec4 outColor;

const float PI = 3.14159265359;

float distributionGGX(vec3 N, vec3 H, float roughness) {
    float a = roughness * roughness;
    float a2 = a * a;
    float NdotH = max(dot(N, H), 0.0);
    float d = NdotH * NdotH * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

float geometrySchlickGGX(float NdotX, float roughness) {
    float r = roughness + 1.0;
    float k = (r * r) / 8.0;
    return NdotX / (NdotX * (1.0 - k) + k);
}

float geometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
    return geometrySchlickGGX(max(dot(N, V), 0.0), roughness) *
           geometrySchlickGGX(max(dot(N, L), 0.0), roughness);
}

vec3 fresnelSchlick(float cosTheta, vec3 F0) {
    return F0 + (1.0 - F0) * pow(clamp(1.0 - cosTheta, 0.0, 1.0), 5.0);
}

void main() {
    vec3 albedo = pc.albedoMetallic.rgb * inColor;
    float metallic = clamp(pc.albedoMetallic.a, 0.0, 1.0);
    float roughness = clamp(pc.emissionRough.a, 0.04, 1.0);
    vec3 emission = pc.emissionRough.rgb;

    vec3 N = normalize(inWorldNormal);
    vec3 V = normalize(g.cameraPosition.xyz - inWorldPos);
    vec3 L = normalize(g.sunDirection.xyz);
    vec3 H = normalize(V + L);

    vec3 F0 = mix(vec3(0.04), albedo, metallic);
    float NdotL = max(dot(N, L), 0.0);

    float NDF = distributionGGX(N, H, roughness);
    float G = geometrySmith(N, V, L, roughness);
    vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);

    vec3 specular = (NDF * G * F) /
                    (4.0 * max(dot(N, V), 0.0) * NdotL + 0.0001);
    vec3 kd = (vec3(1.0) - F) * (1.0 - metallic);

    vec3 direct = (kd * albedo / PI + specular) * g.sunColor.rgb * NdotL;
    vec3 ambient = g.ambient.rgb * albedo;

    outColor = vec4(direct + ambient + emission, 1.0);
}
