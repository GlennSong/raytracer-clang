#version 450
// Phase 1 forward mesh vertex stage (ADR-0057). Ported from the Metal
// vertexMain (shaders/metal/lighting.metal) + the Vertex layout in common.metal.
// The viewProjection is uploaded already carrying the Vulkan clip-space Y-flip
// (the C++ side negates clip row 1), so this stage needs no convention fix-ups.

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inTangent;
layout(location = 3) in vec2 inTexcoord;
layout(location = 4) in vec3 inColor;

// Must match the block layout in mesh.frag and GlobalsUBO in vulkan_renderer.cpp.
struct Light {
    vec4 positionIntensity;
    vec4 directionInner;
    vec4 colorOuter;
    vec4 typeRange;
};
layout(set = 0, binding = 0) uniform Globals {
    mat4  viewProjection;
    mat4  view;
    mat4  invViewProjection;
    mat4  cascadeVP[4];
    vec4  cameraPosition;
    vec4  ambient;
    vec4  cascadeSplit;
    ivec4 counts;          // x lightCount, y cascadeCount, z envMode
    vec4  shadowParams;    // x normalBias, y pcfRadius, z mapSize, w strength
    vec4  skySunDir;
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
} g;

layout(push_constant) uniform Push {
    mat4  model;
    vec4  albedoMetallic;
    vec4  emissionRough;
    uvec4 surfaceFlags;
} pc;

layout(location = 0) out vec3 outWorldPos;
layout(location = 1) out vec3 outWorldNormal;
layout(location = 2) out vec2 outTexcoord;
layout(location = 3) out vec3 outColor;

void main() {
    vec4 world = pc.model * vec4(inPosition, 1.0);
    outWorldPos = world.xyz;
    // Inverse-transpose so non-uniform scale keeps normals perpendicular.
    mat3 normalMatrix = mat3(transpose(inverse(pc.model)));
    outWorldNormal = normalize(normalMatrix * inNormal);
    outTexcoord = inTexcoord;
    outColor = inColor;
    gl_Position = g.viewProjection * world;
}
