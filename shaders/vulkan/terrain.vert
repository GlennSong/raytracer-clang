#version 450
// CDLOD terrain vertex stage (ADR-0036; ports lighting.metal terrainVertexMain).
// Terrain nodes are world-space meshes with an identity model. The tangent slot
// carries each vertex's morph target — the position it collapses to on the next-
// coarser grid — and we blend toward it by camera distance over [morphStart,
// morphEnd], so a node matches the coarser silhouette before its parent takes
// over (no pop, no crack). Feeds the shared lit fragment (mesh.frag).

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inTangent;   // morph-target position for terrain
layout(location = 3) in vec2 inTexcoord;
layout(location = 4) in vec3 inColor;

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
    ivec4 counts;
    vec4  shadowParams;
    vec4  skySunDir;
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
    vec4  fog;
    vec4  shadowTint;
    vec4  wind1;
    vec4  wind2;
} g;

layout(push_constant) uniform Push {
    mat4  model;              // identity for terrain (unused here)
    vec4  albedoMetallic;
    vec4  emissionRough;
    uvec4 surfaceFlags;
    float morphStart;
    float morphEnd;
} pc;

layout(location = 0) out vec3 outWorldPos;
layout(location = 1) out vec3 outWorldNormal;
layout(location = 2) out vec2 outTexcoord;
layout(location = 3) out vec3 outColor;
layout(location = 4) out vec3 outWorldTangent;

void main() {
    float dist = distance(g.cameraPosition.xyz, inPosition);
    float k = clamp((dist - pc.morphStart) / max(pc.morphEnd - pc.morphStart, 1e-3),
                    0.0, 1.0);
    vec3 worldPos = mix(inPosition, inTangent, k);

    outWorldPos = worldPos;
    outWorldNormal = normalize(inNormal);
    outWorldTangent = vec3(1.0, 0.0, 0.0);   // terrain has no normal map
    outTexcoord = inTexcoord;
    outColor = inColor;
    gl_Position = g.viewProjection * vec4(worldPos, 1.0);
}
