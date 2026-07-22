#version 450
// Water surface vertex stage (procedural-planet-plan P2). A trimmed mesh.vert: no
// wind sway; passes world position / normal / tangent / uv / color to water.frag.
// The Globals block and vertex attributes match mesh.vert so it rides the same
// geometry + descriptor layout. Ripples are done per-pixel in the fragment (normal
// perturbation), so the mesh needs no tessellation.

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inTangent;
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
    mat4  model;
    vec4  albedoMetallic;
    vec4  emissionRough;
    uvec4 surfaceFlags;
} pc;

layout(location = 0) out vec3 outWorldPos;
layout(location = 1) out vec3 outWorldNormal;
layout(location = 2) out vec2 outTexcoord;
layout(location = 3) out vec3 outColor;
layout(location = 4) out vec3 outWorldTangent;

void main() {
    vec4 world = pc.model * vec4(inPosition, 1.0);
    outWorldPos = world.xyz;
    mat3 nm = mat3(pc.model);
    outWorldNormal = normalize(nm * inNormal);
    outWorldTangent = normalize(nm * inTangent);
    outTexcoord = inTexcoord;
    outColor = inColor;
    gl_Position = g.viewProjection * world;
}
