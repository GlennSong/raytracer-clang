#version 450
// Procedural-sky skybox (ADR-0057, Phase 4). Fullscreen triangle, no vertex
// buffer; reconstructs the world-space view ray via the inverse of the flipped
// clip transform (so it matches the geometry pass exactly). Drawn first with
// depth test/write off, so it fills the background. Do NOT normalize the ray
// here — far-plane offsets are affine in NDC; the fragment normalizes per pixel.

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
} g;

layout(location = 0) out vec3 outViewDir;

void main() {
    vec2 uv = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    vec2 clip = uv * 2.0 - 1.0;
    gl_Position = vec4(clip, 1.0, 1.0);
    vec4 world = g.invViewProjection * vec4(clip, 1.0, 1.0);
    outViewDir = world.xyz / world.w - g.cameraPosition.xyz;
}
