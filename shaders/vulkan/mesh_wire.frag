#version 450
// Wireframe debug view (ADR-0057; Renderer::wireframe). Used by the LINE-mode
// wire pipeline, which shares the mesh pipeline layout and mesh.vert. Outputs a
// flat line color (carried in the push's albedoMetallic.rgb so the backend can
// set Renderer::wireframeColor without touching the globals UBO). Must still
// write the second MRT attachment (the normal G-buffer) since the scene pass has
// two color attachments — a neutral value is fine; SSAO/SSR are bypassed in the
// debug/wire views anyway.

layout(push_constant) uniform Push {
    mat4  model;
    vec4  albedoMetallic;     // rgb = wireframe color
    vec4  emissionRough;
    uvec4 surfaceFlags;
} pc;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormal;

void main() {
    outColor  = vec4(pc.albedoMetallic.rgb, 1.0);
    outNormal = vec4(0.5, 0.5, 1.0, 1.0);   // neutral (+Z) world normal, max roughness
}
