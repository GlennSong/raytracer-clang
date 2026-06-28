#version 450
// Depth-only shadow caster (ADR-0057, Phase 3). Renders geometry from a
// cascade's light view-projection into one slice of the shadow-map array. No
// fragment stage — fixed-function writes depth. The light VP is built with no
// clip-space Y-flip, and the lit pass samples with the matching unflipped
// NDC->uv mapping, so write and read are self-consistent.

layout(location = 0) in vec3 inPosition;

layout(push_constant) uniform Push {
    mat4 lightViewProj;
    mat4 model;
} pc;

void main() {
    gl_Position = pc.lightViewProj * pc.model * vec4(inPosition, 1.0);
}
