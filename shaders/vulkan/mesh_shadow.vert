#version 450
// Depth-only shadow caster (ADR-0057, Phase 3). Renders geometry from a
// cascade's light view-projection into one slice of the shadow-map array. No
// fragment stage — fixed-function writes depth. The light VP is built with no
// clip-space Y-flip, and the lit pass samples with the matching unflipped
// NDC->uv mapping, so write and read are self-consistent.

layout(location = 0) in vec3 inPosition;
layout(location = 1) in mat4 inModel;   // per instance (ADR-0097); pc.model is no longer read

layout(push_constant) uniform Push {
    mat4 lightViewProj;
    mat4 model;
} pc;

void main() {
    // (the bottom row can carry per-instance DATA -- a grass rank, a signal's state -- never w)
    mat4 M = inModel;
    M[0][3] = 0.0;
    gl_Position = pc.lightViewProj * M * vec4(inPosition, 1.0);
    // FLAG_LOD_BAND (pc.model[2].x = 1): a hard cut at the middle of each band edge, by the INSTANCE's
    // origin (a near tree casts its shadow while it is drawn, its impostor beyond) -- pc.model[0].xyz the
    // camera, pc.model[1] the band (in0, in1, out0, out1). A collapsed instance draws nothing.
    if (pc.model[2].x > 0.5) {
        const vec4 b = pc.model[1];
        const float d = distance(inModel[3].xyz, pc.model[0].xyz);
        if ((b.y > b.x && d < 0.5 * (b.x + b.y)) || (b.w > b.z && d >= 0.5 * (b.z + b.w))) gl_Position = vec4(0.0, 0.0, -2.0, 1.0);
    }
}
