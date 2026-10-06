#version 450
// The alpha-cut shadow caster (ADR-0129): mesh_shadow.vert plus the texcoord, for geometry whose
// albedo alpha cuts it out -- leaf and needle cards -- so a crown casts dappled shade, not the
// solid rectangles of its cards. Paired with mesh_shadow_alpha.frag.
layout(location = 0) in vec3 inPosition;
layout(location = 1) in mat4 inModel;   // per instance (ADR-0097)
layout(location = 5) in vec2 inTexcoord;

layout(push_constant) uniform Push {
    mat4 lightViewProj;
    mat4 model;
} pc;

layout(location = 0) out vec2 outTexcoord;
layout(location = 1) out vec3 outWorldPos;

void main() {
    outTexcoord = inTexcoord;
    mat4 M = inModel;
    M[0][3] = 0.0;   // (per-instance data rides the bottom row, never w)
    const vec4 world = M * vec4(inPosition, 1.0);
    outWorldPos = world.xyz;
    gl_Position = pc.lightViewProj * world;
}
