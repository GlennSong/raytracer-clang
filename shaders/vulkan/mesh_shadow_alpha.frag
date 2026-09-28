#version 450
// The alpha-cut shadow caster's cut (ADR-0129): the material's albedo alpha, the same 0.5 test
// mesh.frag applies to FLAG_ALPHA_TEST surfaces. Depth is fixed-function. A FLAG_LOD_BAND surface
// (pc.model[2].x = 1) casts only on its side of the middle of each band edge, by this fragment's
// distance from the camera (pc.model[0].xyz) -- a near crown and its impostor never both cast.
layout(set = 0, binding = 0) uniform sampler2D albedoMap;   // the material set's first slot
layout(push_constant) uniform Push {
    mat4 lightViewProj;
    mat4 model;
} pc;
layout(location = 0) in vec2 inTexcoord;
layout(location = 1) in vec3 inWorldPos;

void main() {
    if (pc.model[2].x > 0.5) {
        const vec4 b = pc.model[1];
        const float d = distance(inWorldPos, pc.model[0].xyz);
        if ((b.y > b.x && d < 0.5 * (b.x + b.y)) || (b.w > b.z && d >= 0.5 * (b.z + b.w))) discard;
    }
    if (texture(albedoMap, inTexcoord).a < 0.5) discard;
}
