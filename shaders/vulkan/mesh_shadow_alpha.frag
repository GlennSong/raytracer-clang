#version 450
// The alpha-cut shadow caster's cut (ADR-0129): the material's albedo alpha, the same 0.5 test
// mesh.frag applies to FLAG_ALPHA_TEST surfaces. Depth is fixed-function.
layout(set = 0, binding = 0) uniform sampler2D albedoMap;   // the material set's first slot
layout(location = 0) in vec2 inTexcoord;

void main() {
    if (texture(albedoMap, inTexcoord).a < 0.5) discard;
}
