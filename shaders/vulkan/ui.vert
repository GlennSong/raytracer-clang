#version 450
// THE GAME UI LAYER (Renderer::submitUi): one textured quad per draw, its four
// corners (NDC) and UVs in push constants -- no vertex buffer. Two triangles,
// TL-TR-BR and TL-BR-BL.
layout(push_constant) uniform UiPush {
    vec2 pos[4];
    vec2 uv[4];
    vec4 color;
} pc;

layout(location = 0) out vec2 outUv;

void main() {
    const int corner[6] = int[6](0, 1, 2, 0, 2, 3);
    int i = corner[gl_VertexIndex];
    outUv = pc.uv[i];
    gl_Position = vec4(pc.pos[i], 0.0, 1.0);
}
