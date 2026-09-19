#version 450
// The texel times the quad's tint, straight alpha. The swapchain is UNORM and
// the composite already wrote display-encoded colour, so UI texels (uploaded
// UNORM) pass through as authored.
layout(set = 0, binding = 0) uniform sampler2D uiTexture;

layout(push_constant) uniform UiPush {
    vec2 pos[4];
    vec2 uv[4];
    vec4 color;
} pc;

layout(location = 0) in vec2 inUv;
layout(location = 0) out vec4 outColor;

void main() {
    outColor = texture(uiTexture, inUv) * pc.color;
}
