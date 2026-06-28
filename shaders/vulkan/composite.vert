#version 450
// Composite fullscreen triangle (ADR-0057, Phase 5). Outputs a [0,1] UV used to
// sample the HDR scene target. No flip: the HDR target and the swapchain share
// the same Vulkan framebuffer orientation, so a passthrough UV is correct.

layout(location = 0) out vec2 outUV;

void main() {
    vec2 uv = vec2(float((gl_VertexIndex << 1) & 2), float(gl_VertexIndex & 2));
    outUV = uv;
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
}
