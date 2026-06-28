#version 450
// Box blur for the half-res SSAO target (ADR-0057, Phase 5b). The raw SSAO is
// noisy because each pixel samples a randomly-rotated hemisphere kernel; a small
// box blur over the half-res AO removes that noise before the composite reads it.
// Matches the spirit of Metal's AO denoise (a separable/box smooth on the AO
// channel). Single channel R8 in/out; push constant carries the AO texel size.

layout(set = 0, binding = 0) uniform sampler2D aoTex;

layout(push_constant) uniform Push {
    vec2 texel;   // 1.0 / aoResolution
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out float outAO;

void main() {
    // 4x4 box blur (radius 1.5 in half-res space) — cheap and enough to kill the
    // hemisphere-kernel noise on a half-res target.
    float sum = 0.0;
    for (int y = -2; y <= 1; ++y) {
        for (int x = -2; x <= 1; ++x) {
            vec2 off = vec2(float(x) + 0.5, float(y) + 0.5) * pc.texel;
            sum += texture(aoTex, inUV + off).r;
        }
    }
    outAO = sum / 16.0;
}
