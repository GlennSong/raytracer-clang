#version 450
// Bloom (ADR-0057, Phase 5b): one shader for both the bright-pass (soft-knee
// threshold) and a separable 9-tap Gaussian blur, selected by push.brightPass.
// Reuses composite.vert (fullscreen triangle, passthrough UV).

layout(set = 0, binding = 0) uniform sampler2D src;

layout(push_constant) uniform Push {
    float texelX, texelY;   // 1/size of the bloom target
    float dirX, dirY;       // blur direction in texels (0 for bright pass)
    float threshold;
    float knee;
    float brightPass;       // 1.0 = bright-pass, 0.0 = blur
    float _pad;
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

void main() {
    if (pc.brightPass > 0.5) {
        // Soft-knee bright pass (Call of Duty style).
        vec3 c = texture(src, inUV).rgb;
        float br = max(c.r, max(c.g, c.b));
        float soft = clamp(br - pc.threshold + pc.knee, 0.0, 2.0 * pc.knee);
        soft = soft * soft / (4.0 * pc.knee + 1e-5);
        float contrib = max(soft, br - pc.threshold) / max(br, 1e-5);
        outColor = vec4(c * contrib, 1.0);
    } else {
        vec2 stp = vec2(pc.dirX, pc.dirY) * vec2(pc.texelX, pc.texelY);
        float w[5] = float[](0.227027, 0.1945946, 0.1216216, 0.054054, 0.016216);
        vec3 sum = texture(src, inUV).rgb * w[0];
        for (int i = 1; i < 5; ++i) {
            sum += texture(src, inUV + stp * float(i)).rgb * w[i];
            sum += texture(src, inUV - stp * float(i)).rgb * w[i];
        }
        outColor = vec4(sum, 1.0);
    }
}
