#version 450
// Depth-guided bilateral upsample of the half-res cloud overlay onto the
// full-res HDR scene (port of fragmentCloudComposite / cloudOverlaySample in
// clouds.metal). The pipeline blends RGB = src * ONE + dst * SRC_ALPHA
// (dest * transmittance + scattered), alpha write-masked off.

layout(set = 0, binding = 0) uniform sampler2D cloudTex;   // half-res overlay
layout(set = 0, binding = 1) uniform sampler2D depthTex;   // full-res reverse-Z

layout(push_constant) uniform Push {
    float nearPlane;
    float farPlane;
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Ports post_common.metal: linear eye-space compare, relative difference, so
// one sigma behaves the same near and far under reverse-Z.
float linearizeReverseZ(float z, float near, float far) {
    return near * far / (near + z * (far - near));
}
float bilateralDepthWeight(float zCenter, float zSample, float sigmaRel) {
    float c = linearizeReverseZ(zCenter, pc.nearPlane, pc.farPlane);
    float s = linearizeReverseZ(zSample, pc.nearPlane, pc.farPlane);
    float rel = (s - c) / max(c, pc.nearPlane);
    return exp(-rel * rel / (2.0 * sigmaRel * sigmaRel));
}

void main() {
    vec2 cSize = vec2(textureSize(cloudTex, 0));
    ivec2 fullMax = textureSize(depthTex, 0) - 1;
    vec2 fullSize = vec2(textureSize(depthTex, 0));

    float d0 = texelFetch(depthTex, min(ivec2(inUV * fullSize), fullMax), 0).r;

    vec2 pos = inUV * cSize - 0.5;
    vec2 f = fract(pos);
    ivec2 base = ivec2(floor(pos));
    vec4 acc = vec4(0.0);
    float wSum = 0.0;
    for (int j = 0; j < 2; j++) {
        for (int i = 0; i < 2; i++) {
            ivec2 c = clamp(base + ivec2(i, j), ivec2(0), ivec2(cSize) - 1);
            vec2 cuv = (vec2(c) + 0.5) / cSize;
            float dd = texelFetch(depthTex, min(ivec2(cuv * fullSize), fullMax), 0).r;
            // Blend of the bilinear weight and flat 0.25: pure bilinear leaves
            // a weighted remainder of the march's 2x2 dither (stipple); pure
            // flat resolves every full-res pixel in a half-res window to the
            // IDENTICAL value and paints a hard screen-locked 2x2 grid. The
            // depth term stops the deck bleeding across silhouettes.
            float bw = (i != 0 ? f.x : 1.0 - f.x) * (j != 0 ? f.y : 1.0 - f.y);
            float w = mix(0.25, bw, 0.5) * bilateralDepthWeight(d0, dd, 0.10) + 1e-5;
            acc += texelFetch(cloudTex, c, 0) * w;
            wSum += w;
        }
    }
    outColor = acc / wSum;
}
