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
    // A 4x4 tent over the half-res texels (bilinear weights times a 1-2-2-1
    // outer taper): every full-res pixel sees all four phases of the march's
    // 2x2 dither at least twice, so the stipple the 2x2 blend left behind
    // averages out, and cloud edges soften by half a texel — which they
    // should, at half res. The depth term still stops the deck bleeding
    // across building silhouettes.
    vec4 acc = vec4(0.0);
    float wSum = 0.0;
    for (int j = -1; j <= 2; j++) {
        for (int i = -1; i <= 2; i++) {
            ivec2 c = clamp(base + ivec2(i, j), ivec2(0), ivec2(cSize) - 1);
            vec2 cuv = (vec2(c) + 0.5) / cSize;
            float dd = texelFetch(depthTex, min(ivec2(cuv * fullSize), fullMax), 0).r;
            float dx = abs(float(i) - f.x), dy = abs(float(j) - f.y);   // texel distance
            float tent = max(0.0, 2.0 - dx) * max(0.0, 2.0 - dy);
            float w = tent * bilateralDepthWeight(d0, dd, 0.10) + 1e-5;
            acc += texelFetch(cloudTex, c, 0) * w;
            wSum += w;
        }
    }
    outColor = acc / wSum;
}
