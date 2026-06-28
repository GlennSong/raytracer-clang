#version 450
// Depth of field (ADR-0057; ports post.metal dofGather). Single-pass
// scatter-as-gather on the HDR scene: a per-pixel thin-lens circle of confusion
// drives a golden-angle spiral gather, each tap weighted by whether its own CoC
// reaches this pixel so sharp in-focus geometry doesn't smear onto blurred
// neighbors. Composite reads this in place of the HDR when DOF is enabled.
// Full-res. View depth is reconstructed from the depth buffer (forward-Z) via
// invViewProjection + view, matching the SSAO/SSR basis.

struct Light { vec4 a; vec4 b; vec4 c; vec4 d; };
layout(set = 0, binding = 0) uniform Globals {
    mat4  viewProjection;
    mat4  view;
    mat4  invViewProjection;
    mat4  cascadeVP[4];
    vec4  cameraPosition;
    vec4  ambient;
    vec4  cascadeSplit;
    ivec4 counts;
    vec4  shadowParams;
    vec4  skySunDir;
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
} g;

layout(set = 1, binding = 0) uniform sampler2D hdrTex;
layout(set = 1, binding = 1) uniform sampler2D depthTex;

layout(push_constant) uniform Push {
    float focusDistance;   // world units
    float focalLength;     // meters
    float aperture;        // diameter, meters
    float cocScale;        // sensor meters → pixels
    float maxCocPixels;
    float texelX;
    float texelY;
    float _pad;
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

// Positive view-space depth (meters) reconstructed from the depth buffer.
float viewDepth(vec2 uv) {
    float d = texture(depthTex, uv).r;
    vec4 w = g.invViewProjection * vec4(uv * 2.0 - 1.0, d, 1.0);
    vec3 world = w.xyz / w.w;
    return -(g.view * vec4(world, 1.0)).z;
}

// Thin-lens CoC radius in pixels (sensor-plane diameter × cocScale).
float cocRadius(float linZ) {
    float denom = max(linZ * (pc.focusDistance - pc.focalLength), 1e-4);
    float cocMeters = pc.aperture * pc.focalLength * abs(linZ - pc.focusDistance) / denom;
    return min(cocMeters * pc.cocScale * 0.5, pc.maxCocPixels);
}

void main() {
    vec3 center = texture(hdrTex, inUV).rgb;
    float centerCoc = cocRadius(viewDepth(inUV));
    if (centerCoc < 0.5) { outColor = vec4(center, 1.0); return; }   // in focus

    const int TAPS = 24;
    const float GOLDEN = 2.39996323;
    vec2 texel = vec2(pc.texelX, pc.texelY);
    vec3 sum = center;
    float wsum = 1.0;
    for (int i = 0; i < TAPS; ++i) {
        float tapDist = centerCoc * sqrt((float(i) + 0.5) / float(TAPS));   // pixels
        float angle = float(i) * GOLDEN;
        vec2 off = vec2(cos(angle), sin(angle)) * tapDist;                  // pixels
        vec2 uv = clamp(inUV + off * texel, vec2(0.0), vec2(1.0));
        float sampleCoc = cocRadius(viewDepth(uv));
        // Full weight when the sample's blur circle covers this pixel, with a
        // 1px soft transition below that (scatter-as-gather).
        float w = clamp(sampleCoc - tapDist + 1.0, 0.0, 1.0);
        sum += texture(hdrTex, uv).rgb * w;
        wsum += w;
    }
    outColor = vec4(sum / wsum, 1.0);
}
