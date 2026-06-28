#version 450
// Screen-space reflections (ADR-0057, Phase 5b). World-space ray march of the
// reflection ray against the depth buffer; on a hit, samples the HDR scene and
// returns it with a confidence (edge + Fresnel + roughness fade, scaled by
// blendStrength). Half-res; composite blends it by confidence. Reuses the world
// reconstruction basis (invViewProjection) and the roughness packed in normal.a.
// A fixed-step march (no binary refine) — an approximation of Metal's ssrRayMarch.

struct Light {
    vec4 positionIntensity;
    vec4 directionInner;
    vec4 colorOuter;
    vec4 typeRange;
};
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

layout(set = 1, binding = 0) uniform sampler2D depthTex;
layout(set = 1, binding = 1) uniform sampler2D normalTex;
layout(set = 1, binding = 2) uniform sampler2D hdrTex;

layout(push_constant) uniform Push {
    float maxRayDist;
    float thickness;
    float maxRoughness;
    float blendStrength;
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out vec4 outColor;

const int STEPS = 32;

vec3 reconstructWorld(vec2 uv, float depth) {
    vec4 c = g.invViewProjection * vec4(uv * 2.0 - 1.0, depth, 1.0);
    return c.xyz / c.w;
}

void main() {
    outColor = vec4(0.0);
    float depth = texture(depthTex, inUV).r;
    if (depth <= 0.0) return;                       // sky: no reflection (reverse-Z far = 0)

    vec4 nr = texture(normalTex, inUV);
    float roughness = nr.a;
    if (roughness > pc.maxRoughness) return;        // too rough to mirror

    vec3 P = reconstructWorld(inUV, depth);
    vec3 N = normalize(nr.rgb * 2.0 - 1.0);
    vec3 V = normalize(P - g.cameraPosition.xyz);   // camera → surface
    vec3 R = reflect(V, N);

    float stepLen = pc.maxRayDist / float(STEPS);
    vec3 rayPos = P + N * 0.02;                      // bias off the surface
    vec3 prevPos = rayPos;                           // last position in front of geometry
    for (int i = 0; i < STEPS; ++i) {
        prevPos = rayPos;
        rayPos += R * stepLen;
        vec4 clip = g.viewProjection * vec4(rayPos, 1.0);
        if (clip.w <= 0.0) return;
        vec2 suv = (clip.xy / clip.w) * 0.5 + 0.5;
        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) return;

        float sd = texture(depthTex, suv).r;
        if (sd <= 0.0) continue;                     // sky pixel, keep marching (reverse-Z)
        vec3 sceneP = reconstructWorld(suv, sd);
        float rayDist = distance(g.cameraPosition.xyz, rayPos);
        float sceneDist = distance(g.cameraPosition.xyz, sceneP);
        if (rayDist > sceneDist && (rayDist - sceneDist) < pc.thickness + stepLen) {
            // Binary-refine the crossing between the last in-front position and
            // this behind position. The coarse fixed world-step quantises the hit
            // to step boundaries, which makes a tall reflection look repeated /
            // laddered; a few bisections localise it to the true surface.
            vec3 a = prevPos, b = rayPos;
            vec2 hitUV = suv;
            for (int j = 0; j < 5; ++j) {
                vec3 mid = 0.5 * (a + b);
                vec4 mc = g.viewProjection * vec4(mid, 1.0);
                if (mc.w <= 0.0) break;
                vec2 muv = (mc.xy / mc.w) * 0.5 + 0.5;
                if (muv.x < 0.0 || muv.x > 1.0 || muv.y < 0.0 || muv.y > 1.0) break;
                float md = texture(depthTex, muv).r;
                if (md <= 0.0) { a = mid; continue; }   // sky: midpoint is in front (reverse-Z)
                vec3 mP = reconstructWorld(muv, md);
                if (distance(g.cameraPosition.xyz, mid) >
                    distance(g.cameraPosition.xyz, mP)) {
                    b = mid; hitUV = muv;               // behind geometry → pull far end in
                } else {
                    a = mid;                            // in front → pull near end in
                }
            }
            // Hit. Confidence: fade at screen edges, by Fresnel, and by roughness.
            vec2 edge = smoothstep(0.0, 0.15, hitUV) * (1.0 - smoothstep(0.85, 1.0, hitUV));
            float edgeFade = edge.x * edge.y;
            float fresnel = pow(clamp(1.0 - max(dot(N, -V), 0.0), 0.0, 1.0), 3.0);
            float roughFade = 1.0 - smoothstep(0.0, pc.maxRoughness, roughness);
            float conf = edgeFade * mix(0.2, 1.0, fresnel) * roughFade * pc.blendStrength;
            outColor = vec4(texture(hdrTex, hitUV).rgb, clamp(conf, 0.0, 1.0));
            return;
        }
    }
}
