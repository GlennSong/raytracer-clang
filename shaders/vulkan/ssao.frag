#version 450
// World-space SSAO (ADR-0057, Phase 5b). Reconstructs world position from the
// depth buffer via invViewProjection, reads the world normal G-buffer, samples a
// cosine-weighted hemisphere kernel, and compares each sample's camera distance
// against the stored geometry to estimate occlusion. Half-res; composite
// multiplies it in. Approximation of Metal's GTAO (no temporal/blur yet).

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

layout(push_constant) uniform Push {
    float radius;
    float bias;
    float intensity;
    float _pad;
} pc;

layout(location = 0) in vec2 inUV;
layout(location = 0) out float outAO;

const int SAMPLES = 16;

float hash(vec2 p) { return fract(sin(dot(p, vec2(12.9898, 78.233))) * 43758.5453); }

vec3 reconstructWorld(vec2 uv, float depth) {
    vec4 c = g.invViewProjection * vec4(uv * 2.0 - 1.0, depth, 1.0);
    return c.xyz / c.w;
}

// Cosine-weighted hemisphere sample in tangent space (z up), scaled to cluster
// nearer the origin so occlusion favours close geometry.
vec3 kernelSample(int i) {
    float u1 = hash(vec2(float(i) * 0.137, 11.7));
    float u2 = hash(vec2(float(i) * 0.731, 27.3));
    float r = sqrt(u1);
    float theta = 6.2831853 * u2;
    float scale = mix(0.1, 1.0, float(i) / float(SAMPLES));
    scale *= scale;
    return vec3(r * cos(theta), r * sin(theta), sqrt(max(1.0 - u1, 0.0))) * scale;
}

void main() {
    float depth = texture(depthTex, inUV).r;
    if (depth >= 1.0) { outAO = 1.0; return; }   // background

    vec3 P = reconstructWorld(inUV, depth);
    vec3 N = normalize(texture(normalTex, inUV).rgb * 2.0 - 1.0);

    // Random per-pixel rotation for the kernel basis.
    vec3 rnd = normalize(vec3(hash(inUV) * 2.0 - 1.0, hash(inUV + 0.137) * 2.0 - 1.0, 0.0));
    vec3 T = normalize(rnd - N * dot(rnd, N));
    vec3 B = cross(N, T);
    mat3 TBN = mat3(T, B, N);

    vec3 camPos = g.cameraPosition.xyz;
    float occlusion = 0.0;
    for (int i = 0; i < SAMPLES; ++i) {
        vec3 sp = P + TBN * kernelSample(i) * pc.radius;
        vec4 clip = g.viewProjection * vec4(sp, 1.0);
        if (clip.w <= 0.0) continue;
        vec2 suv = (clip.xy / clip.w) * 0.5 + 0.5;
        if (suv.x < 0.0 || suv.x > 1.0 || suv.y < 0.0 || suv.y > 1.0) continue;
        float sd = texture(depthTex, suv).r;
        vec3 storedP = reconstructWorld(suv, sd);
        float sampleDist = distance(camPos, sp);
        float storedDist = distance(camPos, storedP);
        // Occluded when the stored geometry is closer than the sample point.
        if (storedDist < sampleDist - pc.bias) {
            float rangeCheck = smoothstep(0.0, 1.0,
                pc.radius / max(abs(sampleDist - storedDist), 1e-4));
            occlusion += rangeCheck;
        }
    }
    occlusion = 1.0 - (occlusion / float(SAMPLES)) * pc.intensity;
    outAO = clamp(occlusion, 0.0, 1.0);
}
