#version 450
// Water surface shading (procedural-planet-plan P2): animated ripple normals, a GGX
// sun glint (the bright specular smear toward the sun), Schlick Fresnel sky
// reflection, and depth-tinted transmission — turquoise shallows to dark deeps —
// plus a shoreline foam band. Reads the engine's Surface::Water UV convention
// (uv.x = water depth, uv.y = distance to shore), so it drops in for a PLANET ocean
// AND city/coast water bodies alike. Writes the same MRT as mesh.frag (color +
// packed normal/roughness) so SSR/SSAO stay consistent. Output is scene-linear HDR.

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
    vec4  skySunDir;      // xyz toward the sun, w disc intensity
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
    vec4  fog;
    vec4  shadowTint;
    vec4  wind1;          // xyz wind dir, w wind time
    vec4  wind2;
} g;

layout(push_constant) uniform Push {
    mat4  model;
    vec4  albedoMetallic;   // rgb deep-water color
    vec4  emissionRough;    // w roughness
    uvec4 surfaceFlags;
} pc;

layout(location = 0) in vec3 inWorldPos;
layout(location = 1) in vec3 inWorldNormal;
layout(location = 2) in vec2 inTexcoord;    // x = depth, y = shore distance
layout(location = 3) in vec3 inColor;
layout(location = 4) in vec3 inWorldTangent;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec4 outNormal;

const float PI = 3.14159265359;

float hash21(vec2 p) {
    p = fract(p * vec2(0.1031, 0.1030));
    p += dot(p, p.yx + 33.33);
    return fract((p.x + p.y) * p.x);
}
float valueNoise2(vec2 p) {
    vec2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = hash21(i), b = hash21(i + vec2(1, 0));
    float c = hash21(i + vec2(0, 1)), d = hash21(i + vec2(1, 1));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}
// Height of the ripple field at p (two scrolling octaves), used for finite-diff normals.
float rippleHeight(vec2 p, float t) {
    vec2 w = vec2(t * 0.6, t * 0.35);
    float h = valueNoise2(p * 1.7 + w);
    h += 0.5 * valueNoise2(p * 3.9 - w * 1.3);
    return h;
}

float ggx(float ndh, float rough) {
    float a = rough * rough;
    float a2 = a * a;
    float d = (ndh * ndh) * (a2 - 1.0) + 1.0;
    return a2 / max(1e-5, PI * d * d);
}

void main() {
    vec3 N0 = normalize(inWorldNormal);
    vec3 V = normalize(g.cameraPosition.xyz - inWorldPos);
    vec3 L = normalize(g.skySunDir.xyz);

    // Ripple normal: finite-difference the ripple height in the surface tangent plane.
    vec3 T = normalize(inWorldTangent - N0 * dot(inWorldTangent, N0));
    vec3 B = cross(N0, T);
    float t = g.wind1.w;
    vec2 uvp = inWorldPos.xz * 0.15;
    float e = 0.35;
    float hL = rippleHeight(uvp - vec2(e, 0), t);
    float hR = rippleHeight(uvp + vec2(e, 0), t);
    float hD = rippleHeight(uvp - vec2(0, e), t);
    float hU = rippleHeight(uvp + vec2(0, e), t);
    float strength = 0.6;
    vec3 N = normalize(N0 - T * (hR - hL) * strength - B * (hU - hD) * strength);

    // Fresnel (Schlick), water F0 ~= 0.02.
    float ndv = max(dot(N, V), 0.0);
    float fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);

    // Depth-tinted body colour: shallow -> deep by the baked depth (uv.x).
    float depth = clamp(inTexcoord.x, 0.0, 1.0);
    vec3 deep = pc.albedoMetallic.rgb;
    vec3 shallow = mix(vec3(0.10, 0.45, 0.50), deep, 0.35);   // turquoise shallows
    vec3 body = mix(shallow, deep, depth);

    // Sky reflection approximation from the reflected ray's height.
    vec3 R = reflect(-V, N);
    float upBlend = clamp(R.y * 0.5 + 0.5, 0.0, 1.0);
    vec3 sky = mix(g.skyHorizon.rgb, g.skyZenith.rgb, pow(upBlend, 0.5));

    // GGX sun glint.
    vec3 H = normalize(L + V);
    float ndh = max(dot(N, H), 0.0);
    float ndl = max(dot(N, L), 0.0);
    float rough = max(0.02, pc.emissionRough.w * 0.4);
    float spec = ggx(ndh, rough) * fres * ndl;
    vec3 glint = g.skySunColor.rgb * spec * g.skySunDir.w;

    // Shoreline foam: a noisy band where shore distance (uv.y) is small.
    float shore = inTexcoord.y;
    float foamMask = smoothstep(0.12, 0.0, shore) *
                     step(0.5, valueNoise2(inWorldPos.xz * 0.6 + t * 0.5));
    vec3 foam = vec3(0.9) * foamMask;

    vec3 color = mix(body, sky, fres) + glint + foam;
    // Sun-lit brightening of the body so the day side reads.
    color *= (0.25 + 0.9 * ndl);

    outColor = vec4(color, 1.0);
    outNormal = vec4(N * 0.5 + 0.5, rough);
}
