// Water surface (procedural-planet-plan P2) — Metal port of water.vert + water.frag.
// Animated ripple normals, a GGX sun glint, Schlick-Fresnel sky reflection, and
// depth-tinted transmission (uv.x = depth, uv.y = shore distance, the engine's
// Surface::Water convention) — so it serves a planet ocean AND city/coast water.
// Concatenation-ready (relies on the prepended metal_stdlib / shader_types.h).
// Mirrors the SPIR-V-verified GLSL; MSL is compile-unverified in CI. Wire on device:
// add to the concat list, match the engine's Vertex layout + Globals/Draw uniforms
// (WaterUniforms below is a self-contained stand-in), build a pipeline from
// vertexWater / fragmentWater, and route Surface::Water draws to it.

struct WaterVertexIn {
    float3 position [[attribute(0)]];
    float3 normal   [[attribute(1)]];
    float3 tangent  [[attribute(2)]];
    float2 uv       [[attribute(3)]];
    float3 color    [[attribute(4)]];
};

struct WaterGlobals {
    float4x4 viewProjection;
    float4x4 invViewProjection;
    float4   cameraPosition;
    float4   sunDir;        // xyz toward sun, w disc intensity
    float4   sunColor;
    float4   skyZenith;
    float4   skyHorizon;
    float4   windTime;      // x time
};
struct WaterDraw {
    float4x4 model;
    float4   deepColor;
    float4   roughness;     // x
};

struct WaterOut {
    float4 position [[position]];
    float3 worldPos;
    float3 normal;
    float2 uv;
    float3 tangent;
};

struct WaterFragOut {
    float4 color  [[color(0)]];
    float4 normal [[color(1)]];
};

static float w_hash21(float2 p) {
    p = fract(p * float2(0.1031, 0.1030));
    p += dot(p, p.yx + 33.33);
    return fract((p.x + p.y) * p.x);
}
static float w_valueNoise2(float2 p) {
    float2 i = floor(p), f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float a = w_hash21(i), b = w_hash21(i + float2(1, 0));
    float c = w_hash21(i + float2(0, 1)), d = w_hash21(i + float2(1, 1));
    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}
static float w_ripple(float2 p, float t) {
    float2 w = float2(t * 0.6, t * 0.35);
    return w_valueNoise2(p * 1.7 + w) + 0.5 * w_valueNoise2(p * 3.9 - w * 1.3);
}
static float w_ggx(float ndh, float rough) {
    float a = rough * rough, a2 = a * a;
    float den = (ndh * ndh) * (a2 - 1.0) + 1.0;
    return a2 / max(1e-5, M_PI_F * den * den);
}

vertex WaterOut vertexWater(WaterVertexIn v [[stage_in]],
                            constant WaterGlobals& g [[buffer(1)]],
                            constant WaterDraw& d [[buffer(2)]]) {
    WaterOut out;
    float4 world = d.model * float4(v.position, 1.0);
    float3x3 nm = float3x3(d.model[0].xyz, d.model[1].xyz, d.model[2].xyz);
    out.worldPos = world.xyz;
    out.normal = normalize(nm * v.normal);
    out.tangent = normalize(nm * v.tangent);
    out.uv = v.uv;
    out.position = g.viewProjection * world;
    return out;
}

fragment WaterFragOut fragmentWater(WaterOut in [[stage_in]],
                                    constant WaterGlobals& g [[buffer(1)]],
                                    constant WaterDraw& d [[buffer(2)]]) {
    float3 N0 = normalize(in.normal);
    float3 V = normalize(g.cameraPosition.xyz - in.worldPos);
    float3 L = normalize(g.sunDir.xyz);

    float3 T = normalize(in.tangent - N0 * dot(in.tangent, N0));
    float3 B = cross(N0, T);
    float t = g.windTime.x;
    float2 uvp = in.worldPos.xz * 0.15;
    float e = 0.35;
    float hL = w_ripple(uvp - float2(e, 0), t);
    float hR = w_ripple(uvp + float2(e, 0), t);
    float hD = w_ripple(uvp - float2(0, e), t);
    float hU = w_ripple(uvp + float2(0, e), t);
    float strength = 0.6;
    float3 N = normalize(N0 - T * (hR - hL) * strength - B * (hU - hD) * strength);

    float ndv = max(dot(N, V), 0.0);
    float fres = 0.02 + 0.98 * pow(1.0 - ndv, 5.0);

    float depth = clamp(in.uv.x, 0.0, 1.0);
    float3 deep = d.deepColor.rgb;
    float3 shallow = mix(float3(0.10, 0.45, 0.50), deep, 0.35);
    float3 body = mix(shallow, deep, depth);

    float3 R = reflect(-V, N);
    float upBlend = clamp(R.y * 0.5 + 0.5, 0.0, 1.0);
    float3 sky = mix(g.skyHorizon.rgb, g.skyZenith.rgb, pow(upBlend, 0.5));

    float3 H = normalize(L + V);
    float ndh = max(dot(N, H), 0.0);
    float ndl = max(dot(N, L), 0.0);
    float rough = max(0.02, d.roughness.x * 0.4);
    float spec = w_ggx(ndh, rough) * fres * ndl;
    float3 glint = g.sunColor.rgb * spec * g.sunDir.w;

    float shore = in.uv.y;
    float foamMask = smoothstep(0.12, 0.0, shore) *
                     step(0.5, w_valueNoise2(in.worldPos.xz * 0.6 + t * 0.5));
    float3 foam = float3(0.9) * foamMask;

    float3 color = mix(body, sky, fres) + glint + foam;
    color *= (0.25 + 0.9 * ndl);

    WaterFragOut out;
    out.color = float4(color, 1.0);
    out.normal = float4(N * 0.5 + 0.5, rough);
    return out;
}
