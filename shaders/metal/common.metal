// Shared vertex layouts, varyings, and small BRDF helpers used across the
// shader modules. The loader concatenates the modules in dependency order:
// shader_types.h, common, environment, shadows, lighting, post (ADR-0017).

struct Vertex {
    packed_float3 position;
    packed_float3 normal;
    packed_float3 tangent;
    packed_float2 texcoord;
    packed_float3 color;      // per-vertex tint (matches engine Vertex::color)
};

struct VertexOut {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
    float3 vertexColor;
};

// VertexOut + per-instance material values carried through interpolation
// (constant within an instance) for the instanced path.
struct FragmentData {
    float4 position [[position]];
    float3 worldPosition;
    float3 worldNormal;
    float3 worldTangent;
    float2 texcoord;
    float3 vertexColor;
    float3 albedo;
    float metallic;
    float roughness;
    float opacity;
    float flags;
    float3 emission;
    uint textureFlags;
};

// Lit passes write HDR color + view-space normals (for SSR).
struct GBufferOut {
    float4 color [[color(0)]];
    float4 viewNormal [[color(1)]];
};

float fresnelSchlick(float cosTheta, float f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

float3 fresnelSchlickVec(float cosTheta, float3 f0) {
    return f0 + (1.0 - f0) * pow(saturate(1.0 - cosTheta), 5.0);
}

// Checkerboard pattern based on world position
float3 applyCheckerboard(float3 albedo, float3 worldPos) {
    int cx = int(floor(worldPos.x));
    int cz = int(floor(worldPos.z));
    bool dark = ((cx + cz) & 1) != 0;
    return dark ? albedo * 0.3 : albedo;
}

// --- Surface primitives (kept byte-for-byte with scene.cpp) -----------------
// The noise + planar-UV primitives the procedural surface library is built from.
// The library ITSELF lives in surfaces_facade.metal and surface_{road,water,
// terrain}.metal, dispatched by surfaces.metal — all of which are concatenated
// after this file and depend on these primitives.
constant float SURF_PI = 3.14159265;

float hash21(float a, float b) {
    return fract(sin(a * 12.9898 + b * 78.233) * 43758.5453);
}
float vnoise2(float x, float y) {
    float xi = floor(x), yi = floor(y), xf = x - xi, yf = y - yi;
    float a = hash21(xi, yi), b = hash21(xi + 1.0, yi);
    float c = hash21(xi, yi + 1.0), d = hash21(xi + 1.0, yi + 1.0);
    float ux = xf * xf * (3.0 - 2.0 * xf), uy = yf * yf * (3.0 - 2.0 * yf);
    return a * (1.0 - ux) * (1.0 - uy) + b * ux * (1.0 - uy) +
           c * (1.0 - ux) * uy + d * ux * uy;
}
float fbm2(float x, float y) {
    float v = 0.0, amp = 0.5, f = 1.0;
    for (int i = 0; i < 4; ++i) { v += amp * vnoise2(x * f, y * f); f *= 2.0; amp *= 0.5; }
    return v;
}
float tile1(float x, float m) { return x - m * floor(x / m); }
float2 surfUV(float3 p, float3 n) {
    if (abs(n.y) > 0.5) return float2(p.x, p.z);
    float2 t = float2(n.z, -n.x);
    float tl = length(t);
    t = tl < 1e-6 ? float2(1.0, 0.0) : t / tl;
    return float2(p.x * t.x + p.z * t.y, p.y);
}

