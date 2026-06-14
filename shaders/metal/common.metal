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
