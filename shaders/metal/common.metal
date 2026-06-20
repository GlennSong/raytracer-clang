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

// Hash a brick's (column,row) to [0,1] — matches scene.cpp brickHash.
float brickHash21(float a, float b) {
    return fract(sin(a * 12.9898 + b * 78.233) * 43758.5453);
}

// World-space running-bond brick (kept in lockstep with scene.cpp applyBrick).
// `albedo` is the brick base colour; mortar joints darken to neutral grey and
// each brick takes a small colour jitter, so a facade reads as masonry up close.
float3 applyBrick(float3 albedo, float3 worldPos, float3 normal) {
    float u, v;
    if (abs(normal.y) > 0.9) {
        u = worldPos.x; v = worldPos.z;
    } else {
        float2 t = float2(normal.z, -normal.x);
        float tl = length(t);
        t = tl < 1e-6 ? float2(1.0, 0.0) : t / tl;
        u = worldPos.x * t.x + worldPos.z * t.y;
        v = worldPos.y;
    }

    const float courseH = 0.075;   // brick + bed-joint height
    const float brickL  = 0.20;    // brick + head-joint length
    const float mortar  = 0.011;   // joint half-width

    float row = floor(v / courseH);
    float offset = (fmod(abs(row), 2.0) < 1.0) ? 0.0 : brickL * 0.5;
    float uu = u + offset;
    float col = floor(uu / brickL);

    float fy = v - row * courseH;
    float fx = uu - col * brickL;
    float joint = min(min(fy, courseH - fy), min(fx, brickL - fx));

    // Per-brick value jitter, an occasional darker "burnt header", and a faint
    // within-brick gradient so faces aren't dead flat.
    float h = brickHash21(col, row);
    float h2 = brickHash21(col * 1.7 + 3.1, row * 0.9 + 5.7);
    float shade = 0.74 + 0.46 * h;
    if (h2 < 0.12) shade *= 0.6;
    shade *= 0.94 + 0.12 * (fx / brickL);
    float3 brickCol = albedo * shade;
    float3 mortarCol = float3(0.30, 0.29, 0.27);

    float t = saturate((joint - mortar) / 0.004);
    return mix(mortarCol, brickCol, t);
}
