#version 450
// Phase 1 forward mesh vertex stage (ADR-0057). Ported from the Metal
// vertexMain (shaders/metal/lighting_entry.metal) + the Vertex layout in common.metal.
// The viewProjection is uploaded already carrying the Vulkan clip-space Y-flip
// (the C++ side negates clip row 1), so this stage needs no convention fix-ups.

// Two vertex layouts, one shader (ADR-0096). The full layout feeds float3 normal/tangent/colour
// (the missing .w reads as 1); the 32-byte standard one feeds octahedral snorm16 pairs in .xy
// and an 8-bit colour, and its pipeline twin turns kPackedVertex on.
layout(constant_id = 0) const bool kPackedVertex = false;
layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec4 inNormal;
layout(location = 2) in vec4 inTangent;
layout(location = 3) in vec2 inTexcoord;
layout(location = 4) in vec4 inColor;
// The model matrix, per instance (binding 1, a column per location; ADR-0097). A plain
// drawMesh is a one-instance draw, so every mesh reads it here, not from the push block.
layout(location = 5) in mat4 inModel;

// Octahedral pair -> unit vector (the inverse of octEncode() in vulkan_renderer.cpp).
vec3 octDecode(vec2 e) {
    vec3 n = vec3(e, 1.0 - abs(e.x) - abs(e.y));
    float t = max(-n.z, 0.0);
    n.x += n.x >= 0.0 ? -t : t;
    n.y += n.y >= 0.0 ? -t : t;
    return normalize(n);
}

// Must match the block layout in mesh.frag and GlobalsUBO in vulkan_renderer.cpp.
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
    ivec4 counts;          // x lightCount, y cascadeCount, z envMode
    vec4  shadowParams;    // x normalBias, y pcfRadius, z mapSize, w strength
    vec4  skySunDir;
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
    vec4  fog;             // rgb fog color, w density (matches GlobalsUBO)
    vec4  shadowTint;      // rgb artistic tint, w ambientStrength
    vec4  wind1;           // xyz wind dir, w wind time
    vec4  wind2;           // x frequency, y height, z amplitude
} g;

layout(push_constant) uniform Push {
    mat4  model;
    vec4  albedoMetallic;
    vec4  emissionRough;
    uvec4 surfaceFlags;
    float morphStart;      // FLAG_GRASS: the fade band (terrain.vert: its morph band)
    float morphEnd;
} pc;

layout(location = 0) out vec3 outWorldPos;
layout(location = 1) out vec3 outWorldNormal;
layout(location = 2) out vec2 outTexcoord;
layout(location = 3) out vec3 outColor;
layout(location = 4) out vec3 outWorldTangent;

void main() {
    vec4 world = inModel * vec4(inPosition, 1.0);
    vec3 normal = kPackedVertex ? octDecode(inNormal.xy) : inNormal.xyz;
    vec3 tangent = kPackedVertex ? octDecode(inTangent.xy) : inTangent.xyz;

    // Wind sway (FLAG_WIND = bit 2): displace in the wind direction, weighted by
    // height above the model's base (planted root, moving tips) and phase-offset
    // by world XZ so a field doesn't sway in unison. Ports lighting_entry.metal.
    const bool grass = (pc.surfaceFlags.y & (1u << 17)) != 0u;   // FLAG_GRASS
    // Ground cover shrinks to nothing about its planted origin across the fade band, so the
    // field thins away with distance instead of ending at a line.
    if (grass) {
        vec3 origin = inModel[3].xyz;
        float k = 1.0 - smoothstep(pc.morphStart, pc.morphEnd, distance(origin, g.cameraPosition.xyz));
        world.xyz = origin + (world.xyz - origin) * k;
    }
    if ((pc.surfaceFlags.y & 4u) != 0u) {
        float baseY = inModel[3].y;
        // grass sways over its own half-metre height, not a tree's
        float swayHeight = grass ? 0.6 : g.wind2.y;
        float weight = clamp((world.y - baseY) / max(swayHeight, 0.001), 0.0, 1.0);
        weight *= weight;
        float phase = g.wind1.w * g.wind2.x + dot(world.xz, vec2(0.15, 0.1));
        float gust = sin(phase) + 0.3 * sin(phase * 2.3 + 1.7);
        world.xz += g.wind1.xz * (g.wind2.z * weight * gust);
    }

    outWorldPos = world.xyz;
    // Inverse-transpose so non-uniform scale keeps normals perpendicular.
    mat3 normalMatrix = mat3(transpose(inverse(inModel)));
    outWorldNormal = normalize(normalMatrix * normal);
    // Tangent in world space for normal mapping (matches Metal's model*tangent).
    outWorldTangent = normalize((inModel * vec4(tangent, 0.0)).xyz);
    outTexcoord = inTexcoord;
    outColor = inColor.rgb;
    gl_Position = g.viewProjection * world;
}
