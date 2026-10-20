#version 450
// CDLOD terrain vertex stage (ADR-0036; ports lighting_entry.metal terrainVertexMain).
// Terrain nodes are world-space meshes with an identity model. The tangent slot
// carries each vertex's morph target — the position it collapses to on the next-
// coarser grid — and we blend toward it by camera distance over [morphStart,
// morphEnd], so a node matches the coarser silhouette before its parent takes
// over (no pop, no crack). Feeds the shared lit fragment (mesh.frag).

layout(location = 0) in vec3 inPosition;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec3 inTangent;   // the morph target: y its height, x and z the coarser level's normal
layout(location = 3) in vec2 inTexcoord;
layout(location = 4) in vec3 inColor;

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
    vec4  fog;
    vec4  shadowTint;
    vec4  wind1;
    vec4  wind2;
    vec4  skyMoonDir;         // (sky.frag's members, declared to reach the tail)
    vec4  skyMoonSun;
    vec4  skyCelX;
    vec4  skyCelY;
    vec4  skyCelZ;
    vec4  skyStars;
    vec4  skyCity;
    vec4  terrainHorizon;     // xy world origin, z 1/extent, w enabled
    vec4  terrainHorizon2;    // x encodeLo, y encodeHi - encodeLo, z 1: no impostor self-shadow (RT_IMPOSTOR_SELF_SHADOW=0),
                              // w 1: grass bent to the sky, not its ground (RT_GRASS_GROUND=0)
    vec4  viewerBoxA;         // the viewer's enclosure (SceneLighting::viewerBox*): xyz centre, w half along u (0: none)
    vec4  viewerBoxB;         // xy its axis u (x, z), z half across it, w half up
    vec4  glassMode;          // x 1: glass as a filter (0: the old glass, RT_GLASS=old; 2: none, RT_GLASS=none),
                              // y the ceiling over the viewer (0: none)
    vec4  terrainNormal;      // the terrain normal map (binding 5, ADR-0205): xy world origin, z 1/extent, w enabled
    vec4  terrainNormal2;     // x the camera distance it blends in from, y 1/(the blend's length),
                              // z 1: no surface relief (RT_TERRAIN_DETAIL=0, ADR-0207)
    vec4  terrainSurface;     // the cover + macro maps (bindings 6, 7, ADR-0208): xy world origin, z 1/extent, w enabled
    vec4  terrainSurface2;    // x the camera distance the cover map blends in from, y 1/(the blend's length)
} g;

layout(push_constant) uniform Push {
    vec4  features[4];     // the material features (ADR-0098); the model is per instance
    vec4  albedoMetallic;
    vec4  emissionRough;
    uvec4 surfaceFlags;
    float morphStart;
    float morphEnd;
} pc;

layout(location = 0) out vec3 outWorldPos;
layout(location = 1) out vec3 outWorldNormal;
layout(location = 2) out vec2 outTexcoord;
layout(location = 3) out vec3 outColor;
layout(location = 4) out vec3 outWorldTangent;
layout(location = 5) flat out vec3 outInstanceOrigin;   // mesh.frag's (unused for terrain)

void main() {
    float dist = distance(g.cameraPosition.xyz, inPosition);
    float k = clamp((dist - pc.morphStart) / max(pc.morphEnd - pc.morphStart, 1e-3),
                    0.0, 1.0);
    vec3 worldPos = vec3(inPosition.x, mix(inPosition.y, inTangent.y, k), inPosition.z);
    // SNOW HAS DEPTH (ADR-0210; Glenn: the snow "seems painted onto the surface"): on the layered ground (surface 19,
    // whose u is the snow's weight) the surface rises with it -- up to 0.6 m where it lies thick -- so a snowfield
    // has an edge and buries the rock under it; the player walks the ground beneath, as on real snow
    if (pc.surfaceFlags.x == 19u && g.terrainNormal2.z < 0.5) worldPos.y += 0.6 * smoothstep(0.25, 0.9, inTexcoord.x);
    // ...and its normal toward the coarser level's (ADR-0204), so a tile meets its parent shading as the parent does
    vec3 coarseN = vec3(inTangent.x, sqrt(max(0.0, 1.0 - inTangent.x * inTangent.x - inTangent.z * inTangent.z)), inTangent.z);

    outWorldPos = worldPos;
    outWorldNormal = normalize(mix(normalize(inNormal), coarseN, k));
    outWorldTangent = vec3(1.0, 0.0, 0.0);   // terrain has no normal map
    outTexcoord = inTexcoord;
    outColor = inColor;
    gl_Position = g.viewProjection * vec4(worldPos, 1.0);
    outInstanceOrigin = vec3(0.0);
}
