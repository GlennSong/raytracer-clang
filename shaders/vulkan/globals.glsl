// THE GLOBALS (ADR-0372): set 0 as the scene's shaders see it -- the lights, the frame's (or a probe face's) view,
// the sky, the clustered lights and the frame's textures. Shared by mesh.frag and probe_relight.comp; one block, so
// the C++ GlobalsUBO has one mirror to keep in step.

const float PI = 3.14159265359;

struct Light {
    vec4 positionIntensity;   // xyz position (point/spot), w intensity
    vec4 directionInner;      // xyz direction (dir/spot), w innerCosAngle
    vec4 colorOuter;          // rgb color, w outerCosAngle
    vec4 typeRange;           // x type (0 point,1 directional,2 spot), y range
};

layout(set = 0, binding = 0) uniform Globals {
    mat4  viewProjection;
    mat4  view;
    mat4  invViewProjection;
    mat4  cascadeVP[4];
    vec4  cameraPosition;
    vec4  ambient;            // rgb = ambient tint * multiplier (IBL strength), w = facade glass reflectance scale
    vec4  cascadeSplit;       // far view-space depth of cascades 0..3
    ivec4 counts;             // x lightCount, y cascadeCount, z envMode, w debugView
    vec4  shadowParams;       // x normalBias, y pcfRadius, z mapSize, w strength
    vec4  skySunDir;          // xyz dir, w disc intensity
    vec4  skySunColor;
    vec4  skyZenith;
    vec4  skyHorizon;
    vec4  skyGround;
    vec4  skyCloud;
    Light lights[32];
    vec4  fog;                // rgb fog color, w density (0 = off)
    vec4  shadowTint;         // rgb artistic shadow tint, w ambientStrength
    vec4  wind1;              // xyz wind dir, w time (used by mesh.vert)
    vec4  wind2;              // x frequency, y height, z amplitude
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
    ivec4 clusterGrid;        // CLUSTERED LIGHTS (RT_CLUSTERED=1): x tilesX, y tilesY, z slices, w 1: on (bindings 8-10)
    vec4  clusterDepth;       // x the slices' near depth, y 1/log(far/near), z framebuffer width, w height
    vec4  skyPhysical;        // THE PHYSICAL SKY (ADR-0367; envMode 2): x ground radius, y camera height, z the active
    vec4  skyPhysical2;       //   light's top illuminance, w cos(sun radius); x disc visibility, y the other light's,
    vec4  skyPhysical3;       //   z top radius; x 1: the active light is the sun, y the celestial scale, z the glows'
                              //   emission into units (ADR-0369)
    vec4  skyPhysical4;       // the air for the aerial perspective: rgb Rayleigh /m, w its scale height;
    vec4  skyPhysical5;       //   x Mie extinction /m, y its scale height, z the level's aerial density
    vec4  skySH[9];           // the sky's light on surfaces: E / pi, order-2 spherical harmonics (ADR-0371)
} g;
#include "sky_physical.glsl"
// the clustered lights (light_clusters.h): every point and spot, each froxel's offset (low 24 bits) | count (high 8),
// and the froxels' index lists
layout(std430, set = 0, binding = 8) readonly buffer ClusterLightBuf { Light clusterLights[]; };
layout(std430, set = 0, binding = 9) readonly buffer ClusterCellBuf { uint clusterCells[]; };
layout(std430, set = 0, binding = 10) readonly buffer ClusterIndexBuf { uint clusterIndices[]; };

layout(set = 0, binding = 1) uniform sampler2DArrayShadow shadowMap;
layout(set = 0, binding = 2) uniform sampler2D envEquirect;   // HDR env (counts.z==1)
layout(set = 0, binding = 3) uniform sampler2D brdfLut;       // split-sum BRDF LUT
layout(set = 0, binding = 4) uniform sampler2D terrainHorizonMap;   // mountain shadows
layout(set = 0, binding = 5) uniform sampler2D terrainNormalMap;    // the ground's slope per texel (ADR-0205)
layout(set = 0, binding = 6) uniform sampler2D terrainCoverMap;     // its cover weights per texel (ADR-0208)
layout(set = 0, binding = 7) uniform sampler2D terrainMacroMap;     // its convexity, wetness, rock tone, hardness (ADR-0208, -0209)

// THE REFLECTION PROBES (ADR-0373): their count (x), then each one's position (w 1: relit), box min, box max; and their
// relit cubes, a cube array with GGX mips (roughness 0 .. 1 over 5)
// the probes (ADR-0373/0374): x count, yz the plan grid's min corner, w its cell (m); x its size in cells; each probe's
// position (w 1: relit), box min, box max; the grid's cells, each the two probes to try, (first + 1) | (second + 1) << 8
layout(std430, set = 0, binding = 16) readonly buffer ProbeBuf { vec4 probeCount; vec4 probeGridSize; vec4 probeData[192]; uint probeCells[]; };
layout(set = 0, binding = 17) uniform samplerCubeArray probeRadiance;

// The reflection toward R from P out of the probe whose box holds P -- the box's parallax corrected (the ray's exit
// from the box, seen from the probe) -- faded in from the box's walls; `fallback` (the sky's) where none does.
vec3 probeReflection(vec3 P, vec3 R, float roughness, vec3 fallback) {
    // the plan grid's cell (ADR-0374) names two probes; the first relit one whose box holds P
    int k = int(probeGridSize.x);
    if (probeCount.x < 0.5 || k <= 0) return fallback;
    ivec2 gc = ivec2(floor((P.xz - probeCount.yz) / probeCount.w));
    if (any(lessThan(gc, ivec2(0))) || any(greaterThanEqual(gc, ivec2(k)))) return fallback;
    uint pair = probeCells[gc.y * k + gc.x];
    int best = -1;
    for (int j = 0; j < 2 && best < 0; ++j) {
        int i = int((pair >> (8 * j)) & 0xFFu) - 1;
        if (i < 0) break;
        if (probeData[i * 3].w < 0.5) continue;
        vec3 lo = probeData[i * 3 + 1].xyz, hi = probeData[i * 3 + 2].xyz;
        if (all(greaterThanEqual(P, lo)) && all(lessThanEqual(P, hi))) best = i;
    }
    if (best < 0) return fallback;
    vec4 pos = probeData[best * 3];
    vec3 lo = probeData[best * 3 + 1].xyz, hi = probeData[best * 3 + 2].xyz;
    vec3 r = normalize(R);
    vec3 t1 = (hi - P) / r, t2 = (lo - P) / r;
    vec3 tf = max(t1, t2);
    float t = min(min(tf.x, tf.y), tf.z);
    vec3 d = P + r * t - pos.xyz;
    vec3 c = textureLod(probeRadiance, vec4(d, float(best)), roughness * 4.0).rgb;
    vec3 edge = min(P - lo, hi - P);
    float w = clamp(min(min(edge.x, edge.z), edge.y + 100.0) / 4.0, 0.0, 1.0);   // (4 m from a wall; not the floor)
    return mix(fallback, c, w);
}
