#ifndef RAYTRACER_ENGINE_ROCK_H
#define RAYTRACER_ENGINE_ROCK_H

#include "../../renderer/renderer.h"   // RenderMesh
#include "noise.h"

namespace engine {

// A noise-displaced rock (ROADMAP 4 Phase B.1): start from a sphere and push
// each vertex along its radial normal by a noise field, then recompute normals.
// Deterministic for a given Noise. The pragmatic stand-in for SDF rocks (A.1),
// off the Forest critical path but using the same value types.
struct RockParams {
    float radius = 1.0f;
    float displacement = 0.35f;   // fraction of radius the noise can push/pull
    double noiseScale = 2.0;      // bumpiness frequency
    int octaves = 3;
    int stacks = 16;
    int slices = 24;
};

RenderMesh generateRock(const RockParams& params, const Noise& noise);

// A rock as an SDF *graph* (ADR-0022 "a rock is a generator, not rock.cpp"):
// a base sphere smooth-unioned with seeded lumps and minus a few subtracted
// facets, polygonized. The same handful of composable ops a node graph would
// expose — tunable to make rocks of any character. Deterministic per seed.
struct RockSdfParams {
    float baseRadius = 1.0f;
    int lumps = 6;            // extra spheres smooth-unioned on for bumps
    int cuts = 2;             // spheres subtracted for flat facets
    float lumpScale = 0.5f;   // lump radius as a fraction of baseRadius
    double smoothness = 0.3;  // blend radius of the smooth-union
    int resolution = 40;      // polygonization grid cells per axis
    bool faceted = false;     // flat-shade per triangle (stylized low-poly):
                              // pair with a LOW resolution (~14-20) so the
                              // facets are big enough to read as cut planes
};

RenderMesh generateRockSdf(const RockSdfParams& params, uint32_t seed);

}  // namespace engine

#endif
