#ifndef RAYTRACER_ENGINE_ROCK_H
#define RAYTRACER_ENGINE_ROCK_H

#include "../renderer/renderer.h"   // RenderMesh
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

}  // namespace engine

#endif
