#ifndef RAYTRACER_ENGINE_PROCGEN_SURFACE_MAPS_H
#define RAYTRACER_ENGINE_PROCGEN_SURFACE_MAPS_H

#include "tree.h"                      // TextureData
#include "../../renderer/renderer.h"   // RenderMaterial::Surface
#include <cstdint>

namespace engine {

// Bake the procedural city surfaces (brick, concrete, roof tile, asphalt, ...)
// into a tiling PBR texture set — the realtime-correct way to use procedural
// materials (ADR-0039 Phase B): the analytic pattern runs once on the CPU into
// textures, every material that needs "brick" samples the one shared set, and
// the lighting shader just does texture fetches (mip-filtered, no per-pixel
// pattern math). Mirrors the bark-map pipeline (height field -> albedo + normal).
//
// All maps tile seamlessly over [0,1] (the pattern is laid out in integer cells),
// so a surface is sampled with a world-scaled UV: uv = worldPlaneCoord /
// worldTileSize. worldTileSize() returns the metres one tile should span so the
// feature sizes read at human scale.
struct SurfaceMaps {
    TextureData albedo;   // RGB base colour (mortar grey vs brick face, ...)
    TextureData normal;   // RGB tangent-space normal (the relief — bricks proud)
    TextureData mr;       // RGB, glTF-style: G = roughness, B = metallic
    TextureData ao;       // grayscale ambient occlusion (crevice darkening)
    TextureData height;   // grayscale height (source of the normal; future POM)
};

SurfaceMaps surfaceMaps(RenderMaterial::Surface surface, int size = 256,
                        uint32_t seed = 0);

// Metres one tile should span for a surface, so the baked features (brick
// courses, slabs, cobbles) read at the right real-world size when tiled.
double surfaceWorldTileSize(RenderMaterial::Surface surface);

}  // namespace engine

#endif
