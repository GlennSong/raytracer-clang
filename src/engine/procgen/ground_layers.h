#ifndef RAYTRACER_ENGINE_PROCGEN_GROUND_LAYERS_H
#define RAYTRACER_ENGINE_PROCGEN_GROUND_LAYERS_H

// TERRAIN LAYERS (the flora plan): the ground as a few tileable materials -- grass, dirt,
// sand, rock -- height-blended in the shader by the ground-cover map's weights, the way
// modern terrain does it, instead of one vertex colour under procedural noise bumps (which
// lit as orange peel). Stylized: soft painterly colour, the detail in colour and in each
// layer's HEIGHT (alpha), which decides who wins at a transition (sand fills between
// pebbles, rock pokes up through grass). No normal maps: the ground lights smooth.
//
// Each layer is a RECIPE over texture_field's primitives (tileable noise, cells, bands, warps),
// baked RGBA8 (rgb the albedo gamma-encoded, a the height) through texture_field's disk cache
// (bakeCached). Exactly tileable because every primitive is.

#include "tree.h"   // TextureData
#include "../../rt_math.h"

#include <cstdint>

namespace engine {

enum class GroundLayer : uint8_t { Grass = 0, Dirt, Sand, Rock };

// `linearBase` is the layer's mean colour (linear albedo, e.g. GroundCoverParams::grass).
TextureData groundLayerTexture(GroundLayer layer, const Vec3& linearBase, int size, uint32_t seed);
// The same through the disk cache (RT_NOCACHE=1 always builds).
TextureData groundLayerTextureCached(GroundLayer layer, const Vec3& linearBase, int size, uint32_t seed);

// World metres one texture tile spans (the shader's planar scale). Shared with the shader.
constexpr double kGroundLayerTileMetres = 4.0;

}  // namespace engine

#endif
