#ifndef RAYTRACER_ENGINE_PROCGEN_MATERIAL_RECIPES_H
#define RAYTRACER_ENGINE_PROCGEN_MATERIAL_RECIPES_H

// MATERIAL RECIPES (ADR-0098): texture sets composed from texture_field's vocabulary and baked
// through its cache -- the stone of the rock library today. Each is tileable and meant for the
// TRIPLANAR material feature (RenderMaterial::triplanarScale): world-space, no UVs.

#include "tree.h"   // TextureData

#include <cstdint>
#include <string>

namespace engine {

enum class StoneKind { Granite, Sandstone, Basalt };
bool stoneKindFromName(const std::string& n, StoneKind& out);

struct StoneTextures {
    TextureData albedo;   // RGBA: rgb sRGB colour, a height (the top layer settles in low spots)
    TextureData normal;   // RGB tangent-space normal from the same height
};
StoneTextures stoneTextures(StoneKind kind, uint32_t seed, int size = 256);   // cached

}  // namespace engine

#endif
