#ifndef RAYTRACER_ENGINE_PROCGEN_MATERIAL_RECIPES_H
#define RAYTRACER_ENGINE_PROCGEN_MATERIAL_RECIPES_H

// MATERIAL RECIPES (ADR-0098): texture sets composed from texture_field's vocabulary and baked
// through its cache -- the stone of the rock library today. Each is tileable and meant for the
// TRIPLANAR material feature (RenderMaterial::triplanarScale): world-space, no UVs.

#include "tree.h"   // TextureData

#include <cstdint>
#include <string>

namespace engine {

enum class StoneKind { Granite, Sandstone, Basalt, Masonry };   // masonry: dressed blocks (quays, walls)
bool stoneKindFromName(const std::string& n, StoneKind& out);

struct StoneTextures {
    TextureData albedo;   // RGBA: rgb sRGB colour, a height (the top layer settles in low spots)
    TextureData normal;   // RGB tangent-space normal from the same height
};
StoneTextures stoneTextures(StoneKind kind, uint32_t seed, int size = 256);   // cached

// BARK (the tree lab, 2026-10-17): a tree's bark by kind, a tile that wraps both ways -- u ROUND the limb, v ALONG
// it (tree.cpp sweeps it a whole number of times round a limb by its girth, a tile a metre along it).
//   furrowed  deep vertical ridges between long narrow plates (oak, chestnut)
//   plated    irregular flat plates parted by cracks (pine, spruce)
//   smooth    a thin, nearly flat skin with faint mottling (beech, aspen)
//   papery    white with short dark lenticel dashes across it, black patches, peeling bands (birch)
//   ringed    rings round it, fibres along it (palms, cherry)
// RGB is a BRIGHTNESS the bark's vertex colour tints (linear, not gamma), as the tree's leaf textures are; the
// normal is from the same height. Old names map on: "oak" furrowed, "birch" papery, "pine"/"spruce" plated,
// "beech"/"aspen" smooth, "palm"/"cherry" ringed.
enum class BarkKind { Furrowed, Plated, Smooth, Papery, Ringed };
bool barkKindFromName(const std::string& n, BarkKind& out);
struct BarkTextures {
    TextureData albedo;   // RGBA: RGB brightness (linear), A 255
    TextureData normal;   // RGB tangent-space normal (x round the limb, y along it)
};
BarkTextures barkTextures(BarkKind kind, uint32_t seed, int size = 512);   // cached

// A STRAP LEAF's surface (foliage_kit.h addBlade: flax, iris, rush, a palm's leaflets): u across the blade, v
// along it -- fine veins running its length, a paler midrib, the margins a shade darker, a little mottle; the
// normal map ribs it along the veins. Brightness (the blade's vertex colours give the hue); tiles along v.
BarkTextures bladeTextures(uint32_t seed, int size = 256);   // cached

}  // namespace engine

#endif
