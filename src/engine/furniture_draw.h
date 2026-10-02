#ifndef RAYTRACER_ENGINE_FURNITURE_DRAW_H
#define RAYTRACER_ENGINE_FURNITURE_DRAW_H

// DRAWING THE FURNITURE KIT (buildings M4b; the furniture library, M2): placed pieces become INSTANCE GROUPS -- one
// per (piece, variant, finish), the kit's mesh uploaded once for the session, the placements as its transforms --
// with the finish's baked surface maps (wood grain, fabric). Shared by the streamed interiors and the city's
// outdoor furniture.

#include "../renderer/renderer.h"
#include "procgen/city/shape_grammar.h"   // PlacedPiece
#include "world.h"
#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace engine {

class AssetManager;

struct FurnitureDrawCache {
    std::unordered_map<uint64_t, MeshHandle> pieceMesh;             // (piece, variant, finish) -> uploaded mesh
    std::unordered_map<int, std::array<TextureHandle, 4>> surfTex;  // baked surface maps by Surface id
};

// Create the groups for `pieces` (world transforms) in `world`, append their entities to `out`; returns the
// triangles drawn. `tint` is the finish materials' fallback colour (materialFor's wallColor argument).
std::size_t spawnFurnitureGroups(World& world, AssetManager& assets, Renderer* renderer, FurnitureDrawCache& cache,
                                 const std::vector<PlacedPiece>& pieces, const Vec3& tint, double drawDistance,
                                 std::vector<Entity>& out);

}  // namespace engine

#endif
