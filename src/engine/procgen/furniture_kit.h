#ifndef ENGINE_PROCGEN_FURNITURE_KIT_H
#define ENGINE_PROCGEN_FURNITURE_KIT_H

// THE FURNITURE KIT (buildings M4b; Glenn, 2026-09-30: "I think you should legit build furniture. It feels like
// you're repurposing parts from the flooring."). Modelled pieces -- a five-star office chair on casters with a gas
// lift, a desk with a drawer pedestal, a sofa of separate cushions, a made bed, kitchen units with doors, worktop,
// sink and hob -- written as RECIPES over the shape kit's words (MeshBuilder::roundedBox, lathe, tube, cylinder),
// one mesh per MATERIAL so each wears a real one: wood grain, fabric, painted/plastic, metal, ceramic.
//
// A piece is modelled once per (piece, variant) and DRAWN INSTANCED (InstanceGroup): a room places transforms,
// not triangles, so a piece can carry the detail a close look needs. The variant picks the wood and the fabric.
//
// Piece space: x across the piece (centred), z out from the wall it backs onto (0 = against the wall), y up from
// the floor. `size` is the footprint the placer reserves (x, height, z).

#include "../../renderer/renderer.h"
#include <array>
#include <cstdint>
#include <string>

namespace engine {

enum class FurnMat : uint8_t { Wood, Fabric, Hard, Metal, Ceramic, Count };
constexpr int kFurnMatCount = static_cast<int>(FurnMat::Count);

enum class Piece : uint8_t {
    Desk, OfficeChair, Monitor, FilingCabinet,
    Bed, Nightstand, Wardrobe,
    Sofa, CoffeeTable, TvUnit,
    KitchenBase, KitchenSink, KitchenHob, KitchenTall, KitchenWall,
    DiningTable, DiningChair,
    Bathtub, Toilet, Vanity,
    LoungeChair, Planter,
    Picture, Shelving, Rug,   // buildings B: wall art, a closet's shelves, a living-room rug
    DeskPod, Cubicle, MeetingTable, Whiteboard,   // buildings C: the office floor
    Count
};
constexpr int kPieceCount = static_cast<int>(Piece::Count);

struct FurniturePiece {
    std::array<RenderMesh, kFurnMatCount> mesh;   // by FurnMat; an empty mesh = the piece has none of it
    Vec3 size{1, 1, 1};                           // the footprint box: x width, y height, z depth
    bool solid = true;                            // the placer adds its box to the collider
    Real colliderH = 0;                           // the collider's height when not size.y (a tap is not a wall)
};

// The piece, modelled once per (piece, variant) and cached (thread-safe). Variant: bits 0-2 the fabric, 3-4 the
// wood, the rest free for the recipe's own style dice.
const FurniturePiece& furniturePiece(Piece p, uint32_t variant);
const char* furniturePieceName(Piece p);
bool furniturePieceByName(const std::string& name, Piece& out);

}  // namespace engine

#endif
