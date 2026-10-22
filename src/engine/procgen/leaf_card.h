#ifndef RAYTRACER_ENGINE_PROCGEN_LEAF_CARD_H
#define RAYTRACER_ENGINE_PROCGEN_LEAF_CARD_H

// LEAF CARDS DRAWN BY AN L-SYSTEM (Glenn, 2026-10-17: "when I look at the leaf cards I can see where it's cut off ...
// would we need different leaf cards for different trees? Can we generate those cards using the l-system to draw
// those leaves?"). A card is a small 2D twig: a grammar (the same parametric L-system as the trees,
// tree_recipe.h LeafCardSpec) drawn by a 2D turtle -- F(len) a stem, +(a) -(a) turns (degrees), [ ] a side shoot,
// L(scale) a leaf -- with the recipe's LEAF SHAPE at every L:
//   oval      a pointed oval (beech, aspen)          lobed     rounded lobes down each side (oak)
//   serrate   a toothed triangle-ovate (birch)       needle    a long thin needle (spruce, pine)
//   leaflet   a narrow lance (palm and ash leaflets)
// The drawing is fitted INSIDE the card with a margin -- nothing cut off at its edge -- its base at the card's foot
// (where the card meets the branch), supersampled for clean edges, colour dilated under the cut so mips do not
// darken. RGB is a brightness pattern the leaf's vertex colour tints (as tree.h leafTexture); the NORMAL map cups
// each leaf about its midrib, so a card lights leaf by leaf, not as one flat sheet. `variants` x `variants` cards,
// each its own draw of the grammar's ranges, share the atlas. The grammar sees `$row`, the atlas row it is drawn
// in: with TreeParams::tipCards the tree puts row 0 at its tips (a salvia's spikes) and the rest along its stems.

#include "tree.h"          // TextureData
#include "tree_recipe.h"   // LeafCardSpec

#include <cstdint>

namespace engine {

struct LeafCardAtlas {
    TextureData albedo;   // RGBA: RGB brightness (tinted by vertex colour), A the cut-out
    TextureData normal;   // RGBA: RGB the tangent-space normal (x across the card, y along it), A 255
    int tiles = 1;        // cards a side
};

// The atlas: tiles of tileW x tileH pixels (the card's own aspect -- tree.cpp's cards are 1 wide by 1.7 long).
LeafCardAtlas leafCardAtlas(const LeafCardSpec& spec, uint32_t seed, int tileW = 256, int tileH = 436);

// THE ELEMENT ATLAS for near foliage (foliage_kit.h addKitLeaf / addKitBlossom): 2 x 2 square tiles, each ONE leaf of
// the card's shape (drawn up the tile, its stalk at the foot) -- or, with `flowers`, the second row one flower each
// (centred). The same leaf and flower the species' cards draw, whole, one at a time.
LeafCardAtlas foliageElementAtlas(const LeafCardSpec& card, bool flowers, uint32_t seed, int tile = 256);

}  // namespace engine

#endif
