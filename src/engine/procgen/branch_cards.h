#ifndef RAYTRACER_ENGINE_PROCGEN_BRANCH_CARDS_H
#define RAYTRACER_ENGINE_PROCGEN_BRANCH_CARDS_H

// BRANCH CARDS (Glenn, 2026-10-18: "When we get to small enough branches that we instead just use some kind of card to
// stand in for those branches and clusters of leaves and somehow still make that look realistic enough"; the wood
// held to ~8000 triangles a hero tree). How production trees are built (SpeedTree's branch atlases, a game's
// spruce): the trunk and limbs modelled, and every small branch with its leaves a CARD wearing a picture of one.
//
// Ours are baked from the recipe's own growth, not painted. A tree grown with TreeParams::sprigSize records its
// SPRIGS -- the side shoots whose whole spread is card-sized -- and builds none of them. The same tree grown as a
// DONOR (sprigDonor) builds every sprig whole, each into its own meshes; a few of those, chosen across their sizes,
// are drawn into an atlas (real_tree.h renderCardView: colour and the card-frame normal map, as the impostors are).
// Then each sprig of the tree wears the picture nearest it in size, scaled to it, on a card laid along it:
//   a SPINE from where it leaves its limb out along its axis, the card's two halves lifted `fold` off flat about it
//   (a spray's V), across its widest spread, facing the sky -- and with `views` above one, as many cards turned
//   about the spine, each wearing the sprig as seen from its side (the sprig has depth from anywhere; Glenn of one
//   card a sprig: the crowns read flat and thin);
//   its NORMALS bent toward out of the crown (`bend`) -- the crown shades as one volume, clump by clump from the map;
//   darker toward the crown's middle (the shade the cards cannot cast on each other), and each picture's colour
//   carried at colourScale (8 bits keep the dark greens; the vertex colour undoes it, as the impostors do).

#include "leaf_card.h"
#include "tree.h"
#include "tree_recipe.h"

#include <cstdint>
#include <vector>

namespace engine {

struct BranchCardSet {
    LeafCardAtlas atlas;      // albedo RGB the sprigs' own colour x colourScale, A the cut; normal: the card's frame
    // a picture is one sprig seen from `views` sides round its axis (the face first, then turned 180 / views at a
    // time): each view its cell and the frame it shows
    struct View {
        float u0 = 0, v0 = 0, u1 = 1, v1 = 1;   // its cell (v0 the sprig's far end, v1 its base)
        float x0 = 0, x1 = 0, y0 = 0, y1 = 0;   // the frame it shows (m): across, out along the axis
    };
    struct Picture {
        std::vector<View> views;
        float length = 0, halfWidth = 0;        // the sprig it shows (TreeMesh::Sprig)
    };
    std::vector<Picture> pictures;
    double colourScale = 3.0;
    bool empty() const { return pictures.empty(); }
};

// The pictures, from a donor tree's sprigs (grown with sprigSize and sprigDonor) and the leaves' own picture
// (tree_lods.h recipeLeafAtlas). Empty if the donor has no sprigs.
BranchCardSet bakeBranchCards(const TreeMesh& donor, const LeafCardAtlas& leaves, const BranchCardSpec& spec);

// Every sprig of `tree` (grown with sprigSize, not a donor) a card from `set`, into tree.leaves (foliage 5: one
// winding, drawn two-sided). Returns the cards laid.
int wearBranchCards(TreeMesh& tree, const BranchCardSet& set, const BranchCardSpec& spec, uint32_t seed);

}  // namespace engine

#endif
