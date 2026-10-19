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
        int kind = -1;                          // a TWIG SEGMENT's: 0 leafy, 1 leafy Y, 2 bare, 3 bare Y (-1 a sprig)
    };
    float twigLength = 0;                       // the twig segments' own length (m): a card scales from it
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

// TWIG SEGMENTS (Glenn, 2026-10-18, of the sprig pictures: "Those look like small trees ... It needs to be a branch
// without leaves and with leaves or maybe a Y branch without leaves and with leaves ... smaller segments that can then
// be planted along a bigger trunk to fill out"): for a species with near leaves (a leaf kit), the cards are not
// pictures of sprigs but of TWIG SEGMENTS, `twigLength` long, grown from the species' own leaf and bark -- a straight
// twig and a Y twig, each leafy and bare, `variants` of each -- drawn from above (the leaves face the sky).
BranchCardSet bakeTwigCards(const TreeRecipe& r, double twigLength, int variants, uint32_t seed);
// ...laid along the tree's fine branching (TreeMesh::twigs: a segment along every internode a sprig stood for, leafy
// where the tree carries leaves, bare inside, a Y now and then), and leafy segments sprouting along its thin modelled
// limbs (under `fillRadius` m), every `fillSpacing` m -- each segment two planes in a shallow V about its axis.
int wearTwigCards(TreeMesh& tree, const BranchCardSet& set, const BranchCardSpec& spec, double fillRadius, double fillSpacing,
                  uint32_t seed, int perTwig = 1, double clusterScale = 1.0);   // (perTwig: segments a leafy twig, fanned round it --
                                                                                // a big crown's clusters, clusterScale times a twig's size)

// THE NEAR FOLIAGE (Glenn, 2026-10-18: "For up close you'd want to make individual leaves and branches because they
// will have the most scrutiny ... make sure the twigs look like they're solid like the 3D trunks/branches. The leaves
// should have some kind of subsurface effect"): along the tree's fine branching (TreeMesh::twigs) a THIN TUBE of bark
// for every twig (three sides; the thinnest left bare of wood past `twigTris` triangles) in tree.branches -- the trunk's
// own material, lit and shadowed as it is -- and its LEAVES one by one, each a folded blade wearing the species'
// leaf (normal-mapped; the foliage material lets the sun through), about `leaves` of them over the crown (each
// larger as they are fewer, the crown's leaf area kept), turned to the light, darker toward the crown's middle; a
// blossoming species' cupped blossoms at the twig ends. tree.foliage 2 (foliageElementAtlas).
int wearNearLeaves(TreeMesh& tree, const TreeRecipe& r, int leaves, int twigTris, double fillRadius, double fillSpacing,
                   uint32_t seed);

// A CONIFER'S NEEDLES (TreeRecipe::needles): `variants` of each of a main shoot (a twig's needles along it), a side
// shoot (shorter) and, for a pine, a tuft (needles in bundles round a twig's end), drawn from needle geometry -- each
// needle a thin blade off its shoot, turned round it, its normal out from the shoot (the normal map rounds the shoot),
// this year's growth lighter at the end -- in the picture kinds 0, 1, 2.
BranchCardSet bakeNeedleCards(const TreeRecipe& r, int variants, uint32_t seed);
// ...worn along the tree's fine branching: each twig a bark tube (three to five sides by its girth; the thickest first,
// to `twigTris`) and its needles --
// two crossed cards along it (a flat fir's one, opened in a shallow V), the share `reach` from its end -- and its side
// shoots, `shoots` (0..1) of the recipe's, from points ON it, in the branch's plane, forward; a pine's tufts at its end.
int wearNeedles(TreeMesh& tree, const TreeRecipe& r, const BranchCardSet& set, double shoots, int twigTris, int maxPlanes,
                uint32_t seed);   // (at most `maxPlanes` card planes: the side shoots, then the shoots, thinned evenly)

}  // namespace engine

#endif
