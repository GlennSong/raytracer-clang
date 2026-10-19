#ifndef RAYTRACER_ENGINE_PROCGEN_REAL_TREE_H
#define RAYTRACER_ENGINE_PROCGEN_REAL_TREE_H

// REAL TREES (ADR-0129, Glenn: "some more realistic trees over the landscape. Forests with many
// trees"). Built the way production foliage is: a bark skeleton of tapered tubes, and FOLIAGE AS
// ALPHA-CUT CARDS carrying a painted spray (needles on a twig, a cluster of leaves), lit with
// CROWN-BENT NORMALS -- each card's normal points out from the crown's axis, so the crown shades
// as one soft volume instead of a thousand flat planes -- and darkened toward the crown's inside
// (the self-shadowing a card cannot compute).
//
//   conifers  spruce | fir: a straight leader, whorls of branches every half metre, drooping
//             and upturned, crown a narrow cone; pine: a tall clear bole, a flat-topped crown of
//             a few heavy limbs carrying needle tufts
//   broadleaf oak | beech | birch | maple | aspen | willow (hanging sprays) | alder | shrub (a
//             multi-stemmed understory bush): a trunk that forks into limbs, branches out to a crown SHELL
//             (points on an ellipsoid, the way real crowns carry their leaves outside), leaf
//             clusters around every branch tip
//
// Far away a tree is an IMPOSTOR: renderImpostor rasterises the tree (on the CPU, once per
// variant) into a side and a top picture; forest.h stands them as crossed cards.

#include "../../renderer/renderer.h"   // RenderMesh
#include "tree.h"                      // TextureData

#include <cstdint>
#include <string>

namespace engine {

// The STREET TREES of a tropical island town (Glenn, 2026-10-10: "a variety of trees for the city beyond the ones
// seen along the island's forests"), after Shrub so the forest species keep their numbers:
//   royal_palm    a smooth pale-grey column, a green crownshaft, a head of arching feather fronds
//   coconut_palm  a slim ringed trunk that leans and bows, drooping fronds, a bunch of nuts
//   monkeypod     the rain tree: a short trunk forking low into a vast flat umbrella of fine leaves
//   poinciana     the flame tree: a smaller umbrella, a good share of it in red-orange flower
//   jacaranda     an open rounded crown, mostly lilac-purple in bloom
//   plumeria      frangipani: a short candelabra of thick pale limbs, paddle leaves and cream flowers at the tips
enum class RealSpecies : uint8_t { Spruce = 0, Fir, Pine, Oak, Beech, Birch, Maple, Aspen, Willow, Alder, Shrub,
                                   RoyalPalm, CoconutPalm, Monkeypod, Poinciana, Jacaranda, Plumeria, Count };
// a palm: a column and a head of fronds, no limbs (its trunk collider is its column)
inline bool realSpeciesIsPalm(RealSpecies s) { return s == RealSpecies::RoyalPalm || s == RealSpecies::CoconutPalm; }
bool realSpeciesFromName(const std::string& name, RealSpecies& out);
const char* realSpeciesName(RealSpecies s);

struct RealTree {
    RenderMesh bark;      // tubes; vertex colour is the bark (linear); white material
    RenderMesh foliage;   // alpha-cut cards; uv into foliageTexture; vertex colour = tint x shading
    double height = 0.0, crownRadius = 0.0, crownBase = 0.0, trunkRadius = 0.0;
};

// One tree of `species`, about `height` metres; the seed varies everything else.
RealTree realTree(RealSpecies species, uint32_t seed, double height);

// The species' foliage spray (RGBA, `size` square): RGB a near-white brightness pattern the card's
// vertex colour tints, alpha the cut-out (dilated colour under the cut, so mips do not darken).
TextureData realFoliageTexture(RealSpecies species, int size, uint32_t seed);

// Rasterise `tree` into an RGBA picture (w x h), orthographic: `top` false looks along -z (the side;
// the tree fills the height, centred), true looks straight down. Colour is LINEAR and multiplied by
// `colourScale` (so 8 bits keep the dark greens: the impostor's vertex colour undoes it); alpha is
// the silhouette; colour dilated under the cut.
// `normalOut` (optional, same layout): the tree's own surface normal at each pixel (#93, Glenn: "The tree
// cards aren't lit well. I can tell they're the cards"), in the CARD's tangent frame as the mesh shader reads
// it (T = the card's u axis, B = N x T, N = the card's plane): the crown-bent leaf normals keep their
// sideways and up components, so the card lights clump by clump like the model; the component along the
// card's plane normal is folded to a symmetric 0.25..0.75 (a two-sided card is seen from both sides, and
// its back must not light as its front). Encoded n * 0.5 + 0.5; uncovered texels (0.5, 0.5, 1).
void renderImpostor(const RealTree& tree, const TextureData& foliage, bool top, int w, int h,
                    double colourScale, uint8_t* rgbaOut, int strideBytes, uint8_t* normalOut = nullptr);

// MANY-VIEW IMPOSTORS (Glenn, 2026-10-18: "So can we make multi angle imposter cards"): the tree seen from any
// direction -- `azimuth` round it (0: from +z, toward +x as it grows) and `elevation` above the horizon (radians) --
// an orthographic picture framed on its centre (`centreY` up its axis): x -Rb..Rb across (Rb: its reach from the axis),
// y -Hh..Hh up the view (Hh: impostorViewHalfHeight, the same for every view, so one window fits them all). The
// view's frame (the normal map's): T = r = (cos az, 0, -sin az), N = d = toward the viewer, B = u = d x r.
// Colour, alpha and normals as renderImpostor's.
void renderImpostorView(const RealTree& tree, const TextureData& foliage, double azimuth, double elevation, double centreY,
                        double Rb, double Hh, int w, int h, double colourScale, uint8_t* rgbaOut, int strideBytes,
                        uint8_t* normalOut = nullptr);
// ...the window's half height for views up to `maxElevation` (radians): every part of the tree inside it from any of them
double impostorViewHalfHeight(const RealTree& tree, double centreY, double maxElevation);

// The same picture of any bark and foliage, framed by the caller: the side view (along -z) spans x0..x1 across and
// y0..y1 up the image; the top view (`top`, down -y) x0..x1 across and z y0..y1 down it. (renderImpostor frames a
// whole tree; branch_cards.h a sprig in its own frame.)
// The extras a close-up picture wants (branch_cards.h's twig and leaf sprites; Glenn, 2026-10-18: "these sprites will
// need some kind of normal map to give them dimension"): the bark's own maps on the bark (its u round the limb, v
// along it, the vertex tangent along it), the leaves' normal map perturbing theirs (the vertex tangent across the
// leaf), and the parts deeper in the picture shaded darker by `depthShade` (0 none .. 1 black at the back).
struct CardViewExtras {
    const TextureData* barkAlbedo = nullptr;
    const TextureData* barkNormal = nullptr;
    const TextureData* foliageNormal = nullptr;
    double depthShade = 0.0;
    bool trueNormals = false;   // the normal map in the card's frame as it is, no fold (a sprite seen from its face)
    int alphaGrow = 0;          // the cut-out widened this many pixels (thin needles: they thin away further down the mips)
};
void renderCardView(const RenderMesh& bark, const RenderMesh& foliage, const TextureData& fol, bool top, double x0, double x1,
                    double y0, double y1, int w, int h, double colourScale, uint8_t* rgbaOut, int strideBytes,
                    uint8_t* normalOut = nullptr, const CardViewExtras* extras = nullptr);

}  // namespace engine

#endif
