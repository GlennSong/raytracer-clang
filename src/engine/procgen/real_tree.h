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
//   broadleaf oak | beech | birch: a trunk that forks into limbs, branches out to a crown SHELL
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

enum class RealSpecies : uint8_t { Spruce = 0, Fir, Pine, Oak, Beech, Birch, Count };
bool realSpeciesFromName(const std::string& name, RealSpecies& out);
const char* realSpeciesName(RealSpecies s);
bool realSpeciesIsConifer(RealSpecies s);

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
void renderImpostor(const RealTree& tree, const TextureData& foliage, bool top, int w, int h,
                    double colourScale, uint8_t* rgbaOut, int strideBytes);

}  // namespace engine

#endif
