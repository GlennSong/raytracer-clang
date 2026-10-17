#ifndef RAYTRACER_ENGINE_PROCGEN_FOLIAGE_KIT_H
#define RAYTRACER_ENGINE_PROCGEN_FOLIAGE_KIT_H

// THE FOLIAGE KIT -- near foliage as small geometry, not crossed cards (Glenn, 2026-10-17: "the spruce leaves are
// clustered but they're flat ... have a 2d texture image of a strand of spruce blades and then put them around a
// cylinder to create one of the leaves and then cluster those using geometry ... Make a variety of those so we can
// instance different ones and create a tree that way"; the crossed cards "create weird slits and gaps ... could be
// okay mid-tier lods but not close up").
//
//   STRAND  a texture: one shoot drawn flat -- its twig down the middle, needles both sides, a bud at the tip -- by the
//           recipe's `strand` card grammar (leaf_card.h), an atlas of a few of them
//   SHOOT   geometry: `fins` planes through the shoot's own axis, each wearing a strand -- fletching, not a tube: a
//           shoot reads full from the side and end-on alike -- bent down a little along its length
//   SPRAY   geometry: a main shoot with side shoots fanned off it alternately, mostly flat, shortening toward the
//           tip, a few lifted out of the fan for depth; `variants` of them per species, built once
//   TREE    every leafy node of the skeleton takes a spray variant, laid along its twig with the fan level, turned
//           and scaled a little (TreeParams::foliage = 1)
//
// A spray is built in its own frame -- +Y along the main shoot, +X across the fan, +Z the fan's up -- at unit length.

#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"             // Vec3

#include <cstdint>
#include <vector>

namespace engine {

struct SpraySpec {
    bool set = false;
    int sideShoots = 7;          // side shoots off the main one, alternating sides
    double sideAngle = 50.0;     // their angle off the main shoot (deg)
    double sideLength = 0.55;    // the first (longest) side shoot, a fraction of the main shoot
    double sideFalloff = 0.88;   // each next side shoot this x the last
    double lift = 14.0;          // side shoots tilted out of the fan by up to this (deg): depth
    int fins = 3;                // strand planes through each shoot's axis
    double aspect = 0.40;        // a strand's width over its length (the atlas tile's aspect)
    double droop = 0.25;         // the shoots bend down this much (rad) over their length
    int variants = 6;            // sprays built per species
    int perNode = 1;             // sprays on each leafy node of the tree
    int budget = 2500;           // sprays a tree at most (thinned evenly over the crown, as cards are)
    Vec3 color{1, 1, 1}, colorHi{1, 1, 1};   // a spray's vertex colour, drawn between (the strand carries the hues)
};

// The species' spray variants, each in the spray frame at unit length; the strand atlas is `tiles` x `tiles`.
std::vector<RenderMesh> buildSprays(const SpraySpec& spec, int tiles, uint32_t seed);

}  // namespace engine

#endif
