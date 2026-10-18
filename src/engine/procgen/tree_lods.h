#ifndef RAYTRACER_ENGINE_PROCGEN_TREE_LODS_H
#define RAYTRACER_ENGINE_PROCGEN_TREE_LODS_H

// A RECIPE TREE'S LEVELS OF DETAIL (Glenn, 2026-10-17: "hero trees have everything wielded. And then mid level doesn't
// have to wield them all and then furthest would be cards"). One recipe and seed grown as up to three trees, each
// drawn over its own range and crossfaded per pixel into the next (RenderMaterial::FLAG_LOD_BAND):
//   hero   joints at every fork of wood, the near foliage (leaves one by one, sprays)       0 .. hero1
//   mid    joints on the trunk and main limbs only (jointMinRadius 0.10), the same leaves   hero0 .. mid1
//          coarser (TreeParams::leafLod 2.2: fewer, larger, each still turned its own way), no
//          twigs under 1.5 cm (woodMinRadius), 0.6 of the sides
//   light  no joints, no twigs (woodMinRadius), fewer sides, leafLod 3.5: a forest's       mid0 .. (the caller's)
// A species with sprays and no leaf kit (the spruce) wears its crossed cards at the mid and light levels.
//   far    the light tree coarser again (leafLod 6, limbs from 7 cm, four sides)          light0 .. (the caller's)
// The lab's single trees stop at the mid; a forest (forest.h) adds the light and far levels and impostors past them.
// Shared by the lab's tree entities and the forest so the two draw the same trees.

#include "leaf_card.h"
#include "tree_recipe.h"

#include <map>
#include <string>
#include <vector>

namespace engine {

struct RecipeTreeLevel {
    TreeMesh tree;
    LeafCardAtlas atlas;                         // its leaves' picture (recipeLeafAtlas)
    float in0 = 0, in1 = 0, out0 = 0, out1 = 0;  // fade in over [in0, in1], out over [out0, out1] (0, 0: no fade)
    double minDistance = 0, maxDistance = 0;     // culled nearer / farther (0: not)
};

struct RecipeLodBands {
    double hero0 = 24.0, hero1 = 30.0;   // hero -> mid (both 0: no hero level)
    double mid0 = 0.0, mid1 = 0.0;       // mid -> light (both 0: no light level, the mid is the last)
    double light0 = 0.0, light1 = 0.0;   // light -> far (leafLod 6, no limb under 7 cm, four sides; both 0: none)
};

// The levels, nearest first. A recipe with no near kit (no leaf kit, no sprays), `overrides` pinning `foliage`, or
// `lod` false grows one tree for the hero's place (still followed by the light level if the bands ask for one).
std::vector<RecipeTreeLevel> growRecipeLevels(const TreeRecipe& r, uint32_t seed,
                                              const std::map<std::string, double>& overrides = {},
                                              const RecipeLodBands& bands = {}, bool lod = true);

// The picture a recipe tree's leaves wear, by what it grew (TreeMesh::foliage): blades and fronds the strap leaf's
// veins, leaves one by one the element atlas, sprays the strand, cards the species' card (empty: none drawn).
LeafCardAtlas recipeLeafAtlas(const TreeRecipe& r, const TreeMesh& tree);

}  // namespace engine

#endif
