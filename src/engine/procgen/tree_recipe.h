#ifndef RAYTRACER_ENGINE_PROCGEN_TREE_RECIPE_H
#define RAYTRACER_ENGINE_PROCGEN_TREE_RECIPE_H

// TREE RECIPES (Glenn, 2026-10-17: "Could lua be used to define recipes and then the C++ could ingest those recipes
// and build out those trees?"). A species is DATA -- assets/scripts/tree_recipes.lua, read by the scripting layer
// (scripting/tree_recipes_lua.h) -- and C++ grows it: the grammar expanded (ParametricLSystem), the turtle run,
// the pipe model, the gnarl and the tube skin (tree.h skinTree). No Lua runs per branch.
//
// Variety lives in the recipe's RANGES: every number may be a {lo, hi} the seed draws within, so one recipe is a
// species and its seeds the trees of it ("same type of tree and different trees"). Two kinds of number:
//   params  TreeParams fields by name (droop, lobes, leafSize, ...: tree.h)
//   vars    the recipe's own names, written $name in its rules and axiom (an angle, a falloff) -- substituted per
//           seed before the grammar is compiled, so a rule reads "&($pitch)A(l*$side)"
// A recipe with no rules grows from the built-in grammar (growTree) and its params alone.

#include "tree.h"

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace engine {

// A grammar rule and a drawn number: the tree's and its leaf card's.
struct RecipeRule {
    std::string predecessor;     // "A(l)" or guarded "A(l):l>0.5"
    std::string successor;
    double weight = 1.0;
};
struct RecipeRange {
    std::string name;
    double lo = 0, hi = 0;       // lo == hi: a fixed number
};

// THE LEAF CARD (leaf_card.h): a 2D twig grammar and the leaf shape it draws at every L.
struct LeafCardSpec {
    bool set = false;                     // the recipe has a card (else the generic leaf cluster)
    std::string axiom = "S(5)";
    int iterations = 6;
    std::vector<RecipeRule> rules;
    std::vector<RecipeRange> vars;        // drawn per card variant
    std::string shape = "oval";           // oval | lobed | serrate | needle | leaflet | palmate
    double leafLength = 0.30, leafWidth = 0.14;   // a leaf at L(1), in the grammar's units
    int lobes = 3;                        // lobed: lobes down each side; palmate: lobes in all (a maple's 5 or 7)
    int teeth = 9;                        // serrate: teeth down each side
    double stemWidth = 0.012;             // the twig's stroke
    double cup = 0.5;                     // how far a leaf's normal tips across it (0: flat)
    double shadeJitter = 0.18;            // leaf-to-leaf brightness variation
    double tipBright = 0.0;               // the new growth: leaves lighter toward the card's far end (0 none)
    int variants = 2;                     // cards a side of the atlas
    // COLOUR in the card (Glenn, 2026-10-17: "more color"): the atlas RGB is each part's brightness times its tint,
    // and the tree's vertex colour multiplies that. White (the default) leaves the vertex colour the whole colour;
    // a card with blossoms on it tints them apart from its leaves and its twig.
    Vec3 leafTint{1, 1, 1}, stemTint{1, 1, 1};
    // THE FLOWER the card grammar draws at every W(scale), facing along the turtle:
    //   blossom  `petals` round petals, each notched at its tip by `notch` (sakura, apple)
    //   daisy    `petals` narrow rays round a disc (euryops, cosmos, marguerite)
    //   floret   one small two-lipped flower: stacked up a stem they make a spike (salvia, lavender)
    // `eye` is the centre's share of the radius, coloured eyeTint (a sakura's pink eye, a daisy's disc).
    struct Flower {
        bool set = false;
        std::string shape = "blossom";
        double size = 0.08;               // across, at W(1), in the grammar's units
        int petals = 5;
        double notch = 0.2, eye = 0.25;
        Vec3 tint{1, 1, 1}, eyeTint{1, 1, 1};
    } flower;
};

struct TreeRecipe {
    std::string name;
    bool shrub = false;              // kind = "shrub": a shrub or a perennial (knee to head height), not a tree
    std::string axiom = "A(1.6)";   // module literals once its $vars are substituted
    int iterations = 5;
    using Rule = RecipeRule;
    std::vector<Rule> rules;         // empty: the built-in grammar (growTree) from the params
    using Range = RecipeRange;
    std::vector<Range> params;       // TreeParams fields
    std::vector<Range> vars;         // $names in the rules and the axiom
    std::string bark = "oak";        // the bark maps' style (tree.h barkStyleFromName)
    Vec3 barkColor{0.33, 0.24, 0.16}, barkColorHi{0.33, 0.24, 0.16};   // drawn between
    Vec3 leafColor{0.19, 0.44, 0.13}, leafColorHi{0.19, 0.44, 0.13};
    // LEAF PALETTES: a tree's cards take their colour from a palette, blended across the crown in patches of
    // paletteScale metres (one limb red, the next orange: an autumn maple). Several palettes: each seed picks one.
    // Empty: leafColor..leafColorHi, one colour a tree.
    std::vector<std::vector<Vec3>> leafPalettes;
    LeafCardSpec card;
    // NEAR FOLIAGE (foliage_kit.h): the strand -- one shoot drawn flat, its own card grammar -- and the spray the
    // shoots are built into. A tree grown with foliage = 1 wears these instead of the card.
    LeafCardSpec strand;
    SpraySpec spray;
    LeafKitSpec leafKit;             // foliage = 2: every leaf a blade, blossoms cups (wearing the card's leaf and flower)
};

class TreeRecipeBook {
public:
    void set(TreeRecipe r);
    const TreeRecipe* find(const std::string& name) const;
    std::size_t size() const { return recipes_.size(); }
    std::vector<std::string> names() const;
private:
    std::vector<TreeRecipe> recipes_;
};

// Set a TreeParams field by its name (the recipe's and a level's tree block's spelling); false for an unknown name.
bool setTreeParam(TreeParams& p, const std::string& name, double value);
// Every name setTreeParam knows (the reader rejects others: a typo would otherwise silently grow the default).
const std::vector<std::string>& treeParamNames();

// The tree a recipe's ranges give for `seed`: its TreeParams and its vars' values.
TreeParams drawTreeParams(const TreeRecipe& r, uint32_t seed, std::map<std::string, double>* vars = nullptr);

// `text` with every $name replaced by its value in `vars`; `missing` gets a name with no value.
std::string substituteVars(const std::string& text, const std::map<std::string, double>& vars, std::string* missing = nullptr);

// Grow one tree of the recipe. `overrides` (a level's per-tree knobs: the lab's gnarl row) win over the draw.
TreeMesh growRecipeTree(const TreeRecipe& r, uint32_t seed, const std::map<std::string, double>& overrides = {});

}  // namespace engine

#endif
