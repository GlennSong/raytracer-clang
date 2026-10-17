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

struct TreeRecipe {
    std::string name;
    std::string axiom = "A(1.6)";   // module literals once its $vars are substituted
    int iterations = 5;
    struct Rule {
        std::string predecessor;     // "A(l)" or guarded "A(l):l>0.5"
        std::string successor;
        double weight = 1.0;
    };
    std::vector<Rule> rules;         // empty: the built-in grammar (growTree) from the params
    struct Range {
        std::string name;
        double lo = 0, hi = 0;       // lo == hi: a fixed number
    };
    std::vector<Range> params;       // TreeParams fields
    std::vector<Range> vars;         // $names in the rules and the axiom
    std::string bark = "oak";        // the bark maps' style (tree.h barkStyleFromName)
    Vec3 barkColor{0.33, 0.24, 0.16}, barkColorHi{0.33, 0.24, 0.16};   // drawn between
    Vec3 leafColor{0.19, 0.44, 0.13}, leafColorHi{0.19, 0.44, 0.13};
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
