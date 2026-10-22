#ifndef RAYTRACER_ENGINE_SCRIPTING_TREE_RECIPES_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_TREE_RECIPES_LUA_H

// Reads the TREE RECIPES (procgen/tree_recipe.h) from Lua: the global table `tree_recipes` that
// assets/scripts/tree_recipes.lua leaves behind, keyed by species name.

#include "../procgen/tree_recipe.h"
#include <string>

namespace engine {

class ScriptVM;

// Fill `out` from the VM's `tree_recipes` global. False (and `err`) on a malformed recipe: an unknown param name, a
// $name in a rule or the axiom with no var, a rule without both halves -- a typo must not grow a default tree.
bool readTreeRecipes(ScriptVM& vm, TreeRecipeBook& out, std::string* err = nullptr);
bool loadTreeRecipesSource(const std::string& source, TreeRecipeBook& out, std::string* err = nullptr);

// The shipped recipes (assets/scripts/tree_recipes.lua), read afresh each call -- a level's trees reload with it,
// so the tree lab picks up an edited recipe. Empty (and a warning) when the file is missing or malformed. `path`
// gets the file's path for the script watcher.
TreeRecipeBook loadShippedTreeRecipes(std::string* path = nullptr);
// ...the same, parsed again only when the file's text has changed since the last call (a level of many recipe
// trees reads it once; an edit in the tree lab still shows on the reload)
const TreeRecipeBook& shippedTreeRecipes(std::string* path = nullptr);

}  // namespace engine

#endif
