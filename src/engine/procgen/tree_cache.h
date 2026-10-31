#ifndef RAYTRACER_ENGINE_PROCGEN_TREE_CACHE_H
#define RAYTRACER_ENGINE_PROCGEN_TREE_CACHE_H

// THE GROWN TREES ON DISK (Glenn, 2026-10-18, of the island with the recipe trees: its forest built in 121 s against
// the real_tree forest's 42 -- every variant grown at its four levels on every load, the mature oak's ~7 s each): a
// forest's recipe species, all its variants' levels (tree_lods.h RecipeTreeLevel: the meshes, the card atlases, the
// bands), in one file under cache/trees, keyed by everything that grows them -- the recipe file's bytes, the recipe's
// name and age, the seeds, the bands, kTreeCacheVersion and the engine's build (a commit is a new key: a changed
// grower never wears an old tree). RT_TREE_CACHE=0 grows them every time. Kept under RT_TREE_CACHE_MAX_GB (default 4):
// a write deletes the least recently used files past it.

#include "tree_lods.h"

#include <cstdint>
#include <string>
#include <vector>

namespace engine {

constexpr uint32_t kTreeCacheVersion = 2;   // bump when the growers change between commits; 2: ProcRng (ADR-0265)

// The key: the recipe file (its path, hashed by content), what the loader asks for. 0: no cache (RT_TREE_CACHE=0, or
// the file unreadable).
uint64_t recipeSpeciesKey(const std::string& recipeFile, const std::string& recipe, const std::string& age,
                          const std::vector<uint32_t>& seeds, const RecipeLodBands& bands);
std::string treeCacheDir();   // cache/trees (RT_TREE_CACHE_DIR)

// Read / write a species' variants' levels. Read: false (and `out` untouched) on a miss or a file it cannot read.
bool loadRecipeSpecies(uint64_t key, std::vector<std::vector<RecipeTreeLevel>>& out);
bool saveRecipeSpecies(uint64_t key, const std::vector<std::vector<RecipeTreeLevel>>& variants);

}  // namespace engine

#endif
