#ifndef RAYTRACER_ENGINE_SCRIPTING_PREFAB_LIST_H
#define RAYTRACER_ENGINE_SCRIPTING_PREFAB_LIST_H

// The list of prefabs, read (#191, step 7): assets/scripts/prefabs.lua, or a level's own. See procgen/prefab.h.

#include "../procgen/prefab.h"

#include <string>
#include <vector>

namespace engine {

// The prefab named `name`, checked: its line says only what a prefab can say today, and its model is in the list of
// models. False with `error` otherwise. `levelDir`: where a level keeps its own scripts (may be empty). `list` and
// `models`: the two lists' files, for a test with lists of its own.
bool findPrefab(const std::string& name, Prefab& out, const std::string& levelDir = std::string(), std::string* error = nullptr,
                const std::string& list = "prefabs.lua", const std::string& models = "models.lua");

// The names in the list, sorted. Empty with `error` when the list cannot be read.
std::vector<std::string> namedPrefabs(const std::string& levelDir = std::string(), std::string* error = nullptr, const std::string& list = "prefabs.lua");

}  // namespace engine

#endif
