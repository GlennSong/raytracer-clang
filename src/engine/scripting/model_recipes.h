#ifndef RAYTRACER_ENGINE_SCRIPTING_MODEL_RECIPES_H
#define RAYTRACER_ENGINE_SCRIPTING_MODEL_RECIPES_H

// NAMED MODELS (#191, model step 6): the models the world can ask for by name, and how each is built.
//
// `assets/scripts/models.lua` is the list, as data:
//
//     return {
//       sedan  = { recipe = "car_rig_lab", args = { name = "sedan" } },
//       pickup = { recipe = "car_rig_lab", args = { name = "pickup" } },
//       crane  = { files = { "crane_high.glb", "crane_low.glb" }, distances = { 0, 80 } },
//     }
//
// A model is a recipe (a script under assets/scripts, named as `require` names one) run with its arguments, or
// model files read. `obtainModel("sedan")` gives the one sedan, building it the first time and keeping it in the
// model store (procgen/model_store.h); it is built again only when what it was built from has changed: its line in
// the list, its recipe, a library the recipe requires, a file it reads, or another model it uses. A new kind of
// model is a new line here. No C++ changes.
//
// A recipe asks for another named model with `model.get("tire")` and uses it as a piece, so every car shares the one
// tire. A kept model is frozen: shared, so not to be changed.

#include "../procgen/model_store.h"

#include <nlohmann/json.hpp>

#include <memory>
#include <string>
#include <vector>

namespace engine {

// The model named `name` in models.lua, from the store, built or rebuilt if it has to be. Null with `error` when
// there is no such name, its recipe fails, or it needs itself. `levelDir`: where a level keeps its own scripts and
// model files (may be empty). `store`: the process's store when null. `list`: the list's file, for a test with a list
// of its own.
std::shared_ptr<const ProcModel> obtainModel(const std::string& name, const std::string& levelDir = std::string(), std::string* error = nullptr,
                                             ModelStore* store = nullptr, const std::string& list = "models.lua");

// A MODEL STRAIGHT FROM A RECIPE, without a line in the list: the recipe (a script's name, as `require` names one)
// run with `argsJson` (a JSON object, or empty). Kept in the store like any other, under `recipeModelName`, and
// built again only when what it was built from has changed. For a model still being written (the editor's model
// studio), which earns its line in models.lua when it is done.
std::shared_ptr<const ProcModel> obtainRecipeModel(const std::string& recipe, const std::string& argsJson = std::string(),
                                                   const std::string& levelDir = std::string(), std::string* error = nullptr,
                                                   ModelStore* store = nullptr, const std::string& list = "models.lua");
std::string recipeModelName(const std::string& recipe, const std::string& argsJson = std::string());

// Every file a model is built from, as far as is known: its recipe, the list that names it, each library the recipe
// required, each file it read, and the same for every model it uses. What a host watches to rebuild on an edit. A
// model whose recipe failed still gives its recipe's file, so a mistake can be fixed.
std::vector<std::string> modelSourceFiles(const std::string& name, const std::string& levelDir = std::string(), ModelStore* store = nullptr,
                                          const std::string& list = "models.lua");

// A LUA TABLE WRITTEN AS TEXT, as JSON: `{ kind = "tree", seed = 3 }` -> {"kind":"tree","seed":3}. For a host
// that takes a recipe's arguments from a person (the model studio's Arguments box). False with `error` when the
// text is not Lua, or is not a table of named values. Empty text is no arguments: true, with `json` empty.
bool luaTableToJson(const std::string& luaText, std::string& json, std::string* error = nullptr);

// The names in the list, sorted. Empty with `error` when the list cannot be read.
std::vector<std::string> namedModels(const std::string& levelDir = std::string(), std::string* error = nullptr, const std::string& list = "models.lua");

// A LIST SCRIPT (models.lua, prefabs.lua): a script that returns a table of name = { ... }, run and given as JSON.
// False with `error` when it cannot be read, fails, or returns something else.
bool readScriptList(const std::string& file, const std::string& levelDir, nlohmann::json& out, std::string* error = nullptr);

}  // namespace engine

#endif
