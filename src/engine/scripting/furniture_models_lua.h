#ifndef RAYTRACER_ENGINE_SCRIPTING_FURNITURE_MODELS_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_FURNITURE_MODELS_LUA_H

// THE FURNITURE KIT'S MODELS IN LUA (#191; Glenn, 2026-11-06: "port the furniture to use the new meshbuilder in lua"):
// assets/scripts/furniture_models.lua leaves a global `furniture_models`, a recipe per kit piece by its name
// (furniturePieceName), each `function(variant) -> piece` built over the mesh words. A piece is
//   { size = {x, y, z}, solid = true, collider_h = 0, colliders = { {centre, size}, ... },
//     parts = { wood = mesh, fabric = mesh, hard = mesh, metal = mesh, ceramic = mesh, light = mesh } }
// in piece space (procgen/furniture_kit.h): x across, y up, z out from the wall.

#include "../procgen/furniture_kit.h"
#include <cstdint>
#include <string>

namespace engine {

// Build `name` in `variant` from the recipe source `source` (the file's text), in a fresh VM per call. False (and
// `err`) when the source fails, the recipe is missing, or what it returns is malformed.
bool buildFurnitureModelFrom(const std::string& source, const std::string& name, uint32_t variant, FurniturePiece& out,
                             std::string* err = nullptr);

// The same from the shipped assets/scripts/furniture_models.lua, read and run once per process into one VM kept for
// every later piece (thread-safe: one piece at a time).
bool buildFurnitureModel(const std::string& name, uint32_t variant, FurniturePiece& out, std::string* err = nullptr);

}  // namespace engine

#endif
