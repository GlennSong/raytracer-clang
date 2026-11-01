#ifndef RAYTRACER_CITYSIM_PLACE_KINDS_LUA_H
#define RAYTRACER_CITYSIM_PLACE_KINDS_LUA_H

// The place-kind catalog from Lua (assets/scripts/places.lua): global `places` = { order, kinds, dwellings,
// house_sizes }. See citysim/place_kinds.h for what each part means; an unknown type, a recipe two kinds claim or a
// mix with no shares is an error, not a guess.

#include "../place_kinds.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadPlaceKindCatalog(ScriptVM& vm, citysim::PlaceKindCatalog& out, std::string* err);
}  // namespace engine

#endif
