#ifndef RAYTRACER_CITYSIM_PEOPLE_LUA_H
#define RAYTRACER_CITYSIM_PEOPLE_LUA_H

// The people catalog from Lua (assets/scripts/people.lua): global `people` = { stages, bands, households, halls }.
// See citysim/households.h for what each part means; an unknown stage or band, a share-less type or a dwelling size
// no household fits is an error, not a guess.

#include "../households.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadPeopleCatalog(ScriptVM& vm, citysim::PeopleCatalog& out, std::string* err);
}  // namespace engine

#endif
