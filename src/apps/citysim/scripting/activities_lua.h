#ifndef RAYTRACER_CITYSIM_ACTIVITIES_LUA_H
#define RAYTRACER_CITYSIM_ACTIVITIES_LUA_H

// The activity catalog from Lua (assets/scripts/activities.lua): globals `activities` (name -> definition) and
// `menus` (name -> { bands }). See citysim/activities.h for the fields; an unknown site kind, tag or perform is an
// error, not a guess.

#include "../activities.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadActivityCatalog(ScriptVM& vm, citysim::ActivityCatalog& out, std::string* err);
}  // namespace engine

#endif
