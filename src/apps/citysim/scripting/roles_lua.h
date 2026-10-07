#ifndef RAYTRACER_CITYSIM_ROLES_LUA_H
#define RAYTRACER_CITYSIM_ROLES_LUA_H

// The role catalog from Lua (assets/scripts/roles.lua): global `roles` (name -> { day = {...}, night_out = share },
// with `order` naming them -- the first four are the built-in Agent::Role ids). See citysim/city_roles.h for what a
// day's parts mean; an unknown break target or label is an error, not a guess.

#include "../city_roles.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadRoleCatalog(ScriptVM& vm, citysim::RoleCatalog& out, std::string* err);
}  // namespace engine

#endif
