#ifndef RAYTRACER_CITYSIM_ROLES_LUA_H
#define RAYTRACER_CITYSIM_ROLES_LUA_H

// The role catalog from Lua (assets/scripts/roles.lua): global `roles` (name -> { day = {...}, night_out = share,
// job = {...}, staff = {...} }, with `order` naming them). See citysim/city_roles.h for what a day's parts mean; an
// unknown break target or label, or a catalog with no catch-all job role, is an error, not a guess.

#include "../city_roles.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadRoleCatalog(ScriptVM& vm, citysim::RoleCatalog& out, std::string* err);
}  // namespace engine

#endif
