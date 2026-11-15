#ifndef RAYTRACER_ENGINE_SCRIPTING_TAG_NAMES_H
#define RAYTRACER_ENGINE_SCRIPTING_TAG_NAMES_H

// A LIST OF TAG NAMES read from Lua: the one reader of a vehicle's `driving.tags` (which lanes it may use). The roads
// read it to know what bodies to check a junction's paths against (fleet_lua.h fleetVehicleSizes) and the driving
// library to know which paths a class may take (apps/driving/scripting/vehicle_class_lua.cpp).

#include <string>
#include <vector>

struct lua_State;

namespace engine {

// The table at stack index `list`: its names sorted, repeats dropped. `notNames` (optional) gets the 1-based places of
// entries that are not a non-empty string.
void readTagNames(lua_State* L, int list, std::vector<std::string>& names, std::vector<int>* notNames = nullptr);

}  // namespace engine

#endif
