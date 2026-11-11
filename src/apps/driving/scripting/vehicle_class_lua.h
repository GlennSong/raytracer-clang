#ifndef RAYTRACER_APPS_DRIVING_SCRIPTING_VEHICLE_CLASS_LUA_H
#define RAYTRACER_APPS_DRIVING_SCRIPTING_VEHICLE_CLASS_LUA_H

// READS A VEHICLE'S `driving` BLOCK OUT OF LUA (issue #248): the numbers the driving model needs that the vehicle data
// cannot derive -- comfortable speeding-up and braking, hardest braking, top speed, the tag names, and a trailer when
// it pulls one. The block sits on each fleet entry in assets/scripts/vehicles.lua:
//
//   driving = { speed_up = 1.5, brake = 2.0, hardest_brake = 8.0, top_speed = 50, tags = { "car" },
//               trailer = { length = 13.6, hitch_ahead = 0.5 } }   -- (trailer optional)
//
// Units: m/s2 and m/s. The sizes (length, width, wheelbase, turning radius) are NOT here: they are derived from the
// body and the wheels (vehicle_class.h deriveVehicleClass), never written twice.
//
// Every fault is reported (a missing or non-positive number, a tag that is not a string, an unknown key, so a typo is
// caught), one per line, each naming `where` and the key. On any fault `out` is left untouched.
//
// This and rule_set_lua are the only driving files allowed to include Lua.

#include "apps/driving/vehicle_class.h"

#include <string>

struct lua_State;

namespace driving {

// The block is the table at stack index `t` of `L` (the stack is left as found). `where` names it in messages (for
// example "vehicle.fleet[3].driving"). Fills only the authored fields of `out`.
bool readVehicleClassBlock(lua_State* L, int t, const std::string& where, VehicleClass& out, std::string* err);

}  // namespace driving

#endif
