#ifndef RAYTRACER_ENGINE_SCRIPTING_FLEET_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_FLEET_LUA_H

// A LEVEL'S FLEET, READ FROM ITS VEHICLE SCRIPT (vehicles.lua: `vehicle.fleet`, one slot a kind of vehicle). The parts
// of the fleet the ENGINE reads: how many slots, a slot's class and size, its handling, and what the roads are checked
// against (each vehicle tag's most demanding body, ADR-0435). What a slot looks like (its meshes) and how it drives
// (its `driving` block) are read by the city sim, over these (apps/citysim/scripting/vehicle_body.h).
//
// The VM must have the procgen library and the module loader open and the vehicle script run.

#include "../../rt_math.h"
#include "../procgen/city/roads/vehicle_size.h"
#include "tag_names.h"      // readTagNames: a slot's `driving.tags`
#include "vehicle_spec.h"   // VehicleSpec: a slot's handling

#include <string>
#include <vector>

struct lua_State;

namespace engine {

class ScriptVM;

// How many slots `vehicle.fleet` declares. THE FLEET'S LENGTH IS THE ASSET'S TO DECIDE. Returns 0 when there is no
// fleet to count (no global, not an array), which the caller reads as "use the built-in fleet".
int fleetSlotCount(ScriptVM& vm);

// Point `vehicle.fleet` at one of the script's NAMED fleets (vehicles.lua: fleet_kit, classic, fleet_mixed), after the
// script has run -- a level's citysim.fleet. False (err filled, fleet untouched) when there is no such table.
bool selectFleet(ScriptVM& vm, const std::string& name, std::string* err = nullptr);

// Push `vehicle.fleet[slot+1]` onto the stack. On failure the stack is restored to `base` and false is returned with
// `err` filled (`where` names the slot in it).
bool pushFleetSlot(lua_State* L, int base, int slot, const std::string& where, std::string* err);

// The CATALOGUE half of a slot -- its class name and its size -- without building any geometry. A recipe may publish
// its body behind a `build()` function; this never calls it, so reading what a car MEASURES costs nothing.
// `size` is left zero when the slot declares none (a legacy recipe): the caller keeps its built-in dimensions for that
// slot. Returns false only for a missing global, a missing/out-of-range slot, or a malformed entry.
bool loadFleetCatalogueEntry(ScriptVM& vm, int slot, Vec3& size, std::string& className, std::string* err = nullptr);

// The HANDLING half of a slot (item 23, #117): its `handling` table read by the one handling reader into a spec whose
// config is what a physical-tier car or a commandeered one drives with. No geometry is built. False (err filled) when
// the slot has none or it carries no wheels.
bool loadFleetHandling(ScriptVM& vm, int slot, VehicleSpec& out, std::string* err = nullptr);

// A SLOT'S BODY AS IT FOLLOWS A ROAD: its class name; length and width from its `size`; the wheelbase from its
// handling's wheels (the steered axle to the middle of the others, vehicle_size.h) and its largest steering angle.
// False (err filled) when any part is missing or malformed.
struct FleetSlotBody {
    std::string className;
    Real length = 0, width = 0, wheelbase = 0, maxSteerDegrees = 0;
};
bool loadFleetSlotBody(ScriptVM& vm, int slot, FleetSlotBody& out, std::string* err = nullptr);

// WHAT THE ROADS ARE CHECKED AGAINST: every tag any class of the fleet carries, with its most demanding body
// (vehicle_size.h addVehicleSize), sorted by tag. A class is the first slot of its name, as the driving model takes
// them. A slot that cannot be read is left out and named in `notes`. Empty when the script has no fleet.
std::vector<roads::lanes::VehicleSize> fleetVehicleSizes(ScriptVM& vm, std::vector<std::string>* notes = nullptr);

}  // namespace engine

#endif
