#ifndef RAYTRACER_CITYSIM_TRAVEL_LUA_H
#define RAYTRACER_CITYSIM_TRAVEL_LUA_H

// The ways of travelling from Lua (assets/scripts/travel.lua): global `travel` = { traits = { name, ... }, modes = {
// {name, network, speed_kmh, start_minutes, end_minutes, set_off_wait_minutes, own_vehicle, access, requires,
// longest_leg_minutes, begins_in, weight, colour}, ... }, near = {lo, hi}, far = {lo, hi} }.
// Refused (false, `err` says why) on a row without a name, an unknown network, a speed that is not positive, minutes
// below zero, an
// `access` that names no row (or a row that itself needs a vehicle), a `requires` that `traits` does not list, a way
// that needs a vehicle when nothing on the footpaths is open to everyone, or a band whose ends are out of order.

#include "../trip_plan.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadTravelCatalog(ScriptVM& vm, citysim::TravelCatalog& out, std::string* err);
}  // namespace engine

#endif
