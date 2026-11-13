#ifndef RAYTRACER_CITYSIM_TRAVEL_LUA_H
#define RAYTRACER_CITYSIM_TRAVEL_LUA_H

// The ways of travelling from Lua (assets/scripts/travel.lua): global `travel` = { modes = { {name, network,
// speed_kmh, start_minutes, end_minutes, own_vehicle, access, weight, colour}, ... }, near = {lo, hi}, far = {lo, hi} }.
// Refused (false, `err` says why) on a row without a name, an unknown network, a speed that is not positive, an
// `access` that names no row (or a row that itself needs a vehicle), or a band whose ends are out of order.

#include "../trip_plan.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadTravelCatalog(ScriptVM& vm, citysim::TravelCatalog& out, std::string* err);
}  // namespace engine

#endif
