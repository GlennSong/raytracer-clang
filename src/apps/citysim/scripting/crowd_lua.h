#ifndef RAYTRACER_CITYSIM_CROWD_LUA_H
#define RAYTRACER_CITYSIM_CROWD_LUA_H

// THE CROWD A LEVEL MAKES from Lua (assets/scripts/crowd.lua, or the file a level names; city_crowd.h, ADR-0446):
// global `crowd` = { residents, intent, keep_from_ends, places = { {kind, count, homes}, ... },
// household_sizes = { {size, share}, ... }, cars = { {cars, share}, ... }, traits = { {name, share}, ... },
// days = { {share, stops = { {kind | home = true, minutes = {lo, hi}}, ... }}, ... } }.
// Refused (false, `err` says why, naming the key) on a missing or empty list, a count or share below zero, a place row
// with no kind, a stop with neither a kind nor `home`, or a minutes band out of order.

#include "../city_crowd.h"

#include <string>

namespace engine {
class ScriptVM;
bool loadCrowdSpec(ScriptVM& vm, const std::string& file, citysim::CrowdSpec& out, std::string* err);
}  // namespace engine

#endif
