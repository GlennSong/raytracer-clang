#ifndef RAYTRACER_CITYSIM_SIGNALS_LUA_H
#define RAYTRACER_CITYSIM_SIGNALS_LUA_H

// The signals' cycle times from Lua (assets/scripts/signals.lua): global `signals` = { green, yellow, all_red, walk,
// arrow, arrow_yellow }, seconds. A key left out keeps SignalTiming's default; a time that is not positive is an error.

#include "../traffic_signal.h"
#include <string>

namespace engine {
class ScriptVM;
bool loadSignalTiming(ScriptVM& vm, citysim::SignalTiming& out, std::string* err);
}  // namespace engine

#endif
