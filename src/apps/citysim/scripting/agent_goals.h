#ifndef RAYTRACER_APPS_CITYSIM_SCRIPTING_AGENT_GOALS_H
#define RAYTRACER_APPS_CITYSIM_SCRIPTING_AGENT_GOALS_H

#include "../city_goals.h"

#include <string>

namespace engine {

class ScriptVM;

// The agents.lua reader (ADR-0064) — mirrors vehicle_spec's pattern: Lua
// defines DATA at load, C++ consumes it. Run assets/scripts/agents.lua in `vm`
// first (vm.doString) so the global `agents` table exists, then read one
// archetype's goal table — `agents[archetype]` with `entry`, `states` (name +
// action/target/activity/dwell) and `transitions` (from/event/to) — into a
// citysim::GoalTable ready for CitySim::setGoalTables. Load-time only by
// design: the returned table is plain data and every per-tick transition runs
// in C++ (no Lua in the tick).
//
// Returns false (with `err` filled, if non-null) on a missing archetype, an
// unknown state/event/action name, or a malformed table — a broken script must
// fail loudly at load, not misbehave silently at tick time.
bool loadGoalTable(ScriptVM& vm, const std::string& archetype,
                   citysim::GoalTable& out, std::string* err = nullptr);

}  // namespace engine

#endif
