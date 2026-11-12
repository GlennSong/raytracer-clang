#ifndef RAYTRACER_APPS_DRIVING_SCRIPTING_RULE_SET_LUA_H
#define RAYTRACER_APPS_DRIVING_SCRIPTING_RULE_SET_LUA_H

// READS A RULE SET OUT OF LUA (driving-model-design.md 6.1 a, c, d and 6.4; issue #250). The caller has already run the
// rule-set file in `vm`, which defined a global table `rule_set`. This reads the parts C3 owns (name, drives_on,
// ranks, devices, numbers, personality, and the optional default_limits, written like the numbers) into plain structs
// and checks them. It also reads the optional right-of-way tables (issue #260): `control`, each kind of junction
// control as { settle = { question, ... }, everyone_stops = bool, gives_way_on_green = { movement, ... },
// arrow = { movement, ... }, turn_on_red = { movement = ..., after = "full_stop" } }, and `movement_rank`, a list of
// movements, earlier goes first. Other top-level keys are left for the later
// readers and are not errors; unknown keys inside the parts read here ARE, so a typo is caught.
//
// A value in a Lua table carries no line number, so an error names the file and the full key path and says what was
// wrong and what was expected. The reader collects up to kMaxRuleSetErrors and reports them all, one per line. On any
// error `out` is left untouched. There is no built-in rule set and no fallback: a missing `rule_set` is an error.
//
// This is the only driving file allowed to include Lua; the rest of the library never sees it.

#include "apps/driving/rule_set.h"

#include <string>

namespace engine {
class ScriptVM;
}

namespace driving {

// The most faults one load reports (a file that bad needs fixing from the top anyway).
constexpr std::size_t kMaxRuleSetErrors = 20;

// `fileName` (for example "us_left.lua") is only used to name the file in messages.
bool loadRuleSet(engine::ScriptVM& vm, const std::string& fileName, RuleSet& out, std::string* err);

}  // namespace driving

#endif
