#ifndef RAYTRACER_APPS_DRIVING_SCRIPTING_RULES_LUA_H
#define RAYTRACER_APPS_DRIVING_SCRIPTING_RULES_LUA_H

// READS AND COMPILES A RULE SET'S `rules` (driving-model-design.md 6.1 e, 6.4; issue #252). The caller has already run
// the rule-set file in `vm` and read its numbers and personality with loadRuleSet; this reads `rule_set.rules`, a list
// kept in written order:
//
//   rules = {
//     { name = "lane ends", when = "lane_ends_ahead", give = { stop_at = "lane_end" } },
//     { name = "tailgated horn",
//       when = { all = { "car_ahead", { "gap_ahead", "<", "horn_gap" }, { "not", "standing" } } },
//       unless = { "closing_speed", "<", "no_speed" }, give = { action = "horn" } },
//   }
//
// A condition is a fact's name; { "not", condition }; { all = { ... } } or { any = { ... } }; or a comparison
// { fact, op, value } with op one of < <= > >= == ~= (number and place facts) or is / is_not (word facts). The value is
// the name of a rule-set number, a personal setting or another number fact, or (for is) one of the word fact's words.
// Both sides must measure the same thing. A number written out has no unit, so it is allowed only against a fact that
// measures a plain ratio; anything else names a rule-set number, which carries its unit.
//
// A give holds stop_at = <place fact>, top_speed = <a speed: fact, number or setting>, action = <name or list>, and
// <intention kind> = <word> with an optional urgency = <a ratio: written out or named>.
//
// A fact, a rule-set number and a personal setting may not share a name (that is checked here too), so a name is
// never ambiguous.
//
// Every fault is collected (up to kMaxRulesErrors) and reported together, each naming the file and the full key path
// (`us_left.lua: rule_set.rules[4] ("tailgated horn").when.all[2] ...`). On any fault `out` is left untouched.
//
// Lua only here, under scripting/; decide.h never sees it.

#include "apps/driving/decide.h"
#include "apps/driving/notice.h"
#include "apps/driving/rule_set.h"

#include <cstddef>
#include <string>

namespace engine {
class ScriptVM;
}

namespace driving {

// The most faults one load reports.
constexpr std::size_t kMaxRulesErrors = 20;

// `fileName` (for example "us_left.lua") is only used to name the file in messages. `ruleSet` is the one loadRuleSet
// read from the same file: its numbers are turned into values now, and its personal settings are found by place.
bool loadRules(engine::ScriptVM& vm, const std::string& fileName, const FactRegistry& facts, const RuleSet& ruleSet,
               const AnswerVocabulary& vocabulary, CompiledRules& out, std::string* err);

}  // namespace driving

#endif
