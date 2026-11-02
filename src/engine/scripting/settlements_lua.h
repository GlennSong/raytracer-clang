#ifndef RAYTRACER_ENGINE_SCRIPTING_SETTLEMENTS_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_SETTLEMENTS_LUA_H

// The kinds of place on an island from Lua (assets/scripts/settlements.lua): global `settlements` = { order, <kind> =
// {...} }. See procgen/world/settlements.h for what each field means; a field left out keeps the struct's default (a
// town's), an unknown naming or a bad size is an error, not a guess.

#include "../procgen/world/settlements.h"
#include <string>

namespace engine {

class ScriptVM;
bool loadSettlementCatalog(ScriptVM& vm, SettlementCatalog& out, std::string* err = nullptr);
// The shipped catalog (assets/scripts/settlements.lua); defaultSettlements when it is missing or refused (`warn` says
// why).
SettlementCatalog shippedSettlements(std::string* warn = nullptr);

}  // namespace engine

#endif
