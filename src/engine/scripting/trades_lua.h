#ifndef RAYTRACER_ENGINE_SCRIPTING_TRADES_LUA_H
#define RAYTRACER_ENGINE_SCRIPTING_TRADES_LUA_H

// THE TRADES AS DATA (ADR-0360): assets/scripts/trades.lua -> TradeBook (procgen/city/trades.h).

#include "../procgen/city/trades.h"
#include <string>

namespace engine {

// Read `code` (it returns the table) into `out`. False, with `err`, when it does not read; `out` is then untouched.
bool loadTradesSource(const std::string& code, TradeBook& out, std::string* err);
// Read trades.lua once and make it the book in use; absent or broken, the built-in book stands (and it says so).
void ensureTradesLoaded();

}  // namespace engine

#endif
