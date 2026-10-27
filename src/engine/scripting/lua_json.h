#ifndef RAYTRACER_ENGINE_SCRIPTING_LUA_JSON_H
#define RAYTRACER_ENGINE_SCRIPTING_LUA_JSON_H

// Lua values <-> JSON, for the named events (ADR-0253): events.publish reads a Lua table as an event's fields,
// events.on hands one over as a table. Internal to the scripting module like lua_helpers.h.
//
// A Lua table with keys 1..n (and nothing else) is a JSON array, any other table an object; integers stay
// integers. Functions, userdata and threads have no JSON form and become null.

#include "lua_state.h"

#include <nlohmann/json.hpp>

namespace engine {

nlohmann::json luaToJson(lua_State* L, int idx);
void pushJson(lua_State* L, const nlohmann::json& value);

}  // namespace engine

#endif
