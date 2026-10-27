#include "lua_json.h"

#include <algorithm>
#include <string>

namespace engine {
namespace {

// Keys 1..n and nothing else: an array of n (0 for an empty table, which reads as an object).
lua_Integer sequenceLength(lua_State* L, int t) {
    lua_Integer count = 0, maxKey = 0;
    lua_pushnil(L);
    while (lua_next(L, t) != 0) {
        lua_pop(L, 1);   // the value; the key stays for lua_next
        if (!lua_isinteger(L, -1) || lua_tointeger(L, -1) < 1) {
            lua_pop(L, 1);
            return -1;
        }
        maxKey = std::max(maxKey, lua_tointeger(L, -1));
        ++count;
    }
    return count == maxKey ? count : -1;
}

nlohmann::json toJson(lua_State* L, int idx, int depth) {
    switch (lua_type(L, idx)) {
    case LUA_TBOOLEAN: return lua_toboolean(L, idx) != 0;
    case LUA_TNUMBER:
        if (lua_isinteger(L, idx)) return static_cast<long long>(lua_tointeger(L, idx));
        return lua_tonumber(L, idx);
    case LUA_TSTRING: return std::string(lua_tostring(L, idx));
    case LUA_TTABLE: {
        if (depth > 32) return nullptr;   // a table that holds itself
        const int t = lua_absindex(L, idx);
        const lua_Integer n = sequenceLength(L, t);
        if (n > 0) {
            nlohmann::json a = nlohmann::json::array();
            for (lua_Integer i = 1; i <= n; ++i) {
                lua_geti(L, t, i);
                a.push_back(toJson(L, -1, depth + 1));
                lua_pop(L, 1);
            }
            return a;
        }
        nlohmann::json o = nlohmann::json::object();
        lua_pushnil(L);
        while (lua_next(L, t) != 0) {
            // Copy the key before reading it as text: lua_tostring on a number key would change it in place.
            lua_pushvalue(L, -2);
            const char* key = lua_tostring(L, -1);
            if (key != nullptr) o[key] = toJson(L, -2, depth + 1);
            lua_pop(L, 2);
        }
        return o;
    }
    default: return nullptr;
    }
}

}  // namespace

nlohmann::json luaToJson(lua_State* L, int idx) {
    return toJson(L, idx, 0);
}

void pushJson(lua_State* L, const nlohmann::json& value) {
    switch (value.type()) {
    case nlohmann::json::value_t::boolean: lua_pushboolean(L, value.get<bool>()); return;
    case nlohmann::json::value_t::number_integer:
    case nlohmann::json::value_t::number_unsigned: lua_pushinteger(L, value.get<lua_Integer>()); return;
    case nlohmann::json::value_t::number_float: lua_pushnumber(L, value.get<double>()); return;
    case nlohmann::json::value_t::string: lua_pushstring(L, value.get<std::string>().c_str()); return;
    case nlohmann::json::value_t::array: {
        lua_createtable(L, static_cast<int>(value.size()), 0);
        lua_Integer i = 1;
        for (const nlohmann::json& v : value) {
            pushJson(L, v);
            lua_seti(L, -2, i++);
        }
        return;
    }
    case nlohmann::json::value_t::object: {
        lua_createtable(L, 0, static_cast<int>(value.size()));
        for (const auto& [k, v] : value.items()) {
            pushJson(L, v);
            lua_setfield(L, -2, k.c_str());
        }
        return;
    }
    default: lua_pushnil(L); return;
    }
}

}  // namespace engine
