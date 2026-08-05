#ifndef RAYTRACER_ENGINE_SCRIPTING_LUA_HELPERS_H
#define RAYTRACER_ENGINE_SCRIPTING_LUA_HELPERS_H

// Shared Lua argument/stack helpers for binding TUs. Internal to the scripting
// module like lua_state.h: only binding .cpp files include this (citysim's
// vehicle_body.cpp counts — it is a binding TU for the citysim surface).
//
// One definition instead of per-surface copies: these were duplicated across
// gameplay_bindings / procgen_bindings / vehicle_spec / vehicle_body, where
// the copies could — and did — start to drift (TECH_DEBT, "binding helpers
// are copy-pasted across surfaces").

#include "lua_state.h"
#include "../world.h"        // Entity
#include "../../rt_math.h"   // Vec3

#include <cstdint>
#include <string>

namespace engine {

// Entities cross to Lua as ONE packed integer (generation<<32 | index):
// opaque to scripts, round-trips exactly, and lets `alive` check the
// generation. Encode and decode live side by side on purpose — split across
// files they must match by hand, and a drift corrupts entity ids silently.
inline lua_Integer packEntity(Entity e) {
    return static_cast<lua_Integer>(
        (static_cast<uint64_t>(e.generation) << 32) | static_cast<uint64_t>(e.index));
}

inline Entity unpackEntity(lua_Integer packed) {
    auto bits = static_cast<uint64_t>(packed);
    Entity e;
    e.index = static_cast<uint32_t>(bits & 0xffffffffu);
    e.generation = static_cast<uint32_t>(bits >> 32);
    return e;
}

// A Vec3 argument is a 3-element Lua array, e.g. {x, y, z}. Errors (luaL) if
// the value at `idx` is not a table of numbers.
inline Vec3 checkVec3(lua_State* L, int idx) {
    idx = lua_absindex(L, idx);  // we push while reading; pin the table index
    luaL_checktype(L, idx, LUA_TTABLE);
    Vec3 v;
    lua_geti(L, idx, 1); v.x = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 2); v.y = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 3); v.z = luaL_checknumber(L, -1); lua_pop(L, 1);
    return v;
}

inline void pushVec3(lua_State* L, const Vec3& v) {
    lua_createtable(L, 3, 0);
    lua_pushnumber(L, v.x); lua_seti(L, -2, 1);
    lua_pushnumber(L, v.y); lua_seti(L, -2, 2);
    lua_pushnumber(L, v.z); lua_seti(L, -2, 3);
}

// Read an optional numeric field `key` from the table at `idx` (fallback when
// absent). Used to turn a Lua params table into a C++ params struct.
inline double optField(lua_State* L, int idx, const char* key, double fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    double v = luaL_optnumber(L, -1, fallback);
    lua_pop(L, 1);
    return v;
}

// Table-field readers over an ABSOLUTE stack index `t` (spec-table style:
// missing or mistyped fields keep the default rather than raising).
inline double numField(lua_State* L, int t, const char* key, double def) {
    lua_getfield(L, t, key);
    double v = lua_isnumber(L, -1) ? lua_tonumber(L, -1) : def;
    lua_pop(L, 1);
    return v;
}

inline bool boolField(lua_State* L, int t, const char* key, bool def) {
    lua_getfield(L, t, key);
    bool v = lua_isboolean(L, -1) ? (lua_toboolean(L, -1) != 0) : def;
    lua_pop(L, 1);
    return v;
}

// "" when absent.
inline std::string strField(lua_State* L, int t, const char* key) {
    lua_getfield(L, t, key);
    const char* s = lua_isstring(L, -1) ? lua_tostring(L, -1) : nullptr;
    std::string v = s ? s : "";
    lua_pop(L, 1);
    return v;
}

// A {x,y,z} array-field: missing components keep `def`.
inline Vec3 vec3Field(lua_State* L, int t, const char* key, Vec3 def) {
    lua_getfield(L, t, key);
    Vec3 r = def;
    if (lua_istable(L, -1)) {
        int st = lua_gettop(L);
        lua_rawgeti(L, st, 1);
        if (lua_isnumber(L, -1)) r.x = lua_tonumber(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, st, 2);
        if (lua_isnumber(L, -1)) r.y = lua_tonumber(L, -1);
        lua_pop(L, 1);
        lua_rawgeti(L, st, 3);
        if (lua_isnumber(L, -1)) r.z = lua_tonumber(L, -1);
        lua_pop(L, 1);
    }
    lua_pop(L, 1);
    return r;
}

}  // namespace engine

#endif
