#include "carry_items.h"
#include "lua_state.h"     // luaState() + the Lua C API (scripting-internal)
#include "lua_helpers.h"   // numField / strField
#include "script_vm.h"

namespace engine {

bool loadCarryCatalogue(ScriptVM& vm, const std::string& code, CarryCatalogue& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    if (luaL_loadstring(L, code.c_str()) != LUA_OK || lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (err) *err = lua_tostring(L, -1) ? lua_tostring(L, -1) : "carry_items.lua did not run";
        lua_settop(L, base);
        return false;
    }
    if (!lua_istable(L, -1)) {
        if (err) *err = "carry_items.lua returns no table";
        lua_settop(L, base);
        return false;
    }
    const int t = lua_gettop(L);
    out.limit = numField(L, t, "limit", out.limit);
    out.items.clear();
    lua_getfield(L, t, "items");
    if (lua_istable(L, -1)) {
        const int it = lua_gettop(L);
        lua_pushnil(L);
        while (lua_next(L, it) != 0) {   // name = { size = {x,y,z}, mass =, color = {r,g,b} }
            if (lua_type(L, -2) == LUA_TSTRING && lua_istable(L, -1)) {
                const int e = lua_gettop(L);
                CarryItem c;
                c.name = lua_tostring(L, -2);
                c.mass = numField(L, e, "mass", c.mass);
                auto vec = [&](const char* key, Vec3& v) {
                    lua_getfield(L, e, key);
                    if (lua_istable(L, -1)) {
                        Real k[3] = {v.x, v.y, v.z};
                        for (int i = 0; i < 3; ++i) {
                            lua_rawgeti(L, -1, i + 1);
                            if (lua_isnumber(L, -1)) k[i] = lua_tonumber(L, -1);
                            lua_pop(L, 1);
                        }
                        v = Vec3(k[0], k[1], k[2]);
                    }
                    lua_pop(L, 1);
                };
                vec("size", c.size);
                vec("color", c.color);
                out.items.push_back(c);
            }
            lua_pop(L, 1);
        }
    }
    lua_settop(L, base);
    return true;
}

}  // namespace engine
