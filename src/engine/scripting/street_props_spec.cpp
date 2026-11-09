#include "street_props_spec.h"

#include "lua_state.h"      // luaState() + the Lua C API (scripting-internal)
#include "lua_helpers.h"
#include "procgen_bindings.h"
#include "procgen_mesh.h"   // luaToMesh
#include "script_modules.h"
#include "script_vm.h"
#include "../script_assets.h"

#include <tuple>

namespace engine {

namespace {

StreetPropCollider readCollider(lua_State* L, int t) {
    StreetPropCollider c;
    lua_getfield(L, t, "collider");
    if (lua_istable(L, -1)) c.half = vec3Field(L, lua_gettop(L), "half", c.half);
    lua_pop(L, 1);
    c.breaksAt = numField(L, t, "breaks_at", 0.0);
    c.mass = numField(L, t, "mass", c.mass);
    return c;
}

}  // namespace

bool loadStreetProps(const std::string& levelDir, StreetPropsSpec& out, std::string* err) {
    const std::string src = loadScriptCode("street_props.lua", levelDir);
    if (src.empty()) {
        if (err) *err = "street_props.lua not found";
        return false;
    }
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, makeModuleSource(levelDir));
    if (!vm.doString("__street_props = (function() " + src + "\nend)()", err)) return false;
    lua_State* L = luaState(vm);
    lua_getglobal(L, "__street_props");
    const int t = lua_gettop(L);
    if (!lua_istable(L, t)) {
        lua_pop(L, 1);
        if (err) *err = "street_props.lua returned no table";
        return false;
    }
    const int n = static_cast<int>(luaL_len(L, t));
    for (int i = 1; i <= n; ++i) {
        lua_rawgeti(L, t, i);
        const int k = lua_gettop(L);
        if (lua_istable(L, k)) {
            StreetPropKind kind;
            kind.name = strField(L, k, "name");
            kind.every = numField(L, k, "every", kind.every);
            kind.verge = numField(L, k, "verge", kind.verge);
            kind.junction = numField(L, k, "junction", kind.junction);
            kind.mainOnly = boolField(L, k, "main_only", false);
            kind.collider = readCollider(L, k);
            lua_getfield(L, k, "mesh");
            if (lua_isfunction(L, -1)) {
                if (lua_pcall(L, 0, 1, 0) == LUA_OK) kind.mesh = luaToMesh(L, -1);
                else if (err) *err = std::string("street prop ") + kind.name + ": " + lua_tostring(L, -1);
            }
            lua_pop(L, 1);
            lua_getfield(L, k, "spray");
            if (lua_istable(L, -1)) {
                const int sp = lua_gettop(L);
                kind.sprayEffect = strField(L, sp, "effect");
                kind.sprayRate = numField(L, sp, "rate", 0.0);
                kind.sprayUp = numField(L, sp, "up", 0.0);
            }
            lua_pop(L, 1);
            if (kind.mesh) out.kinds.push_back(std::move(kind));
        }
        lua_pop(L, 1);
    }
    lua_getfield(L, t, "legacy");
    if (lua_istable(L, -1)) {
        const int lg = lua_gettop(L);
        for (const char* name : {"bin", "bike_rack", "news_boxes"}) {
            lua_getfield(L, lg, name);
            if (lua_istable(L, -1)) out.legacy[name] = readCollider(L, lua_gettop(L));
            lua_pop(L, 1);
        }
    }
    lua_pop(L, 1);
    for (const auto& [key, rr, hh] : {std::tuple<const char*, Real*, Real*>{"tree_trunk", &out.trunkRadius, &out.trunkHeight},
                                      std::tuple<const char*, Real*, Real*>{"lamp_post", &out.lampRadius, &out.lampHeight}}) {
        lua_getfield(L, t, key);
        if (lua_istable(L, -1)) {
            *rr = numField(L, lua_gettop(L), "radius", 0.0);
            *hh = numField(L, lua_gettop(L), "height", 0.0);
        }
        lua_pop(L, 1);
    }
    lua_getfield(L, t, "knock");
    if (lua_istable(L, -1)) {
        const int k = lua_gettop(L);
        out.knockPush = numField(L, k, "push", out.knockPush);
        out.knockUp = numField(L, k, "up", out.knockUp);
        out.knockUpPerSpeed = numField(L, k, "up_per_speed", out.knockUpPerSpeed);
    }
    lua_pop(L, 1);
    lua_pop(L, 1);
    return true;
}

}  // namespace engine
