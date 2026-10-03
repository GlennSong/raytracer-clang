#include "furniture_library_lua.h"
#include "lua_state.h"
#include "lua_helpers.h"
#include "script_vm.h"
#include "../script_assets.h"
#include "../../log.h"

#include <mutex>

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

}  // namespace

bool readFurnitureLibrary(ScriptVM& vm, FurnitureLibrary& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    lua_getglobal(L, "furniture_library");
    if (!lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "no global `furniture_library`"); }
    const int lib = lua_gettop(L);
    out.clear();
    lua_pushnil(L);
    while (lua_next(L, lib) != 0) {
        // key at -2, entry at -1
        if (!lua_isstring(L, -2) || !lua_istable(L, -1)) { lua_settop(L, base); return fail(err, "an entry is not name = {...}"); }
        const std::string name = lua_tostring(L, -2);
        const int t = lua_gettop(L);
        FurnitureAsset a;
        if (!furniturePieceByName(name, a.piece)) { lua_settop(L, base); return fail(err, "unknown piece '" + name + "'"); }
        a.family = strField(L, t, "family");
        lua_getfield(L, t, "tags");
        if (lua_istable(L, -1)) {
            const int n = static_cast<int>(luaL_len(L, -1));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, -1, i);
                if (lua_isstring(L, -1)) a.tags.push_back(lua_tostring(L, -1));
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "spots");
        if (lua_istable(L, -1)) {
            const int st = lua_gettop(L), n = static_cast<int>(luaL_len(L, st));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, st, i);
                if (lua_istable(L, -1)) {
                    FurnSpot s;
                    s.id = strField(L, lua_gettop(L), "id");
                    s.at = vec3Field(L, lua_gettop(L), "at", Vec3(0, 0, 0));
                    a.spots.push_back(s);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        if (a.spots.size() > 32) { lua_settop(L, base); return fail(err, name + ": more than 32 spots"); }
        lua_getfield(L, t, "clearance");   // {front = m, side = m}
        if (lua_istable(L, -1)) {
            a.clearFront = numField(L, lua_gettop(L), "front", 0.0);
            a.clearSide = numField(L, lua_gettop(L), "side", 0.0);
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "anchors");
        if (lua_istable(L, -1)) {
            const int at = lua_gettop(L), n = static_cast<int>(luaL_len(L, at));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, at, i);
                if (lua_istable(L, -1)) {
                    const int e = lua_gettop(L);
                    FurnAnchor an;
                    an.id = strField(L, e, "id");
                    an.at = vec3Field(L, e, "at", an.at);
                    an.w = numField(L, e, "w", an.w);
                    an.d = numField(L, e, "d", an.d);
                    an.chance = numField(L, e, "chance", 1.0);
                    an.yaw = numField(L, e, "yaw", 0.0);
                    lua_getfield(L, e, "accepts");
                    if (lua_istable(L, -1)) {
                        const int k = static_cast<int>(luaL_len(L, -1));
                        for (int j = 1; j <= k; ++j) {
                            lua_rawgeti(L, -1, j);
                            if (lua_isstring(L, -1)) an.accepts.push_back(lua_tostring(L, -1));
                            lua_pop(L, 1);
                        }
                    }
                    lua_pop(L, 1);
                    if (an.accepts.empty()) { lua_settop(L, base); return fail(err, name + ": anchor '" + an.id + "' accepts nothing"); }
                    a.anchors.push_back(an);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        lua_getfield(L, t, "verbs");
        if (lua_istable(L, -1)) {
            const int vt = lua_gettop(L), n = static_cast<int>(luaL_len(L, vt));
            for (int i = 1; i <= n; ++i) {
                lua_rawgeti(L, vt, i);
                if (lua_istable(L, -1)) {
                    const int e = lua_gettop(L);
                    FurnVerb v;
                    const std::string vn = strField(L, e, "verb");
                    if (!verbByName(vn, v.verb)) { lua_settop(L, base); return fail(err, name + ": unknown verb '" + vn + "'"); }
                    v.label = strField(L, e, "label");
                    v.eye = vec3Field(L, e, "eye", v.eye);
                    v.look = vec3Field(L, e, "look", v.look);
                    v.exit = vec3Field(L, e, "exit", v.exit);
                    lua_getfield(L, e, "spots");
                    if (lua_istable(L, -1)) {
                        const int sn = static_cast<int>(luaL_len(L, -1));
                        for (int k = 1; k <= sn; ++k) {
                            lua_rawgeti(L, -1, k);
                            const std::string sid = lua_isstring(L, -1) ? lua_tostring(L, -1) : "";
                            lua_pop(L, 1);
                            bool found = false;
                            for (std::size_t j = 0; j < a.spots.size(); ++j)
                                if (a.spots[j].id == sid) { v.spots |= 1u << j; found = true; }
                            if (!found) { lua_settop(L, base); return fail(err, name + ": verb uses unknown spot '" + sid + "'"); }
                        }
                    }
                    lua_pop(L, 1);
                    if (v.spots == 0) { lua_settop(L, base); return fail(err, name + ": a verb with no spots"); }
                    a.verbs.push_back(v);
                }
                lua_pop(L, 1);
            }
        }
        lua_pop(L, 1);
        out.set(std::move(a));
        lua_pop(L, 1);   // the entry; the key stays for lua_next
    }
    lua_settop(L, base);
    return true;
}

bool loadFurnitureLibrarySource(const std::string& source, FurnitureLibrary& out, std::string* err) {
    ScriptVM vm;
    std::string e;
    if (!vm.doString(source, &e)) return fail(err, "furniture_library.lua: " + e);
    return readFurnitureLibrary(vm, out, err);
}

bool ensureFurnitureLibraryLoaded() {
    static std::once_flag once;
    static bool ok = false;
    std::call_once(once, [] {
        const std::string src = loadScriptCode("furniture_library.lua", "");
        if (src.empty()) { LOG_WARN << "[furniture] no furniture_library.lua: nothing is interactive"; return; }
        std::string err;
        FurnitureLibrary lib;
        if (!loadFurnitureLibrarySource(src, lib, &err)) { LOG_WARN << "[furniture] " << err; return; }
        FurnitureLibrary::global() = lib;
        ok = true;
        LOG_INFO << "[furniture] library: " << lib.size() << " described pieces";
    });
    return ok;
}

}  // namespace engine
