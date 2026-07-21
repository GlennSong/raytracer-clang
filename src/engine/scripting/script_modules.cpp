#include "script_modules.h"

#include "lua_state.h"

#include <new>
#include <string>

namespace engine {

namespace {

const char* const kSourceMeta = "engine.ScriptModuleSource";

// Upvalue slots on the `require` closure.
constexpr int kUpSource = 1;   // full userdata wrapping the ScriptModuleSource
constexpr int kUpCache = 2;    // table: name -> module value

// Marker stored in the cache while a module is mid-load, so a cycle is an error
// instead of unbounded recursion.
int kLoadingTag = 0;

// Identifiers only. This is the confinement: reject anything that could name a
// location rather than a module. Checked BEFORE the source callback runs, so a
// hostile name never reaches the filesystem layer at all.
bool validModuleName(const std::string& name) {
    if (name.empty() || name.size() > 64) return false;
    for (const char c : name) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '_';
        if (!ok) return false;
    }
    return true;
}

int sourceGc(lua_State* L) {
    auto* fn = static_cast<ScriptModuleSource*>(
        luaL_checkudata(L, 1, kSourceMeta));
    fn->~ScriptModuleSource();
    return 0;
}

int luaRequire(lua_State* L) {
    const char* raw = luaL_checkstring(L, 1);
    const std::string name = raw != nullptr ? raw : "";
    if (!validModuleName(name))
        return luaL_error(L, "require: '%s' is not a module name (letters, "
                             "digits and underscore only — modules are named, "
                             "not paths)", name.c_str());

    // Cached? A cycle is the kLoading sentinel still sitting there.
    lua_pushvalue(L, lua_upvalueindex(kUpCache));
    lua_getfield(L, -1, name.c_str());
    if (lua_touserdata(L, -1) == &kLoadingTag)
        return luaL_error(L, "require: cyclic dependency on '%s'", name.c_str());
    if (!lua_isnil(L, -1)) {
        lua_remove(L, -2);   // drop the cache table, leave the value
        return 1;
    }
    lua_pop(L, 1);           // the nil; cache table stays at -1

    // Mark in-progress before running the body, so a cycle is caught.
    lua_pushlightuserdata(L, &kLoadingTag);
    lua_setfield(L, -2, name.c_str());

    auto* source = static_cast<ScriptModuleSource*>(
        lua_touserdata(L, lua_upvalueindex(kUpSource)));
    std::string code, path;
    if (source == nullptr || !(*source)(name, code, path)) {
        lua_pushnil(L);      // clear the marker so a later retry can work
        lua_setfield(L, -2, name.c_str());
        return luaL_error(L, "require: module '%s' not found", name.c_str());
    }

    // Chunkname "@<name>.lua" so tracebacks name the module, not "[string ...]".
    const std::string chunk = "@" + name + ".lua";
    if (luaL_loadbuffer(L, code.c_str(), code.size(), chunk.c_str()) != LUA_OK)
        return lua_error(L);
    lua_call(L, 0, 1);

    // A module that returns nothing still counts as loaded (Lua's convention),
    // otherwise a side-effect-only module re-runs on every require.
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        lua_pushboolean(L, 1);
    }

    lua_pushvalue(L, -1);
    lua_setfield(L, -3, name.c_str());   // cache[name] = value
    lua_remove(L, -2);                   // drop the cache table
    return 1;
}

}  // namespace

void openModuleLoader(ScriptVM& vm, ScriptModuleSource source) {
    lua_State* L = luaState(vm);

    // Full userdata, not light: the callback owns captured state (a levelDir
    // string, a path-recording vector) that must outlive this call and be
    // destroyed with the VM.
    void* mem = lua_newuserdatauv(L, sizeof(ScriptModuleSource), 0);
    new (mem) ScriptModuleSource(std::move(source));
    if (luaL_newmetatable(L, kSourceMeta)) {
        lua_pushcfunction(L, sourceGc);
        lua_setfield(L, -2, "__gc");
    }
    lua_setmetatable(L, -2);

    lua_newtable(L);                       // the per-VM module cache
    lua_pushcclosure(L, luaRequire, 2);
    lua_setglobal(L, "require");
}

}  // namespace engine
