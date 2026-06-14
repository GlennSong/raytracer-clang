#include "script_system.h"

#include "lua_state.h"
#include "gameplay_bindings.h"
#include "script_behaviour.h"
#include "../world.h"
#include "../../log.h"

#include <cstdint>

namespace engine {
namespace {

// Pack an Entity to the integer scripts see (must match gameplay_bindings'
// toEntity: generation<<32 | index).
lua_Integer packEntity(Entity e) {
    return static_cast<lua_Integer>(
        (static_cast<uint64_t>(e.generation) << 32) | static_cast<uint64_t>(e.index));
}

// Load a behaviour's chunk, which must return a table (the per-entity instance);
// store it in the registry and record the ref. Returns false (logging) on error.
bool loadInstance(lua_State* L, ScriptBehaviour& sb) {
    if (luaL_loadstring(L, sb.source.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        LOG_ERROR << "[script] behaviour load error: " << lua_tostring(L, -1);
        lua_pop(L, 1);
        return false;
    }
    if (!lua_istable(L, -1)) {
        LOG_ERROR << "[script] behaviour must `return` a table of hooks";
        lua_pop(L, 1);
        return false;
    }
    sb.instanceRef = luaL_ref(L, LUA_REGISTRYINDEX);   // pops the table
    return true;
}

// Call instance:hook(e[, dt]) if it exists. The instance table must be at the
// top of the stack on entry, and is left there on return.
void callHook(lua_State* L, const char* hook, Entity e, double dt, bool hasDt) {
    lua_getfield(L, -1, hook);                 // [inst, fn?]
    if (!lua_isfunction(L, -1)) {
        lua_pop(L, 1);                         // [inst]
        return;
    }
    lua_pushvalue(L, -2);                       // self = inst -> [inst, fn, inst]
    lua_pushinteger(L, packEntity(e));         // [inst, fn, inst, e]
    int nargs = 2;
    if (hasDt) {
        lua_pushnumber(L, dt);                  // [inst, fn, inst, e, dt]
        nargs = 3;
    }
    if (lua_pcall(L, nargs, 0, 0) != LUA_OK) {  // [inst] or [inst, errmsg]
        LOG_ERROR << "[script] " << hook << "() error: " << lua_tostring(L, -1);
        lua_pop(L, 1);                          // [inst]
    }
}

}  // namespace

ScriptSystem::ScriptSystem() {
    openGameplayLibrary(vm_);
}

void ScriptSystem::update(FrameContext& ctx) {
    tick(ctx.world, ctx.frameDelta);
}

void ScriptSystem::tick(World& world, double dt) {
    lua_State* L = luaState(vm_);
    setActiveWorld(vm_, &world);

    // Mutating ScriptBehaviour's own fields (refs/flags) is allowed inside each;
    // scripts only touch Transform, so the no-structural-mutation contract holds.
    world.each<ScriptBehaviour>([&](Entity e, ScriptBehaviour& sb) {
        if (sb.failed) return;
        if (sb.instanceRef < 0 && !loadInstance(L, sb)) {
            sb.failed = true;
            return;
        }

        lua_rawgeti(L, LUA_REGISTRYINDEX, sb.instanceRef);   // [inst]
        if (!sb.started) {
            callHook(L, "start", e, dt, /*hasDt=*/false);
            sb.started = true;
        }
        callHook(L, "update", e, dt, /*hasDt=*/true);
        lua_pop(L, 1);                                        // pop inst
    });

    setActiveWorld(vm_, nullptr);
}

}  // namespace engine
