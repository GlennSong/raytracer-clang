#include "gameplay_bindings.h"

#include "lua_state.h"
#include "../world.h"
#include "../components.h"   // Transform
#include "../../rt_math.h"
#include "../../log.h"

#include <cstdint>

namespace engine {
namespace {

// Registry key under which the active World pointer lives during a tick.
constexpr const char* kWorldKey = "engine.gameplay.world";

World* activeWorld(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, kWorldKey);
    auto* w = static_cast<World*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return w;
}

// Entities cross to Lua as a packed integer (generation<<32 | index) — opaque to
// scripts, round-trips exactly, and lets `alive` check the generation.
Entity toEntity(lua_Integer packed) {
    auto bits = static_cast<uint64_t>(packed);
    Entity e;
    e.index = static_cast<uint32_t>(bits & 0xffffffffu);
    e.generation = static_cast<uint32_t>(bits >> 32);
    return e;
}

Vec3 checkVec3(lua_State* L, int idx) {
    idx = lua_absindex(L, idx);
    luaL_checktype(L, idx, LUA_TTABLE);
    Vec3 v;
    lua_geti(L, idx, 1); v.x = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 2); v.y = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 3); v.z = luaL_checknumber(L, -1); lua_pop(L, 1);
    return v;
}

void pushVec3(lua_State* L, const Vec3& v) {
    lua_createtable(L, 3, 0);
    lua_pushnumber(L, v.x); lua_seti(L, -2, 1);
    lua_pushnumber(L, v.y); lua_seti(L, -2, 2);
    lua_pushnumber(L, v.z); lua_seti(L, -2, 3);
}

// Resolve the entity arg to a live Transform*, or raise a Lua error.
Transform* checkTransform(lua_State* L, int idx) {
    World* world = activeWorld(L);
    if (world == nullptr) {
        luaL_error(L, "no active world (gameplay call outside a tick)");
        return nullptr;
    }
    Entity e = toEntity(luaL_checkinteger(L, idx));
    Transform* t = world->get<Transform>(e);
    if (t == nullptr) luaL_error(L, "entity has no Transform (or is dead)");
    return t;
}

int l_log(lua_State* L) {
    LOG_INFO << "[script] " << luaL_checkstring(L, 1);
    return 0;
}

int l_entity_alive(lua_State* L) {
    World* world = activeWorld(L);
    bool alive = world != nullptr && world->alive(toEntity(luaL_checkinteger(L, 1)));
    lua_pushboolean(L, alive);
    return 1;
}

int l_entity_get_position(lua_State* L) {
    pushVec3(L, checkTransform(L, 1)->position);
    return 1;
}

int l_entity_set_position(lua_State* L) {
    Transform* t = checkTransform(L, 1);
    t->position = checkVec3(L, 2);
    return 0;
}

int l_entity_translate(lua_State* L) {
    Transform* t = checkTransform(L, 1);
    t->position = t->position + checkVec3(L, 2);
    return 0;
}

int l_entity_set_yaw(lua_State* L) {
    Transform* t = checkTransform(L, 1);
    t->orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), luaL_checknumber(L, 2));
    return 0;
}

}  // namespace

void openGameplayLibrary(ScriptVM& vm) {
    lua_State* L = luaState(vm);

    lua_pushcfunction(L, l_log);
    lua_setglobal(L, "log");

    static const luaL_Reg kEntityFns[] = {
        {"alive", l_entity_alive},
        {"get_position", l_entity_get_position},
        {"set_position", l_entity_set_position},
        {"translate", l_entity_translate},
        {"set_yaw", l_entity_set_yaw},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kEntityFns);
    lua_setglobal(L, "entity");
}

void setActiveWorld(ScriptVM& vm, World* world) {
    lua_State* L = luaState(vm);
    if (world != nullptr) {
        lua_pushlightuserdata(L, world);
    } else {
        lua_pushnil(L);
    }
    lua_setfield(L, LUA_REGISTRYINDEX, kWorldKey);
}

}  // namespace engine
