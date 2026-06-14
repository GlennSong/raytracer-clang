#include "gameplay_bindings.h"

#include "lua_state.h"
#include "../world.h"
#include "../components.h"          // Transform
#include "../input/input_map.h"
#include "../../renderer/renderer.h"  // CameraState
#include "../../rt_math.h"
#include "../../log.h"

#include <cstdint>

namespace engine {
namespace {

// Registry key under which the active GameplayContext pointer lives during a tick.
constexpr const char* kCtxKey = "engine.gameplay.ctx";

GameplayContext* gameplayCtx(lua_State* L) {
    lua_getfield(L, LUA_REGISTRYINDEX, kCtxKey);
    auto* g = static_cast<GameplayContext*>(lua_touserdata(L, -1));
    lua_pop(L, 1);
    return g;
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

double optField(lua_State* L, int idx, const char* key, double fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    double v = luaL_optnumber(L, -1, fallback);
    lua_pop(L, 1);
    return v;
}

Vec3 fieldVec3(lua_State* L, int idx, const char* key) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    Vec3 v = checkVec3(L, -1);
    lua_pop(L, 1);
    return v;
}

Vec3 optFieldVec3(lua_State* L, int idx, const char* key, Vec3 fallback) {
    idx = lua_absindex(L, idx);
    lua_getfield(L, idx, key);
    Vec3 v = lua_isnoneornil(L, -1) ? fallback : checkVec3(L, -1);
    lua_pop(L, 1);
    return v;
}

World* requireWorld(lua_State* L) {
    GameplayContext* g = gameplayCtx(L);
    if (g == nullptr || g->world == nullptr) {
        luaL_error(L, "no active world (gameplay call outside a tick)");
    }
    return g->world;
}

Transform* checkTransform(lua_State* L, int idx) {
    World* world = requireWorld(L);
    Entity e = toEntity(luaL_checkinteger(L, idx));
    Transform* t = world->get<Transform>(e);
    if (t == nullptr) luaL_error(L, "entity has no Transform (or is dead)");
    return t;
}

// --- log / entity.* ---

int l_log(lua_State* L) {
    LOG_INFO << "[script] " << luaL_checkstring(L, 1);
    return 0;
}

int l_entity_alive(lua_State* L) {
    GameplayContext* g = gameplayCtx(L);
    bool alive = g != nullptr && g->world != nullptr &&
                 g->world->alive(toEntity(luaL_checkinteger(L, 1)));
    lua_pushboolean(L, alive);
    return 1;
}
int l_entity_get_position(lua_State* L) {
    pushVec3(L, checkTransform(L, 1)->position);
    return 1;
}
int l_entity_set_position(lua_State* L) {
    checkTransform(L, 1)->position = checkVec3(L, 2);
    return 0;
}
int l_entity_translate(lua_State* L) {
    Transform* t = checkTransform(L, 1);
    t->position = t->position + checkVec3(L, 2);
    return 0;
}
int l_entity_set_yaw(lua_State* L) {
    checkTransform(L, 1)->orientation =
        Quat::fromAxisAngle(Vec3(0, 1, 0), luaL_checknumber(L, 2));
    return 0;
}

// --- input.* ---

InputMap* requireInput(lua_State* L) {
    GameplayContext* g = gameplayCtx(L);
    if (g == nullptr || g->input == nullptr) luaL_error(L, "input unavailable");
    return g->input;
}
int l_input_pressed(lua_State* L) {
    lua_pushboolean(L, requireInput(L)->pressed(luaL_checkstring(L, 1)));
    return 1;
}
int l_input_held(lua_State* L) {
    lua_pushboolean(L, requireInput(L)->held(luaL_checkstring(L, 1)));
    return 1;
}
int l_input_axis(lua_State* L) {
    lua_pushnumber(L, requireInput(L)->axis(luaL_checkstring(L, 1)));
    return 1;
}

// --- camera.* (the active view; aim source for the gun) ---

const CameraState& requireCamera(lua_State* L) {
    GameplayContext* g = gameplayCtx(L);
    if (g == nullptr || g->camera == nullptr) luaL_error(L, "camera unavailable");
    return *g->camera;
}
int l_camera_eye(lua_State* L) {
    pushVec3(L, requireCamera(L).position);
    return 1;
}
int l_camera_forward(lua_State* L) {
    const CameraState& c = requireCamera(L);
    pushVec3(L, normalize(c.target - c.position));
    return 1;
}
int l_camera_right(lua_State* L) {
    const CameraState& c = requireCamera(L);
    Vec3 fwd = normalize(c.target - c.position);
    pushVec3(L, normalize(cross(fwd, c.up)));
    return 1;
}

// --- spawn.* (deferred; applied by ScriptSystem after iteration) ---

int l_spawn_block(lua_State* L) {
    GameplayContext* g = gameplayCtx(L);
    if (g == nullptr || g->spawns == nullptr) {
        return luaL_error(L, "spawn unavailable outside a gameplay tick");
    }
    luaL_checktype(L, 1, LUA_TTABLE);
    SpawnCommand c;
    c.position = fieldVec3(L, 1, "position");
    c.velocity = fieldVec3(L, 1, "velocity");
    c.size = optField(L, 1, "size", c.size);
    c.restitution = optField(L, 1, "restitution", c.restitution);
    c.friction = optField(L, 1, "friction", c.friction);
    c.color = optFieldVec3(L, 1, "color", c.color);
    c.emission = optFieldVec3(L, 1, "emission", c.emission);
    g->spawns->push_back(c);
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

    static const luaL_Reg kInputFns[] = {
        {"pressed", l_input_pressed},
        {"held", l_input_held},
        {"axis", l_input_axis},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kInputFns);
    lua_setglobal(L, "input");

    static const luaL_Reg kCameraFns[] = {
        {"eye", l_camera_eye},
        {"forward", l_camera_forward},
        {"right", l_camera_right},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kCameraFns);
    lua_setglobal(L, "camera");

    static const luaL_Reg kSpawnFns[] = {
        {"block", l_spawn_block},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kSpawnFns);
    lua_setglobal(L, "spawn");
}

void setGameplayContext(ScriptVM& vm, GameplayContext* ctx) {
    lua_State* L = luaState(vm);
    if (ctx != nullptr) {
        lua_pushlightuserdata(L, ctx);
    } else {
        lua_pushnil(L);
    }
    lua_setfield(L, LUA_REGISTRYINDEX, kCtxKey);
}

}  // namespace engine
