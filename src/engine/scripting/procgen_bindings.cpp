#include "procgen_bindings.h"

#include "lua_state.h"
#include "../procgen/sdf.h"
#include "../procgen/noise.h"
#include "../../renderer/renderer.h"   // RenderMesh
#include "../../rt_math.h"

#include <memory>
#include <new>
#include <utility>

namespace engine {
namespace {

// Wrapped C++ values cross the C boundary as full userdata, each with a
// metatable whose __gc destroys the held object. The names are unique registry
// keys (luaL_newmetatable / luaL_checkudata).
constexpr const char* kSdfMt = "engine.procgen.Sdf";
constexpr const char* kMeshMt = "engine.procgen.Mesh";

using MeshPtr = std::shared_ptr<RenderMesh>;

// --- Sdf userdata (a Field) ---

void pushSdf(lua_State* L, Sdf sdf) {
    void* mem = lua_newuserdatauv(L, sizeof(Sdf), 0);
    new (mem) Sdf(std::move(sdf));
    luaL_setmetatable(L, kSdfMt);
}

Sdf& checkSdf(lua_State* L, int idx) {
    return *static_cast<Sdf*>(luaL_checkudata(L, idx, kSdfMt));
}

int sdfGc(lua_State* L) {
    static_cast<Sdf*>(lua_touserdata(L, 1))->~Sdf();
    return 0;
}

// --- Mesh userdata ---

void pushMesh(lua_State* L, MeshPtr mesh) {
    void* mem = lua_newuserdatauv(L, sizeof(MeshPtr), 0);
    new (mem) MeshPtr(std::move(mesh));
    luaL_setmetatable(L, kMeshMt);
}

int meshGc(lua_State* L) {
    static_cast<MeshPtr*>(lua_touserdata(L, 1))->~MeshPtr();
    return 0;
}

// --- argument helpers ---

// A Vec3 is a 3-element Lua array, e.g. {x, y, z}.
Vec3 checkVec3(lua_State* L, int idx) {
    idx = lua_absindex(L, idx);  // we push while reading; pin the table index
    luaL_checktype(L, idx, LUA_TTABLE);
    Vec3 v;
    lua_geti(L, idx, 1); v.x = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 2); v.y = luaL_checknumber(L, -1); lua_pop(L, 1);
    lua_geti(L, idx, 3); v.z = luaL_checknumber(L, -1); lua_pop(L, 1);
    return v;
}

// --- sdf.* ---

int l_sdf_sphere(lua_State* L) {
    Vec3 c = checkVec3(L, 1);
    double r = luaL_checknumber(L, 2);
    pushSdf(L, sdfSphere(c, r));
    return 1;
}

int l_sdf_box(lua_State* L) {
    Vec3 c = checkVec3(L, 1);
    Vec3 h = checkVec3(L, 2);
    pushSdf(L, sdfBox(c, h));
    return 1;
}

int l_sdf_capsule(lua_State* L) {
    Vec3 a = checkVec3(L, 1);
    Vec3 b = checkVec3(L, 2);
    double r = luaL_checknumber(L, 3);
    pushSdf(L, sdfCapsule(a, b, r));
    return 1;
}

int l_sdf_union(lua_State* L) {
    pushSdf(L, sdfUnion(checkSdf(L, 1), checkSdf(L, 2)));
    return 1;
}

int l_sdf_intersect(lua_State* L) {
    pushSdf(L, sdfIntersect(checkSdf(L, 1), checkSdf(L, 2)));
    return 1;
}

int l_sdf_subtract(lua_State* L) {
    pushSdf(L, sdfSubtract(checkSdf(L, 1), checkSdf(L, 2)));
    return 1;
}

int l_sdf_smooth_union(lua_State* L) {
    Sdf a = checkSdf(L, 1);
    Sdf b = checkSdf(L, 2);
    double k = luaL_checknumber(L, 3);
    pushSdf(L, sdfSmoothUnion(a, b, k));
    return 1;
}

// --- noise.* (seed-first so the surface stays a pure function) ---

int l_noise_value2(lua_State* L) {
    auto seed = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    lua_pushnumber(L, Noise(seed).noise2(x, y));
    return 1;
}

int l_noise_fbm2(lua_State* L) {
    auto seed = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    int octaves = static_cast<int>(luaL_optinteger(L, 4, 4));
    lua_pushnumber(L, Noise(seed).fbm2(x, y, octaves));
    return 1;
}

int l_noise_fbm3(lua_State* L) {
    auto seed = static_cast<uint32_t>(luaL_checkinteger(L, 1));
    double x = luaL_checknumber(L, 2);
    double y = luaL_checknumber(L, 3);
    double z = luaL_checknumber(L, 4);
    int octaves = static_cast<int>(luaL_optinteger(L, 5, 4));
    lua_pushnumber(L, Noise(seed).fbm3(x, y, z, octaves));
    return 1;
}

// --- polygonize(field, {min=, max=}, resolution) -> Mesh ---

int l_polygonize(lua_State* L) {
    Sdf field = checkSdf(L, 1);
    luaL_checktype(L, 2, LUA_TTABLE);
    lua_getfield(L, 2, "min"); Vec3 mn = checkVec3(L, -1); lua_pop(L, 1);
    lua_getfield(L, 2, "max"); Vec3 mx = checkVec3(L, -1); lua_pop(L, 1);
    int res = static_cast<int>(luaL_checkinteger(L, 3));
    luaL_argcheck(L, res >= 2, 3, "resolution must be >= 2");

    RenderMesh mesh = polygonizeSdf(field, SdfBounds{mn, mx}, res);
    pushMesh(L, std::make_shared<RenderMesh>(std::move(mesh)));
    return 1;
}

void registerMetatable(lua_State* L, const char* name, lua_CFunction gc) {
    if (luaL_newmetatable(L, name)) {
        lua_pushcfunction(L, gc);
        lua_setfield(L, -2, "__gc");
    }
    lua_pop(L, 1);
}

}  // namespace

void openProcgenLibrary(ScriptVM& vm) {
    lua_State* L = luaState(vm);

    registerMetatable(L, kSdfMt, sdfGc);
    registerMetatable(L, kMeshMt, meshGc);

    static const luaL_Reg kSdfFns[] = {
        {"sphere", l_sdf_sphere},
        {"box", l_sdf_box},
        {"capsule", l_sdf_capsule},
        {"union", l_sdf_union},
        {"intersect", l_sdf_intersect},
        {"subtract", l_sdf_subtract},
        {"smooth_union", l_sdf_smooth_union},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kSdfFns);
    lua_setglobal(L, "sdf");

    static const luaL_Reg kNoiseFns[] = {
        {"value2", l_noise_value2},
        {"fbm2", l_noise_fbm2},
        {"fbm3", l_noise_fbm3},
        {nullptr, nullptr},
    };
    luaL_newlib(L, kNoiseFns);
    lua_setglobal(L, "noise");

    lua_pushcfunction(L, l_polygonize);
    lua_setglobal(L, "polygonize");
}

bool runProcgenMesh(ScriptVM& vm, const std::string& code,
                    std::shared_ptr<RenderMesh>& out, std::string* error) {
    lua_State* L = luaState(vm);
    if (luaL_loadstring(L, code.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 1, 0) != LUA_OK) {
        if (error != nullptr) {
            const char* msg = lua_tostring(L, -1);
            *error = msg != nullptr ? msg : "unknown Lua error";
        }
        lua_pop(L, 1);
        return false;
    }

    auto* mesh = static_cast<MeshPtr*>(luaL_testudata(L, -1, kMeshMt));
    if (mesh == nullptr) {
        if (error != nullptr) *error = "procgen script did not return a Mesh";
        lua_pop(L, 1);
        return false;
    }
    out = *mesh;
    lua_pop(L, 1);
    return true;
}

}  // namespace engine
