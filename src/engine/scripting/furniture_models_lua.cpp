#include "furniture_models_lua.h"
#include "lua_state.h"
#include "lua_helpers.h"
#include "procgen_bindings.h"
#include "procgen_mesh.h"   // luaToMesh
#include "script_vm.h"
#include "../script_assets.h"

#include <memory>
#include <mutex>

namespace engine {

namespace {

bool fail(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

// the finishes by the names a recipe gives its parts, in FurnMat's order
const char* const kPartNames[kFurnMatCount] = {"wood", "fabric", "hard", "metal", "ceramic", "light"};

// Run furniture_models[name](variant) in `vm` and read the piece it returns into `out`.
bool runRecipe(ScriptVM& vm, const std::string& name, uint32_t variant, FurniturePiece& out, std::string* err) {
    lua_State* L = luaState(vm);
    const int base = lua_gettop(L);
    auto bail = [&](const std::string& msg) { lua_settop(L, base); return fail(err, name + ": " + msg); };
    lua_getglobal(L, "furniture_models");
    if (!lua_istable(L, -1)) return bail("no global `furniture_models`");
    lua_getfield(L, -1, name.c_str());
    if (!lua_isfunction(L, -1)) return bail("no recipe");
    lua_pushinteger(L, static_cast<lua_Integer>(variant));
    if (lua_pcall(L, 1, 1, 0) != LUA_OK) return bail(lua_tostring(L, -1) ? lua_tostring(L, -1) : "error");
    const int t = lua_gettop(L);
    if (!lua_istable(L, t)) return bail("the recipe returned no table");
    out = FurniturePiece{};
    lua_getfield(L, t, "size");
    if (!lua_istable(L, -1)) return bail("no size");
    out.size = checkVec3(L, -1);
    lua_pop(L, 1);
    lua_getfield(L, t, "solid");
    if (lua_isboolean(L, -1)) out.solid = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    out.colliderH = static_cast<Real>(optField(L, t, "collider_h", 0.0));
    lua_getfield(L, t, "colliders");
    if (lua_istable(L, -1)) {
        const lua_Integer n = luaL_len(L, -1);
        for (lua_Integer i = 1; i <= n; ++i) {
            lua_geti(L, -1, i);
            lua_geti(L, -1, 1);
            lua_geti(L, -2, 2);
            out.colliders.push_back({checkVec3(L, -2), checkVec3(L, -1)});
            lua_pop(L, 3);
        }
    }
    lua_pop(L, 1);
    lua_getfield(L, t, "parts");
    if (!lua_istable(L, -1)) return bail("no parts");
    for (int k = 0; k < kFurnMatCount; ++k) {
        lua_getfield(L, -1, kPartNames[k]);
        if (!lua_isnil(L, -1)) {
            const std::shared_ptr<RenderMesh> m = luaToMesh(L, -1);
            if (!m) return bail(std::string("part `") + kPartNames[k] + "` is not a mesh");
            out.mesh[static_cast<std::size_t>(k)] = *m;
        }
        lua_pop(L, 1);
    }
    lua_settop(L, base);
    return true;
}

}  // namespace

bool buildFurnitureModelFrom(const std::string& source, const std::string& name, uint32_t variant, FurniturePiece& out,
                             std::string* err) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::string e;
    if (!vm.doString(source, &e)) return fail(err, "furniture_models.lua: " + e);
    return runRecipe(vm, name, variant, out, err);
}

bool buildFurnitureModel(const std::string& name, uint32_t variant, FurniturePiece& out, std::string* err) {
    static std::mutex mu;
    static std::unique_ptr<ScriptVM> vm;
    static std::string loadError;
    std::lock_guard<std::mutex> lock(mu);
    if (!vm && loadError.empty()) {
        const std::string src = loadScriptCode("furniture_models.lua", "");
        auto fresh = std::make_unique<ScriptVM>();
        openProcgenLibrary(*fresh);
        std::string e;
        if (src.empty()) loadError = "no furniture_models.lua";
        else if (!fresh->doString(src, &e)) loadError = "furniture_models.lua: " + e;
        else vm = std::move(fresh);
    }
    if (!vm) return fail(err, loadError);
    return runRecipe(*vm, name, variant, out, err);
}

}  // namespace engine
