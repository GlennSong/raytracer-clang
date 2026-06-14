#include "lua_state.h"

#include "../../check.h"

namespace engine {

namespace {

// Open only the pure standard libraries — the procgen sandbox (ADR-0023).
// Deliberately excludes io, os, package/require, and debug: no filesystem, no
// clock/ambient entropy, no native loading. base/table/string/math/utf8 are
// pure, so a script stays a deterministic (params, seed) -> content function.
void openSandboxLibs(lua_State* L) {
    static const luaL_Reg kLibs[] = {
        {LUA_GNAME, luaopen_base},
        {LUA_TABLIBNAME, luaopen_table},
        {LUA_STRLIBNAME, luaopen_string},
        {LUA_MATHLIBNAME, luaopen_math},
        {LUA_UTF8LIBNAME, luaopen_utf8},
        {nullptr, nullptr},
    };
    for (const luaL_Reg* lib = kLibs; lib->func != nullptr; ++lib) {
        luaL_requiref(L, lib->name, lib->func, 1);
        lua_pop(L, 1);  // requiref leaves the module on the stack
    }
}

}  // namespace

ScriptVM::ScriptVM() : impl_(std::make_unique<Impl>()) {
    impl_->L = luaL_newstate();
    CHECK(impl_->L != nullptr);  // only fails on OOM
    openSandboxLibs(impl_->L);
}

ScriptVM::~ScriptVM() {
    if (impl_ && impl_->L) lua_close(impl_->L);
}

ScriptVM::ScriptVM(ScriptVM&&) noexcept = default;
ScriptVM& ScriptVM::operator=(ScriptVM&&) noexcept = default;

bool ScriptVM::doString(const std::string& code, std::string* error) {
    lua_State* L = impl_->L;
    if (luaL_loadstring(L, code.c_str()) != LUA_OK ||
        lua_pcall(L, 0, 0, 0) != LUA_OK) {
        if (error != nullptr) {
            const char* msg = lua_tostring(L, -1);
            *error = msg != nullptr ? msg : "unknown Lua error";
        }
        lua_pop(L, 1);  // error message
        return false;
    }
    return true;
}

void ScriptVM::setGlobalNumber(const std::string& name, double value) {
    lua_State* L = impl_->L;
    lua_pushnumber(L, value);
    lua_setglobal(L, name.c_str());
}

bool ScriptVM::getGlobalNumber(const std::string& name, double& out) const {
    lua_State* L = impl_->L;
    lua_getglobal(L, name.c_str());
    const bool ok = lua_isnumber(L, -1) != 0;
    if (ok) out = lua_tonumber(L, -1);
    lua_pop(L, 1);
    return ok;
}

bool ScriptVM::getGlobalBool(const std::string& name, bool& out) const {
    lua_State* L = impl_->L;
    lua_getglobal(L, name.c_str());
    const bool ok = lua_isboolean(L, -1);
    if (ok) out = lua_toboolean(L, -1) != 0;
    lua_pop(L, 1);
    return ok;
}

}  // namespace engine
