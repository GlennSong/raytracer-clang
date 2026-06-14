#ifndef RAYTRACER_ENGINE_SCRIPTING_LUA_STATE_H
#define RAYTRACER_ENGINE_SCRIPTING_LUA_STATE_H

// Internal to the scripting module (ADR-0023): the ONE place the Lua C API is
// included. Engine/game headers must never include this — they use the sealed
// script_vm.h. Lua ships its headers as plain C (no extern "C" guards; that is
// why lua.hpp exists in full distributions), and the bare lua/lua source tree
// has no lua.hpp, so we wrap the includes ourselves.
extern "C" {
#include "lua.h"
#include "lualib.h"
#include "lauxlib.h"
}

#include "script_vm.h"

namespace engine {

// The sealed state. Lives here (not in script_vm.cpp) so binding TUs — e.g.
// procgen_bindings.cpp — can reach the lua_State without re-deriving it.
struct ScriptVM::Impl {
    lua_State* L = nullptr;
};

// The raw state behind a ScriptVM (scripting-module-internal use only).
inline lua_State* luaState(ScriptVM& vm) { return vm.impl()->L; }

}  // namespace engine

#endif
