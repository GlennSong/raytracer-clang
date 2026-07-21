#ifndef RAYTRACER_ENGINE_SCRIPTING_SCRIPT_MODULES_H
#define RAYTRACER_ENGINE_SCRIPTING_SCRIPT_MODULES_H

#include <functional>
#include <string>

namespace engine {

class ScriptVM;

// SHARED LUA MODULES — an opt-in `require` for the procgen sandbox (ADR-0023).
//
// The sandbox opens no `package`, so scripts could not include one another and
// every shared table had to be copy-pasted. assets/scripts/twin_cities.lua says
// so out loud: "sandbox has no require, so the essentials are ported here rather
// than shared". assets/scripts/vehicle_classes.lua was written as a shared data
// file and, with no way to load it, was dead on arrival.
//
// THIS DOES NOT WEAKEN THE SANDBOX, and the distinction is the whole design:
//
//   * Lua passes an IDENTIFIER, never a path. Names must match [A-Za-z0-9_]+ —
//     no dots, no slashes, no `..`, no extension — so a script cannot express a
//     location, and there is no path to sanitise. `require "vehicle_classes"`,
//     never `require "../../etc/passwd"`.
//   * C++ maps identifiers to files under a fixed script root. The VM still
//     never touches the filesystem; the host does, exactly as it already does to
//     read the entity script itself.
//   * `luaopen_package` stays CLOSED. This is a plain C closure, not a
//     `package.searchers` entry, so there is no path list to append to and no
//     native `.so` loading.
//
// Net effect is a tighter seal than before: adding this came with nil'ing the
// `dofile`/`loadfile` that luaopen_base had been handing out all along
// (script_vm.cpp), which were real arbitrary-path reads.
//
// Determinism (ADR-0002) holds: same inputs, same modules, same output. The
// module cache is per-lua_State, so two VMs on two threads share no mutable
// state and ADR-0023's one-VM-per-thread rule is untouched.

// Resolve `name` to Lua source. Return false if there is no such module.
// `resolvedPath` is for logging and for the hot-reload watch list.
using ScriptModuleSource = std::function<bool(const std::string& name,
                                              std::string& source,
                                              std::string& resolvedPath)>;

// Install the global `require` on `vm`. Layered on top of a VM the way
// openProcgenLibrary layers `mesh.*` — a bare ScriptVM still has no `require`,
// which is what keeps the sandbox test in tests/test_script_vm.cpp honest.
//
// Cycles error rather than hanging; a module returning nil caches as `true`
// (Lua's own convention) so it is not re-run.
void openModuleLoader(ScriptVM& vm, ScriptModuleSource source);

}  // namespace engine

#endif
