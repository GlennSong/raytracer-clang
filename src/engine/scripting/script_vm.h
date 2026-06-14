#ifndef RAYTRACER_ENGINE_SCRIPTING_SCRIPT_VM_H
#define RAYTRACER_ENGINE_SCRIPTING_SCRIPT_VM_H

#include <memory>
#include <string>

namespace engine {

// The engine's embedded scripting VM (ADR-0023): a single Lua interpreter,
// sealed so no `lua_*` type reaches engine/game headers — exactly how `Window`
// seals GLFW (ADR-0001) and `PhysicsWorld` seals Jolt (ADR-0012). One ScriptVM
// owns one lua_State; create one per thread (Lua has no global lock), so procgen
// scripts fan out on the JobSystem (ADR-0014) without contention.
//
// The VM is a *sandbox*: it opens only the pure standard libraries (base, table,
// string, math, utf8) — never io/os/package/require/debug — so a script cannot
// touch the filesystem, read the clock, or load native code. That is the
// determinism + safety contract procgen needs (ADR-0021/0002): a script is a
// pure (params, seed) -> content function. Binding surfaces (e.g. the procgen
// library, see procgen_bindings.h) are layered on top of this base.
class ScriptVM {
public:
    ScriptVM();
    ~ScriptVM();

    ScriptVM(ScriptVM&&) noexcept;
    ScriptVM& operator=(ScriptVM&&) noexcept;
    ScriptVM(const ScriptVM&) = delete;
    ScriptVM& operator=(const ScriptVM&) = delete;

    // Run a chunk of Lua source. Returns true on success; on failure returns
    // false and, if `error` is non-null, fills it with Lua's error message.
    bool doString(const std::string& code, std::string* error = nullptr);

    // Set a global the script can read (e.g. a per-variant `seed`), so hosts
    // pass parameters in without compiling Lua source.
    void setGlobalNumber(const std::string& name, double value);

    // Read a global the script set (convenience for hosts/tests). Returns false
    // if the global is missing or not of the requested type.
    bool getGlobalNumber(const std::string& name, double& out) const;
    bool getGlobalBool(const std::string& name, bool& out) const;

    // The sealed state. Defined only in the scripting module's internal header
    // (lua_state.h); incomplete here so the Lua C API stays out of public
    // headers. Binding TUs reach the lua_State through it.
    struct Impl;
    Impl* impl() const { return impl_.get(); }

private:
    std::unique_ptr<Impl> impl_;
};

}  // namespace engine

#endif
