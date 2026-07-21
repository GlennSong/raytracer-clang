#include "test_framework.h"

#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/script_modules.h"
#include "../src/engine/scripting/procgen_bindings.h"

#include <map>
#include <string>

using namespace engine;

// The host-controlled `require` (script_modules.h). Driven by an IN-MEMORY
// module source, so these are hermetic — no fixture files, no cwd sensitivity,
// and the confinement tests can assert the filesystem layer was never reached
// by counting callback invocations.

namespace {

// A module source over a fixed map, counting lookups.
struct FakeModules {
    std::map<std::string, std::string> files;
    int lookups = 0;

    ScriptModuleSource source() {
        return [this](const std::string& name, std::string& code,
                      std::string& path) {
            ++lookups;
            auto it = files.find(name);
            if (it == files.end()) return false;
            code = it->second;
            path = "memory://" + name + ".lua";
            return true;
        };
    }
};

}  // namespace

TEST_CASE(modules_require_returns_the_module_value) {
    FakeModules mods;
    mods.files["kit"] = "return { answer = 42 }";
    ScriptVM vm;
    openModuleLoader(vm, mods.source());

    double v = 0;
    CHECK(vm.doString("out = require('kit').answer"));
    CHECK(vm.getGlobalNumber("out", v));
    CHECK_APPROX(v, 42.0, 1e-9);
    CHECK(mods.lookups == 1);
}

// Caching is identity, not just equality — two requires must be the SAME table,
// or a module holding shared state would silently fork.
TEST_CASE(modules_are_cached_and_run_once) {
    FakeModules mods;
    mods.files["counter"] = "runs = (runs or 0) + 1; return { n = runs }";
    ScriptVM vm;
    openModuleLoader(vm, mods.source());

    bool same = false;
    CHECK(vm.doString("same = (require('counter') == require('counter'))"));
    CHECK(vm.getGlobalBool("same", same));
    CHECK(same);

    double runs = 0;
    CHECK(vm.doString("r = runs"));
    CHECK(vm.getGlobalNumber("r", runs));
    CHECK_APPROX(runs, 1.0, 1e-9);   // body ran once despite two requires
    CHECK(mods.lookups == 1);        // ...and the host was asked once
}

// A module with no return value still counts as loaded, or a side-effect-only
// module re-runs on every require.
TEST_CASE(modules_returning_nil_cache_as_true) {
    FakeModules mods;
    mods.files["side"] = "hits = (hits or 0) + 1";
    ScriptVM vm;
    openModuleLoader(vm, mods.source());

    bool ok = false;
    CHECK(vm.doString("a = require('side'); b = require('side'); ok = (a == true and b == true)"));
    CHECK(vm.getGlobalBool("ok", ok));
    CHECK(ok);

    double hits = 0;
    CHECK(vm.doString("h = hits"));
    CHECK(vm.getGlobalNumber("h", hits));
    CHECK_APPROX(hits, 1.0, 1e-9);
}

TEST_CASE(modules_missing_module_errors_cleanly) {
    FakeModules mods;
    ScriptVM vm;
    openModuleLoader(vm, mods.source());

    std::string err = "unset";
    CHECK(!vm.doString("require('nope')", &err));
    CHECK(err != "unset");
    CHECK(!err.empty());
    CHECK(mods.lookups == 1);   // the name was legal, so the host WAS consulted
}

// THE CONFINEMENT TEST. Every one of these must be rejected by name validation
// BEFORE the source callback runs — asserted by the lookup count staying zero.
// If a path form ever reaches the filesystem layer, this fails.
TEST_CASE(modules_reject_anything_that_names_a_path) {
    const char* kBad[] = {
        "../x", "../../etc/passwd", "/etc/passwd", "a/b", "a\\b",
        "a.b", "kit.lua", "", " ", "a b", "a-b", "a:b", "~/x", "./x",
    };
    for (const char* bad : kBad) {
        FakeModules mods;
        mods.files["a"] = "return {}";
        ScriptVM vm;
        openModuleLoader(vm, mods.source());

        std::string err = "unset";
        const std::string code =
            std::string("require([[") + bad + "]])";
        CHECK(!vm.doString(code, &err));
        CHECK(mods.lookups == 0);   // never reached the host
    }
}

TEST_CASE(modules_cycles_error_rather_than_hang) {
    FakeModules mods;
    mods.files["a"] = "local b = require('b'); return { b = b }";
    mods.files["b"] = "local a = require('a'); return { a = a }";
    ScriptVM vm;
    openModuleLoader(vm, mods.source());

    std::string err = "unset";
    CHECK(!vm.doString("require('a')", &err));
    CHECK(err.find("cyclic") != std::string::npos);
}

// The loader must not re-open what the sandbox closed. This is the regression
// guard against someone "simplifying" it into luaopen_package later.
TEST_CASE(modules_loader_does_not_reopen_the_sandbox) {
    FakeModules mods;
    ScriptVM vm;
    openModuleLoader(vm, mods.source());

    const char* kForbidden[] = {"io", "os", "package", "dofile", "loadfile"};
    for (const char* name : kForbidden) {
        bool nil = false;
        CHECK(vm.doString(std::string("ok = (") + name + " == nil)"));
        CHECK(vm.getGlobalBool("ok", nil));
        CHECK(nil);
    }
}

// The cache is per-lua_State, so two VMs share nothing — the property that keeps
// ADR-0023's one-VM-per-thread rule and determinism intact.
TEST_CASE(modules_are_per_vm_and_deterministic) {
    FakeModules mods;
    mods.files["kit"] = "return { n = 7 }";

    double a = 0, b = 0;
    {
        ScriptVM vm;
        openModuleLoader(vm, mods.source());
        CHECK(vm.doString("require('kit').n = 99; out = require('kit').n"));
        CHECK(vm.getGlobalNumber("out", a));
    }
    {
        ScriptVM vm;   // a fresh VM must NOT see the mutation above
        openModuleLoader(vm, mods.source());
        CHECK(vm.doString("out = require('kit').n"));
        CHECK(vm.getGlobalNumber("out", b));
    }
    CHECK_APPROX(a, 99.0, 1e-9);
    CHECK_APPROX(b, 7.0, 1e-9);
}

// Modules see the binding surface, so shared code can build geometry.
TEST_CASE(modules_see_the_procgen_bindings) {
    FakeModules mods;
    mods.files["shapes"] = "return { unit = function() return mesh.box({1,1,1}) end }";
    ScriptVM vm;
    openProcgenLibrary(vm);
    openModuleLoader(vm, mods.source());

    std::string err = "unset";
    CHECK(vm.doString("ok = (require('shapes').unit() ~= nil)", &err));
    bool ok = false;
    CHECK(vm.getGlobalBool("ok", ok));
    CHECK(ok);
}
