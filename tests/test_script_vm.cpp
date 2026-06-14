#include "test_framework.h"

#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/procgen/sdf.h"
#include "../src/renderer/renderer.h"

#include <cmath>
#include <memory>
#include <string>

using namespace engine;  // namespace migration (ADR-0015)

// --- the VM seal (ADR-0023) ---

TEST_CASE(script_vm_runs_and_reads_globals) {
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString("answer = 6 * 7", &err));
    double a = 0;
    CHECK(vm.getGlobalNumber("answer", a));
    CHECK_APPROX(a, 42.0, 1e-12);
}

TEST_CASE(script_vm_reports_errors) {
    ScriptVM vm;
    std::string err = "unset";
    CHECK(!vm.doString("this is not valid lua", &err));
    CHECK(!err.empty());
    CHECK(err != "unset");
}

TEST_CASE(script_vm_sandbox_blocks_io_os_require) {
    ScriptVM vm;
    // The procgen sandbox opens no io/os/package, so those globals are nil and a
    // script cannot reach the filesystem, the clock, or native loading.
    bool b = false;
    CHECK(vm.doString("ok_io = (io == nil)"));
    CHECK(vm.getGlobalBool("ok_io", b)); CHECK(b);
    b = false;
    CHECK(vm.doString("ok_os = (os == nil)"));
    CHECK(vm.getGlobalBool("ok_os", b)); CHECK(b);
    b = false;
    CHECK(vm.doString("ok_req = (require == nil)"));
    CHECK(vm.getGlobalBool("ok_req", b)); CHECK(b);

    // ...but the pure libraries ARE available.
    double pi = 0;
    CHECK(vm.doString("p = math.pi"));
    CHECK(vm.getGlobalNumber("p", pi));
    CHECK_APPROX(pi, 3.14159265358979, 1e-10);
}

// --- the procgen binding surface ---

TEST_CASE(procgen_script_builds_an_sdf_sphere_mesh) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local s = sdf.sphere({0, 0, 0}, 1.0)
        return polygonize(s, {min = {-1.5,-1.5,-1.5}, max = {1.5,1.5,1.5}}, 24)
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 50);
    CHECK(mesh->indices.size() % 3 == 0);

    // Same property test_sdf.cpp asserts for the C++ path: vertices sit on the
    // unit sphere (within a grid cell).
    const double cell = 3.0 / 24;
    bool onSurface = true;
    for (const Vertex& v : mesh->vertices)
        if (std::fabs(v.position.length() - 1.0) > 1.5 * cell) onSurface = false;
    CHECK(onSurface);
}

TEST_CASE(procgen_script_matches_the_cpp_substrate) {
    // ADR-0023: the Lua front-end and the C++ generators are the SAME substrate.
    // An identical recipe must yield an identical mesh.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> viaScript;
    const char* code = R"LUA(
        local a = sdf.sphere({-0.6, 0, 0}, 0.7)
        local b = sdf.sphere({ 0.6, 0, 0}, 0.7)
        return polygonize(sdf.smooth_union(a, b, 0.4),
                          {min = {-2,-2,-2}, max = {2,2,2}}, 32)
    )LUA";
    CHECK(runProcgenMesh(vm, code, viaScript, nullptr));
    if (!viaScript) { CHECK(false); return; }

    Sdf blob = sdfSmoothUnion(sdfSphere(Vec3(-0.6, 0, 0), 0.7),
                              sdfSphere(Vec3(0.6, 0, 0), 0.7), 0.4);
    RenderMesh viaCpp =
        polygonizeSdf(blob, SdfBounds{Vec3(-2, -2, -2), Vec3(2, 2, 2)}, 32);

    CHECK(viaScript->vertices.size() == viaCpp.vertices.size());
    CHECK(viaScript->indices.size() == viaCpp.indices.size());
}

TEST_CASE(procgen_non_mesh_return_is_an_error) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    CHECK(!runProcgenMesh(vm, "return 42", mesh, &err));
    CHECK(!err.empty());
    CHECK(mesh == nullptr);
}

TEST_CASE(procgen_noise_is_seed_deterministic) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    double v1 = 0, v2 = 0, v3 = 0;
    CHECK(vm.doString("a = noise.fbm2(1234, 1.5, 2.5)"));
    CHECK(vm.doString("b = noise.fbm2(1234, 1.5, 2.5)"));   // same seed+coords
    CHECK(vm.doString("c = noise.fbm2(9999, 1.5, 2.5)"));   // different seed
    CHECK(vm.getGlobalNumber("a", v1));
    CHECK(vm.getGlobalNumber("b", v2));
    CHECK(vm.getGlobalNumber("c", v3));
    CHECK_APPROX(v1, v2, 1e-12);   // deterministic
    CHECK(std::fabs(v1 - v3) > 1e-9);  // seed actually matters
}
