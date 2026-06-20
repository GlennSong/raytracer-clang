#include "test_framework.h"

#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"
#include "../src/engine/procgen/sdf.h"
#include "../src/engine/mesh_builder.h"
#include "../src/renderer/renderer.h"

#include <algorithm>
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

TEST_CASE(procgen_script_grows_a_building) {
    // ADR-0038/0028: the split/shape grammar is an L1 sibling exposed to Lua, the
    // same way the L-system is. building.grow returns a mesh; taller floors -> more
    // geometry; building.height is a cheap query.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        return building.grow{ floors = 6, width = 18, depth = 14,
                              ground_retail = true, walkable_ground = true, seed = 3 }
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 100);
    CHECK(mesh->indices.size() % 3 == 0);

    double h6 = 0, h12 = 0;
    CHECK(vm.doString("h6  = building.height{ floors = 6 }"));
    CHECK(vm.doString("h12 = building.height{ floors = 12 }"));
    CHECK(vm.getGlobalNumber("h6", h6));
    CHECK(vm.getGlobalNumber("h12", h12));
    CHECK(h12 > h6);
    CHECK_APPROX(h6, 6 * 3.2 + 4.5 + 1.1, 1e-6);   // matches the C++ grammar
}

TEST_CASE(procgen_script_builds_street_furniture) {
    // ADR-0042: the street kit is exposed as tunable Lua recipes wrapping the
    // same C++ builder the city uses. A lamp and a signal each return a mesh, and
    // a taller lamp yields a taller mesh (the params reach the builder).
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> lamp, signal;
    std::string err;
    CHECK(runProcgenMesh(vm, "return streetfurniture.lamp{ height = 4.6 }", lamp, &err));
    CHECK(runProcgenMesh(vm, "return streetfurniture.traffic_signal{}", signal, &err));
    if (!lamp || !signal) { CHECK(false); return; }
    CHECK(lamp->vertices.size() > 12);
    CHECK(signal->indices.size() % 3 == 0);
    CHECK(signal->vertices.size() > lamp->vertices.size());   // signal is richer

    // A taller lamp reaches higher (the height param flows to the C++ builder).
    auto topY = [](const RenderMesh& m) {
        double y = -1e30;
        for (const Vertex& v : m.vertices) y = std::max(y, double(v.position.y));
        return y;
    };
    std::shared_ptr<RenderMesh> tall;
    CHECK(runProcgenMesh(vm, "return streetfurniture.lamp{ height = 8.0 }", tall, nullptr));
    if (tall) CHECK(topY(*tall) > topY(*lamp));
}

TEST_CASE(procgen_script_authors_with_scope_ops) {
    // ADR-0042 Phase 2: the split/shape-grammar op-vocabulary is exposed, so Lua
    // can subdivide a scope and emit geometry per cell — author props/details,
    // not just whole buildings. The ops wrap the same C++ grammar.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> mesh;
    std::string err;
    const char* code = R"LUA(
        local s = scope{ origin = {0, 0, 0}, size = {4, 3, 2} }
        local bays = s:split('x', { 1, 1, 1, 1 })      -- four equal bays
        local parts = {}
        for i = 1, #bays do parts[i] = bays[i]:box{0.2, 0.6, 0.8} end
        return mesh.merge(parts)
    )LUA";
    CHECK(runProcgenMesh(vm, code, mesh, &err));
    if (!mesh) { CHECK(false); return; }
    CHECK(mesh->vertices.size() > 0);
    CHECK(mesh->indices.size() % 3 == 0);

    // split returns the requested number of cells; divide sizes by target.
    double n = 0, halved = 0, divided = 0;
    CHECK(vm.doString("n = #(scope{origin={0,0,0},size={4,3,2}}:split('x',{1,1,1,1}))"));
    CHECK(vm.getGlobalNumber("n", n));
    CHECK(n == 4);
    CHECK(vm.doString("divided = #(scope{origin={0,0,0},size={10,3,2}}:divide('x', 2.0))"));
    CHECK(vm.getGlobalNumber("divided", divided));
    CHECK(divided == 5);                                  // 10 / 2 = 5 cells

    // inset shrinks the footprint but keeps height; corner/center read back.
    CHECK(vm.doString("halved = (scope{origin={0,0,0},size={4,3,4}}:inset(1.0)):size()[1]"));
    CHECK(vm.getGlobalNumber("halved", halved));
    CHECK_APPROX(halved, 2.0, 1e-6);                     // 4 - 2*1 = 2 wide

    // mesh.quad emits a single panel (two triangles).
    std::shared_ptr<RenderMesh> quad;
    CHECK(runProcgenMesh(vm,
        "return mesh.quad({0,0,0},{1,0,0},{1,1,0},{0,1,0},{0,0,1},{1,1,1})",
        quad, nullptr));
    if (quad) CHECK(quad->indices.size() == 6);
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

// --- the deepened surface: a whole generator written in Lua ---

TEST_CASE(procgen_script_builds_a_full_lsystem_tree) {
    // A complete grammar generator authored in Lua: define rules, expand the
    // axiom, then skin the turtle string into one welded SDF surface. This is
    // the "how deep can you go" proof — the same pipeline as the C++ generators.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> tree;
    std::string err;
    const char* code = R"LUA(
        local sys = lsystem.create()
        sys:rule("X", "F[+X][-X]FX")
        sys:rule("F", "FF")
        local symbols = sys:expand("X", 3, 7)
        return lsystem.turtle_mesh_sdf(symbols,
            {length = 0.6, radius = 0.10, angle_deg = 28, leaf_radius = 0.0},
            0.08, 48)
    )LUA";
    CHECK(runProcgenMesh(vm, code, tree, &err));
    if (!tree) { CHECK(false); return; }
    CHECK(tree->vertices.size() > 100);     // a real welded canopy of branches
    CHECK(tree->indices.size() % 3 == 0);
}

TEST_CASE(procgen_script_skins_a_parametric_tree) {
    // The grammar-first path (ADR-0030): a script defines a *parametric* grammar
    // (successor params are expressions over the predecessor's), expands it, and
    // skins the modules into the real tree (curved generalized-cylinder bark +
    // alpha-cut leaf cards) via tree.skin — the same generator the C++ growTree
    // uses, now driven entirely from Lua.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> bark;
    std::string err;
    const char* code = R"LUA(
        local sys = lsystem.parametric()
        sys:rule("A(l,w)",
          "F(l,w)/(137.5)[&(38)A(l*0.8,w*0.6)]/(137.5)[&(38)A(l*0.8,w*0.6)]/(137.5)A(l*0.85,w*0.7)")
        local modules = sys:expand("A(1.4,0.12)", 5, 7)
        local bark, leaves = tree.skin(modules,
            { ring_segments = 6, droop = 0.2, leaves_per_tip = 4,
              bark_color = {0.3, 0.2, 0.13}, leaf_color = {0.2, 0.45, 0.13} }, 7)
        assert(leaves ~= nil, "expected leaf cards")
        return bark
    )LUA";
    CHECK(runProcgenMesh(vm, code, bark, &err));
    if (!bark) { CHECK(false); return; }
    CHECK(bark->vertices.size() > 100);     // a swept canopy of branches
    CHECK(bark->indices.size() % 3 == 0);
}

TEST_CASE(procgen_model_returns_multiple_parts) {
    // Multi-part model (ADR-0032): a script returns a list of {mesh, material}
    // parts — here bark (opaque, bark texture) + leaves (alpha-cut, leaf texture)
    // from one tree. The loader turns each into its own InstanceGroup.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::vector<ScriptMeshPart> parts;
    std::string err;
    const char* code = R"LUA(
        local sys = lsystem.parametric()
        sys:rule("A(l,w)", "F(l,w)/(137.5)[&(40)A(l*0.8,w*0.6)]/(137.5)A(l*0.85,w*0.7)")
        local modules = sys:expand("A(1.2,0.1)", 4, 5)
        local bark, leaves = tree.skin(modules, { droop = 0.2, leaves_per_tip = 3 }, 5)
        return {
            { mesh = bark,   texture = "bark" },
            { mesh = leaves, texture = "leaf", alpha_test = true, roughness = 0.6 },
        }
    )LUA";
    CHECK(runProcgenModel(vm, code, parts, &err));
    CHECK(parts.size() == 2u);
    if (parts.size() == 2) {
        CHECK(parts[0].texture == "bark");
        CHECK(parts[0].mesh && !parts[0].mesh->vertices.empty());
        CHECK(parts[0].hasMaterial);
        CHECK(parts[1].texture == "leaf");
        CHECK(parts[1].alphaTest);
        CHECK(parts[1].mesh && !parts[1].mesh->vertices.empty());
    }
}

TEST_CASE(procgen_model_single_mesh_is_one_default_part) {
    // Back-compat: a bare single-mesh return is one part with no material of its
    // own (the caller applies the species' default).
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::vector<ScriptMeshPart> parts;
    std::string err;
    CHECK(runProcgenModel(vm, "return mesh.box({1, 1, 1})", parts, &err));
    CHECK(parts.size() == 1u);
    if (!parts.empty()) CHECK(!parts[0].hasMaterial);
}

TEST_CASE(procgen_script_kitbashes_meshes) {
    // mesh primitives + transform + merge: two boxes welded into one buffer.
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> combined;
    const char* code = R"LUA(
        local a = mesh.box({1, 1, 1})
        local b = mesh.translate(mesh.box({1, 1, 1}), {2, 0, 0})
        return mesh.merge({a, b})
    )LUA";
    CHECK(runProcgenMesh(vm, code, combined, nullptr));
    if (!combined) { CHECK(false); return; }

    RenderMesh oneBox = MeshBuilder::box(Vec3(1, 1, 1));
    CHECK(combined->vertices.size() == oneBox.vertices.size() * 2);
    CHECK(combined->indices.size() == oneBox.indices.size() * 2);
    // The second box was shifted +2 in x: some vertex must sit out there.
    bool shifted = false;
    for (const Vertex& v : combined->vertices)
        if (v.position.x > 1.5) shifted = true;
    CHECK(shifted);
}

TEST_CASE(procgen_script_builds_terrain) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    std::shared_ptr<RenderMesh> ground;
    const char* code = R"LUA(
        return terrain({size = 50, resolution = 16, height_scale = 5}, 99)
    )LUA";
    CHECK(runProcgenMesh(vm, code, ground, nullptr));
    if (!ground) { CHECK(false); return; }
    CHECK(ground->vertices.size() == 17 * 17);   // (resolution + 1)^2
    CHECK(ground->indices.size() % 3 == 0);
}

TEST_CASE(procgen_scatter_returns_frames) {
    ScriptVM vm;
    openProcgenLibrary(vm);
    // scatter yields a plain Lua array of {position, yaw, scale} — the Frame
    // value type. A script consumes it like any table.
    CHECK(vm.doString(R"LUA(
        local frames = scatter({region_size = 80, count = 200, seed = 3},
                               {size = 80, resolution = 32}, 99)
        n = #frames
        ok = (n > 0) and (frames[1].position ~= nil)
              and (type(frames[1].yaw) == "number")
    )LUA"));
    double n = 0;
    bool ok = false;
    CHECK(vm.getGlobalNumber("n", n));
    CHECK(n > 0);
    CHECK(vm.getGlobalBool("ok", ok));
    CHECK(ok);
}
