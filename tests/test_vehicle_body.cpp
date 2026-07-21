#include "test_framework.h"

#include "../src/apps/citysim/city_meshes.h"        // carVariantCount
#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"   // openProcgenLibrary
#include "../src/engine/scripting/script_modules.h"     // openModuleLoader
#include "../src/engine/script_assets.h"                // makeModuleSource
#include "../src/apps/citysim/scripting/vehicle_body.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

using namespace engine;

// The vehicles.lua fleet reader (ADR-0065): the shipped asset must build a
// recognizable, vertex-coloured car for EVERY fleet slot the sim uses, each with
// named light attachment markers; a broken recipe must fail loudly at load (so
// the render bridge falls back to the C++ fleetCarMesh) rather than at draw time.

namespace {

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// A VM with the shipped assets/scripts/vehicles.lua loaded (global `vehicle`).
struct VehiclesVM {
    ScriptVM vm;
    bool loaded = false;
    VehiclesVM() {
        // vehicles.lua now `require`s vehicle_classes + vehicle_forms, so the VM
        // needs the same surface the real hosts give it: the procgen bindings
        // plus a module loader rooted at the asset scripts. RT_SOURCE_DIR keeps
        // it resolving whatever the test's working directory is.
        openProcgenLibrary(vm);
        openModuleLoader(vm, makeModuleSource(std::string(RT_SOURCE_DIR) + "/assets/scripts"));
        std::string src =
            readFile(std::string(RT_SOURCE_DIR) + "/assets/scripts/vehicles.lua");
        std::string err;
        loaded = !src.empty() && vm.doString(src, &err);
        if (!loaded) std::printf("    vehicles.lua load error: %s\n", err.c_str());
    }
};

bool anyColored(const RenderMesh& m) {
    for (const Vertex& v : m.vertices)
        if (v.color.x != 1.0 || v.color.y != 1.0 || v.color.z != 1.0) return true;
    return false;
}

}  // namespace

TEST_CASE(vehicles_lua_builds_every_fleet_slot) {
    VehiclesVM a;
    CHECK(a.loaded);
    // Every sim fleet slot must have a Lua recipe that yields a real mesh.
    for (int slot = 0; slot < citysim::carVariantCount(); ++slot) {
        CarBodyRecipe body;
        std::string err;
        bool ok = loadFleetCarBody(a.vm, slot, body, &err);
        if (!ok) std::printf("    slot %d: %s\n", slot, err.c_str());
        CHECK(ok);
        CHECK(body.mesh.vertices.size() > 0);
        CHECK(body.mesh.indices.size() > 0);
        CHECK(body.mesh.indices.size() % 3 == 0);   // whole triangles
        CHECK(anyColored(body.mesh));                // paint/glass, not all white
        CHECK(body.lights.size() >= 2);              // at least front + rear lamps
        for (const Attachment& lt : body.lights) CHECK(!lt.name.empty());
    }
}

TEST_CASE(vehicles_lua_light_markers_are_named_and_positioned) {
    VehiclesVM a;
    CHECK(a.loaded);
    CarBodyRecipe body;
    std::string err;
    CHECK(loadFleetCarBody(a.vm, 0, body, &err));
    // The shipped sedan carries four named corner lamps (head/tail, L/R).
    CHECK(body.lights.size() == 4);
    bool head = false, tail = false;
    for (const Attachment& lt : body.lights) {
        if (lt.name.rfind("headlight_", 0) == 0) { head = true; CHECK(lt.pos.z > 0); }
        if (lt.name.rfind("taillight_", 0) == 0) { tail = true; CHECK(lt.pos.z < 0); }
    }
    CHECK(head);
    CHECK(tail);
}

TEST_CASE(vehicles_lua_reader_rejects_malformed_recipe) {
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString(R"lua(
        vehicle = { fleet = {
            { lights = { { name = "headlight_r", pos = { 0, 0, 1 } } } },  -- no parts
            { parts = { { pos = { 0, 0, 0 }, size = { 0, 1, 1 }, color = { 1, 0, 0 } } } },
        } }
    )lua", &err));
    CarBodyRecipe body;
    CHECK(!loadFleetCarBody(vm, 0, body, &err));   // slot 1: `parts` is mandatory
    CHECK(!err.empty());
    CHECK(!loadFleetCarBody(vm, 1, body, &err));   // slot 2: a zero-size part
    CHECK(!loadFleetCarBody(vm, 9, body, &err));   // out of range (fleet has 2)
    CHECK(!err.empty());
}

TEST_CASE(vehicles_lua_reader_needs_the_library_loaded) {
    ScriptVM vm;   // never ran vehicles.lua
    CarBodyRecipe body;
    std::string err;
    CHECK(!loadFleetCarBody(vm, 0, body, &err));   // no global `vehicle` table
    CHECK(!err.empty());
}
