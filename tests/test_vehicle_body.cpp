#include "test_framework.h"

#include <cmath>

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
    // The shipped sedan carries four corner lamps (head/tail x L/R) at the
    // right END of the car. It may carry OTHER named attachments too — the
    // fleet is built by mesh.car now, which also marks the driver seat — so
    // this pins the lamp set, not the marker count (the old box fleet's
    // hand-authored "exactly 4" was a property of the retired box recipe).
    int heads = 0, tails = 0;
    for (const Attachment& lt : body.lights) {
        CHECK(!lt.name.empty());
        if (lt.name.rfind("headlight_", 0) == 0) { ++heads; CHECK(lt.pos.z > 0); }
        if (lt.name.rfind("taillight_", 0) == 0) { ++tails; CHECK(lt.pos.z < 0); }
    }
    CHECK(heads == 2);   // left + right
    CHECK(tails == 2);
    // The lamps sit ON the body, not floating: within its own half-extents.
    for (const Attachment& lt : body.lights)
        CHECK(std::fabs(lt.pos.x) < 2.0 && std::fabs(lt.pos.z) < 4.0);
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

TEST_CASE(fleet_cars_stand_on_their_wheels) {
    VehiclesVM a;
    CHECK(a.loaded);
    // A parked car's wheels must reach the ROAD. The fleet bakes its own
    // wheels around mesh.car's shell, and the class package puts its floor at
    // -height/2 with the body floating a ride-height above it — so the mesh
    // must extend BELOW the body's rocker, down to (near) that floor. Copying
    // the drivable spec's suspension-attachment formula lifted every wheel by
    // a ride height: the car sat on its rocker with the wheels tucked inside,
    // and nothing touched the ground.
    for (int slot = 0; slot < citysim::carVariantCount(); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        if (body.mesh.vertices.empty()) continue;
        double lo = 1e30, hi = -1e30;
        // Rocker line: the lowest point of the middle 40% of the car's LENGTH
        // that is not a wheel — approximated by the lowest vertex near the
        // centreline (x within 25% of half-width), where no wheel sits.
        double zMin = 1e30, zMax = -1e30, xMax = 0;
        for (const Vertex& v : body.mesh.vertices) {
            lo = std::min(lo, static_cast<double>(v.position.y));
            hi = std::max(hi, static_cast<double>(v.position.y));
            zMin = std::min(zMin, static_cast<double>(v.position.z));
            zMax = std::max(zMax, static_cast<double>(v.position.z));
            xMax = std::max(xMax, std::fabs(static_cast<double>(v.position.x)));
        }
        double rocker = 1e30;
        for (const Vertex& v : body.mesh.vertices)
            if (std::fabs(v.position.x) < xMax * 0.25)
                rocker = std::min(rocker, static_cast<double>(v.position.y));
        const double height = hi - lo;
        // Wheels reach below the centreline rocker by a real fraction of the
        // car's height (a tucked-up wheel makes these equal).
        std::printf("    [slot %2d] height %.2f, floor %.2f, rocker %.2f, drop %.3f\n",
                    slot, height, lo, rocker, rocker - lo);
        CHECK(rocker - lo > height * 0.04);
    }
}
