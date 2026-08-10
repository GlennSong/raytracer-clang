#include "test_framework.h"

#include <cmath>

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
// the render bridge draws no car for that slot) rather than at draw time.

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
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
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

TEST_CASE(every_fleet_slot_has_exactly_four_corner_lamps) {
    VehiclesVM a;
    CHECK(a.loaded);
    // EVERY slot, not just the sedan: syncCarLamps overlays one emissive
    // instance per marker, so a slot carrying a duplicate or a third lamp per
    // end draws visibly doubled lights on that class of car.
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        int heads = 0, tails = 0;
        for (const Attachment& lt : body.lights) {
            if (lt.name.rfind("headlight_", 0) == 0) ++heads;
            if (lt.name.rfind("taillight_", 0) == 0) ++tails;
        }
        // No two lamp markers may share a position — coincident markers stack
        // two lit lenses in one place, which reads as one over-bright lamp, and
        // two markers a few cm apart read as a doubled one.
        int tooClose = 0;
        for (std::size_t i = 0; i < body.lights.size(); ++i)
            for (std::size_t j = i + 1; j < body.lights.size(); ++j) {
                const Vec3 d = body.lights[i].pos - body.lights[j].pos;
                if (d.length() < 0.05) {
                    ++tooClose;
                    std::printf("    [slot %2d] coincident: %s / %s\n", slot,
                                body.lights[i].name.c_str(),
                                body.lights[j].name.c_str());
                }
            }
        std::printf("    [slot %2d] %d markers: %d head, %d tail\n", slot,
                    static_cast<int>(body.lights.size()), heads, tails);
        CHECK(heads == 2);
        CHECK(tails == 2);
        CHECK(tooClose == 0);
    }
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

TEST_CASE(fleet_publishes_a_wheel_less_chassis) {
    VehiclesVM a;
    CHECK(a.loaded);
    // A car the player COMMANDEERS (ADR-0062) becomes a real Vehicle and grows
    // physics wheels, so it needs the same car MINUS the baked wheelset —
    // otherwise it drives on eight. Before this existed, promotion fell back to
    // the retired C++ box body: the whole city was mesh.car and the one car in
    // the player's hands was a box.
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        CHECK(!body.chassis.vertices.empty());
        CHECK(body.chassis.indices.size() % 3 == 0);
        CHECK(anyColored(body.chassis));
        // Strictly less car than the full instance, and the difference is the
        // wheels: the chassis must not reach as far DOWN, because the wheels are
        // what touch the road (fleet_cars_stand_on_their_wheels pins that drop).
        CHECK(body.chassis.indices.size() < body.mesh.indices.size());
        double meshFloor = 1e30, chassisFloor = 1e30;
        for (const Vertex& v : body.mesh.vertices)
            meshFloor = std::min(meshFloor, static_cast<double>(v.position.y));
        for (const Vertex& v : body.chassis.vertices)
            chassisFloor = std::min(chassisFloor, static_cast<double>(v.position.y));
        CHECK(chassisFloor > meshFloor);
    }
}

TEST_CASE(fleet_wheel_layout_matches_the_drawn_wheels) {
    VehiclesVM a;
    CHECK(a.loaded);
    // The promoted car's Jolt wheels are built from this layout. It must
    // describe the wheels that were actually DRAWN — the failure it replaces is
    // a radius guessed from the body's height, which put the simulated wheels
    // several centimetres outside their arches.
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        CHECK(body.wheels.size() == 4);

        int steered = 0, handBrake = 0;
        double loX = 1e30, hiX = -1e30, loY = 1e30, loZ = 1e30, hiZ = -1e30;
        for (const Vertex& v : body.mesh.vertices) {
            loX = std::min(loX, static_cast<double>(v.position.x));
            hiX = std::max(hiX, static_cast<double>(v.position.x));
            loY = std::min(loY, static_cast<double>(v.position.y));
            loZ = std::min(loZ, static_cast<double>(v.position.z));
            hiZ = std::max(hiZ, static_cast<double>(v.position.z));
        }
        for (const WheelPlacement& w : body.wheels) {
            CHECK(w.radius > 0.15 && w.radius < 1.0);
            CHECK(w.width > 0.05);
            CHECK(w.driven);                    // 4WD, like the drivable spec
            if (w.steered) ++steered;
            if (w.handBrake) ++handBrake;
            // Front wheels steer, rear wheels take the handbrake — never both.
            CHECK(w.steered != w.handBrake);
            // Inside the car it belongs to, not floating beside it.
            CHECK(w.pos.x > loX && w.pos.x < hiX);
            CHECK(w.pos.z > loZ && w.pos.z < hiZ);
            // A wheel RESTS on the ground: its contact point is the car's floor,
            // which is exactly what the baked wheels reach.
            CHECK(std::fabs((w.pos.y - w.radius) - loY) < 0.02);
        }
        CHECK(steered == 2);
        CHECK(handBrake == 2);

        // The layout beside the box it sits in, and beside the two numbers the
        // city_vehicles fallback would have guessed from that box (a half-track
        // of half-width minus 10 cm, an axle at 0.34 x length). Printed, not
        // asserted — the fallback may stay wrong for the built-in fleet; this
        // is here so drift in either is visible in the test log.
        const WheelPlacement& f = body.wheels[0];
        std::printf("    [slot %2d] car %.2f x %.2f   axle |x| %.3f |z| %.3f "
                    "r %.3f   (guess would be |x| %.3f |z| %.3f)\n",
                    slot, hiX - loX, hiZ - loZ, std::fabs(f.pos.x),
                    std::fabs(f.pos.z), f.radius,
                    (hiX - loX) * 0.5 - 0.10, (hiZ - loZ) * 0.34);
    }
}

TEST_CASE(fleet_wheels_are_round) {
    VehiclesVM a;
    CHECK(a.loaded);
    // THE SQUISHED WHEELS. Every fleet car used to be scaled to fit a hardcoded
    // per-slot box (the C++ kFleet table) that disagreed with its class package
    // — and disagreed by a DIFFERENT amount per axis. A wheel rolls in the Y-Z
    // plane, so a non-uniform fit turns it into an ellipse: the pickup's wheels
    // were squashed 10%, the hatchback's 8%. The catalogue is the size now and
    // nothing is scaled, so a wheel is as round as the cylinder it was cut from.
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        CHECK(body.wheels.size() == 4);
        // The wheelset mesh is everything the chassis is not.
        const std::size_t wheelVerts =
            body.mesh.vertices.size() - body.chassis.vertices.size();
        CHECK(wheelVerts > 0);

        for (const WheelPlacement& w : body.wheels) {
            // Vertices belonging to THIS wheel: the wheelset lives at the tail
            // of the merged mesh, and each wheel is isolated by its x band.
            double loY = 1e30, hiY = -1e30, loZ = 1e30, hiZ = -1e30;
            int n = 0;
            for (std::size_t i = body.chassis.vertices.size();
                 i < body.mesh.vertices.size(); ++i) {
                const Vertex& v = body.mesh.vertices[i];
                if (std::fabs(v.position.x - w.pos.x) > w.width) continue;
                if (std::fabs(v.position.z - w.pos.z) > w.radius * 1.5) continue;
                ++n;
                loY = std::min(loY, static_cast<double>(v.position.y));
                hiY = std::max(hiY, static_cast<double>(v.position.y));
                loZ = std::min(loZ, static_cast<double>(v.position.z));
                hiZ = std::max(hiZ, static_cast<double>(v.position.z));
            }
            CHECK(n > 0);
            const double dY = hiY - loY, dZ = hiZ - loZ;
            const double oval = dY > dZ ? dY / dZ : dZ / dY;
            std::printf("    [slot %2d] wheel %.3f x %.3f (r %.3f) oval %.4f\n",
                        slot, dY, dZ, w.radius, oval);
            // A circle: the two in-plane diameters agree, and both are 2r.
            CHECK(oval < 1.02);
            CHECK(std::fabs(dY - 2 * w.radius) < 0.02);
            CHECK(std::fabs(dZ - 2 * w.radius) < 0.02);
        }
    }
}

TEST_CASE(fleet_catalogue_is_the_size_the_car_is_built_at) {
    VehiclesVM a;
    CHECK(a.loaded);
    // The catalogue entry the SIM adopts must be the size the car was actually
    // drawn at — that equality is the whole point of retiring the slot box, and
    // it is what keeps follow gaps, parking bays and collider proxies honest.
    // Reading it must also be CHEAP: no geometry is built here.
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        Vec3 size(0, 0, 0);
        std::string className;
        std::string err;
        CHECK(loadFleetCatalogueEntry(a.vm, slot, size, className, &err));
        CHECK(size.x > 1.0 && size.x < 3.0);     // width
        CHECK(size.y > 1.0 && size.y < 3.5);     // height
        CHECK(size.z > 3.0 && size.z < 8.0);     // length
        CHECK(!className.empty());

        CarBodyRecipe body;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        CHECK(std::fabs(body.size.x - size.x) < 1e-9);
        CHECK(std::fabs(body.size.z - size.z) < 1e-9);
        // The drawn shell fits inside that box and very nearly fills it.
        double loX = 1e30, hiX = -1e30, loZ = 1e30, hiZ = -1e30;
        for (const Vertex& v : body.chassis.vertices) {
            loX = std::min(loX, static_cast<double>(v.position.x));
            hiX = std::max(hiX, static_cast<double>(v.position.x));
            loZ = std::min(loZ, static_cast<double>(v.position.z));
            hiZ = std::max(hiZ, static_cast<double>(v.position.z));
        }
        std::printf("    [slot %2d] %-10s catalogue %.2f x %.2f, shell %.2f x %.2f\n",
                    slot, className.c_str(), size.x, size.z, hiX - loX, hiZ - loZ);
        // The shell fills its box, running exactly 1 cm long: mesh.car sets each
        // lamp lens 5 mm proud of the nose and tail caps so it reads as
        // flush-mounted rather than buried (car_mesh.cpp kLampProud). That is
        // bodywork detail, not a fit error — anything beyond it is.
        CHECK(hiX - loX <= size.x + 0.002);
        CHECK(hiZ - loZ <= size.z + 0.012);
        CHECK(hiZ - loZ > size.z * 0.90);
    }
}

TEST_CASE(legacy_boxes_only_recipe_offers_no_chassis) {
    // A recipe that publishes no separate wheelset has no honest wheel-less
    // form: its boxes ARE the whole car. The reader must say so with an empty
    // chassis rather than hand out a body the wheels were never removed from —
    // the consumer falls back to the built-in fleet mesh on exactly that signal.
    ScriptVM vm;
    std::string err;
    CHECK(vm.doString(R"lua(
        vehicle = { fleet = {
            { parts = { { pos = { 0, 0, 0 }, size = { 2, 1, 4 }, color = { 1, 0, 0 } } } },
        } }
    )lua", &err));
    CarBodyRecipe body;
    CHECK(loadFleetCarBody(vm, 0, body, &err));
    CHECK(!body.mesh.vertices.empty());   // the car itself still builds
    CHECK(body.chassis.vertices.empty()); // ...but there is no physics-car form
    CHECK(body.wheels.empty());
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
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
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
