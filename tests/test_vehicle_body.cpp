#include "test_framework.h"

#include <algorithm>
#include <cmath>

#include "../src/engine/scripting/script_vm.h"
#include "../src/engine/scripting/procgen_bindings.h"   // openProcgenLibrary
#include "../src/engine/scripting/script_modules.h"     // openModuleLoader
#include "../src/engine/script_assets.h"                // makeModuleSource
#include "../src/apps/citysim/scripting/vehicle_body.h"
#include "../src/engine/scripting/vehicle_spec.h"       // loadVehicleSpec (ride-height gate)
#include "../src/engine/procgen/vehicle/occupant.h"     // the seated rider (back bench)
#include "../src/engine/vehicle_lamps.h"                // LensRole: the lamp lenses' tags

#include <cstdio>
#include <cstdlib>
#include <map>
#include <tuple>
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
        // vehicles.lua `require`s vehicle_kit, so the VM
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

// A TWO-WHEELER's slot (the motorcycle, 2026-11-04), by its own handling's `two_wheeler`: one wheel and one lamp per
// end, no glass, an open seat. The car rules below hold for every other slot unchanged.
bool twoWheelSlot(ScriptVM& vm, int slot) {
    VehicleSpec h;
    return loadFleetHandling(vm, slot, h) && h.config.twoWheeler;
}

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
        const int perEnd = twoWheelSlot(a.vm, slot) ? 1 : 2;   // a bike: one lamp each end, on its centre line
        CHECK(heads == perEnd);
        CHECK(tails == perEnd);
        CHECK(tooClose == 0);
    }
}

// THE LAMPS ARE THE BODY'S (ADR-0342, #186): every fleet body -- the kit fleet, the classic set, the bikes, and the
// semi with its trailer -- carries its own lamp lenses, tagged by role in their u (engine/vehicle_lamps.h), and the
// renderer lights those. A body without a head and a tail lens drives dark; a head lens behind the middle lights
// the wrong end. A bike has an indicator a side at each end; a semi's trailer its tail, turn and marker lamps.
namespace {
struct LensCount { int head = 0, tail = 0, indicatorL = 0, indicatorR = 0, marker = 0, wrongEnd = 0; };
LensCount countLenses(const RenderMesh& m) {
    LensCount c;
    for (const Vertex& v : m.vertices) {
        if (v.u > engine::kLensTagBase - 0.5f) continue;
        const int role = static_cast<int>(engine::kLensTagBase - v.u + 0.5f);
        if (role == static_cast<int>(engine::LensRole::Head)) { ++c.head; c.wrongEnd += v.position.z <= 0; }
        else if (role == static_cast<int>(engine::LensRole::Tail)) { ++c.tail; c.wrongEnd += v.position.z >= 0; }
        else if (role == static_cast<int>(engine::LensRole::Indicator)) (v.position.x > 0 ? c.indicatorL : c.indicatorR)++;
        else if (role == static_cast<int>(engine::LensRole::Marker)) ++c.marker;
    }
    return c;
}
}  // namespace

TEST_CASE(every_fleet_body_carries_its_own_lit_lenses) {
    VehiclesVM a;
    CHECK(a.loaded);
    std::string err;
    // (a drivable spec as a fleet slot: its body and lamps; the wheels are not what is checked)
    CHECK(a.vm.doString("function __asSlot(s) return { body = s.body, wheels = mesh.box({0.1, 0.1, 0.1}), lights = s.lights } end", &err));
    for (const char* fleet : {"fleet_kit", "fleet_semis"}) {
        if (std::string(fleet) == "fleet_semis")
            CHECK(a.vm.doString("vehicle.fleet_semis = {"
                                "  { class = 'semi', size = {2.6, 4.1, 22}, build = function() return __asSlot(vehicle.kit_drivable('semi', {}, {})) end },"
                                "  { class = 'semi_tanker', size = {2.6, 4.1, 22}, build = function() return __asSlot(vehicle.kit_drivable('semi_tanker', {}, {})) end } }", &err));
        CHECK(a.vm.doString(std::string("vehicle.fleet = vehicle.") + fleet, &err));
        for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
            CarBodyRecipe body;
            const bool ok = loadFleetCarBody(a.vm, slot, body, &err);
            if (!ok) std::printf("    [%s %2d] %s\n", fleet, slot, err.c_str());
            CHECK(ok);
            const LensCount c = countLenses(body.mesh);
            std::printf("    [%s %2d] lens verts: head %d, tail %d, indicators %d/%d, markers %d, wrong end %d\n", fleet, slot,
                        c.head, c.tail, c.indicatorL, c.indicatorR, c.marker, c.wrongEnd);
            CHECK(c.head > 0);
            CHECK(c.tail > 0);
            CHECK(c.wrongEnd == 0);
            if (twoWheelSlot(a.vm, slot)) CHECK(c.indicatorL > 0 && c.indicatorR > 0);
            if (std::string(fleet) == "fleet_semis") CHECK(c.indicatorL > 0 && c.indicatorR > 0 && c.marker > 0);
        }
    }
    // ...and the bikes the player rides, which are no fleet slot yet (#186: into traffic next)
    for (const char* bike : {"motorcycle", "moped"}) {
        CHECK(a.vm.doString(std::string("vehicle.fleet = { { class = '") + bike + "', size = {0.7, 1.2, 2.0}, build = function() return __asSlot(vehicle." +
                                bike + "(0)) end } }", &err));
        CarBodyRecipe body;
        const bool ok = loadFleetCarBody(a.vm, 0, body, &err);
        if (!ok) std::printf("    [%s] %s\n", bike, err.c_str());
        CHECK(ok);
        const LensCount c = countLenses(body.mesh);
        std::printf("    [%s] lens verts: head %d, tail %d, indicators %d/%d\n", bike, c.head, c.tail, c.indicatorL, c.indicatorR);
        CHECK(c.head > 0 && c.tail > 0 && c.wrongEnd == 0 && c.indicatorL > 0 && c.indicatorR > 0);
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
        const bool bike = twoWheelSlot(a.vm, slot);
        const std::size_t perAxle = bike ? 1 : 2;
        CHECK(body.wheels.size() == 2 * perAxle);

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
            CHECK(w.driven || (bike && w.steered));   // 4WD, like the drivable spec (a bike: its rear wheel)
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
        CHECK(steered == static_cast<int>(perAxle));
        CHECK(handBrake == static_cast<int>(perAxle));

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
        CHECK(body.wheels.size() == (twoWheelSlot(a.vm, slot) ? 2u : 4u));
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
        CHECK(!className.empty());
        // The length bound said "a car is 3 to 8 m", which was true while every
        // fleet slot WAS a car. The transit slot is an 11.4 m bus, so the bound
        // is per-class now rather than being quietly loosened for everyone --
        // an 8 m "car" should still fail.
        const bool transit = (className.rfind("bus", 0) == 0 || className == "fire_engine");   // (a 9.2 m fire engine too)
        const bool bike = twoWheelSlot(a.vm, slot);   // a motorcycle: 0.7 m over the bars, about 2 m long
        CHECK(size.x > (bike ? 0.4 : 1.0) && size.x < (bike ? 1.2 : 3.0));             // width
        CHECK(size.y > (bike ? 0.5 : 1.0) && size.y < (transit ? 4.0 : 3.5));          // height
        CHECK(size.z > (bike ? 1.4 : 3.0) && size.z < (transit ? 15.0 : bike ? 2.8 : 8.0));   // length

        CarBodyRecipe body;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        CHECK(std::fabs(body.size.x - size.x) < 1e-9);
        CHECK(std::fabs(body.size.z - size.z) < 1e-9);
        // The drawn shell fits inside that box and very nearly fills it.
        double loX = 1e30, hiX = -1e30, loZ = 1e30, hiZ = -1e30;
        // (a bike's wheels ARE its ends, standing past its guards: its drawn length is the shell with them)
        for (const Vertex& v : (bike ? body.mesh : body.chassis).vertices) {
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
        // a bike's wheels stand ON its centre line, so its "rocker" is the lowest point of the frame (the shell without
        // its wheels)
        if (twoWheelSlot(a.vm, slot)) {
            rocker = 1e30;
            for (const Vertex& v : body.chassis.vertices) rocker = std::min(rocker, static_cast<double>(v.position.y));
        }
        const double height = hi - lo;
        // Wheels reach below the centreline rocker by a real fraction of the
        // car's height (a tucked-up wheel makes these equal).
        std::printf("    [slot %2d] height %.2f, floor %.2f, rocker %.2f, drop %.3f\n",
                    slot, height, lo, rocker, rocker - lo);
        CHECK(rocker - lo > height * 0.04);
    }
}

// THE PLAYER'S CAR RESTS WHERE IT IS DRAWN (device: "inconsistencies with the
// simulated cars versus the player's physics based car"). The drivable spec
// publishes each wheel's drawn resting centre; the host must turn that into a
// Jolt attach point lifted by the spring's settle, with the street travel
// (PhysicsWorld::kStreet*). test_driving_lab.cpp proves that rule settles a
// chassis exactly at its drawn height; this proves the shipped sedan spec
// actually arrives with it applied. Without it the car rode 0.30 m high.
TEST_CASE(drivable_spec_rests_the_wheels_where_they_are_drawn) {
    VehiclesVM v;
    CHECK(v.loaded);
    if (!v.loaded) return;
    VehicleSpec spec;
    std::string err;
    const bool ok = loadVehicleSpec(v.vm, "return vehicle.sedan(seed, {})", 7u, spec, &err);
    if (!ok) std::printf("    spec error: %s\n", err.c_str());
    CHECK(ok);
    CHECK(spec.config.wheels.size() == 4);
    const Real H = spec.config.chassisHalfExtent.y * 2.0;
    for (const PhysicsWorld::VehicleWheel& w : spec.config.wheels) {
        CHECK(std::fabs(w.suspensionMin - PhysicsWorld::kStreetSuspensionMin) < 1e-9);
        CHECK(std::fabs(w.suspensionMax - PhysicsWorld::kStreetSuspensionMax) < 1e-9);
        // attach = drawn rest centre (-H/2 + r) + the settle the spring takes back
        const Real restCentre = -H * 0.5 + w.radius;
        const Real want = restCentre + PhysicsWorld::kStreetSuspensionRestDrop;
        if (std::fabs(w.position.y - want) >= 1e-6)
            std::printf("    wheel attach y=%.4f, want %.4f (rest centre %.4f)\n",
                        w.position.y, want, restCentre);
        CHECK(std::fabs(w.position.y - want) < 1e-6);
    }
}


// THE CITY BUS (Glenn, 2026-09-18: "build an actual bus model with an interior
// and exterior and have it look like a city bus", "we should see the npcs
// sitting on the bus. There should be a driver"). The bus slot is the only one
// ALWAYS seen into (`seeInto`): its saloon is in its shell, and it publishes a
// driver's seat front-left and a saloon's worth of passenger seats with a gap on
// the kerb side where the middle door is. Since fleet v2 every slot publishes its
// GLASS as a part (drawn opaque and reflective, clear near the player) and an
// ordinary car publishes its CABIN apart and a driver's seat, but no passenger
// seats: nobody rides in ambient traffic.
TEST_CASE(fleet_bus_is_a_bus_you_can_see_into) {
    VehiclesVM a;
    CHECK(a.loaded);
    int buses = 0;
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        CHECK(!body.glass.vertices.empty() || twoWheelSlot(a.vm, slot));   // (a bike has no glass)
        if (body.className.rfind("bus", 0) != 0) {
            // an open seat (the convertible, a bike) is seen into like the bus; every other car shows its cabin only
            // near the player
            CHECK(!body.seeInto || body.openSeat);
            // fleet v2 kit bodies carry no cabin yet (the city keeps their glass opaque); mesh.car ones do
            CHECK(body.hasDriverSeat);
            CHECK(body.seats.empty());
            continue;
        }
        ++buses;
        CHECK(body.seeInto);
        // its two door leaves apart from the body, each sliding back along the side (ADR-0356)
        CHECK(body.doorLeaves.size() == 2);
        for (const auto& leaf : body.doorLeaves) CHECK(leaf.slide.z < -0.8 && !leaf.body.vertices.empty());
        CHECK(body.interior.vertices.empty());   // the saloon rides in the shell
        const Real W = body.size.x, H = body.size.y, L = body.size.z;
        CHECK(!body.glass.vertices.empty());
        CHECK(body.hasDriverSeat);
        CHECK(body.driverSeat.x < 0);            // driver on the left...
        CHECK(body.driverSeat.z > L * 0.30);     // ...at the front
        CHECK(body.seats.size() >= 24);
        std::vector<Real> kerbZ;
        for (const Vec3& s : body.seats) {
            CHECK(std::fabs(s.x) < W * 0.5);
            CHECK(std::fabs(s.z) < L * 0.5);
            // Hip height above the road: floor plus a bus seat, ~0.6-0.9 m.
            const Real hip = s.y + H * 0.5;
            CHECK(hip > 0.5 && hip < 1.0);
            if (s.x > 0.3) kerbZ.push_back(s.z);
        }
        std::sort(kerbZ.begin(), kerbZ.end());
        Real widest = 0;
        for (std::size_t i = 1; i < kerbZ.size(); ++i)
            widest = std::max(widest, kerbZ[i] - kerbZ[i - 1]);
        std::printf("    [bus] %zu passenger seats, driver at (%.2f, %.2f, %.2f), "
                    "widest kerb-side gap %.2f m\n",
                    body.seats.size(), body.driverSeat.x, body.driverSeat.y,
                    body.driverSeat.z, widest);
        CHECK(widest > 1.1);                      // the middle door
    }
    CHECK(buses == 2);   // (all seats and standing room)
}

// THE OFF-ROADER (#41): vehicle.offroad reads back as a part-time four-wheel-drive truck -- 2WD (rear) until
// the driver engages 4WD, big wheels on long, soft travel resting where they are drawn, low gearing, near-
// locking differentials, real drag -- and its lamps and driver's seat come from the kit body.
TEST_CASE(offroad_spec_is_a_part_time_4x4_on_long_travel) {
    VehiclesVM v;
    CHECK(v.loaded);
    VehicleSpec spec;
    std::string err;
    const bool ok = loadVehicleSpec(v.vm, "return vehicle.offroad(seed, {})", 3u, spec, &err);
    if (!ok) std::printf("    spec error: %s\n", err.c_str());
    CHECK(ok);
    if (!ok) return;
    const PhysicsWorld::VehicleConfig& c = spec.config;
    CHECK(spec.partTime4wd);
    CHECK_APPROX(c.frontDriveShare, 0.0, 1e-9);   // starts in 2WD (rear)
    CHECK(c.wheels.size() == 4);
    CHECK(c.dragArea > 0.0);
    CHECK(c.gearRatios.size() == 5 && c.gearRatios[0] > 3.0);
    CHECK(c.axleLimitedSlip < 1.4);     // near-locking per axle...
    CHECK(c.centerLimitedSlip >= 1e29); // ...and open between them (a fixed 50/50 split in 4WD)
    const Real H = c.chassisHalfExtent.y * 2.0;
    for (const PhysicsWorld::VehicleWheel& w : c.wheels) {
        CHECK(w.radius > 0.45);
        CHECK(w.driven);
        CHECK_APPROX(w.suspensionMax, 0.40, 1e-9);
        CHECK_APPROX(w.suspensionFrequency, 1.3, 1e-9);
        // attach = the drawn resting centre + the rig's rest drop
        CHECK_APPROX(w.position.y, -H * 0.5 + w.radius + 0.30, 1e-6);
    }
    CHECK(spec.hasDriverSeat);
    int heads = 0, tails = 0;
    for (const auto& l : spec.lights) {
        heads += l.name.rfind("headlight", 0) == 0;
        tails += l.name.rfind("taillight", 0) == 0;
    }
    CHECK(heads == 2 && tails == 2);
    // clear glass + the cabin fixed to the chassis; the rest ride its doors, hood and hatch (ADR-0352)
    int fixed = 0;
    for (const auto& part : spec.parts) fixed += part.panel < 0;
    CHECK(fixed == 2);
    CHECK(spec.panels.size() >= 6);
}

// THE CAR PICKER (Glenn: "cycle through all of them ... It should tell me if the car is 2WD or 4WD"): every
// entry in vehicle.drivable builds, and the drive the picker SHOWS is the drive the car HAS -- fwd puts all
// the torque on the front axle, rwd on the rear, awd splits it, 4wd is part-time (starts rear, switchable).
TEST_CASE(car_picker_catalogue_builds_and_says_the_true_drivetrain) {
    VehiclesVM v;
    CHECK(v.loaded);
    std::vector<DrivableEntry> cat;
    std::string err;
    CHECK(loadDrivableCatalogue(v.vm, cat, &err));
    CHECK(cat.size() >= 10);
    for (const DrivableEntry& e : cat) {
        VehicleSpec spec;
        const bool ok = loadVehicleSpec(v.vm, "return vehicle." + e.recipe + "(seed, {})", 5u, spec, &err);
        if (!ok) std::printf("    %s: %s\n", e.recipe.c_str(), err.c_str());
        CHECK(ok);
        if (!ok) continue;
        const Real share = spec.config.frontDriveShare;   // < 0: from the driven flags (all four: AWD)
        bool matches = false;
        if (e.drive == "fwd") matches = std::fabs(share - 1.0) < 1e-9 && !spec.partTime4wd;
        else if (e.drive == "rwd") matches = std::fabs(share) < 1e-9 && !spec.partTime4wd;
        else if (e.drive == "awd") matches = (share < 0 || std::fabs(share - 0.5) < 1e-9) && !spec.partTime4wd;
        else if (e.drive == "4wd") matches = spec.partTime4wd && std::fabs(share) < 1e-9;
        std::printf("    %-16s %-18s %-4s share %+.2f part-time %d  %s\n", e.recipe.c_str(), e.label.c_str(), e.drive.c_str(),
                    share, spec.partTime4wd ? 1 : 0, matches ? "ok" : "MISMATCH");
        CHECK(matches);
        CHECK(spec.config.wheels.size() >= (spec.config.twoWheeler ? 2u : 4u));   // (the bikes ride on two, ADR-0235);
        CHECK(spec.body != nullptr);
    }
    CHECK(driveLabel("4wd").find("4WD") != std::string::npos);
    CHECK(driveLabel("fwd").find("2WD") != std::string::npos);
}

// THE MIXED FLEET (Glenn: "get the new cars into island 8 along with the classic cars"). Since the classic cars
// retired (ADR-0349) vehicle.fleet_mixed is the kit fleet ordered for the island: every car and truck, the fire
// engine, then the city bus's two bodies (all seats, standing room); a level's `citysim.fleet` points vehicle.fleet at it (selectFleet, after the script runs
// -- the first cut appended the switch to the script text, after vehicles.lua's `return`, and every such level drew no
// cars), and the kit fleet stays the default.
TEST_CASE(fleet_mixed_carries_the_kit_bodies_and_its_two_buses) {
    VehiclesVM v;
    CHECK(v.loaded);
    CHECK(fleetSlotCount(v.vm) == 19);   // the default: the kit fleet (the fire engine, 2026-10-13; the motorcycle, 2026-11-04;
                                         // the off-roader, the taxi and the moped, 2026-11-06; the standing-room bus)
    std::string err;
    CHECK(!selectFleet(v.vm, "no_such_fleet", &err));   // a bad name leaves the default...
    CHECK(fleetSlotCount(v.vm) == 19);
    CHECK(selectFleet(v.vm, "fleet_mixed", &err));      // ...a good one switches it (the city bridge's call)
    const int n = fleetSlotCount(v.vm);
    std::printf("    fleet_mixed: %d slots\n", n);
    CHECK(n == 19);
    int buses = 0, sedans = 0, fireEngines = 0;
    for (int s = 0; s < n; ++s) {
        Vec3 size;
        std::string cls, e;
        CHECK(loadFleetCatalogueEntry(v.vm, s, size, cls, &e));
        const bool bike = twoWheelSlot(v.vm, s);
        CHECK(size.x > (bike ? 0.4 : 1.0) && size.z > (bike ? 1.4 : 3.0));
        if (cls.rfind("bus", 0) == 0) ++buses;
        if (cls == "fire_engine") ++fireEngines;
        if (cls == "sedan") ++sedans;
        if (s >= n - 2) CHECK(cls.rfind("bus", 0) == 0);   // the two bus bodies last
    }
    CHECK(buses == 2);
    CHECK(fireEngines == 1);
    CHECK(sedans == 3);
    CarBodyRecipe kitBody, busBody;
    CHECK(loadFleetCarBody(v.vm, 0, kitBody, &err));
    CHECK(loadFleetCarBody(v.vm, n - 1, busBody, &err));
    CHECK(!kitBody.mesh.vertices.empty() && !busBody.mesh.vertices.empty());
}

// #42 (Glenn, 2026-10-03: "I can see through the bottom of some cars and also the classic van/suv has a polygon
// missing in the roof. Not sure if that's supposed to be a sunroof"): the classic bodies' windscreen and backlight
// bands lay along the FLAT roof (SUV 0.85 m2 of roof glass, van 1.29, no forward-facing screen at all). No body
// may carry glass lying flat in its roof, and every body has a floor. (Open edges are printed, not gated: the
// classic shell cuts real apertures its inner skin closes, so they are not holes; RT_HOLE_DUMP=<slot> lists them.)
TEST_CASE(fleet_bodies_have_no_roof_glass_and_a_floor) {
    VehiclesVM v;
    CHECK(v.loaded);
    std::string err;
    CHECK(selectFleet(v.vm, "fleet_mixed", &err));
    const int n = fleetSlotCount(v.vm);
    for (int s = 0; s < n; ++s) {
        CarBodyRecipe b;
        if (!loadFleetCarBody(v.vm, s, b, &err)) continue;
        // weld by position, count each undirected edge
        std::map<std::tuple<long, long, long>, int> ids;
        auto id = [&](const Vec3& p) {
            const auto k = std::make_tuple(std::lround(p.x * 2000), std::lround(p.y * 2000), std::lround(p.z * 2000));
            auto it = ids.find(k);
            if (it != ids.end()) return it->second;
            const int nid = static_cast<int>(ids.size());
            ids[k] = nid;
            return nid;
        };
        std::map<std::pair<int, int>, int> edges;
        std::map<std::pair<int, int>, Vec3> edgeAt;
        Real top = -1e9, bottom = 1e9;
        auto add = [&](const RenderMesh& m) {
            for (const Vertex& vx : m.vertices) { top = std::max(top, vx.position.y); bottom = std::min(bottom, vx.position.y); }
            for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
                const Vec3 P[3] = {m.vertices[m.indices[i]].position, m.vertices[m.indices[i + 1]].position,
                                   m.vertices[m.indices[i + 2]].position};
                const int I[3] = {id(P[0]), id(P[1]), id(P[2])};
                for (int e = 0; e < 3; ++e) {
                    const int a = I[e], c = I[(e + 1) % 3];
                    if (a == c) continue;
                    const auto key = std::make_pair(std::min(a, c), std::max(a, c));
                    ++edges[key];
                    edgeAt[key] = (P[e] + P[(e + 1) % 3]) * 0.5;
                }
            }
        };
        add(b.chassis.vertices.empty() ? b.mesh : b.chassis);
        add(b.glass);
        int open = 0, roofOpen = 0, lowOpen = 0;
        for (const auto& [k, c] : edges) {
            if (c != 1) continue;
            ++open;
            const Real y = edgeAt[k].y;
            if (y > bottom + 0.8 * (top - bottom)) ++roofOpen;
            if (y < bottom + 0.35 * (top - bottom)) ++lowOpen;
        }
        // a floor: an up-facing triangle low in the body, inside the shell or the cabin
        bool floor = false;
        for (const RenderMesh* m : {&b.mesh, &b.interior})
            for (std::size_t i = 0; i + 2 < m->indices.size() && !floor; i += 3) {
                const Vec3 A = m->vertices[m->indices[i]].position, B = m->vertices[m->indices[i + 1]].position,
                           C = m->vertices[m->indices[i + 2]].position;
                const Vec3 nrm = cross(B - A, C - A);
                const Real y = (A.y + B.y + C.y) / 3;
                if (nrm.length() > 1e-9 && std::fabs(nrm.y) / nrm.length() > 0.8 && y > bottom + 0.15 &&
                    y < bottom + 0.45 * (top - bottom) && std::fabs((A.x + B.x + C.x) / 3) < b.size.x * 0.3)
                    floor = true;
            }
        if (std::getenv("RT_HOLE_DUMP") && std::atoi(std::getenv("RT_HOLE_DUMP")) == s)
            for (const auto& [k, c] : edges)
                if (c == 1) std::printf("      open %.3f %.3f %.3f\n", edgeAt[k].x, edgeAt[k].y, edgeAt[k].z);
        // the glass by which way it faces: a flat pane on the roof is a sunroof nobody asked for
        Real gTop = 0, gFront = 0, gRear = 0, gSide = 0;
        for (std::size_t i = 0; i + 2 < b.glass.indices.size(); i += 3) {
            const Vec3 A = b.glass.vertices[b.glass.indices[i]].position, B = b.glass.vertices[b.glass.indices[i + 1]].position,
                       C = b.glass.vertices[b.glass.indices[i + 2]].position;
            Vec3 nrm = cross(B - A, C - A);
            const Real area = nrm.length() * 0.5;
            if (area < 1e-9) continue;
            nrm = nrm * (1.0 / (2 * area));
            if (std::fabs(nrm.y) > 0.9) gTop += area;
            else if (nrm.z > 0.3) gFront += area;
            else if (nrm.z < -0.3) gRear += area;
            else gSide += area;
        }
        std::printf("      glass m2 (both faces): flat %.2f  front %.2f  rear %.2f  side %.2f\n", gTop, gFront, gRear, gSide);
        CHECK(gTop < 0.3);                 // no sunroof (a raked pane near its top edge may tip past 26 deg)
        CHECK(gFront + gRear > 0.3 || b.glass.vertices.empty());   // and the screens are still there (a bike has none)
        std::printf("    slot %2d %-10s open edges %4d (roof %3d, low %3d)  floor %s  interior %s  see-into %d\n", s,
                    b.className.c_str(), open, roofOpen, lowOpen, floor ? "yes" : "NO", b.interior.vertices.empty() ? "-" : "yes",
                    b.seeInto ? 1 : 0);
        CHECK(floor);
    }
}

// A back seat is the model's own bench (rear_hip), not a guess (Glenn, in a cab: "He's poking out the back of the
// car" -- the city put a back seat 0.85 m behind the driver of a model with no bench, in the boot). Where a body
// has one it is behind the driver, inside the body, and a car's seated passenger there keeps their head under the
// roof. (The kit sedan has none: 1.6 m from the windscreen's foot to the rear window's, so a bench 0.85 m behind
// the front seats would be under the rear window. A cab of that model seats its rider in front.)
TEST_CASE(fleet_back_bench_is_inside_the_car) {
    for (const char* fleet : {"", "fleet_mixed"}) {
        VehiclesVM v;
        CHECK(v.loaded);
        std::string err;
        if (*fleet) CHECK(selectFleet(v.vm, fleet, &err));
        int withBench = 0;
        for (int s = 0; s < fleetSlotCount(v.vm); ++s) {
            CarBodyRecipe body;
            Vec3 size;
            std::string cls;
            if (!loadFleetCarBody(v.vm, s, body, &err) || !loadFleetCatalogueEntry(v.vm, s, size, cls, &err)) continue;
            if (!body.hasRearHip) continue;
            ++withBench;
            CHECK(body.hasDriverSeat);
            CHECK(body.rearHip.z < body.driverSeat.z - 0.5);   // behind the driver
            Real top = -1e9, back = 1e9;
            for (const Vertex& vx : body.mesh.vertices) {
                top = std::max(top, static_cast<Real>(vx.position.y));
                back = std::min(back, static_cast<Real>(vx.position.z));
            }
            CHECK(body.rearHip.z > back + 0.4);   // ...and short of the tail
            const RenderMesh rider = buildSeatedOccupant(carPassenger());
            Real head = -1e9;
            for (const Vertex& vx : rider.vertices) head = std::max(head, static_cast<Real>(vx.position.y));
            if (body.rearHip.y + head >= top) std::printf("    %s slot %d (%s): head %.2f over the top %.2f\n", fleet, s,
                                                          cls.c_str(), body.rearHip.y + head, top);
            CHECK(body.rearHip.y + head < top);   // the head under the roof
        }
        std::printf("    %s: %d slots with a back bench\n", *fleet ? fleet : "default", withBench);
        CHECK(withBench > 0);
    }
}


// ONE HANDLING BUILDER (item 23, #117): a fleet slot's `handling` IS the drivable handling of its kit -- the config a
// physical-tier traffic car and a commandeered one drive with is the one the player gets dropping the same car from
// the menu. The kit slots match field for field, wheels and all; every slot carries a whole car: four or more wheels on the ground datum, mass, torque and brakes.
TEST_CASE(every_fleet_slot_drives_with_its_kit_s_drivable_handling) {
    VehiclesVM v;
    CHECK(v.loaded);
    if (!v.loaded) return;
    for (const char* fleet : {"fleet_kit", "fleet_mixed"}) {
        std::string err;
        CHECK(selectFleet(v.vm, fleet, &err));
        const int slots = fleetSlotCount(v.vm);
        CHECK(slots > 0);
        int matched = 0;
        for (int i = 0; i < slots; ++i) {
            VehicleSpec h;
            const bool ok = loadFleetHandling(v.vm, i, h, &err);
            if (!ok) std::printf("    %s[%d]: %s\n", fleet, i, err.c_str());
            CHECK(ok);
            if (!ok) continue;
            const PhysicsWorld::VehicleConfig& c = h.config;
            CHECK(c.wheels.size() >= (c.twoWheeler ? 2u : 4u));
            CHECK(c.mass > (c.twoWheeler ? 100.0 : 500.0));   // (a 220 kg motorcycle)
            CHECK(c.engineTorque > 0.3 * c.mass);
            CHECK(c.brakeTorque > c.mass);
            for (const PhysicsWorld::VehicleWheel& w : c.wheels)   // attach = drawn rest centre + the rig's drop
                CHECK_APPROX(w.position.y, -c.chassisHalfExtent.y + w.radius + h.suspensionRestDrop, 1e-6);
            // the slot's own drivable recipe (vehicles.lua catalogue `drivable`): kit_<kit>
            const std::string slotLua = "vehicle.fleet[" + std::to_string(i + 1) + "]";
            bool kitBody = false;   // (a classic slot's drawn wheels sit half a tyre inside its drivable spec's)
            CHECK(v.vm.doString("__kit_body = (" + slotLua + ".drivable or ''):sub(1, 4) == 'kit_'", &err));
            v.vm.getGlobalBool("__kit_body", kitBody);
            VehicleSpec d;
            if (!loadVehicleSpec(v.vm, "local s = " + slotLua + "; if not s.drivable then return nil end; "
                                 "return vehicle[s.drivable](seed, {})", 1u, d, &err))
                continue;   // (no drivable recipe: the fire engine, the bus)
            const PhysicsWorld::VehicleConfig& k = d.config;
            ++matched;
            CHECK(c.mass == k.mass);
            CHECK(c.engineTorque == k.engineTorque);
            CHECK(c.brakeTorque == k.brakeTorque && c.handBrakeTorque == k.handBrakeTorque);
            CHECK(c.comOffsetY == k.comOffsetY && c.maxSteerDegrees == k.maxSteerDegrees);
            CHECK(c.frontDriveShare == k.frontDriveShare && c.dragArea == k.dragArea && c.lateralGrip == k.lateralGrip);
            CHECK(c.gearRatios == k.gearRatios);
            CHECK(c.chassisHalfExtent.x == k.chassisHalfExtent.x && c.chassisHalfExtent.z == k.chassisHalfExtent.z);
            CHECK(c.wheels.size() == k.wheels.size());
            if (kitBody)
            for (std::size_t w = 0; w < std::min(c.wheels.size(), k.wheels.size()); ++w) {
                CHECK(c.wheels[w].position.x == k.wheels[w].position.x && c.wheels[w].position.z == k.wheels[w].position.z);
                CHECK(c.wheels[w].radius == k.wheels[w].radius && c.wheels[w].steered == k.wheels[w].steered);
                CHECK(c.wheels[w].suspensionMax == k.wheels[w].suspensionMax);
            }
        }
        std::printf("    [handling] %s: %d slots, %d match their drivable recipe\n", fleet, slots, matched);
        CHECK(matched >= 10);
    }
    CHECK(selectFleet(v.vm, "fleet_kit"));
}

// THE MID-RANGE BODY (Glenn, 2026-11-06: "faceted lower poly cars with baked normals"): every smooth kit car in the
// fleet carries a cheaper whole car for past `mid_from` -- under two thirds of the near body's triangles (the lenses,
// grille and wheels are the same at both; the body shell itself is a quarter), its own wheels in it,
// and no bigger than the near car (it replaces it on the same instance).
TEST_CASE(every_smooth_fleet_car_carries_a_cheaper_mid_range_body) {
    VehiclesVM a;
    CHECK(a.loaded);
    int withMid = 0;
    for (int slot = 0; slot < fleetSlotCount(a.vm); ++slot) {
        CarBodyRecipe body;
        std::string err;
        CHECK(loadFleetCarBody(a.vm, slot, body, &err));
        if (twoWheelSlot(a.vm, slot) || body.className.rfind("bus", 0) == 0) continue;
        const std::size_t nearTris = body.mesh.indices.size() / 3, midTris = body.mid.indices.size() / 3;
        std::printf("    [%2d %-12s] near %zu tris, mid %zu tris from %.0f m\n", slot, body.className.c_str(), nearTris, midTris,
                    body.midFrom);
        CHECK(midTris > 0);
        CHECK(body.midFrom > 0);
        CHECK(midTris * 3 < nearTris * 2);
        double nearR = 0, midR = 0;
        for (const Vertex& v : body.mesh.vertices) nearR = std::max(nearR, static_cast<double>(v.position.length()));
        for (const Vertex& v : body.mid.vertices) midR = std::max(midR, static_cast<double>(v.position.length()));
        CHECK(midR < nearR * 1.05 + 0.05);
        if (midTris > 0) ++withMid;
    }
    CHECK(withMid > 8);
}

// THE MOVING PANELS (ADR-0352): every drivable kit car carries its doors (both sides), a hood unless the cab sits over
// the engine, and a trunk, hatch or gate (a pickup's tailgate drops); the panels and the body are the whole car (no face lost
// or doubled); and each hinge OPENS -- a door's middle swings out from the side, a lid's middle rises -- never into
// the car.
TEST_CASE(kit_cars_carry_doors_hood_and_trunk_that_open_outward) {
    VehiclesVM a;
    CHECK(a.loaded);
    std::string err;
    const bool ok = a.vm.doString(R"lua(
      local kit = require "vehicle_kit"
      local function turn(p, pivot, axis, deg)   -- Rodrigues: p about the line (pivot, axis)
        local t = math.rad(deg)
        local c, s = math.cos(t), math.sin(t)
        local v = { p[1] - pivot[1], p[2] - pivot[2], p[3] - pivot[3] }
        local k = axis
        local kv = k[1] * v[1] + k[2] * v[2] + k[3] * v[3]
        local x = { k[2] * v[3] - k[3] * v[2], k[3] * v[1] - k[1] * v[3], k[1] * v[2] - k[2] * v[1] }
        local r = {}
        for i = 1, 3 do r[i] = v[i] * c + x[i] * s + k[i] * kv * (1 - c) + pivot[i] end
        return r
      end
      local bad = {}
      local function fail(m) bad[#bad + 1] = m end
      for _, name in ipairs({ "sedan", "taxi", "hatchback", "suv", "jeep", "offroad", "convertible", "pickup", "step_van" }) do
        local P = kit.SPECS[name]()
        local B = kit.body(P)
        local have = {}
        for _, pn in ipairs(B:panels()) do have[pn] = #B:select{ panel = pn } end
        if not have.door_front_l or not have.door_front_r then fail(name .. ": no front doors") end
        if (have.door_rear_l ~= nil) ~= (have.door_rear_r ~= nil) then fail(name .. ": rear doors on one side only") end
        local total = 0
        for _, n in pairs(have) do total = total + n end
        if total ~= B:face_count() then fail(name .. ": panels + body " .. total .. " faces of " .. B:face_count()) end
        local car = kit.build(P, 0, { panels = true })
        local list = {}
        for pn, panel in pairs(car.panels or {}) do
          list[#list + 1] = pn
          local c = { 0.5 * (panel.lo[1] + panel.hi[1]), 0.5 * (panel.lo[2] + panel.hi[2]), 0.5 * (panel.lo[3] + panel.hi[3]) }
          local h = panel.hinge
          local o = turn(c, h.pivot, h.axis, h.open)
          if pn:sub(1, 5) == "door_" then
            if math.abs(o[1]) < math.abs(c[1]) + 0.15 then fail(name .. " " .. pn .. ": swings in (x " .. c[1] .. " -> " .. o[1] .. ")") end
          elseif pn == "tailgate" then
            if o[2] > c[2] - 0.10 or o[3] > c[3] - 0.10 then fail(name .. " tailgate: does not drop back") end
          elseif o[2] < c[2] + 0.10 then fail(name .. " " .. pn .. ": does not lift (y " .. c[2] .. " -> " .. o[2] .. ")") end
          local n = 0
          for _ in pairs(panel.parts) do n = n + 1 end
          if n == 0 then fail(name .. " " .. pn .. ": no mesh") end
        end
        table.sort(list)
        print("    [panels] " .. name .. ": " .. table.concat(list, " "))
      end
      if #bad > 0 then error(table.concat(bad, "; ")) end
    )lua", &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
}

// A CAR'S DAMAGE IS DATA (ADR-0355): the drivable spec carries vehicles.lua's KIT_DAMAGE -- the default, equal to the
// rule's, and a car's own overrides (the convertible folds easier; a sub-table overrides field by field).
TEST_CASE(drivable_spec_carries_its_damage_tuning) {
    VehiclesVM v;
    CHECK(v.loaded);
    std::string err;
    VehicleSpec sedan, soft;
    CHECK(loadVehicleSpec(v.vm, "return vehicle.sedan(seed, {})", 1u, sedan, &err));
    CHECK(loadVehicleSpec(v.vm, "return vehicle.kit_drivable('convertible', {}, {})", 1u, soft, &err) ||
          loadVehicleSpec(v.vm, "return vehicle.convertible(seed, {})", 1u, soft, &err));
    const DamageRule base;
    CHECK_APPROX(sedan.damage.knockFull, base.knockFull, 1e-9);
    CHECK_APPROX(sedan.damage.dentDepthMax, base.dentDepthMax, 1e-9);
    CHECK_APPROX(sedan.damage.roughMisfire, base.roughMisfire, 1e-9);
    CHECK_APPROX(soft.damage.knockFull, 34.0, 1e-9);
    CHECK_APPROX(soft.damage.dentDepthMax, base.dentDepthMax, 1e-9);   // (its other fields: the default's)
}

// A BUS DRIVEN IS A BUS YOU CAN SEE INTO (Glenn: "the bus has no interior"): the drivable bus carries its saloon as a
// part fixed to the chassis, as a car carries its cabin.
TEST_CASE(drivable_bus_carries_its_saloon) {
    VehiclesVM v;
    CHECK(v.loaded);
    std::string err;
    for (const char* bus : {"kit_bus", "kit_bus_standing"}) {
        VehicleSpec spec;
        CHECK(loadVehicleSpec(v.vm, std::string("return vehicle.") + bus + "(seed, {})", 1u, spec, &err));
        std::size_t fixedVerts = 0;
        for (const auto& part : spec.parts)
            if (part.panel < 0 && part.opacity >= 1.0f && part.mesh) fixedVerts += part.mesh->vertices.size();
        std::printf("    [bus] %s: %zu vertices of interior fixed to the chassis\n", bus, fixedVerts);
        CHECK(fixedVerts > 2000);   // the seats, the poles, the cab
    }
}

// A BUS's DOORS ARE WHERE ITS SALOON'S ARE (ADR-0356): the body's door panels (vehicle_kit.lua panel_plan, which
// copies car_interior.cpp's bus layout) stand over the saloon's own door markers, on the kerb side, and slide open
// along the side; the bus is lined inside.
TEST_CASE(bus_doors_stand_over_the_saloons_doorways) {
    VehiclesVM a;
    CHECK(a.loaded);
    std::string err;
    const bool ok = a.vm.doString(R"lua(
      local kit = require "vehicle_kit"
      for _, name in ipairs({ "bus", "bus_standing" }) do
        local car = kit.build(kit.SPECS[name](), 0, { panels = true })
        assert(car.panels and car.panels.bus_door_1 and car.panels.bus_door_2, name .. ": no bus doors")
        assert(car.doors and #car.doors >= 2, name .. ": no saloon doors")
        for i = 1, 2 do
          local p = car.panels["bus_door_" .. i]
          local d = car.doors[i]
          local mid = 0.5 * (p.lo[3] + p.hi[3])
          assert(math.abs(mid - d[3]) < 0.35, name .. " door " .. i .. ": panel at z " .. mid .. ", doorway at " .. d[3])
          assert(p.lo[1] > 0 and d[1] > 0, name .. " door " .. i .. ": not on the kerb side")
          assert(p.hinge.slide and p.hinge.slide[3] < -0.8, name .. " door " .. i .. ": does not slide along")
        end
        assert(car.parts.liner, name .. ": not lined inside")
        print(string.format("    [bus doors] %s: z %.2f / %.2f over doorways %.2f / %.2f", name,
          0.5 * (car.panels.bus_door_1.lo[3] + car.panels.bus_door_1.hi[3]), 0.5 * (car.panels.bus_door_2.lo[3] + car.panels.bus_door_2.hi[3]),
          car.doors[1][3], car.doors[2][3]))
      end
    )lua", &err);
    if (!ok) std::printf("    %s\n", err.c_str());
    CHECK(ok);
}
