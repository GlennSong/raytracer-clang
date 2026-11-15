#ifndef RAYTRACER_APPS_CITYSIM_SCRIPTING_VEHICLE_BODY_H
#define RAYTRACER_APPS_CITYSIM_SCRIPTING_VEHICLE_BODY_H

#include "../../../renderer/renderer.h"   // RenderMesh, Vertex, Vec3
#include "../../../engine/scripting/vehicle_spec.h"   // VehicleSpec, readVehicleHandling
#include "../../../engine/scripting/fleet_lua.h"      // the fleet's slots: count, class and size, handling
#include "../../driving/vehicle_class.h"              // driving::VehicleClass (#248)

#include <string>
#include <vector>

namespace engine {

class ScriptVM;

// A named ATTACHMENT MARKER on a car body (ADR-0065): a lamp position (front /
// rear corner), the seam where a future emissive lens entity or light actuator
// will be spawned. Parsed at load with the body; not yet rendered (owed — see
// the ADR-0065 register row in docs/decisions.md).
struct Attachment {
    std::string name;
    Vec3 pos;
};

// One wheel of a fleet car, as data: where it sits on the body (+Z forward,
// x>0 the driver's left), how big it is, and what it does. Published by the recipe so a
// PROMOTED car (ADR-0062) builds its Jolt wheels from the same numbers the
// drawn wheels were cut from — the alternative, deriving a radius and an axle
// offset from the body's bounding box, put the simulated wheels several
// centimetres outside the arches drawn for them.
struct WheelPlacement {
    Vec3 pos;
    Real radius = 0.3;
    Real width = 0.2;
    bool steered = false;
    bool driven = true;
    bool handBrake = false;
};

// The DATA a `vehicle.fleet[slot+1]` Lua recipe describes.
//
// `mesh` is the whole car — shell plus baked wheels — which is what instanced
// ambient traffic draws. `chassis` is the SAME car without the wheels, for a
// promoted car that grows real physics wheels; it is EMPTY when the recipe
// publishes no separate wheelset (a legacy boxes-only recipe), and an empty
// chassis means "this recipe cannot dress a physics car", not "the car has no
// body". `wheels` is the wheelset as data, for that car's Jolt config.
// `size` is the recipe's CATALOGUE ENTRY (x=width, y=height, z=length): the
// space this car occupies, straight from its vehicle_classes package. The sim
// adopts it, so car-following gaps, parking bays and collider proxies agree
// with the drawn body by construction. Zero when the recipe declares none — a
// legacy recipe — and the caller keeps the built-in table for that slot.
struct CarBodyRecipe {
    RenderMesh mesh;
    RenderMesh chassis;
    std::vector<WheelPlacement> wheels;
    std::vector<Attachment> lights;
    Vec3 size{0, 0, 0};
    std::string className;
    // THE GLASS, its own part for every slot. A see-into vehicle (the bus: `seeInto`) always draws it
    // clear; ordinary traffic draws it opaque and reflective, and clear only near the player (fleet v2).
    RenderMesh glass;
    // The cabin, apart from the shell, in its own matte material: ordinary traffic's drawn only near the player, a
    // see-into vehicle's (a bus's saloon, an open car's seats) always (ADR-0383).
    RenderMesh interior;
    // The MID-RANGE form (`mid`): the whole car, shell + dark glass + wheels, cheaper, drawn from `midFrom` metres out
    // (the near body, its glass and cabin stop there). Empty: the near body is drawn at every distance.
    RenderMesh mid;
    double midFrom = 0.0;
    // The FAR stand-in (`far`): the car's own silhouette in a few hundred triangles, in its colour, for the city's far
    // band and the GPU crowd. Empty: the band's two-box stand-in.
    RenderMesh far;
    bool seeInto = false;
    // `glass_opacity`: the clear glass's (a see-into or near car's), read as its absorption by mesh.frag's glass
    // filter -- a bus's tinted saloon glass is darker than a car's clear side window
    double glassOpacity = 0.30;
    bool openSeat = false;   // `open_seat`: nothing round the driver (a convertible): a crash throws them (ADR-0252)
    // Seat hip points (body-local), for drawing people IN the vehicle: the
    // driver, and one per passenger seat. Empty when the recipe has none.
    std::vector<Vec3> seats;
    std::vector<Vec3> doors;   // floor points a passenger steps off from
    // A BUS's DOOR LEAVES (ADR-0356): drawn apart from the body, each slid by `slide` (body frame) as the bus stands at
    // its stop with its doors open -- its painted leaf and its glass
    struct DoorLeaf {
        RenderMesh body, glass;
        Vec3 slide{0, 0, 0};
    };
    std::vector<DoorLeaf> doorLeaves;
    Vec3 driverSeat{0, 0, 0};
    bool hasDriverSeat = false;
    // `rear_hip`: the back bench's hip point (its middle), when the cabin has one. Without it a car has no back
    // seat (Glenn, in a cab: "He's poking out the back of the car" -- the city guessed one 0.85 m behind the driver,
    // in a car that had no bench, and sat him in the boot).
    Vec3 rearHip{0, 0, 0};
    bool hasRearHip = false;
};

// The vehicles.lua fleet reader (ADR-0065) — mirrors agent_goals' pattern: Lua
// authors DATA at load, C++ consumes it. Run assets/scripts/vehicles.lua in
// `vm` first (vm.doString) so the global `vehicle` table and its `fleet` array
// exist, then read one fleet slot — `vehicle.fleet[slot+1]` (Lua is 1-indexed),
// with `body` / `wheels` (Meshes), `wheel_layout`, `parts` (a legacy box
// composition: {pos, size, color}) and `lights` (named lamp markers) — into a
// CarBodyRecipe. Box parts are built with the same winding as the C++ addBox,
// ready for the citysim instanced renderer to upload. Load-time only by design:
// the returned meshes are plain data (no ECS, no sim state).
//
// Returns false (with `err` filled, if non-null) on a missing global, a missing
// or out-of-range slot, a recipe carrying neither `body` nor `parts`, or a
// malformed part / wheel / light — a broken recipe must fail loudly at load so
// the render bridge draws no car for that slot, never misbehaves silently.
bool loadFleetCarBody(ScriptVM& vm, int slot, CarBodyRecipe& out,
                      std::string* err = nullptr);

// (How many slots the fleet has, choosing a named fleet, a slot's class and size and its handling: the engine's fleet
// reader, engine/scripting/fleet_lua.h, included above. The roads read the fleet with it too.)

// THE SLOT AS THE DRIVING MODEL SEES IT (#248): its body as it follows a road (fleet_lua.h loadFleetSlotBody: class
// name, length, width, wheelbase, largest steering angle) and the tightest turn from that; then its `driving` block for
// the rest. No geometry is built. False (err filled) when any part is missing or malformed.
bool loadFleetDrivingClass(ScriptVM& vm, int slot, driving::VehicleClass& out, std::string* err = nullptr);

// THE SLOT AS THE PLAYER'S CAR (ADR-0355; Glenn: "Why is an SUV in the world different than mine? I can't open the
// hatch"): its `drivable` recipe (the menu's -- "kit_suv") and its paint `color`, so a car taken from the traffic is
// built as the car N drops: panels, dents, its damage tuning. False when the slot names none.
bool loadFleetDrivable(ScriptVM& vm, int slot, std::string& recipe, Vec3& color);

}  // namespace engine

#endif
