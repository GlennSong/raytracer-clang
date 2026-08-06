#ifndef RAYTRACER_APPS_CITYSIM_SCRIPTING_VEHICLE_BODY_H
#define RAYTRACER_APPS_CITYSIM_SCRIPTING_VEHICLE_BODY_H

#include "../../../renderer/renderer.h"   // RenderMesh, Vertex, Vec3

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
// x>0 right), how big it is, and what it does. Published by the recipe so a
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

// How many slots `vehicle.fleet` declares. THE FLEET'S LENGTH IS THE ASSET'S TO
// DECIDE: it used to be a C++ constant (the size of city_meshes' colour table),
// so a thirteenth slot in vehicles.lua was silently never built and a shorter
// fleet left every missing slot falling back to a box car. Returns 0 when there
// is no fleet to count (no global, not an array), which the caller reads as
// "use the built-in fleet".
int fleetSlotCount(ScriptVM& vm);

// The CATALOGUE half of a slot — its class name and its size — without building
// any geometry. A recipe may publish its body behind a `build()` function; this
// never calls it, so reading what a car MEASURES costs nothing while reading
// what it LOOKS like costs a mesh.car shell. The sim needs only the former, and
// needs it for every headless run.
//
// `size` is left zero when the slot declares none (a legacy recipe): the caller
// keeps its built-in dimensions for that slot. Returns false only for a missing
// global, a missing/out-of-range slot, or a malformed entry.
bool loadFleetCatalogueEntry(ScriptVM& vm, int slot, Vec3& size,
                             std::string& className, std::string* err = nullptr);

}  // namespace engine

#endif
