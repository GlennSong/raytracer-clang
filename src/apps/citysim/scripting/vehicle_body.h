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

// The DATA a `vehicle.fleet[slot+1]` Lua recipe describes: a body mesh built
// from vertex-coloured boxes + the named light attachment markers.
struct CarBodyRecipe {
    RenderMesh mesh;
    std::vector<Attachment> lights;
};

// The vehicles.lua fleet reader (ADR-0065) — mirrors agent_goals' pattern: Lua
// authors DATA at load, C++ consumes it. Run assets/scripts/vehicles.lua in
// `vm` first (vm.doString) so the global `vehicle` table and its `fleet` array
// exist, then read one fleet slot — `vehicle.fleet[slot+1]` (Lua is 1-indexed),
// with `parts` (a box composition: {pos, size, color}) and `lights` (named lamp
// markers) — into a CarBodyRecipe. The mesh is built with the same winding as
// the C++ addBox, ready for the citysim instanced renderer to upload. Load-time
// only by design: the returned mesh is plain data (no ECS, no sim state).
//
// Returns false (with `err` filled, if non-null) on a missing global, a missing
// or out-of-range slot, an empty/missing `parts`, or a malformed part / light —
// a broken recipe must fail loudly at load so the render bridge can fall back to
// the C++ fleetCarMesh, never misbehave silently.
bool loadFleetCarBody(ScriptVM& vm, int slot, CarBodyRecipe& out,
                      std::string* err = nullptr);

}  // namespace engine

#endif
