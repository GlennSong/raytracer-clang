#ifndef RAYTRACER_APPS_CITYSIM_CITY_MESHES_H
#define RAYTRACER_APPS_CITYSIM_CITY_MESHES_H

#include "../../renderer/renderer.h"   // RenderMesh, RenderMaterial
#include "city_sim.h"                  // VehicleType, Agent::State, fleet table
#include "traffic_signal.h"            // SignalState

namespace citysim {

// Procedural meshes + materials for the citysim render bridge: everything here
// is pure construction (vertex-coloured boxes, flat widget geometry, material
// constants) with no ECS or sim-state dependency.

// Build the fleet body mesh for slot `slot` (wraps into the fleet) — the same
// vertex-coloured car the instanced renderer uses, so a PROMOTED physical
// Vehicle (ADR-0062: the player commandeers an ambient car) looks identical to
// the instanced car it replaces. `withWheels` false for that promotion —
// VehicleSystem gives a real Vehicle physics wheels. x=width, y=height,
// z=length (travel +Z).
engine::RenderMesh fleetCarMesh(int slot, bool withWheels = true);

// A vertex-coloured car that mirrors the PLAYER's car body (vehicles.lua
// `car_body`): a low hull, a set-back greenhouse cabin, dark windshield + rear
// glass, and pale front / red rear corner lights — so NPC cars share the
// player's look. `withWheels` bakes wheels into the instanced mesh (ambient
// traffic); a PROMOTED physical car omits them — VehicleSystem gives it physics
// wheels. Body STYLE varies the hull/cabin for a mixed fleet. `size` is
// x=width, y=height, z=length (travel axis, +Z); faces +Z.
engine::RenderMesh buildCarMesh(int style, engine::Vec3 color, engine::Vec3 size,
                                bool withWheels = true);

// The mesh style that draws each body TYPE (matches buildCarMesh's switch).
int styleForType(VehicleType t);

// A body slot's mesh size (x=width, y=height, z=length), from the shared fleet
// table (city_sim kFleet) — so the drawn car is exactly the size the sim follows
// and collides at.
engine::Vec3 fleetBodySize(int slot);

// Number of car paint/body variants (one colour per fleet slot, mirroring
// city_sim kFleet slot for slot). Each variant is its own instance group (a
// group shares one mesh + material); a driver keeps slot vehicleIndex % this,
// so a given car keeps its shape, size, and colour.
int carVariantCount();

// A simple articulated PERSON (device ask: "people should look like people"):
// six vertex-coloured boxes — head, torso, two legs, two arms — centred like the
// old walker box (y in [-0.9, 0.9], facing +Z). `swing` pitches the limbs about
// hip/shoulder pivots (radians; legs opposed, arms counter-swing), so a small
// set of these meshes IS the walk cycle — walkers hop between shared pose
// meshes by walk phase, no skeleton needed. `outfit` picks a deterministic
// shirt/pants/skin combination.
engine::RenderMesh buildPersonMesh(engine::Real swing, int outfit);
int personOutfitCount();

// A ground-PROJECTED ring of unit outer radius: a flat glowing band lying just
// above the pavement, like painted road markings — always visible from any
// camera the way lane paint is (regular depth, no reliance on an overlay depth
// state). Wide enough (20% of radius) to read at street-view distance.
// Instanced with scale (radius, 1, radius).
engine::RenderMesh ringXZ(engine::Real innerFrac = 0.80, int segs = 40);

// A flat arrow along +Z from the origin to z=1 (a shaft + a head), facing +Y —
// the debug trajectory vector. Instanced with scale (width, 1, length) + yaw.
engine::RenderMesh arrowXZ();

// A flat ground strip: a unit quad along +Z (x in [-0.5, 0.5], z in [0, 1]),
// facing +Y — the debug navgraph lane line. Instanced with scale
// (width, 1, length) + yaw.
engine::RenderMesh stripXZ();

// A ground-facing OUTLINE wedge of unit radius centred on +Z: the two straight
// edges from the origin plus the arc between them, built from thin quads (band
// ~4% of radius) like ringXZ's band so it draws with the same widgetMaterial
// technique — the debug vision-cone view. The half-angle is baked into the
// mesh; instanced with scale (range, 1, range) + yaw.
engine::RenderMesh wedgeXZ(engine::Real halfAngleRad, int segs = 12);

// Materials the citysim instance groups draw with (hue mostly carried in each
// mesh's vertex colours).
engine::RenderMaterial carMaterial();
engine::RenderMaterial pedMaterial();
engine::RenderMaterial signalMaterial(SignalState s);      // emissive lit lens
engine::RenderMaterial signalPostMaterial();               // pole/arm/head assembly
engine::RenderMaterial widgetMaterial(engine::Vec3 color); // debug ground paint

// Traffic-light semantics (user-requested): GREEN = going, RED = stopped,
// AMBER = braking/avoiding, violet = turning, teal = following a leader.
engine::Vec3 stateColor(Agent::State s);

}  // namespace citysim

#endif
