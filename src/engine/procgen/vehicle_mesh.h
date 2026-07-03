#ifndef RAYTRACER_ENGINE_PROCGEN_VEHICLE_MESH_H
#define RAYTRACER_ENGINE_PROCGEN_VEHICLE_MESH_H

#include "../../renderer/renderer.h"

namespace engine {

// Lofted, CURVED car bodies (ADR-0062). The old vehicles were compositions of
// axis-aligned boxes — every one read as a cybertruck. This sweeps a rounded
// superellipse cross-section along a style-specific ROOFLINE (nose, hood, raked
// windshield, roof, rear glass, deck) with plan-view taper, so the shell has
// real automotive curves; the greenhouse (windows) is painted into the same
// mesh as dark glass between the belt line and the roof skin, split by pillars.
// Wheels are proper cylinders. Vertex colours carry paint/glass/tyres — no
// textures needed, and one instanced mesh per style+colour as before.
//
// Conventions match the old box cars: +Z is the nose, origin at the body
// centre, `size` = (width, height, length); the shell spans roughly
// [-H/2, +H/2] vertically with the wheels reaching just below.
enum class CarShellStyle : int {
    Sedan = 0, Hatchback, SUV, Pickup, Van, BoxTruck,
};

// `withWheels` = false for callers that render wheels as separate entities
// (the player's physics wheels), true for the instanced NPC fleet.
RenderMesh buildCarShell(CarShellStyle style, const Vec3& paint, const Vec3& size,
                         bool withWheels = true);

}  // namespace engine

#endif
