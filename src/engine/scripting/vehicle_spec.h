#ifndef RAYTRACER_ENGINE_SCRIPTING_VEHICLE_SPEC_H
#define RAYTRACER_ENGINE_SCRIPTING_VEHICLE_SPEC_H

#include "../../rt_math.h"
#include "../physics/physics_world.h"   // PhysicsWorld::VehicleConfig
#include "../world.h"                    // Entity, World
#include <memory>
#include <string>

namespace engine {

class ScriptVM;
class AssetManager;
struct RenderMesh;

// The data a `vehicle.*` Lua recipe returns (ADR-0057): a body mesh + material
// intent and the physics handling config. The Lua authoring mirrors flora/gun;
// this is the C++ side the host reads back.
struct VehicleSpec {
    std::shared_ptr<RenderMesh> body;
    Vec3 albedo{0.70, 0.12, 0.12};
    float metallic = 0.6f;
    float roughness = 0.35f;
    PhysicsWorld::VehicleConfig config;
};

// Run a `vehicle.*` recipe in `vm` (which must already have openProcgenLibrary
// AND the vehicles.lua library loaded, so the global `vehicle` table exists) and
// read the returned spec table into `out`. Sets the `seed` global first. Returns
// false (with `err` filled, if non-null) on a script error or a malformed return.
// UNVERIFIED: the Lua submodule can't be fetched here, so this hasn't compiled.
bool loadVehicleSpec(ScriptVM& vm, const std::string& recipe, uint32_t seed,
                     VehicleSpec& out, std::string* err = nullptr);

// Spawn a drivable vehicle entity from a spec: Transform + PrevTransform, a
// Renderable (body uploaded via `assets`), and a Vehicle component — VehicleSystem
// then creates the Jolt vehicle and brings it to life. Returns the new entity.
Entity spawnVehicle(World& world, AssetManager& assets, const VehicleSpec& spec,
                    const Vec3& position, Real yawDegrees);

}  // namespace engine

#endif
