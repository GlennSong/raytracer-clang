#ifndef RAYTRACER_ENGINE_SCRIPTING_VEHICLE_SPEC_H
#define RAYTRACER_ENGINE_SCRIPTING_VEHICLE_SPEC_H

#include "../../rt_math.h"
#include "../physics/physics_world.h"   // PhysicsWorld::VehicleConfig
#include "../world.h"                    // Entity, World
#include <memory>
#include <string>
#include <vector>
#include <string>

namespace engine {

class ScriptVM;
class AssetManager;
struct RenderMesh;

// The data a `vehicle.*` Lua recipe returns (ADR-0059): a body mesh + material
// intent and the physics handling config. The Lua authoring mirrors flora/gun;
// this is the C++ side the host reads back.
struct VehicleSpec {
    std::shared_ptr<RenderMesh> body;
    Vec3 albedo{0.70, 0.12, 0.12};
    float metallic = 0.6f;
    float roughness = 0.35f;
    PhysicsWorld::VehicleConfig config;

    // Extra body meshes that need their OWN material — transparent glass, the
    // matte cabin interior — because a Renderable carries one material and glass
    // needs opacity < 1. `mesh.car` returns these as separate parts precisely so
    // the body stays opaque and the glass can be see-through; merging them into
    // `body` (the old path) forced the glass opaque and you saw a solid panel.
    struct Part {
        std::shared_ptr<RenderMesh> mesh;
        Vec3 albedo{1, 1, 1};        // white: hue rides the mesh's vertex colours
        float metallic = 0.0f;
        float roughness = 0.5f;
        float opacity = 1.0f;
        Vec3 emission{0, 0, 0};
    };
    std::vector<Part> parts;

    // Named lamp mount points from the recipe's `lights` array — the same marker
    // format citysim's traffic already uses, so one generator feeds both. Names
    // are "headlight_l/r" and "taillight_l/r"; the prefix picks the end and the
    // trailing l/r picks the indicator side. Empty means "use chassis corners".
    struct LampMarker {
        std::string name;
        Vec3 pos{0, 0, 0};
    };
    std::vector<LampMarker> lights;

    // Where the occupant's hip goes. `mesh.car` publishes this alongside the lamp
    // markers as "driver_seat"; hasDriverSeat distinguishes "at the origin" from
    // "not supplied", which would otherwise put the driver in the boot.
    Vec3 driverSeat{0, 0, 0};
    bool hasDriverSeat = false;

    // The suspension the wheels get (the street rig unless the recipe's `suspension` says otherwise) and
    // whether the drivetrain is a part-time 4x4 the driver switches (`drive = "4wd"`).
    Real suspensionMin = PhysicsWorld::kStreetSuspensionMin;
    Real suspensionMax = PhysicsWorld::kStreetSuspensionMax;
    Real suspensionFrequency = PhysicsWorld::kStreetSuspensionFrequency;
    Real suspensionDamping = PhysicsWorld::kStreetSuspensionDamping;
    Real suspensionRestDrop = PhysicsWorld::kStreetSuspensionRestDrop;
    bool partTime4wd = false;
};

// Run a `vehicle.*` recipe in `vm` (which must already have openProcgenLibrary
// AND the vehicles.lua library loaded, so the global `vehicle` table exists) and
// read the returned spec table into `out`. Sets the `seed` global first. Returns
// false (with `err` filled, if non-null) on a script error or a malformed return.
// UNVERIFIED: the Lua submodule can't be fetched here, so this hasn't compiled.
bool loadVehicleSpec(ScriptVM& vm, const std::string& recipe, uint32_t seed,
                     VehicleSpec& out, std::string* err = nullptr);

// The DRIVABLE CATALOGUE (vehicles.lua `vehicle.drivable`): what the player can pick and drop. `drive` is
// "fwd" | "rwd" | "awd" | "4wd" (part-time). The VM must have vehicles.lua loaded.
struct DrivableEntry { std::string recipe, label, drive; };
bool loadDrivableCatalogue(ScriptVM& vm, std::vector<DrivableEntry>& out, std::string* err = nullptr);
// "4WD (part-time, Z)" style text for a drive code.
std::string driveLabel(const std::string& drive);

// Spawn a drivable vehicle entity from a spec: Transform + PrevTransform, a
// Renderable (body uploaded via `assets`), and a Vehicle component — VehicleSystem
// then creates the Jolt vehicle and brings it to life. Returns the new entity.
Entity spawnVehicle(World& world, AssetManager& assets, const VehicleSpec& spec,
                    const Vec3& position, Real yawDegrees);

}  // namespace engine

#endif
