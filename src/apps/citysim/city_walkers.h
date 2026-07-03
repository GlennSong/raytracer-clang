#ifndef RAYTRACER_APPS_CITYSIM_CITY_WALKERS_H
#define RAYTRACER_APPS_CITYSIM_CITY_WALKERS_H

#include "../../engine/system.h"
#include "../../engine/ai/traffic_sense.h"   // KnockdownTimer
#include "../../engine/systems/physics_system.h"
#include "../../renderer/renderer.h"         // engine::MeshHandle
#include "city_render.h"

#include <unordered_map>
#include <vector>

namespace citysim {

// Physical pedestrians (ADR-0062) — the walker counterpart of CityVehicleSystem.
// Each CitySim pedestrian agent gets a REAL body: an entity with a kinematic
// character capsule (the same CharacterController + PhysicsWorld::moveCharacter
// mechanism that moves the player), driven each step toward its planner ghost.
// So walkers collide-and-slide off parked cars, poles, kerbs, the player, and
// each other's world — they can be BLOCKED and SHOVED, where the old instanced
// ghosts walked through everything.
//
// Knockdown-and-recover: a fast vehicle passing through a walker's body floors
// it (KnockdownTimer) — it lies flat for a spell, then gets up and walks on.
// The trigger is a proximity + speed test against real vehicle poses (no Jolt
// contact plumbing needed), so the policy is deterministic and tunable.
//
// The ghost tether applies to walkers too: a plan can never outrun its body.
// The render bridge cedes ped ownership (setPedsExternallyOwned) and receives
// each walker's REAL pose for the debug widgets.
//
// Viewer/editor only (needs Jolt via PhysicsSystem), not engine_core. UNVERIFIED
// on device; the knockdown timer and pose plumbing are headless-tested.
class CityWalkerSystem : public engine::System {
public:
    CityWalkerSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physics_(physics) {}

    void fixedUpdate(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

private:
    void spawnWalkers(engine::FrameContext& ctx);
    void driveWalkers(engine::FrameContext& ctx);

    struct Walker {
        engine::Entity entity;             // the character-capsule entity
        int agentId = -1;                  // the CitySim ghost that plans for it
        engine::KnockdownTimer knock;      // down-and-recover state
        engine::StuckDetector blocked;     // wants to walk, body isn't moving
        engine::Real backoff = 0;          // blocked -> stand still this long
        engine::Vec2 facing{0, 1};         // last real travel direction (render yaw)
        int outfit = 0;                    // deterministic shirt/pants/skin pick
        engine::Real stride = 0;           // walk-cycle phase (advances with speed)
    };

    CityRenderSystem& city_;
    engine::PhysicsSystem& physics_;
    std::vector<Walker> walkers_;
    // Shared pose meshes, acquired lazily per (outfit, pose): the walk cycle is
    // a handful of pre-built limb-swing poses the walkers hop between by phase.
    engine::MeshHandle poseMesh(engine::AssetManager& assets, int outfit, int pose);
    std::unordered_map<int, engine::MeshHandle> poseMeshes_;
    bool spawned_ = false;
};

}  // namespace citysim

#endif
