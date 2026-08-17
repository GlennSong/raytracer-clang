#ifndef RAYTRACER_ENGINE_SCRIPTING_SCRIPT_SYSTEM_H
#define RAYTRACER_ENGINE_SCRIPTING_SCRIPT_SYSTEM_H

#include "../system.h"
#include "script_vm.h"

namespace engine {

class World;
class InputMap;
class AssetManager;
struct CameraState;

// Drives MonoBehaviour-style scripts (ADR-0024): each frame it finds entities
// with a ScriptBehaviour, lazily loads each one's chunk into a per-entity
// instance table, calls start() once, then update(e, dt) every frame — the same
// lifecycle the engine's C++ Systems have, exposed to Lua. Owns one gameplay
// ScriptVM (effectful bindings; not the procgen sandbox).
//
// Gameplay scripting is intentionally NOT in the deterministic sandbox: scripts
// read/mutate the World. It runs in the variable-rate update() (frame logic),
// not fixedUpdate() — physics stays the deterministic fixed-step authority
// (ADR-0002/0012). Entity spawns requested from scripts are deferred and applied
// after iteration (the World::each no-structural-mutation contract, ADR-0006).
class ScriptSystem : public System {
public:
    ScriptSystem();

    void update(FrameContext& ctx) override;

    // Services the gameplay surface reads each tick. update() fills these from
    // FrameContext; tests set only what they exercise. `assets` may be null —
    // then spawned entities get physics components but no Renderable (headless).
    // `events` may be null — then sound.play errors if a script calls it.
    void setServices(InputMap* input, const CameraState* camera,
                     AssetManager* assets, EventBus* events = nullptr);

    // View mode for camera.first_person(): true when the view is the player's
    // own eyes (on foot, not the V shoulder rig, not a vehicle chase camera).
    // update() derives it each frame from Settings + InVehicle; tests and
    // headless ticks keep the default (true) unless they set it.
    void setFirstPersonView(bool firstPerson) { firstPersonView_ = firstPerson; }

    // Headless-testable core: run start/update for every ScriptBehaviour in
    // `world`, advancing by `dt`, then apply any deferred spawns. update()
    // forwards to this (like PhysicsSystem).
    void tick(World& world, double dt);

private:
    ScriptVM vm_;
    InputMap* input_ = nullptr;
    const CameraState* camera_ = nullptr;
    AssetManager* assets_ = nullptr;
    EventBus* events_ = nullptr;
    bool firstPersonView_ = true;
};

}  // namespace engine

#endif
