#ifndef RAYTRACER_ENGINE_SCRIPTING_SCRIPT_SYSTEM_H
#define RAYTRACER_ENGINE_SCRIPTING_SCRIPT_SYSTEM_H

#include "../system.h"
#include "script_vm.h"

namespace engine {

class World;

// Drives MonoBehaviour-style scripts (ADR-0024): each frame it finds entities
// with a ScriptBehaviour, lazily loads each one's chunk into a per-entity
// instance table, calls start() once, then update(e, dt) every frame — the same
// lifecycle the engine's C++ Systems have, exposed to Lua. Owns one gameplay
// ScriptVM (effectful bindings; not the procgen sandbox).
//
// Gameplay scripting is intentionally NOT in the deterministic sandbox: scripts
// read/mutate the World. It runs in the variable-rate update() (frame logic),
// not fixedUpdate() — physics stays the deterministic fixed-step authority
// (ADR-0002/0012).
class ScriptSystem : public System {
public:
    ScriptSystem();

    void update(FrameContext& ctx) override;

    // Headless-testable core: run start/update for every ScriptBehaviour in
    // `world`, advancing by `dt`. update() forwards to this (like PhysicsSystem).
    void tick(World& world, double dt);

private:
    ScriptVM vm_;
};

}  // namespace engine

#endif
