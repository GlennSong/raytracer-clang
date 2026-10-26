#ifndef RAYTRACER_APPS_CITYSIM_WALKER_RAGDOLLS_H
#define RAYTRACER_APPS_CITYSIM_WALKER_RAGDOLLS_H

// TOWNSPEOPLE KNOCKED FLYING (Glenn, 2026-10-24: "this might also go for agents you run into, but we wouldn't
// want them all to ragdoll because I'm sure that would be stupid expensive"). A walker a car floors becomes, for a
// few seconds, a ragdoll of the six blocks (engine/block_body.h) in its own outfit, thrown away from the car --
// one of at most kSlots at once; past that, the oldest gets up early. Only walkers with a physical body can be
// hit at all (the nearest few hundred), and only a hit costs anything: six Jolt bodies a slot.

#include "../../engine/block_body.h"
#include "../../engine/body_draw.h"
#include "../../engine/system.h"
#include "../../engine/physics/physics_world.h"
#include "../../engine/procgen/city/polygon.h"   // Vec2

#include <array>

namespace citysim {

class WalkerRagdolls {
public:
    static constexpr int kSlots = 6;

    // Throw a walker standing at `centre` (its capsule centre), facing `facing`, at `velocity`. Returns the slot
    // (the oldest is recycled when all are busy) -- or -1 without physics.
    int knock(engine::World& world, engine::AssetManager& assets, engine::PhysicsWorld& pw, int owner,
              const engine::Vec3& centre, engine::Vec2 facing, engine::Real scale, int outfit,
              const engine::Vec3& velocity);
    // Each step: draw the busy slots where their bodies are; a slot whose body has lain still long enough (or
    // has been down `maxDown` s) is freed, and its owner told where the torso lies (the walker gets up there).
    struct Up { int owner; engine::Vec3 torso; };
    std::vector<Up> step(engine::World& world, engine::PhysicsWorld& pw, engine::Real dt);
    // Is this owner down in a slot now?
    bool down(int owner) const;
    // "N down: agent A torso (x y z) speed s; ..." for `knock?`
    std::string describe(engine::PhysicsWorld& pw) const;
    // Free them all (the system stopping).
    void clear(engine::World& world, engine::PhysicsWorld& pw);

private:
    struct Slot {
        engine::PhysicsWorld::RagdollId body = engine::PhysicsWorld::INVALID_RAGDOLL;
        std::array<engine::Entity, engine::BlockBody::kParts> parts{};
        bool made = false;
        int owner = -1;
        engine::Real age = 0, still = 0, scale = 1;
    };
    std::array<Slot, kSlots> slots_;
    engine::BodyMeshes meshes_;                 // the limbs' skinned tubes, by fold (as the player's)
    engine::AssetManager* assets_ = nullptr;   // (from knock)
    void stow(engine::World& world, Slot& s);
};

}  // namespace citysim

#endif
