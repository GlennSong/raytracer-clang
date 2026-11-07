#ifndef RAYTRACER_ENGINE_CARRY_SYSTEM_H
#define RAYTRACER_ENGINE_CARRY_SYSTEM_H

#include "../system.h"
#include "../control/control_commands.h"
#include "../scripting/carry_items.h"
#include "../world.h"
#include <vector>

namespace engine {

class PhysicsSystem;

// CARRYING, AND THE TRUNK (ADR-0358; Glenn, 2026-11-07, the vehicle plan's "trunk carry"). On foot, the interaction
// broker's E on a loose thing light enough to lift (Carryable: a panel off a car, a bin knocked off, a box) picks it up:
// its body goes and it rides in front of the player. While carried: E puts it down (a body again, where it is held),
// or -- beside a car whose cargo space is open (the trunk lid or hatch up, a pickup's bed always) -- E puts it IN, laid
// where it fits (carry_stow.h) and riding the car from then on; held E puts it down instead. An open cargo space
// offers what is in it: E takes it out. `carry_spawn [name]` puts a carry_items.lua thing down in front of the player.
// After VehicleSystem, so a stowed thing rides the car where the car was put this step.
class CarrySystem : public System {
public:
    explicit CarrySystem(PhysicsSystem& physics) : physicsSys_(physics) {}
    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;        // offers, the broker's commands, the held thing in hand
    void fixedUpdate(FrameContext& ctx) override;   // stowed things ride their car; a thing put down gets its weight

private:
    PhysicsSystem& physicsSys_;
    CarryCatalogue catalogue_;
    Entity held_;
    struct Settle { Entity e; float mass = 10; Vec3 velocity; };
    std::vector<Settle> settle_;   // put down: its weight once its body is made
    ControlCommands::Binding spawnCmd_;
    std::vector<std::string> spawnStaged_;
    void pickUp(FrameContext& ctx, Entity player, Entity e);
    void putDown(FrameContext& ctx, const Vec3& velocity);
    bool stow(FrameContext& ctx, Entity car, const Vec3& from);
    void spawn(FrameContext& ctx, const std::string& name, const Vec3& at, Real yaw);
};

}  // namespace engine

#endif
