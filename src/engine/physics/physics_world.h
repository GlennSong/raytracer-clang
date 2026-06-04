#ifndef RAYTRACER_ENGINE_PHYSICS_WORLD_H
#define RAYTRACER_ENGINE_PHYSICS_WORLD_H

#include "../../rt_math.h"

#include <cstdint>
#include <memory>

// Thin wrapper that seals Jolt behind a Jolt-free interface (ADR-0012), the same
// way Window seals GLFW: no JPH:: type appears in this header, so engine/game
// code and tests depend only on our own math types. The PhysicsSystem (ECS
// integration, Step C) and tests drive bodies through this.

enum class BodyMotion { Static, Kinematic, Dynamic };

// Opaque handle to a physics body (Jolt's BodyID encoded as a uint32).
using PhysicsBodyId = uint32_t;
constexpr PhysicsBodyId INVALID_PHYSICS_BODY = 0xFFFFFFFFu;

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    bool initialize();
    void shutdown();

    // Body creation. orientation defaults to identity; static bodies are added
    // asleep, dynamic/kinematic active.
    PhysicsBodyId addBox(const Vec3& halfExtent, const Vec3& position,
                         const Quat& orientation, BodyMotion motion);
    PhysicsBodyId addSphere(Real radius, const Vec3& position,
                            const Quat& orientation, BodyMotion motion);
    void removeBody(PhysicsBodyId id);

    void setLinearVelocity(PhysicsBodyId id, const Vec3& velocity);
    Vec3 bodyPosition(PhysicsBodyId id) const;
    Quat bodyOrientation(PhysicsBodyId id) const;

    void setGravity(const Vec3& gravity);

    // Call once after the initial static bodies are added.
    void optimizeBroadPhase();

    // Advance the simulation. Driven from a fixed-timestep hook (ADR-0002).
    void update(Real deltaTime, int collisionSteps = 1);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};

#endif
