#ifndef RAYTRACER_ENGINE_PHYSICS_WORLD_H
#define RAYTRACER_ENGINE_PHYSICS_WORLD_H

#include "../../rt_math.h"

#include <cstdint>
#include <memory>
#include <vector>

namespace engine {

// Thin wrapper that seals Jolt behind a Jolt-free interface (ADR-0012), the same
// way Window seals GLFW: no JPH:: type appears in this header, so engine/game
// code and tests depend only on our own math types. The PhysicsSystem (ECS
// integration, Step C) and tests drive bodies through this.

enum class BodyMotion { Static, Kinematic, Dynamic };

// Opaque handle to a physics body (Jolt's BodyID encoded as a uint32).
using PhysicsBodyId = uint32_t;
constexpr PhysicsBodyId INVALID_PHYSICS_BODY = 0xFFFFFFFFu;

// Opaque handle to a virtual character controller (an index into the world's
// character list). Distinct from PhysicsBodyId — a CharacterVirtual is a
// kinematic collide-and-slide proxy, not a simulated rigid body, so it can step
// up curbs and stairs instead of bouncing off them.
using CharacterId = uint32_t;
constexpr CharacterId INVALID_CHARACTER = 0xFFFFFFFFu;

// Mirrors Jolt's CharacterBase::EGroundState through the Jolt-free seal.
enum class GroundState { OnGround, OnSteepGround, NotSupported, InAir };

class JobSystem;   // our thread pool (src/job_system.h)

class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    // Pass a JobSystem to run Jolt's step on our shared pool (ADR-0014); leave it
    // null to run single-threaded (the ADR-0012 default — used by unit tests).
    bool initialize(engine::JobSystem* jobSystem = nullptr);
    void shutdown();

    // Body creation. orientation defaults to identity; static bodies are added
    // asleep, dynamic/kinematic active.
    PhysicsBodyId addBox(const Vec3& halfExtent, const Vec3& position,
                         const Quat& orientation, BodyMotion motion,
                         Real restitution = 0.0, Real friction = 0.2,
                         bool lockRotation = false, bool continuous = false);
    PhysicsBodyId addSphere(Real radius, const Vec3& position,
                            const Quat& orientation, BodyMotion motion,
                            Real restitution = 0.0, Real friction = 0.2,
                            bool lockRotation = false);
    PhysicsBodyId addCapsule(Real halfHeight, Real radius, const Vec3& position,
                             const Quat& orientation, BodyMotion motion,
                             Real restitution = 0.0, Real friction = 0.2,
                             bool lockRotation = false, bool continuous = false);
    // A static triangle mesh (terrain, baked geometry). vertices/indices are in
    // world space; the body is always static. Indexed triangles, 3 indices each.
    PhysicsBodyId addMesh(const std::vector<Vec3>& vertices,
                          const std::vector<uint32_t>& indices,
                          const Vec3& position, Real friction = 0.5);
    void removeBody(PhysicsBodyId id);

    void setLinearVelocity(PhysicsBodyId id, const Vec3& velocity);
    Vec3 getLinearVelocity(PhysicsBodyId id) const;
    Vec3 bodyPosition(PhysicsBodyId id) const;
    Quat bodyOrientation(PhysicsBodyId id) const;

    void setGravity(const Vec3& gravity);

    // --- Character controller (CharacterVirtual) -----------------------------
    // A capsule controller that walks the world by collide-and-slide and steps up
    // ledges up to stepHeight (curbs, stairs, low cubes) instead of being blocked
    // by them. `position` is the capsule centre, matching the Capsule collider
    // convention. Returns INVALID_CHARACTER on failure.
    CharacterId addCharacter(Real halfHeight, Real radius, const Vec3& position,
                             Real stepHeight = 0.4, Real maxSlopeDegrees = 50.0);
    void removeCharacter(CharacterId id);
    // Drive the character for one step: `velocity` is the desired horizontal
    // velocity (y is ignored — gravity and ground-sticking are applied here), dt
    // the step length. Call once per fixed update.
    void moveCharacter(CharacterId id, const Vec3& velocity, Real dt);
    Vec3 characterPosition(CharacterId id) const;
    Vec3 characterVelocity(CharacterId id) const;
    GroundState characterGroundState(CharacterId id) const;
    void setCharacterPosition(CharacterId id, const Vec3& position);

    // Call once after the initial static bodies are added.
    void optimizeBroadPhase();

    // Advance the simulation. Driven from a fixed-timestep hook (ADR-0002).
    void update(Real deltaTime, int collisionSteps = 1);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};


}  // namespace engine

#endif
