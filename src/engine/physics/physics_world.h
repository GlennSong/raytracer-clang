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

// A NEW touch between two bodies, captured from Jolt's ContactListener during
// update() (ADR-0071). Edge-triggered by construction (Jolt's OnContactAdded —
// persisted/resting contacts don't refire), so it is the natural trigger for
// impact reactions: sound cues, hit decals, gameplay damage. `approachSpeed`
// is the closing speed along the contact normal at the touch (m/s) — the
// severity knob.
struct ContactEvent {
    PhysicsBodyId bodyA = INVALID_PHYSICS_BODY;
    PhysicsBodyId bodyB = INVALID_PHYSICS_BODY;
    Vec3 position;       // world-space contact point
    Vec3 normal;         // world-space contact normal
    Real approachSpeed = 0;
};

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
    // Live bodies in the world (ADR-0080 gates: "body count back to
    // baseline" after an interior releases). 0 before initialize().
    int bodyCount() const;

    void setLinearVelocity(PhysicsBodyId id, const Vec3& velocity);
    Vec3 getLinearVelocity(PhysicsBodyId id) const;

    // Drive a KINEMATIC body toward a target pose over `dt` (Jolt MoveKinematic):
    // the body moves with the velocity needed to arrive, so it pushes dynamic
    // bodies and blocks characters instead of tunnelling. For sim-owned movers
    // (e.g. agent-driven cars) whose pose is decided outside the physics step.
    void moveKinematic(PhysicsBodyId id, const Vec3& position,
                       const Quat& orientation, Real dt);
    // TELEPORT a body: set the pose directly and zero its velocities — no
    // sweep, so nothing on the straight line to the target gets hit. For
    // discontinuous moves (parking a possessed car's proxy far away, respawn):
    // MoveKinematic across such a jump is a one-frame hypersonic sweep that
    // batters everything it passes through.
    void teleport(PhysicsBodyId id, const Vec3& position, const Quat& orientation);
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
    // JUMP. moveCharacter owns the vertical axis (and zeroes it on the
    // ground), so a jump cannot be a velocity the caller passes in — it is
    // staged here and consumed by the next moveCharacter, which also releases
    // the ground-stick for that step so the leap isn't sucked back down.
    // Refused (returns false) unless the character is on the ground: no
    // mid-air jumps, no double jumps, without the caller tracking state.
    bool jumpCharacter(CharacterId id, Real speed);
    // Resize the capsule (crouch / stand), keeping the FEET planted rather
    // than the centre — a crouch settles down, it doesn't sink into the floor.
    // Returns false when the new capsule would not fit where the character is
    // standing, which is what keeps a crouched player under a ledge crouched.
    bool setCharacterHeight(CharacterId id, Real halfHeight, Real radius);
    Vec3 characterPosition(CharacterId id) const;

    // Cast a ray against every body (static + dynamic). Returns true on hit
    // and fills hitPoint. Powers point-and-teleport (device: "click somewhere
    // and immediately jump there") and any future picking.
    bool castRay(const Vec3& origin, const Vec3& dirAndLength,
                 Vec3& hitPoint) const;
    Vec3 characterVelocity(CharacterId id) const;
    GroundState characterGroundState(CharacterId id) const;
    void setCharacterPosition(CharacterId id, const Vec3& position);

    // --- Wheeled vehicle (Jolt VehicleConstraint, ADR-0059) ------------------
    // A physics-driven car: a dynamic box chassis with raycast wheels, suspension,
    // steering, engine + brakes — Jolt's WheeledVehicleController, tuned arcade-
    // forgiving by the caller's VehicleConfig. All Jolt-free here; the constraint
    // lives in the .cpp. Exercised headlessly against the vendored Jolt by
    // tests/test_driving_lab.cpp — drive those gates when tuning.

    // One wheel, positioned relative to the chassis origin (chassis-local metres).
    // THE STREET-CAR SUSPENSION RULE, shared by every path that builds a car
    // from a drawn body (the drivable spec in vehicle_spec.cpp, a commandeered
    // fleet car in city_vehicles.cpp). `VehicleWheel::position` is the
    // ATTACHMENT point; Jolt hangs the wheel below it by the spring's settle,
    // which for these values is clamp(max - staticDeflection, min, max) =
    // 0.075 m, mass-independent (tests/test_driving_lab.cpp measures it). A
    // spec that publishes the DRAWN resting centre must therefore lift the
    // attach point by that drop, or the chassis parks the drop too high —
    // with the stock 0.20/0.45 travel that was 0.375 m: the player's car rode
    // jacked up with its wheels dangling while the sim's cars sat on the road
    // (device: "inconsistencies with the simulated cars versus the player's
    // physics based car").
    static constexpr Real kStreetSuspensionMin = 0.05;
    static constexpr Real kStreetSuspensionMax = 0.15;
    static constexpr Real kStreetStaticDeflection = 0.075;   // at the 1.5 Hz default
    static constexpr Real kStreetSuspensionRestDrop =
        kStreetSuspensionMax - kStreetStaticDeflection;

    struct VehicleWheel {
        Vec3 position{0, 0, 0};        // attachment point (chassis-local)
        Real radius = 0.35;
        Real width = 0.25;
        Real suspensionMin = 0.20;     // suspension travel (m)
        Real suspensionMax = 0.45;
        Real suspensionFrequency = 1.5;// spring frequency (Hz)
        Real suspensionDamping = 0.5;  // spring damping ratio
        bool steered = false;          // turns with steering input (front wheels)
        bool driven = true;            // receives engine torque
        bool handBrake = false;        // gets the handbrake (rear wheels)
    };

    struct VehicleConfig {
        Vec3 chassisHalfExtent{1.0, 0.5, 2.1};   // x=half-width, y=half-height, z=half-length
        Real mass = 1500.0;            // kg
        Real maxSteerDegrees = 30.0;
        Real engineTorque = 500.0;     // peak engine torque (Nm)
        Real maxRPM = 6000.0;
        Real brakeTorque = 1500.0;
        Real handBrakeTorque = 4000.0;
        Real friction = 1.0;           // chassis material friction
        Real comOffsetY = -0.4;        // centre-of-mass offset below the chassis centre (m);
                                       // lower = harder to roll in corners (anti-tip)
        // Metres carved off the UNDERSIDE of the collision box (the visual body
        // is untouched). The specs use the full BODY box as the chassis, and the
        // fleet layout parks each wheel one radius above that box's floor — so a
        // commandeered car's collision floor rested exactly ON the road, and a
        // 0.15 m kerb face was a wall its bumper corner hit before the wheels
        // could climb: the car stopped dead at kerbs. 0.22 lifts the collision
        // floor to bumper-lip height, comfortably over the city's 0.15 kerbs,
        // while a real wall still hits the box (see driving_lab kerb/wall gates).
        Real floorClearance = 0.22;
        std::vector<VehicleWheel> wheels;
    };

    using VehicleId = uint32_t;
    static constexpr VehicleId INVALID_VEHICLE = 0xFFFFFFFFu;

    // Create a vehicle at the given pose; returns INVALID_VEHICLE on failure.
    VehicleId addVehicle(const VehicleConfig& config, const Vec3& position,
                         const Quat& orientation);
    void removeVehicle(VehicleId id);
    // Driver input, each in [-1,1] (brake/handBrake in [0,1]): forward (throttle,
    // negative = reverse), right (steering), brake, handBrake. Wakes the chassis.
    void setVehicleInput(VehicleId id, Real forward, Real right, Real brake,
                         Real handBrake = 0.0);
    // Un-flip a rolled vehicle: keep its heading, level out pitch/roll, lift it a
    // little, and zero its velocity. Drives a "recover" button.
    void resetVehicleUpright(VehicleId id);
    Vec3 vehiclePosition(VehicleId id) const;
    Quat vehicleOrientation(VehicleId id) const;
    Vec3 vehicleVelocity(VehicleId id) const;
    int vehicleWheelCount(VehicleId id) const;
    // World transform of wheel `wheel` (for rendering a wheel mesh).
    Mat4 wheelTransform(VehicleId id, int wheel) const;
    // The chassis rigid-body handle (so the body's transform can be read like any
    // other, and the player can be parented/seated relative to it).
    PhysicsBodyId vehicleBody(VehicleId id) const;

    // Contacts recorded since the last drain (thread-safe internally: Jolt's
    // ContactListener fires on physics worker threads mid-step). Call after
    // update(); PhysicsSystem maps these to entities and publishes them on the
    // EventBus (ADR-0071).
    std::vector<ContactEvent> drainContactEvents();

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
