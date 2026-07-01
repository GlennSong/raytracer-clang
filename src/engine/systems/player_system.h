#ifndef RAYTRACER_ENGINE_PLAYER_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "../physics/physics_world.h"

namespace engine {

class PhysicsSystem;

// Decides when a falling character should snap back to spawn. Pure logic,
// extracted from PlayerSystem so it is testable without Jolt or a renderer.
//
// Two regimes: before the character has ever found footing, a large drop below
// the spawn is legitimate (the default player is dropped in from ~200 m up), so
// only a fall past kFallInitialDrop triggers. Once grounded, falling
// kFallAfterGrounded below the last footing triggers. If a respawn never leads
// to new footing (the level has no collider under the spawn), the net gives up
// after kMaxFailedRespawns rather than teleport-cycling forever.
struct FallRespawnTracker {
    Real spawnY = 0;
    Real lastSafeY = 0;
    bool hasGrounded = false;      // found footing since the last (re)spawn?
    int failedRespawns = 0;        // consecutive respawns with no footing between

    static constexpr Real kFallAfterGrounded = 40.0;
    static constexpr Real kFallInitialDrop = 300.0;
    static constexpr int kMaxFailedRespawns = 2;

    void onSpawnCaptured(Real y) { spawnY = y; lastSafeY = y; }
    void onGrounded(Real y) { lastSafeY = y; hasGrounded = true; failedRespawns = 0; }
    bool shouldRespawn(Real y) const {
        if (failedRespawns >= kMaxFailedRespawns) return false;   // no ground: give up
        Real ref = hasGrounded ? lastSafeY : spawnY;
        Real limit = hasGrounded ? kFallAfterGrounded : kFallInitialDrop;
        return y < ref - limit;
    }
    // Called on BOTH the automatic and the manual (R key) respawn, so the fall
    // reference is always re-anchored at spawn. A manual respawn re-arms the
    // give-up counter (explicit user intent); an automatic one that fired
    // without intervening footing counts toward giving up.
    void onRespawn(bool manual) {
        if (manual) failedRespawns = 0;
        else if (!hasGrounded) ++failedRespawns;
        lastSafeY = spawnY;
        hasGrounded = false;
    }
};

class PlayerSystem : public System {
public:
    PlayerSystem(FlyCameraController& camera, PhysicsSystem& physics)
        : camera(camera), physicsSys(physics) {}

    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

private:
    FlyCameraController& camera;
    PhysicsSystem& physicsSys;
    Entity playerEntity;
    Real moveSpeed = 6.0;
    Real eyeHeight = 0.7;
    // Snap the character back to spawn (shared by the automatic safety net and
    // the manual R key; both must reset the fall tracker the same way).
    void respawn(CharacterId characterId, bool manual);

    Vec3 spawnPos{0, 0, 0};       // captured authored spawn; "respawn" returns the player here
    bool spawnCaptured = false;
    FallRespawnTracker fall;
};

}  // namespace engine

#endif
