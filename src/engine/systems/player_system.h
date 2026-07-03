#ifndef RAYTRACER_ENGINE_PLAYER_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "../camera/follow_camera_controller.h"
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

// Drives the on-foot player character and its camera. The FLY controller is
// the player's HEADING either way (CameraSystem feeds it mouse/stick look):
// first person renders from its pinned eye; third person (V — the on-foot
// camera toggle) frames the same heading from an over-the-shoulder follow rig
// (FollowCameraController::applyShoulderPreset), fly pitch tilting the rig.
// The mode is published to Settings ("playerThirdPerson") so the render-side
// body system (apps/citysim/city_player_body.*) shows/hides the player's
// person mesh without a dependency on this system.
class PlayerSystem : public System {
public:
    PlayerSystem(FlyCameraController& camera, PhysicsSystem& physics)
        : camera(camera), physicsSys(physics) {}

    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

    bool thirdPersonActive() const { return thirdPerson; }

private:
    FlyCameraController& camera;
    PhysicsSystem& physicsSys;
    FollowCameraController shoulder;   // third-person on-foot rig (shoulder preset)
    bool thirdPerson = false;
    // Fly pitch -> shoulder tilt clamps: enough to look up without dropping the
    // eye through the pavement, and down without flipping overhead.
    static constexpr Real kShoulderPitchMin = -70.0;
    static constexpr Real kShoulderPitchMax = 30.0;
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
