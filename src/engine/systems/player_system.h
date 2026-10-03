#ifndef RAYTRACER_ENGINE_PLAYER_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "../camera/follow_camera_controller.h"
#include "../physics/physics_world.h"

#include <algorithm>
#include <cmath>

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

// MOMENTUM IN THE AIR (#83, Glenn: "If I let go of space my momentum doesn't continue. I drop straight
// down."). On the ground the keys ARE the horizontal velocity; in the air the body keeps what it left the
// ground with, and the keys only nudge it -- at most `airAccel` m/s^2 toward what they ask, and nothing
// at all when none is held. Pure, for tests.
inline Vec3 airborneVelocity(const Vec3& carried, const Vec3& input, Real dt, Real airAccel = 4.0) {
    const Real inLen = std::sqrt(input.x * input.x + input.z * input.z);
    if (inLen < 1e-4) return Vec3(carried.x, 0, carried.z);
    Vec3 dv(input.x - carried.x, 0, input.z - carried.z);
    const Real dl = std::sqrt(dv.x * dv.x + dv.z * dv.z), cap = airAccel * dt;
    if (dl > cap) dv = dv * (cap / dl);
    return Vec3(carried.x + dv.x, 0, carried.z + dv.z);
}

// SWIMMING (#43). Water deeper than kSwimDepth at the feet floats the player: the eyes held just above the
// surface with a slow bob, swim pace, jump to rise, crouch to dive; let go and the water brings you back
// up. Walking out happens by itself: shallower than kWadeDepth with the bed under the feet.
struct SwimState {
    static constexpr Real kSwimDepth = 1.3;    // water above the feet that lifts you off them (chest deep)
    static constexpr Real kWadeDepth = 1.05;   // ...and shallow enough to stand again
    static constexpr Real kSpeed = 2.2;        // m/s, a steady crawl
    static constexpr Real kEyeAbove = 0.25;    // eyes this far above the surface, afloat
    bool swimming = false;
    // depth: water surface minus the feet (<= 0 on dry land); touching: the capsule is on something.
    bool update(Real depth, bool touching) {
        if (!swimming && depth > kSwimDepth) swimming = true;
        else if (swimming && depth < kWadeDepth && touching) swimming = false;
        return swimming;
    }
    // Vertical speed: toward the floating height (a buoyant spring, bobbing), or up/down on request.
    static Real verticalSpeed(Real centreY, Real floatY, bool rise, bool dive, Real t) {
        if (dive) return -1.6;
        const Real bob = 0.05 * std::sin(t * 1.7);
        const Real toward = std::clamp((floatY + bob - centreY) * 2.2, Real(-1.4), Real(1.3));
        return rise ? std::max(toward, centreY < floatY - 0.1 ? Real(1.6) : toward) : toward;
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
    // Jump: staged on the PER-FRAME press edge and consumed by one fixed step.
    // Reading the edge in fixedUpdate directly would fire twice whenever a
    // frame runs two steps (a catch-up burst), which is a double-height jump.
    bool jumpRequested = false;
    // Crouch: a held state, so it is level-triggered in fixedUpdate. The
    // STANDING capsule is captured on first sight (it is level-authored, not
    // ours to assume) and the crouched one derived from it; standing back up
    // can be REFUSED by the physics fit test, which is what keeps a player
    // under a ledge crouched instead of teleporting through it.
    bool crouched = false;
    bool standCaptured = false;
    Real standHalfHeight = 0.4;
    Real standRadius = 0.3;
    // ~0.95 m apex under the default gravity — clears a kerb and a bollard,
    // not a wall.
    static constexpr Real kJumpSpeed = 4.3;
    // Crouched capsule as a fraction of the standing half-height (the radius
    // is unchanged — shoulders don't narrow), and the pace penalty.
    static constexpr Real kCrouchHalfScale = 0.35;
    static constexpr Real kCrouchSpeedScale = 0.4;
    static constexpr Real kRunSpeedScale = 1.9;   // Shift / L3 held
    // Snap the character back to spawn (shared by the automatic safety net and
    // the manual R key; both must reset the fall tracker the same way).
    void respawn(CharacterId characterId, bool manual);

    Vec3 spawnPos{0, 0, 0};       // captured authored spawn; "respawn" returns the player here
    bool spawnCaptured = false;
    FallRespawnTracker fall;
    Vec3 lastBodyPos_{0, 0, 0};   // where physics left the player last step
    bool haveLastBodyPos_ = false;
    Vec3 airVel_{0, 0, 0};        // horizontal velocity carried through the air (airborneVelocity)
    SwimState swim_;              // afloat in deep water (#43)
    Real swimClock_ = 0;
    Real swimLevel_ = 0;          // the surface of the water the player is in
public:
    bool swimming() const { return swim_.swimming; }
private:
};

}  // namespace engine

#endif
