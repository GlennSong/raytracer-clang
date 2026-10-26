#ifndef RAYTRACER_ENGINE_PLAYER_BODY_RULES_H
#define RAYTRACER_ENGINE_PLAYER_BODY_RULES_H

// The player body's small pure rules, each testable alone: the net under a fall, momentum in the air, when the
// water floats you, a ladder's pace. (Moved out of systems/player_system.h with the body modes, ADR-0246.)

#include "../../rt_math.h"

#include <algorithm>
#include <cmath>

namespace engine {

// Decides when a falling character should snap back to spawn. Pure logic,
// kept apart from PlayerBody so it is testable without Jolt or a renderer.
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
    // Out of it too with the feet well ABOVE the surface, touching nothing: teleported away, flung out on a rope
    // (a swimmer's feet hang ~1.3 m below it). It was only ever left wading: teleported up from the pool, the player
    // stayed "swimming" in the air, and jump rose instead of opening the chute.
    static constexpr Real kLeftAbove = 1.0;
    bool update(Real depth, bool touching) {
        if (!swimming && depth > kSwimDepth) swimming = true;
        else if (swimming && depth < kWadeDepth && touching) swimming = false;
        else if (swimming && depth <= -kLeftAbove) swimming = false;   // (no water under: depth -1)
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

// THE ONE IMPACT RULE (ADR-0247): what a step LOST of the speed the body meant to have, along the way it meant to
// go -- a landing loses the fall, a wall the run into it, a slam the swing. Speed kept along the way, or turned
// aside (a skim along the ground, a shove off a corner), is not lost. Every mode reports its own meant and got to
// PlayerBody::impact, and past the mode's tolerance the body goes limp.
inline Real speedLost(const Vec3& meant, const Vec3& got) {
    const Real m = std::sqrt(meant.x * meant.x + meant.y * meant.y + meant.z * meant.z);
    if (m < 1e-6) return 0;
    const Real kept = (got.x * meant.x + got.y * meant.y + got.z * meant.z) / m;
    return m - std::max(Real(0), kept);
}

// CLIMBING a ladder (engine/ladder.h): a steady climb, quicker with run held -- a fixed ladder's pace, about a
// rung (0.3 m) every quarter second.
struct ClimbSpeed {
    static constexpr Real kClimb = 1.1;   // m/s
    static constexpr Real kRun = 1.7;
};

}  // namespace engine

#endif
