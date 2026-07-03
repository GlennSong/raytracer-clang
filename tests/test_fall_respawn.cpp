#include "test_framework.h"

#include "../src/engine/systems/player_system.h"

using namespace engine;

// FallRespawnTracker: the pure decision logic behind the auto-respawn safety
// net (PlayerSystem). These walk the exact numeric scenarios: the default
// aerial spawn settling legitimately, the fall-off-a-ledge trigger, the manual
// R-key reset, and the give-up path when a level has no ground at all.

TEST_CASE(fall_respawn_initial_settle_does_not_trigger) {
    // Default player: dropped in at y=200 over a floor at 0 (rest y=0.7). The
    // whole legitimate 199.3 m first fall must stay inside the initial-drop
    // envelope.
    FallRespawnTracker f;
    f.onSpawnCaptured(200.0);
    CHECK(!f.shouldRespawn(150.0));
    CHECK(!f.shouldRespawn(0.7));
    CHECK(!f.shouldRespawn(200.0 - FallRespawnTracker::kFallInitialDrop + 0.1));
    // Past the envelope (no collider anywhere): triggers.
    CHECK(f.shouldRespawn(200.0 - FallRespawnTracker::kFallInitialDrop - 0.1));
}

TEST_CASE(fall_respawn_after_footing_uses_small_limit) {
    FallRespawnTracker f;
    f.onSpawnCaptured(200.0);
    f.onGrounded(0.7);
    CHECK(!f.shouldRespawn(0.7 - FallRespawnTracker::kFallAfterGrounded + 0.1));
    CHECK(f.shouldRespawn(0.7 - FallRespawnTracker::kFallAfterGrounded - 0.1));
}

TEST_CASE(fall_respawn_manual_r_rearms_at_spawn) {
    // Regression: grounded high on a plateau (y=100), press R (spawn 200, real
    // ground at 0). The old code kept lastSafeY=100, so the legitimate re-settle
    // fired a spurious auto-respawn at y<60. After onRespawn the reference must
    // be the spawn again with the big initial-drop limit.
    FallRespawnTracker f;
    f.onSpawnCaptured(200.0);
    f.onGrounded(100.0);
    f.onRespawn(/*manual=*/true);
    CHECK(!f.shouldRespawn(60.0));    // mid-fall past the old plateau limit: no fire
    CHECK(!f.shouldRespawn(0.7));     // settles fine
    CHECK(f.shouldRespawn(-101.0));   // still armed past the initial-drop envelope
}

TEST_CASE(fall_respawn_gives_up_without_ground) {
    // No collider anywhere: each auto respawn re-falls. After kMaxFailedRespawns
    // consecutive failures the net disarms instead of teleport-cycling forever.
    FallRespawnTracker f;
    f.onSpawnCaptured(200.0);
    Real deep = 200.0 - FallRespawnTracker::kFallInitialDrop - 1.0;
    for (int i = 0; i < FallRespawnTracker::kMaxFailedRespawns; ++i) {
        CHECK(f.shouldRespawn(deep));
        f.onRespawn(/*manual=*/false);
    }
    CHECK(!f.shouldRespawn(deep));            // gave up
    f.onRespawn(/*manual=*/true);             // explicit R re-arms it
    CHECK(f.shouldRespawn(deep));
    // Finding footing also fully re-arms.
    f.onRespawn(/*manual=*/false);
    f.onGrounded(0.7);
    CHECK(f.failedRespawns == 0);
    CHECK(f.shouldRespawn(0.7 - FallRespawnTracker::kFallAfterGrounded - 1.0));
}
