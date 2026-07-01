#include "player_system.h"
#include "physics_system.h"
#include "../components.h"

namespace engine {

void PlayerSystem::onStart(FrameContext& ctx) {
    camera.positionLocked = true;
    ctx.actions.bindButton("player_respawn", KeyCode::R);   // fell off the level? snap back to spawn
}

void PlayerSystem::fixedUpdate(FrameContext& ctx) {
    Real dt = ctx.clock.fixedStep();

    ctx.world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform& t, CharacterController& cc, ControlledBy&) {
            playerEntity = e;
            if (cc.characterId == INVALID_CHARACTER) return;
            if (!spawnCaptured) { spawnPos = t.position; spawnCaptured = true; }   // authored spawn

            // Detached freecam (cam_detach): the movement axes drive the fly
            // camera, not the player — otherwise WASD would walk the player while
            // you fly. Pass zero intent so the character still settles under
            // gravity but holds its ground.
            Vec3 desired;
            if (camera.positionLocked) {
                Real forward = ctx.actions.axis("cam_forward");
                Real right = ctx.actions.axis("cam_right");

                Vec3 camForward = camera.forward();
                camForward.y = 0;
                camForward = normalize(camForward);
                Vec3 camRight = camera.right();

                Vec3 moveDir = camForward * forward + camRight * right;
                Real len = moveDir.length();
                if (len > 1.0) moveDir = moveDir / len;
                desired = moveDir * moveSpeed;
            }

            physicsSys.physicsWorld().moveCharacter(cc.characterId, desired, dt);
            t.position = physicsSys.physicsWorld().characterPosition(cc.characterId);

            // Auto-respawn safety net: if the character falls far below solid
            // ground — off a ledge, or straight through a level whose ground under
            // the spawn has no collider — snap it back so starting a level never
            // dead-ends in an endless fall. (Manual respawn on R is unchanged.)
            //
            // Anchor the "too far down" test on the last grounded height, not the
            // spawn: the default player is dropped in from well above the level, so
            // that first settle is a legitimate long fall. Until the character has
            // touched ground once, allow a big drop (past the ~200 m spawn height);
            // afterwards a much smaller fall below the last footing triggers.
            GroundState gs = physicsSys.physicsWorld().characterGroundState(cc.characterId);
            if (gs == GroundState::OnGround || gs == GroundState::OnSteepGround) {
                lastSafeY = t.position.y;
                hasGrounded = true;
            }
            constexpr Real kFallAfterGrounded = 40.0;   // fell off after finding footing
            constexpr Real kFallInitialDrop   = 300.0;  // still settling from the aerial spawn
            Real ref   = hasGrounded ? lastSafeY : spawnPos.y;
            Real limit = hasGrounded ? kFallAfterGrounded : kFallInitialDrop;
            if (spawnCaptured && t.position.y < ref - limit) {
                physicsSys.physicsWorld().setCharacterPosition(cc.characterId, spawnPos);
                t.position = spawnPos;
                lastSafeY = spawnPos.y;
                hasGrounded = false;   // treat the recovery as a fresh drop
            }
        });
}

void PlayerSystem::update(FrameContext& ctx) {
    // Respawn: snap the character back to its spawn (and re-settle under gravity). Works even when
    // detached/flying, so you can always recover a player that has fallen off or through the level.
    if (ctx.actions.pressed("player_respawn") && spawnCaptured && ctx.world.alive(playerEntity)) {
        if (auto* cc = ctx.world.get<CharacterController>(playerEntity))
            if (cc->characterId != INVALID_CHARACTER) {
                physicsSys.physicsWorld().setCharacterPosition(cc->characterId, spawnPos);
                if (auto* t = ctx.world.get<Transform>(playerEntity)) t->position = spawnPos;
            }
    }

    // Detached freecam: CameraSystem's fly view stands as-is.
    if (!camera.positionLocked) return;

    if (!ctx.world.alive(playerEntity)) return;
    auto* t = ctx.world.get<Transform>(playerEntity);
    if (!t) return;

    camera.eye = t->position + Vec3(0, eyeHeight, 0);

    // A placed SceneCamera owns the view this frame; keep the eye pinned above
    // so returning to first person is seamless, but don't write over it.
    if (ctx.view.activeCameraEntity.valid()) return;

    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    ctx.view.camera = camera.cameraState(aspect);
}

}  // namespace engine
