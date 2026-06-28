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
            // Seated in a vehicle (ADR-0057): VehicleSystem owns the pose and the
            // chase camera owns the view, so on-foot movement is suppressed.
            if (ctx.world.has<InVehicle>(e)) return;
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

    // Driving: the chase camera (CameraSystem follow) owns the view; don't pin
    // the first-person eye over it.
    if (ctx.world.alive(playerEntity) && ctx.world.has<InVehicle>(playerEntity))
        return;

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
