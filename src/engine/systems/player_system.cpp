#include "player_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../camera/scene_camera.h"
#include "../../log.h"

#include <algorithm>

namespace engine {

void PlayerSystem::onStart(FrameContext& ctx) {
    camera.positionLocked = true;
    shoulder.applyShoulderPreset();
    thirdPerson = ctx.settings.getBool("playerThirdPerson", false);
    ctx.actions.bindButton("player_respawn", KeyCode::R);   // fell off the level? snap back to spawn
    // First <-> third person on foot. V is also CameraSystem's viewport-cycle
    // key; see the placed-camera guard in update().
    ctx.actions.bindButton("player_camera_toggle", KeyCode::V);
}

void PlayerSystem::fixedUpdate(FrameContext& ctx) {
    Real dt = ctx.clock.fixedStep();

    ctx.world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform& t, CharacterController& cc, ControlledBy&) {
            playerEntity = e;
            // Seated in a vehicle (ADR-0059): VehicleSystem owns the pose and the
            // chase camera owns the view, so on-foot movement is suppressed.
            if (ctx.world.has<InVehicle>(e)) return;
            if (cc.characterId == INVALID_CHARACTER) return;
            if (!spawnCaptured) {                                  // authored spawn
                spawnPos = t.position;
                spawnCaptured = true;
                fall.onSpawnCaptured(t.position.y);
            }

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
            // dead-ends in an endless fall. The decision logic (initial-drop vs
            // after-footing thresholds, and giving up when respawning never finds
            // ground) lives in FallRespawnTracker — see player_system.h.
            GroundState gs = physicsSys.physicsWorld().characterGroundState(cc.characterId);
            if (gs == GroundState::OnGround || gs == GroundState::OnSteepGround)
                fall.onGrounded(t.position.y);
            if (spawnCaptured && fall.shouldRespawn(t.position.y)) {
                respawn(cc.characterId, /*manual=*/false);
                t.position = spawnPos;
            }
        });
}

void PlayerSystem::respawn(CharacterId characterId, bool manual) {
    physicsSys.physicsWorld().setCharacterPosition(characterId, spawnPos);
    fall.onRespawn(manual);
}

void PlayerSystem::update(FrameContext& ctx) {
    // Respawn: snap the character back to its spawn (and re-settle under gravity). Works even when
    // detached/flying, so you can always recover a player that has fallen off or through the level.
    if (ctx.actions.pressed("player_respawn") && spawnCaptured && ctx.world.alive(playerEntity)) {
        if (auto* cc = ctx.world.get<CharacterController>(playerEntity))
            if (cc->characterId != INVALID_CHARACTER) {
                respawn(cc->characterId, /*manual=*/true);
                if (auto* t = ctx.world.get<Transform>(playerEntity)) t->position = spawnPos;
            }
    }

    // First <-> third person on foot (V). V also cycles placed-camera
    // viewports (CameraSystem's cam_cycle_next): while any placed SceneCamera
    // exists the key keeps that meaning and this toggle stands down, so one
    // press never does two things.
    if (ctx.actions.pressed("player_camera_toggle")) {
        bool placedCameras = false;
        ctx.world.each<SceneCamera>([&](Entity, SceneCamera&) { placedCameras = true; });
        if (!placedCameras) {
            thirdPerson = !thirdPerson;
            // Published for the body system (apps/citysim/city_player_body.*):
            // it shows the player's person mesh only in third person.
            ctx.settings.setBool("playerThirdPerson", thirdPerson);
            LOG_INFO << "On-foot camera: "
                     << (thirdPerson ? "third person (over the shoulder)"
                                     : "first person");
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

    // The eye stays pinned in BOTH modes: the fly controller is the heading
    // (and other eye readers — the gun script — keep working), and switching
    // back to first person is seamless.
    camera.eye = t->position + Vec3(0, eyeHeight, 0);

    // A placed SceneCamera owns the view this frame; keep the eye pinned above
    // so returning to first person is seamless, but don't write over it.
    if (ctx.view.activeCameraEntity.valid()) return;

    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    if (thirdPerson) {
        // Over the shoulder: mouse look still steers the player heading — the
        // fly controller owns yaw/pitch exactly as in first person (CameraSystem
        // feeds it look input); the shoulder rig just frames that heading from
        // behind, fly pitch tilting the rig.
        shoulder.setTarget(t->position, camera.yaw);
        shoulder.orbitPitch =
            std::clamp(camera.pitch, kShoulderPitchMin, kShoulderPitchMax);
        ctx.view.camera = shoulder.cameraState(aspect);
    } else {
        ctx.view.camera = camera.cameraState(aspect);
    }
}

}  // namespace engine
