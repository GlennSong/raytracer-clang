#include "player_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../camera/scene_camera.h"
#include "../../log.h"

#include <algorithm>

namespace engine {

void PlayerSystem::onStart(FrameContext& ctx) {
    // cameraStartDetached boots the freecam already detached (the state the
    // cam_detach toggle reaches) so settings-driven poses frame headless
    // captures instead of the spawn point.
    camera.positionLocked = !ctx.settings.getBool("cameraStartDetached", false);
    shoulder.applyShoulderPreset();
    // Boot in FIRST person (device: "we should start default as first
    // person"). The flag is published for the body/script/shooting systems
    // and stays a live setting within the session, but the persisted value
    // no longer decides the boot camera.
    thirdPerson = false;
    ctx.settings.setBool("playerThirdPerson", false);
    ctx.actions.bindButton("player_respawn", KeyCode::R);   // fell off the level? snap back to spawn
    // FAST TRAVEL (device: "click somewhere and immediately jump there and
    // continue play"): T teleports the player to the surface point under the
    // crosshair — fly somewhere in the freecam, look at a street, press T,
    // and you are PLAYING there (the camera re-attaches automatically).
    ctx.actions.bindButton("player_teleport", KeyCode::T);
    // XR: the same action fires on a quick gaze-pinch (see update() — the
    // release edge, so a long HOLD can mean something else to the shell).
    ctx.actions.bindButton("player_teleport", XrButton::Pinch);
    // First <-> third person on foot. V is also CameraSystem's viewport-cycle
    // key; see the placed-camera guard in update().
    ctx.actions.bindButton("player_camera_toggle", KeyCode::V);
    // JUMP and CROUCH. Both keys are shared with driving actions (Space is the
    // brake, LeftControl the handbrake) — the same context gating T already
    // uses: these fire only on foot, those only while seated. The debug
    // overlay's Controls table paints such keys yellow, which is honest.
    ctx.actions.bindButton("player_jump", KeyCode::Space);
    ctx.actions.bindButton("player_jump", GamepadButton::A);
    ctx.actions.bindButton("player_crouch", KeyCode::LeftControl);
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
            // CROUCH (held). The standing capsule is whatever the level
            // authored; capture it once and derive the crouched one, so a
            // level with a taller player crouches proportionally.
            if (!standCaptured) {
                standHalfHeight = cc.halfHeight;
                standRadius = cc.radius;
                standCaptured = true;
            }
            const bool wantCrouch =
                camera.positionLocked && ctx.actions.held("player_crouch");
            if (wantCrouch != crouched) {
                const Real targetHalf =
                    wantCrouch ? standHalfHeight * kCrouchHalfScale
                               : standHalfHeight;
                // Standing up can FAIL (a ledge overhead) — the physics fit
                // test is the authority, so the player stays crouched instead
                // of popping through the ceiling.
                if (physicsSys.physicsWorld().setCharacterHeight(
                        cc.characterId, targetHalf, standRadius)) {
                    crouched = wantCrouch;
                    // Publish the live capsule: the third-person body mesh is
                    // scaled from it (apps/citysim/city_player_body), so the
                    // drawn person shrinks with the collider instead of
                    // floating over a crouched capsule.
                    cc.halfHeight = targetHalf;
                    t.position = physicsSys.physicsWorld().characterPosition(
                        cc.characterId);
                }
            }

            // JUMP: one press, one leap. Refused mid-air and while crouched by
            // the physics gate (grounded-only) and the crouch check here.
            if (jumpRequested) {
                jumpRequested = false;
                if (camera.positionLocked && !crouched)
                    physicsSys.physicsWorld().jumpCharacter(cc.characterId,
                                                            kJumpSpeed);
            }

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
                desired = moveDir * moveSpeed *
                          (crouched ? kCrouchSpeedScale : Real(1));
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
    // Point-and-teleport: ray onto whatever physics surface it hits — deck,
    // rooftop, terrain — then stand the player there and re-attach the camera
    // so play continues in place. Two triggers, one landing:
    //   - Desktop (T press): ray from the eye through the crosshair.
    //   - XR (quick pinch RELEASE, < 0.4s): the gaze ray at the pinch. The
    //     release edge — not the press — so a long hold stays free for shell
    //     gestures (the host's hold-for-menu), and cancels never teleport.
    auto teleportAlong = [&](const Vec3& origin, const Vec3& dir) {
        if (!ctx.world.alive(playerEntity)) return;
        auto* cc = ctx.world.get<CharacterController>(playerEntity);
        if (!cc || cc->characterId == INVALID_CHARACTER) return;
        Vec3 hit;
        if (!physicsSys.physicsWorld().castRay(origin, dir * 4000.0, hit)) return;
        const Vec3 stand = hit + Vec3(0, 1.4, 0);
        physicsSys.physicsWorld().setCharacterPosition(cc->characterId, stand);
        if (auto* t = ctx.world.get<Transform>(playerEntity)) {
            t->position = stand;
            if (auto* pt = ctx.world.get<PrevTransform>(playerEntity))
                pt->value = *t;   // no interpolation streak
        }
        fall.onGrounded(stand.y);   // new baseline: a teleport is
                                    // never a "fall" to respawn from
        camera.positionLocked = true;   // re-attach: back to playing
        LOG_INFO << "Teleported to (" << hit.x << ", " << hit.y
                 << ", " << hit.z << ")";
    };
    if (ctx.xr.active) {
        // Idle look-target: a faint ring where the HEAD is pointing, always
        // on — "if I pinched now, roughly there". The pinch marker below is
        // the precise gaze-driven version.
        if (!ctx.xr.pinchHeld && ctx.xr.originBaseValid) {
            const Mat4& h = ctx.xr.originFromHead;
            Vec3 headPos(h.m[0][3], h.m[1][3], h.m[2][3]);
            Vec3 headFwd(-h.m[0][2], -h.m[1][2], -h.m[2][2]);
            Vec3 hit;
            if (physicsSys.physicsWorld().castRay(ctx.xr.originBase + headPos,
                                                  headFwd * 4000.0, hit)) {
                ctx.debug.circle(hit + Vec3(0, 0.03, 0), Vec3(0, 1, 0), 0.2,
                                 Vec3(0.45, 0.55, 0.6));
            }
        }
        // While the pinch is held, show WHERE it will land: raycast the live
        // gaze and ring the hit point. The marker is the aim feedback that
        // makes blink teleport legible — pinch, sweep your gaze, release.
        if (ctx.xr.pinchHeld && ctx.xr.gazeValid && ctx.xr.originBaseValid) {
            Vec3 origin = ctx.xr.originBase + ctx.xr.gazeOrigin;
            Vec3 hit;
            if (physicsSys.physicsWorld().castRay(origin, ctx.xr.gazeDir * 4000.0,
                                                  hit)) {
                const Vec3 up(0, 1, 0);
                const Vec3 c = hit + Vec3(0, 0.03, 0);
                const Vec3 teal(0.25, 0.9, 1.0);
                ctx.debug.circle(c, up, 0.35, teal);
                ctx.debug.circle(c, up, 0.18, teal);
                ctx.debug.line(c, c + Vec3(0, 0.9, 0), teal);
            }
        }
        // Teleport on a QUICK release (the marker is aimed while holding);
        // longer holds belong to the shell (hold ≈1.2s exits to the menu),
        // and cancels never teleport.
        if (ctx.actions.released("player_teleport") && ctx.xr.pinchEnded
            && ctx.xr.pinchHeldSeconds < 0.8 && ctx.xr.gazeValid
            && ctx.xr.originBaseValid) {
            // ORIGIN-space ray → world: the base is translation-only, so the
            // direction passes through unchanged.
            teleportAlong(ctx.xr.originBase + ctx.xr.gazeOrigin, ctx.xr.gazeDir);
        }
    } else if (ctx.actions.pressed("player_teleport") &&
               !(ctx.world.alive(playerEntity) &&
                 ctx.world.has<InVehicle>(playerEntity))) {
        // T shares a key with vehicle_flip; while driving, T means "flip the
        // car upright", not "teleport the (stowed) walker down the camera ray".
        teleportAlong(camera.eye, camera.forward());
    }

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
    // Stage the jump on the frame's press edge; fixedUpdate consumes it once
    // (see player_system.h — reading the edge there would double-jump on a
    // catch-up frame that runs two steps).
    // ...but NOT while seated: Space is the brake in a car, and staging a
    // jump on every brake press would fire it the moment the player got out.
    if (ctx.actions.pressed("player_jump") &&
        !(ctx.world.alive(playerEntity) && ctx.world.has<InVehicle>(playerEntity)))
        jumpRequested = true;

    // The one-shot beside the V key (device: "how do I switch between third
    // and first person?"): `person first|third|toggle` on the socket and the
    // Teleport panel's checkbox write player.setThirdPerson (1/0; 2 =
    // toggle), consumed here — no placed-camera guard, it is explicit.
    {
        const double want = ctx.settings.getDouble("player.setThirdPerson", -1.0);
        if (want >= 0.0) {
            ctx.settings.setDouble("player.setThirdPerson", -1.0);
            thirdPerson = want >= 1.5 ? !thirdPerson : (want > 0.5);
            ctx.settings.setBool("playerThirdPerson", thirdPerson);
            LOG_INFO << "On-foot camera: "
                     << (thirdPerson ? "third person (over the shoulder)" : "first person")
                     << " (set)";
        }
    }
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
        } else {
            LOG_INFO << "On-foot camera: V cycles the level's placed cameras here — "
                        "use `person first|third` (socket) or Debug > Teleport";
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
    // Crouched, the eye rides the shorter capsule: the Transform is the
    // capsule CENTRE, which the resize already lowered, so the offset above
    // it has to shrink by the same ratio or the view would float above a
    // crouched head.
    const Real eyeScale =
        crouched && standCaptured
            ? (standHalfHeight * kCrouchHalfScale + standRadius) /
                  std::max(Real(1e-3), standHalfHeight + standRadius)
            : Real(1);
    camera.eye = t->position + Vec3(0, eyeHeight * eyeScale, 0);

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
        // Scroll dollies the shoulder arm (into its preset zoom range). Feed
        // ZOOM ONLY: the fly controller owns yaw/pitch, so a full update() would
        // double-apply the look delta on top of it — with zero look delta
        // orbitYaw += 0 is a no-op and the orbitPitch write below overrides
        // update()'s pitch clamp, leaving only the distance dolly.
        CameraInput zoom;
        zoom.zoomDelta = ctx.input.scrollDelta;
        shoulder.update(zoom, ctx.frameDelta);
        shoulder.orbitPitch =
            std::clamp(camera.pitch, kShoulderPitchMin, kShoulderPitchMax);
        shoulder.farPlane = camera.farPlane;  // fly carries the world-extent value
        ctx.view.camera = shoulder.cameraState(aspect);
    } else {
        ctx.view.camera = camera.cameraState(aspect);
    }
}

}  // namespace engine
