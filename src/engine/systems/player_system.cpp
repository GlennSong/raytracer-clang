#include "player_system.h"
#include "underwater_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../interaction.h"   // Seated
#include "../ladder.h"        // Climbing
#include "../player_pose.h"
#include "../interact_broker.h"   // projectToScreen
#include "../block_body.h"
#include "../procgen/person.h"
#include "../camera/scene_camera.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#ifdef RT_ENABLE_IMGUI
#include <imgui.h>
#endif

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
    ctx.actions.setActionContext("player_respawn", engine::InputContext::OnFoot);
    // FAST TRAVEL (device: "click somewhere and immediately jump there and
    // continue play"): T teleports the player to the surface point under the
    // crosshair — fly somewhere in the freecam, look at a street, press T,
    // and you are PLAYING there (the camera re-attaches automatically).
    ctx.actions.bindButton("player_teleport", KeyCode::T);
    // XR: the same action fires on a quick gaze-pinch (see update() — the
    // release edge, so a long HOLD can mean something else to the shell).
    ctx.actions.bindButton("player_teleport", XrButton::Pinch);
    ctx.actions.setActionContext("player_teleport", engine::InputContext::OnFoot);
    // First <-> third person on foot. V is also CameraSystem's viewport-cycle
    // key; see the placed-camera guard in update().
    ctx.actions.bindButton("player_camera_toggle", KeyCode::V);
    // JUMP and CROUCH. Both keys are shared with driving actions (Space is the
    // brake, LeftControl the handbrake) — the same context gating T already
    // uses: these fire only on foot, those only while seated. The debug
    // overlay's Controls table paints such keys yellow, which is honest.
    ctx.actions.bindButton("player_jump", KeyCode::Space);
    ctx.actions.bindButton("player_jump", GamepadButton::A);
    ctx.actions.setActionContext("player_jump", engine::InputContext::OnFoot);
    ctx.actions.bindButton("player_crouch", KeyCode::LeftControl);
    ctx.actions.bindButton("player_crouch", GamepadButton::B);
    // RUN (Glenn, 2026-10-02: "I thought shift was run"): Shift held, or L3 (the pad convention).
    ctx.actions.bindButton("player_run", KeyCode::LeftShift);
    ctx.actions.bindButton("player_run", GamepadButton::LeftThumb);
    ctx.actions.setActionContext("player_run", engine::InputContext::OnFoot);
    ctx.actions.setActionContext("player_crouch", engine::InputContext::OnFoot);
    tune_.load(ctx.settings);   // Debug > Movement's saved numbers (movement.* in settings.json)
    ctx.actions.bindButton("slot_5", KeyCode::Num5);   // the parachute (jump in the air opens it)
    ctx.actions.bindButton("slot_6", KeyCode::Num6);   // the wingsuit (jump in the air opens it; jump again: the chute)
    ctx.actions.bindButton("slot_7", KeyCode::Num7);   // the grapple (hold fire to hang on the rope)
    ctx.actions.bindButton("slot_8", KeyCode::Num8);   // the skateboard (ridden on the ground; W kicks)
    ctx.actions.bindButton("slot_9", KeyCode::Num9);   // the scooter
    ctx.actions.bindButton("player_limp", KeyCode::G);  // go limp (the ragdoll); jump gets up
    ctx.actions.setActionContext("player_limp", engine::InputContext::OnFoot);
    stamina_ = tune_.staminaMax;
}

void PlayerSystem::fixedUpdate(FrameContext& ctx) {
    Real dt = ctx.clock.fixedStep();
    tune_.load(ctx.settings);   // live: the panel and `set movement.<key> <value>` both write the settings

    ctx.world.each<Transform, CharacterController, ControlledBy>(
        [&](Entity e, Transform& t, CharacterController& cc, ControlledBy&) {
            playerEntity = e;
            // Seated in a vehicle (ADR-0059): VehicleSystem owns the pose and the
            // chase camera owns the view, so on-foot movement is suppressed.
            if (ctx.world.has<InVehicle>(e)) return;
            // A PASSENGER (a bus): the carrier owns the position every step.
            if (ctx.world.has<Passenger>(e)) return;
            // SEATED on a piece of furniture (InteractionSystem): the body stays where it stood; the camera sits.
            if (ctx.world.has<Seated>(e)) return;
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
            // CLIMBING (engine/ladder.h; Glenn, 2026-10-13: "Can we make ladders as interactable?"): forward climbs,
            // back climbs down, no gravity; at the top the body steps off onto what the ladder lands on, at the foot
            // onto the ground in front of it; jump lets go.
            if (Climbing* cl = ctx.world.get<Climbing>(e)) {
                const LadderSpec L = cl->ladder;
                const Real feetOff = cc.halfHeight + cc.radius;
                const Real up = camera.positionLocked ? ctx.actions.axis("cam_forward") : Real(0);
                const Real speed = ctx.actions.held("player_run") ? ClimbSpeed::kRun : ClimbSpeed::kClimb;
                const Vec3 out(L.out.x, 0, L.out.y);
                auto off = [&](const Vec3& feetAt) {
                    physicsSys.physicsWorld().setCharacterPosition(cc.characterId, feetAt + Vec3(0, feetOff + 0.05, 0));
                    t.position = physicsSys.physicsWorld().characterPosition(cc.characterId);
                    fall.onGrounded(t.position.y);
                    airVel_ = Vec3(0, 0, 0);
                    ctx.world.remove<Climbing>(e);   // (another type's pool: safe inside this walk)
                };
                if (jumpRequested) {   // let go: a step back off the rungs, and down
                    jumpRequested = false;
                    off(Vec3(L.foot.x, cl->feetY, L.foot.z) + out * (cc.radius + 0.6));
                } else if (cl->feetY >= L.landY - 0.02 && up > 0.3) {
                    off(L.exit);                                                   // over the top
                } else if (cl->feetY <= L.foot.y + 0.01 && up < -0.3) {
                    off(L.foot + out * (cc.radius + 0.45));                      // down at the foot
                } else {
                    cl->feetY = std::clamp(cl->feetY + up * speed * dt, L.foot.y, L.landY);
                    const Vec3 target = Vec3(L.foot.x, cl->feetY + feetOff, L.foot.z) + out * (cc.radius + 0.14);
                    const Vec3 v = (target - t.position) * (1.0 / std::max(dt, Real(1e-4)));
                    physicsSys.physicsWorld().moveCharacterFree(cc.characterId, v, dt);
                    t.position = physicsSys.physicsWorld().characterPosition(cc.characterId);
                    fall.onGrounded(t.position.y);   // a rung is footing: never a fall
                    climbCycle_ += std::fabs(up) * speed * dt / 0.6;
                }
                move_ = Move::Walk;
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            // SWIMMING (#43): water deep enough at the feet floats the player (UnderwaterSystem knows
            // whose water is where). While afloat, crouch means dive, not a smaller capsule.
            PhysicsWorld& pw = physicsSys.physicsWorld();
            {
                const Real feetY = t.position.y - (cc.halfHeight + cc.radius);
                const UnderwaterSystem::Surface water = UnderwaterSystem::surfaceAt(ctx.world, t.position.x, t.position.z);
                const Real depth = water.kind != UnderwaterSystem::Water::None ? Real(water.level) - feetY : Real(-1);
                const GroundState g = pw.characterGroundState(cc.characterId);
                const bool was = swim_.swimming;
                swim_.update(depth, g != GroundState::InAir);
                cc.swimming = swim_.swimming;
                if (swim_.swimming != was) {
                    LOG_INFO << (swim_.swimming ? "[swim] afloat" : "[swim] standing again") << " at (" << t.position.x << ", "
                             << t.position.z << "), water depth " << depth << " m";
                    event(swim_.swimming ? "afloat" : "out of the water");
                    if (swim_.swimming) {
                        move_ = Move::Walk;   // into the water off a wall or a ledge: swim
                        swimVy_ = pw.characterVelocity(cc.characterId).y;   // the plunge carries on in
                    }
                }
                swimLevel_ = water.level;
            }
            if (jumpRequested) {   // the press is the gate's to spend (a jump, a pull-up, a lunge up a wall)
                jumpRequested = false;
                if (camera.positionLocked) gate_.wantJump(tune_);
            }
            // a TELEPORT (further than any step goes) ends whatever the body was doing: no canopy, wall or ledge
            // comes along to the new place
            if (haveLastBodyPos_ && move_ != Move::Walk && (t.position - lastBodyPos_).length() > 4.0) {
                if (ragdoll_ != PhysicsWorld::INVALID_RAGDOLL) {
                    pw.removeRagdoll(ragdoll_);
                    ragdoll_ = PhysicsWorld::INVALID_RAGDOLL;
                }
                move_ = Move::Walk;
                airVel_ = Vec3(0, 0, 0);
                event("teleported");
            }
            if (move_ == Move::Ragdoll) {
                stepRagdoll(ctx, t, cc, dt);
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            if (limpRequested_) {
                limpRequested_ = false;
                // thrown (off a bike): the velocity the thrower handed over, else the body's own
                Vec3 v = pw.characterVelocity(cc.characterId);
                const std::string given = ctx.settings.getString("player.limpVelocity", "");
                if (!given.empty()) {
                    ctx.settings.setString("player.limpVelocity", "");
                    double x = 0, y = 0, z = 0;
                    if (std::sscanf(given.c_str(), "%lf %lf %lf", &x, &y, &z) == 3) v = Vec3(x, y, z);
                }
                if (!swim_.swimming) {
                    goLimp(t, cc, v, given.empty() ? "went limp" : "thrown off");
                    publishPose(ctx, e, t, cc);
                    lastBodyPos_ = t.position;
                    haveLastBodyPos_ = true;
                    return;
                }
            }
            regrab_ = std::max(Real(0), regrab_ - dt);
            landed_ = std::max(Real(0), landed_ - dt * 3.0);
            if (move_ == Move::Mantle) {
                stepMantle(t, cc, dt);
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            // the grapple's aim, and a shot: from walking, falling or gliding
            if (gear_ == 7) aim_ = findGrappleAnchor(pw, t.position + Vec3(0, eyeHeight, 0), camera.forward(), tune_);
            else aim_.ok = false;
            if (grappleFire_) {
                grappleFire_ = false;
                if (gear_ == 7 && !swim_.swimming && move_ != Move::Grapple) tryGrapple(t, cc);
            }
            if (move_ == Move::Grapple) {
                stepGrapple(ctx, t, cc, dt);
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            if (move_ == Move::Parachute || move_ == Move::Wingsuit) {
                stepGlide(ctx, t, cc, dt);
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            // ON A BOARD: with one in hand, the ground is ridden, not walked (off it again when the hand changes)
            if (move_ == Move::Board && gear_ != 8 && gear_ != 9) {
                move_ = Move::Walk;
                event("off the board");
            }
            if (move_ == Move::Walk && (gear_ == 8 || gear_ == 9) && !swim_.swimming && !crouched &&
                pw.characterGroundState(cc.characterId) == GroundState::OnGround) {
                Vec3 h = pw.characterVelocity(cc.characterId);
                h.y = 0;
                boardSpeed_ = h.length();
                if (boardSpeed_ < 0.5) {
                    h = camera.forward();
                    h.y = 0;
                }
                boardHeading_ = h.lengthSquared() > 1e-8 ? normalize(h) : Vec3(0, 0, -1);
                kickClock_ = 0;
                move_ = Move::Board;
                event(gear_ == 8 ? "on the skateboard" : "on the scooter");
            }
            if (move_ == Move::Board) {
                stepBoard(ctx, t, cc, dt);
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            if (move_ == Move::Climb) {
                stepClimb(ctx, t, cc, dt);
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            const bool wantCrouch =
                camera.positionLocked && ctx.actions.held("player_crouch") && !swim_.swimming;
            if (wantCrouch != crouched) {
                const Real targetHalf =
                    wantCrouch ? standHalfHeight * kCrouchHalfScale
                               : standHalfHeight;
                // Standing up can FAIL (a ledge overhead) — the physics fit
                // test is the authority, so the player stays crouched instead
                // of popping through the ceiling.
                if (pw.setCharacterHeight(cc.characterId, targetHalf, standRadius)) {
                    crouched = wantCrouch;
                    // Publish the live capsule: the third-person body mesh is
                    // scaled from it (apps/citysim/city_player_body), so the
                    // drawn person shrinks with the collider instead of
                    // floating over a crouched capsule.
                    cc.halfHeight = targetHalf;
                    t.position = pw.characterPosition(cc.characterId);
                }
            }

            // A TELEPORT IS NEVER A FALL. Something moved the player further
            // than any physics step can -- a walker covers well under 1 m a
            // frame even at terminal velocity -- so this was a teleport: F's
            // fast travel, the `teleport` verb, the Teleport panel. The fall
            // tracker's baseline is still the OLD height, and without this a
            // teleport to lower ground read as a long drop, tripped the
            // auto-respawn below, and put the player back at the spawn point
            // (Glenn: "When I tap F again I go back to where I started").
            //
            // T's own teleport already did this reset (teleportAlong); the
            // Teleport SYSTEM, which F uses, never did. Detecting the jump here
            // covers every teleport path, including ones not written yet,
            // instead of relying on each to remember.
            if (haveLastBodyPos_) {
                const Real jx = t.position.x - lastBodyPos_.x;
                const Real jy = t.position.y - lastBodyPos_.y;
                const Real jz = t.position.z - lastBodyPos_.z;
                if (jx * jx + jy * jy + jz * jz > 4.0 * 4.0)
                    fall.onGrounded(t.position.y);
            }

            moveSpeed = tune_.walkSpeed;
            const bool run = camera.positionLocked && ctx.actions.held("player_run");
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
                          (crouched ? kCrouchSpeedScale : run ? tune_.runScale : Real(1));
            }

            if (swim_.swimming) {
                // afloat: the stroke's pace, the water holds you at the surface; jump rises, crouch dives
                swimClock_ += dt;
                gate_.buffered = -1;   // jump held is "rise" here, not a leap
                const Real floatY = swimLevel_ + SwimState::kEyeAbove - eyeHeight;
                const bool rise = camera.positionLocked && ctx.actions.held("player_jump");
                const bool dive = camera.positionLocked && ctx.actions.held("player_crouch");
                const Real stroking = std::min(Real(1), desired.length() / std::max(moveSpeed, Real(0.1)));
                if (stroking > 0.1)
                    strokePhase_ += dt / std::max(tune_.strokePeriod, Real(0.1)) * (run ? 1.35 : 1.0);
                const Real pace = tune_.swimSpeed * (run ? tune_.swimRunScale : Real(1)) *
                                  strokeSpeedScale(strokeSurgePhase(strokePhase_), tune_.strokeSurge);
                // the water takes the way on gradually: a dive carries in, a stroke builds speed
                const Vec3 stroke = desired * (pace / std::max(moveSpeed, Real(0.1)));
                const Real take = 1.0 - std::exp(-dt * 3.0);
                Vec3 v(airVel_.x + (stroke.x - airVel_.x) * take, 0, airVel_.z + (stroke.z - airVel_.z) * take);
                const Real bob = 0.04 * std::sin(swimClock_ * 1.7);
                swimVy_ = swimVerticalStep(swimVy_, t.position.y, floatY + bob, rise, dive, dt, tune_);
                v.y = swimVy_;
                airVel_ = Vec3(v.x, 0, v.z);   // leaving the water keeps the stroke's speed
                pw.moveCharacterFree(cc.characterId, v, dt);
                const Real yWas = t.position.y;
                t.position = pw.characterPosition(cc.characterId);
                if (dt > 0) swimVy_ = (t.position.y - yWas) / dt;   // the pool floor stops a plunge
                fall.onGrounded(t.position.y);   // floating is footing: never a fall
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }

            // THE JUMP, A PULL-UP, A GRAB (engine/locomotion.h). Facing: where the keys push, else the view.
            const GroundState before = pw.characterGroundState(cc.characterId);
            Vec3 facing(desired.x, 0, desired.z);
            const bool pushing = facing.length() > 0.5;
            if (pushing) {
                facing = normalize(facing);
            } else {
                facing = camera.forward();
                facing.y = 0;
                facing = facing.lengthSquared() > 1e-8 ? normalize(facing) : Vec3(0, 0, -1);
            }
            // A ledge in front when jump is pressed on the ground: pull up onto it rather than hop at it.
            if (camera.positionLocked && gate_.buffered > 0 && !crouched && before == GroundState::OnGround &&
                tryMantle(t, cc, facing, tune_.groundMantleMax)) {
                gate_.buffered = -1;
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            bool launched = false;
            if (camera.positionLocked && !crouched) {
                const JumpGate::Launch l =
                    gate_.take(before, pw.characterGroundNormal(cc.characterId), tune_, dt);
                if (l.go) {
                    pw.launchCharacter(cc.characterId, tune_.jumpSpeed);
                    airVel_ = Vec3(desired.x, 0, desired.z) + l.push;
                    desired = airVel_;
                    launched = true;
                    event(before == GroundState::OnGround ? "jump"
                          : before == GroundState::InAir  ? "jump (just off an edge)"
                                                          : "jump (off steep ground)");
                }
            }
            if (!launched && before == GroundState::InAir && gate_.buffered > 0 && tryDeploy(t, cc)) {
                gate_.buffered = -1;
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            if (!launched && before != GroundState::OnGround) {
                // in the air (or sliding): a ledge in reach is grabbed and pulled up onto, a climbable wall held
                if (camera.positionLocked && pushing && tryMantle(t, cc, facing, tune_.mantleMaxHeight)) {
                    publishPose(ctx, e, t, cc);
                    lastBodyPos_ = t.position;
                    haveLastBodyPos_ = true;
                    return;
                }
                if (camera.positionLocked && pushing && tune_.grabInAir && tryGrab(t, cc, facing)) {
                    publishPose(ctx, e, t, cc);
                    lastBodyPos_ = t.position;
                    haveLastBodyPos_ = true;
                    return;
                }
            }
            if (!launched && before == GroundState::OnGround && camera.positionLocked && pushing &&
                tune_.grabFromGround && tryGrab(t, cc, facing)) {
                publishPose(ctx, e, t, cc);
                lastBodyPos_ = t.position;
                haveLastBodyPos_ = true;
                return;
            }
            // In the air the keys only nudge what the body left the ground with (#83); on the
            // ground they set it, and that is what a jump or a step off a ledge carries.
            if (!launched) {
                if (before == GroundState::InAir) {
                    airVel_ = airborneVelocity(airVel_, desired, dt, tune_.airAccel);
                    desired = airVel_;
                } else {
                    airVel_ = Vec3(desired.x, 0, desired.z);
                }
            }
            const Real vyBefore = pw.characterVelocity(cc.characterId).y;
            pw.moveCharacter(cc.characterId, desired, dt);
            t.position = pw.characterPosition(cc.characterId);

            // Auto-respawn safety net: if the character falls far below solid
            // ground — off a ledge, or straight through a level whose ground under
            // the spawn has no collider — snap it back so starting a level never
            // dead-ends in an endless fall. The decision logic (initial-drop vs
            // after-footing thresholds, and giving up when respawning never finds
            // ground) lives in FallRespawnTracker — see player_system.h.
            GroundState gs = pw.characterGroundState(cc.characterId);
            gate_.observe(gs, pw.characterVelocity(cc.characterId).y, dt);
            if (gs == GroundState::OnGround || gs == GroundState::OnSteepGround)
                fall.onGrounded(t.position.y);
            if (gs == GroundState::OnGround) {
                stamina_ = std::min(tune_.staminaMax, stamina_ + tune_.staminaRegen * dt);
                if (before == GroundState::InAir && vyBefore < -2.0)
                    landed_ = std::min(Real(1), -vyBefore / 7.0);
                if (before == GroundState::InAir && vyBefore < -tune_.limpFallSpeed) {
                    char buf[64];
                    std::snprintf(buf, sizeof(buf), "landed hard at %.0f m/s", -vyBefore);
                    goLimp(t, cc, Vec3(airVel_.x, vyBefore * 0.3, airVel_.z), buf);
                    publishPose(ctx, e, t, cc);
                    lastBodyPos_ = t.position;
                    haveLastBodyPos_ = true;
                    return;
                }
            }
            if (haveLastBodyPos_) {
                const Real hx = t.position.x - lastBodyPos_.x, hz = t.position.z - lastBodyPos_.z;
                const Real hd = std::sqrt(hx * hx + hz * hz);
                if (hd < 1.0 && gs == GroundState::OnGround) stride_ += hd / 1.6;
            }
            // ...but only when there is nothing below to land on: a jump off a tower with a chute to open is a
            // fall of 40 m and more, and the net is for falling through the world, not off it
            PhysicsWorld::RayHit below;
            if (spawnCaptured && fall.shouldRespawn(t.position.y) &&
                !pw.castRayHit(t.position, Vec3(0, -5000, 0), below)) {
                respawn(cc.characterId, /*manual=*/false);
                t.position = spawnPos;
            }
            publishPose(ctx, e, t, cc);
            lastBodyPos_ = t.position;
            haveLastBodyPos_ = true;
        });
}

// --- past walking (engine/locomotion.h) --------------------------------------

void PlayerSystem::event(const std::string& what) {
    lastEvent_ = what;
    LOG_INFO << "[move] " << what;
}

bool PlayerSystem::tryMantle(Transform& t, CharacterController& cc, const Vec3& facing, Real maxHeight) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    Ledge l;
    if (!findLedge(pw, cc.characterId, t.position, cc.halfHeight, cc.radius, facing, maxHeight, tune_, l))
        return false;
    mantle_.start(t.position, l, tune_);
    move_ = Move::Mantle;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "pull up onto a ledge %.2f m above the feet", l.height);
    event(buf);
    return true;
}

bool PlayerSystem::tryGrab(Transform& t, CharacterController& cc, const Vec3& facing) {
    if (regrab_ > 0 || stamina_ <= 0) return false;
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const WallHit w = probeWall(pw, t.position + Vec3(0, cc.halfHeight * 0.5, 0), facing, cc.radius + 0.3, tune_);
    if (!w.hit || !w.climbable) return false;
    const Vec3 n = normalize(Vec3(w.normal.x, 0, w.normal.z));
    if (dot(n, facing) > -0.6) return false;   // brushing past it, not reaching for it
    climbNormal_ = w.normal;
    lunge_ = 0;
    move_ = Move::Climb;
    airVel_ = Vec3(0, 0, 0);
    event("grab the wall");
    return true;
}

void PlayerSystem::stepMantle(Transform& t, CharacterController& cc, Real dt) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    mantle_.t += dt;
    const Vec3 target = mantle_.at(mantle_.t);
    pw.moveCharacterFree(cc.characterId, (target - t.position) * (1.0 / std::max(dt, Real(1e-4))), dt);
    t.position = pw.characterPosition(cc.characterId);
    fall.onGrounded(t.position.y);   // hands on the edge: never a fall
    if ((t.position - target).length() > 0.35) {
        move_ = Move::Walk;
        airVel_ = Vec3(0, 0, 0);
        event("pull-up blocked");
    } else if (mantle_.done()) {
        move_ = Move::Walk;
        airVel_ = Vec3(0, 0, 0);
        event("up");
    }
}

void PlayerSystem::stepClimb(FrameContext& ctx, Transform& t, CharacterController& cc, Real dt) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const Real inF = camera.positionLocked ? ctx.actions.axis("cam_forward") : Real(0);
    const Real inR = camera.positionLocked ? ctx.actions.axis("cam_right") : Real(0);
    const bool run = camera.positionLocked && ctx.actions.held("player_run");
    Vec3 right, up;
    wallBasis(climbNormal_, right, up);
    Vec3 dir = up * inF + right * inR;
    const Real len = dir.length();
    if (len > 1.0) dir = dir / len;
    const bool moving = len > 0.15;
    const Vec3 away = normalize(Vec3(climbNormal_.x, 0, climbNormal_.z));
    auto letGo = [&](Real cooldown, const std::string& why) {
        move_ = Move::Walk;
        regrab_ = cooldown;
        airVel_ = away * 0.6;
        event(why);
    };
    stamina_ -= (moving ? (run ? 2.0 : 1.0) : tune_.climbIdleDrain) * dt;
    if (camera.positionLocked && ctx.actions.held("player_crouch")) return letGo(0.5, "let go of the wall");
    // JUMP on the wall: back held kicks off it; otherwise a lunge the way the keys point (up by default)
    if (gate_.buffered > 0) {
        gate_.buffered = -1;
        if (inF < -0.5) {
            letGo(0.6, "jump off the wall");
            airVel_ = away * tune_.wallKick;
            pw.launchCharacter(cc.characterId, tune_.jumpSpeed);
            gate_.sinceLaunch = 0;
            pw.moveCharacter(cc.characterId, airVel_, dt);
            t.position = pw.characterPosition(cc.characterId);
            return;
        }
        if (stamina_ > 0) {
            lunge_ = tune_.climbLungeTime;
            lungeDir_ = moving ? dir * (1.0 / std::max(len, Real(1e-3))) : up;
            stamina_ -= tune_.climbJumpCost;
            event("lunge");
        }
    }
    Vec3 v = dir * (tune_.climbSpeed * (run ? tune_.climbRunScale : Real(1)));
    if (lunge_ > 0) {
        v = lungeDir_ * tune_.climbLungeSpeed;
        lunge_ -= dt;
    }
    const bool goingUp = inF > 0.2 || (lunge_ > 0 && lungeDir_.y > 0.3);
    const Real tall = 2 * (cc.halfHeight + cc.radius) + 0.6;   // a ledge reached from the wall: hands over the head
    // THE TOP: the head clears the wall -- pull up over it
    const WallHit atHead = probeWall(pw, t.position + Vec3(0, cc.halfHeight + cc.radius * 0.5, 0), -away,
                                     cc.radius + 0.6, tune_);
    if (goingUp && !atHead.hit && tryMantle(t, cc, -away, tall)) return;
    // STICK to the wall at the chest; turn into an inside corner, reach round an outside one
    WallHit at = probeWall(pw, t.position, -away, cc.radius + 0.7, tune_);
    if (std::fabs(inR) > 0.3) {
        const Vec3 side = right * (inR > 0 ? 1.0 : -1.0);
        const WallHit across = probeWall(pw, t.position, side, cc.radius + 0.2, tune_);
        if (across.hit && across.climbable) {
            at = across;
        } else if (!at.hit || !at.climbable) {
            const Vec3 o = t.position + side * (cc.radius + 0.35) - away * (cc.radius + 0.45);
            const WallHit round = probeWall(pw, o, -side, cc.radius + 0.7, tune_);
            if (round.hit && round.climbable) at = round;
        }
    }
    if (!at.hit || !at.climbable) {
        if (tryMantle(t, cc, -away, tall)) return;
        return letGo(0.4, at.hit ? "the wall stopped being climbable" : "lost the wall");
    }
    climbNormal_ = normalize(climbNormal_ * 0.5 + at.normal * 0.5);
    const Real gap = dot(t.position - at.point, at.normal) - (cc.radius + 0.06);
    v = v - at.normal * std::clamp(gap * 8.0, Real(-3), Real(3));
    // THE FOOT: climbing down onto walkable ground
    if (inF < -0.2) {
        PhysicsWorld::RayHit g;
        if (pw.castRayHit(t.position, Vec3(0, -(cc.halfHeight + cc.radius + 0.15), 0), g) &&
            g.normal.y >= tune_.climbSteepest)
            return letGo(0.3, "down off the wall");
    }
    if (stamina_ <= 0) return letGo(1.2, "too tired to hold on");
    pw.moveCharacterFree(cc.characterId, v, dt);
    const Vec3 before = t.position;
    t.position = pw.characterPosition(cc.characterId);
    fall.onGrounded(t.position.y);   // holding on is footing
    const Vec3 moved = t.position - before;
    climbCycle_ += (moved - at.normal * dot(moved, at.normal)).length() / 0.6;
}

// --- gone limp -----------------------------------------------------------------

void PlayerSystem::goLimp(Transform& t, CharacterController& cc, const Vec3& velocity, const std::string& why) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    if (ragdoll_ != PhysicsWorld::INVALID_RAGDOLL) pw.removeRagdoll(ragdoll_);
    BodyPose pose;
    pose.position = t.position;
    pose.scale = (cc.halfHeight + cc.radius) / BlockBody::kHalfHeight;
    Vec3 face(velocity.x, 0, velocity.z);
    if (face.length() < 0.5) {
        face = camera.forward();
        face.y = 0;
    }
    face = face.lengthSquared() > 1e-8 ? normalize(face) : Vec3(0, 0, 1);
    const Real tilt = move_ == Move::Wingsuit ? 1.4 : 0.0;   // a suit goes down flat
    pose.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(face.x, face.z)) *
                       Quat::fromAxisAngle(Vec3(1, 0, 0), tilt);
    ragdoll_ = pw.addRagdoll(blockRagdoll(pose), velocity);
    if (ragdoll_ == PhysicsWorld::INVALID_RAGDOLL) return;
    move_ = Move::Ragdoll;
    limpFor_ = 0;
    stillFor_ = 0;
    gate_.buffered = -1;
    event(why);
}

void PlayerSystem::stepRagdoll(FrameContext& ctx, Transform& t, CharacterController& cc, Real dt) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    limpFor_ += dt;
    Vec3 torso;
    Quat rot;
    if (!pw.ragdollPart(ragdoll_, BlockBody::Torso, torso, rot)) {
        move_ = Move::Walk;
        return;
    }
    t.position = torso;   // the view follows the body
    stillFor_ = pw.ragdollSpeed(ragdoll_) < 0.4 ? stillFor_ + dt : 0;
    const UnderwaterSystem::Surface water = UnderwaterSystem::surfaceAt(ctx.world, torso.x, torso.z);
    const bool inWater = water.kind != UnderwaterSystem::Water::None && torso.y < water.level;
    const bool wantUp = gate_.buffered > 0 && limpFor_ > 0.8 && pw.ragdollSpeed(ragdoll_) < 2.5;
    // fell off the world while limp: the net still catches it
    PhysicsWorld::RayHit below;
    if (spawnCaptured && fall.shouldRespawn(torso.y) && !pw.castRayHit(torso, Vec3(0, -5000, 0), below)) {
        pw.removeRagdoll(ragdoll_);
        ragdoll_ = PhysicsWorld::INVALID_RAGDOLL;
        move_ = Move::Walk;
        respawn(cc.characterId, /*manual=*/false);
        t.position = spawnPos;
        return;
    }
    if (!(inWater || wantUp || stillFor_ > tune_.getUpAfter)) return;
    // UP: stand the capsule over where the body lies (on the ground under the torso), or float it in the water
    const Real lift = cc.halfHeight + cc.radius;
    Vec3 stand = torso + Vec3(0, lift, 0);
    PhysicsWorld::RayHit ground;
    if (!inWater && pw.castRayHit(torso + Vec3(0, 1.0, 0), Vec3(0, -3.0, 0), ground)) {
        // (the ray may meet the body itself: its parts are leaving the world below, so look past them)
        stand = Vec3(torso.x, ground.point.y + lift + 0.05, torso.z);
    }
    pw.removeRagdoll(ragdoll_);
    ragdoll_ = PhysicsWorld::INVALID_RAGDOLL;
    for (Real up : {0.0, 0.3, 0.7, 1.2})
        if (pw.characterFitsAt(cc.characterId, stand + Vec3(0, up, 0))) {
            stand = stand + Vec3(0, up, 0);
            break;
        }
    pw.setCharacterPosition(cc.characterId, stand);
    t.position = stand;
    fall.onGrounded(stand.y);
    airVel_ = Vec3(0, 0, 0);
    gate_.buffered = -1;
    move_ = Move::Walk;
    event(inWater ? "came up in the water" : "got up");
}

// --- boards ------------------------------------------------------------------

void PlayerSystem::stepBoard(FrameContext& ctx, Transform& t, CharacterController& cc, Real dt) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const Real inF = camera.positionLocked ? ctx.actions.axis("cam_forward") : Real(0);
    const Real inR = camera.positionLocked ? ctx.actions.axis("cam_right") : Real(0);
    const GroundState g = pw.characterGroundState(cc.characterId);
    const bool grounded = g == GroundState::OnGround || g == GroundState::OnSteepGround;
    if (grounded) {
        boardSpeed_ = boardStep(boardSpeed_, boardHeading_, kickClock_, inF, inR, pw.characterGroundNormal(cc.characterId),
                                gear_ == 9 ? tune_.scooterPushMax : tune_.boardPushMax, dt, tune_);
        stamina_ = std::min(tune_.staminaMax, stamina_ + tune_.staminaRegen * dt);
    }
    // the ollie: up, board and all
    if (gate_.buffered > 0 && grounded) {
        gate_.buffered = -1;
        gate_.sinceLaunch = 0;
        pw.launchCharacter(cc.characterId, tune_.boardOllie);
        event(gear_ == 8 ? "ollie" : "hop");
    }
    const Vec3 want = boardHeading_ * boardSpeed_;
    const Vec3 was = t.position;
    const Real vyBefore = pw.characterVelocity(cc.characterId).y;
    pw.moveCharacter(cc.characterId, want, dt);
    t.position = pw.characterPosition(cc.characterId);
    const GroundState after = pw.characterGroundState(cc.characterId);
    if (after == GroundState::OnGround || after == GroundState::OnSteepGround) fall.onGrounded(t.position.y);
    // into something: a wall takes the speed; hard enough and you're off and limp
    const Vec3 moved(t.position.x - was.x, 0, t.position.z - was.z);
    const Real got = dot(moved, boardHeading_) / std::max(dt, Real(1e-4));
    if (boardSpeed_ > 0.5 && got < boardSpeed_ * 0.4) {
        if (boardSpeed_ > tune_.boardCrash) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "bailed at %.1f m/s", boardSpeed_);
            const Vec3 thrown = boardHeading_ * (boardSpeed_ * 0.7) + Vec3(0, 2.5, 0);
            boardSpeed_ = 0;
            return goLimp(t, cc, thrown, buf);
        }
        boardSpeed_ = std::max(Real(0), got);
    }
    // a hard landing off a drop still drops you
    if (!grounded && (after == GroundState::OnGround) && vyBefore < -tune_.limpFallSpeed)
        return goLimp(t, cc, want + Vec3(0, vyBefore * 0.3, 0), "landed hard on the board");
    airVel_ = want;
}

// --- the grapple -------------------------------------------------------------

bool PlayerSystem::tryGrapple(Transform& t, CharacterController& cc) {
    if (!aim_.ok) {
        event("nothing to hook");
        return false;
    }
    PhysicsWorld& pw = physicsSys.physicsWorld();
    anchor_ = aim_.point;
    const Vec3 hand = t.position + Vec3(0, cc.halfHeight + cc.radius * 0.5, 0);
    ropeLen_ = std::max(tune_.grappleMinRope, (anchor_ - hand).length());
    ropeVel_ = pw.characterVelocity(cc.characterId);
    // hooked from the ground: a hard pull for a moment, so the body leaves it and swings
    yank_ = pw.characterGroundState(cc.characterId) == GroundState::OnGround ? 0.5 : 0.0;
    if (move_ == Move::Walk || move_ == Move::Parachute || move_ == Move::Wingsuit) move_ = Move::Grapple;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "hooked %.0f m away", aim_.distance);
    event(buf);
    return true;
}

void PlayerSystem::stepGrapple(FrameContext& ctx, Transform& t, CharacterController& cc, Real dt) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const Real handUp = cc.halfHeight + cc.radius * 0.5;
    auto release = [&](const std::string& why) {
        move_ = Move::Walk;
        airVel_ = Vec3(ropeVel_.x, 0, ropeVel_.z);
        event(why);
    };
    if (!grappleHeld_) return release("let go of the rope");
    const bool reelIn = camera.positionLocked && ctx.actions.held("player_jump");
    const bool reelOut = camera.positionLocked && ctx.actions.held("player_crouch");
    gate_.buffered = -1;   // jump is the reel here, not a leap
    if (yank_ > 0) {
        yank_ -= dt;
        ropeLen_ = std::max(tune_.grappleMinRope, ropeLen_ - 2.0 * tune_.grappleReelIn * dt);
    }
    if (reelIn) ropeLen_ = std::max(tune_.grappleMinRope, ropeLen_ - tune_.grappleReelIn * dt);
    if (reelOut) ropeLen_ = std::min(tune_.grappleRange, ropeLen_ + tune_.grappleReelOut * dt);
    // pump along the swing, steer across it
    const Real inF = camera.positionLocked ? ctx.actions.axis("cam_forward") : Real(0);
    const Real inR = camera.positionLocked ? ctx.actions.axis("cam_right") : Real(0);
    Vec3 along(ropeVel_.x, 0, ropeVel_.z);
    if (along.length() < 1.0) {
        along = camera.forward();
        along.y = 0;
    }
    along = along.lengthSquared() > 1e-8 ? normalize(along) : Vec3(0, 0, -1);
    const Vec3 across(-along.z, 0, along.x);
    const Vec3 accel = along * (inF * tune_.grapplePump) + across * (inR * tune_.grappleSteer);
    const Vec3 handWas = t.position + Vec3(0, handUp, 0);
    Vec3 hand = handWas;
    ropeVel_ = ropeStep(hand, ropeVel_, anchor_, ropeLen_, accel, dt);
    pw.moveCharacterFree(cc.characterId, (hand - handWas) * (1.0 / std::max(dt, Real(1e-4))), dt);
    const Vec3 was = t.position;
    t.position = pw.characterPosition(cc.characterId);
    const Vec3 meant = ropeVel_;
    ropeVel_ = (t.position - was) * (1.0 / std::max(dt, Real(1e-4)));   // what walls and the ground let it do
    if ((meant - ropeVel_).length() > tune_.limpImpact) {   // swung into something, hard
        char buf[64];
        std::snprintf(buf, sizeof(buf), "slammed into it at %.0f m/s", meant.length());
        return goLimp(t, cc, meant * 0.4, buf);
    }
    fall.onGrounded(t.position.y);
    // reeled right up to an edge: pull up over it
    const Vec3 toAnchor = anchor_ - (t.position + Vec3(0, handUp, 0));
    if (reelIn && toAnchor.length() < tune_.grappleMinRope + 0.6) {
        Vec3 face(toAnchor.x, 0, toAnchor.z);
        if (face.length() > 0.05 && tryMantle(t, cc, normalize(face), 2 * (cc.halfHeight + cc.radius) + 0.6)) return;
    }
    // down on the ground with the rope slack: walking again
    const GroundState g = pw.characterGroundState(cc.characterId);
    if (g == GroundState::OnGround && toAnchor.length() < ropeLen_ - 0.15 && !reelIn && yank_ <= 0)
        return release("down off the rope");
}

bool PlayerSystem::tryDeploy(Transform& t, CharacterController& cc) {
    if (gear_ != 5 && gear_ != 6) return false;
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const Real feet = cc.halfHeight + cc.radius;
    PhysicsWorld::RayHit ground;
    if (pw.castRayHit(t.position, Vec3(0, -(feet + tune_.deployMinHeight), 0), ground)) {
        event("too low to open");
        return false;
    }
    glideVel_ = pw.characterVelocity(cc.characterId);
    const Vec3 h(glideVel_.x, 0, glideVel_.z);
    Vec3 look = camera.forward();
    look.y = 0;
    chuteHeading_ = h.length() > 0.5 ? normalize(h) : look.lengthSquared() > 1e-6 ? normalize(look) : Vec3(0, 0, -1);
    chuteOpen_ = 0;
    move_ = gear_ == 5 ? Move::Parachute : Move::Wingsuit;
    event(gear_ == 5 ? "parachute open" : "wingsuit spread");
    return true;
}

void PlayerSystem::stepGlide(FrameContext& ctx, Transform& t, CharacterController& cc, Real dt) {
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const Real inF = camera.positionLocked ? ctx.actions.axis("cam_forward") : Real(0);
    const Vec3 look = camera.forward();
    auto stop = [&](const std::string& why) {
        move_ = Move::Walk;
        airVel_ = Vec3(glideVel_.x, 0, glideVel_.z);
        event(why);
    };
    if (camera.positionLocked && ctx.actions.held("player_crouch")) {   // cut away: freefall
        regrab_ = 0.5;
        return stop(move_ == Move::Parachute ? "cut away the canopy" : "folded the wingsuit");
    }
    if (gate_.buffered > 0) {
        gate_.buffered = -1;
        if (move_ == Move::Wingsuit) {   // the suit's own chute
            move_ = Move::Parachute;
            chuteOpen_ = 0;
            const Vec3 h(glideVel_.x, 0, glideVel_.z);
            if (h.length() > 0.5) chuteHeading_ = normalize(h);
            event("parachute open (off the wingsuit)");
        }
    }
    if (move_ == Move::Parachute) {
        chuteOpen_ = std::min(Real(1), chuteOpen_ + dt / std::max(tune_.chuteOpenTime, Real(0.05)));
        glideVel_ = parachuteStep(glideVel_, chuteHeading_, look, inF, chuteOpen_, dt, tune_);
    } else {
        glideVel_ = wingsuitStep(glideVel_, look, dt, tune_);
    }
    const Vec3 was = t.position;
    pw.moveCharacterFree(cc.characterId, glideVel_, dt);
    t.position = pw.characterPosition(cc.characterId);
    fall.onGrounded(t.position.y);   // flying is not falling
    // DOWN: on the ground, or into something hard enough to stop the way
    const Vec3 moved = t.position - was;
    const Real want = glideVel_.length() * dt;
    const GroundState g = pw.characterGroundState(cc.characterId);
    const bool blocked = want > 0.05 && moved.length() < want * 0.4;
    if (g == GroundState::OnGround || g == GroundState::OnSteepGround || blocked) {
        const Real impact = glideVel_.length();
        landed_ = std::min(Real(1), impact / 8.0);
        char buf[96];
        if (move_ == Move::Wingsuit && impact > 20.0) {
            std::snprintf(buf, sizeof(buf), "crashed the wingsuit at %.0f m/s", impact);
            const Vec3 hit = glideVel_ * 0.5;
            glideVel_ = Vec3(0, 0, 0);
            return goLimp(t, cc, hit, buf);
        } else {
            std::snprintf(buf, sizeof(buf), "%s at %.1f m/s", move_ == Move::Parachute ? "landed the chute" : "landed",
                          impact);
            glideVel_ = glideVel_ * 0.4;
        }
        stop(buf);
    }
}

void PlayerSystem::publishPose(FrameContext& ctx, Entity e, const Transform& t, const CharacterController& cc) {
    PlayerPose* p = ctx.world.get<PlayerPose>(e);
    if (!p) {
        ctx.world.add<PlayerPose>(e, PlayerPose{});   // (another type's pool: safe inside this walk)
        p = ctx.world.get<PlayerPose>(e);
        if (!p) return;
    }
    if (!camera.positionLocked) {   // the freecam: nobody's eye, no arms
        p->mode = PlayerPose::Mode::Hidden;
        return;
    }
    PhysicsWorld& pw = physicsSys.physicsWorld();
    const Vec3 v = pw.characterVelocity(cc.characterId);
    p->speed = std::sqrt(v.x * v.x + v.z * v.z);
    p->stride = stride_;
    p->run = camera.positionLocked && ctx.actions.held("player_run");
    p->crouched = crouched;
    p->stamina = std::clamp(stamina_ / std::max(tune_.staminaMax, Real(0.1)), Real(0), Real(1));
    p->landed = landed_;
    p->climbCycle = climbCycle_;
    p->swimPhase = strokePhase_;
    p->underwater = swim_.swimming && camera.eye.y < swimLevel_ - 0.05;
    p->progress = 0;
    if (const Climbing* cl = ctx.world.get<Climbing>(e)) {
        // on the rails, a hand on each, alternating a rung apart as the body goes up
        const LadderSpec& L = cl->ladder;
        const Vec3 out(L.out.x, 0, L.out.y), side(L.out.y, 0, -L.out.x);
        const Real phase = std::sin(climbCycle_ * 3.14159265358979);
        const Real y = cl->feetY + 1.35;
        p->mode = PlayerPose::Mode::Ladder;
        p->handL = Vec3(L.foot.x, y + 0.15 * phase, L.foot.z) - side * (L.width * 0.5) + out * 0.04;
        p->handR = Vec3(L.foot.x, y - 0.15 * phase, L.foot.z) + side * (L.width * 0.5) + out * 0.04;
        p->wallNormal = out;
        return;
    }
    if (move_ == Move::Mantle) {
        const Ledge& l = mantle_.ledge;
        const Vec3 side(l.normal.z, 0, -l.normal.x);
        p->mode = PlayerPose::Mode::Mantle;
        p->handL = l.edge - side * 0.24 - l.normal * 0.06 + Vec3(0, 0.03, 0);
        p->handR = l.edge + side * 0.24 - l.normal * 0.06 + Vec3(0, 0.03, 0);
        p->wallNormal = l.normal;
        p->progress = mantle_.progress();
        return;
    }
    if (move_ == Move::Climb) {
        Vec3 right, up;
        wallBasis(climbNormal_, right, up);
        // hand over hand: each reaches up the wall in turn as the body climbs
        const Real s = std::sin(climbCycle_ * 3.14159265358979);
        const Vec3 wall = t.position - climbNormal_ * (cc.radius + 0.04);
        p->mode = PlayerPose::Mode::Climb;
        // round eye height (the eye is 0.7 over the centre and a hand's breadth off the wall: much higher and
        // the hands leave the view)
        p->handL = wall + up * (0.78 + 0.13 * s) - right * 0.24;
        p->handR = wall + up * (0.78 - 0.13 * s) + right * 0.24;
        p->wallNormal = climbNormal_;
        return;
    }
    p->velocity = v;
    p->airspeed = v.length();
    p->anchor = anchor_;
    if (move_ == Move::Board) {
        p->mode = PlayerPose::Mode::Board;
        p->board = gear_ == 9 ? 1 : 0;
        p->boardHeading = boardHeading_;
        p->kick = kickClock_ > 0 ? 1.0 - kickClock_ / std::max(tune_.boardKickTime, Real(0.05)) : 0;
        const Vec3 side(-boardHeading_.z, 0, boardHeading_.x);   // the board's left
        const Vec3 bar = t.position + boardHeading_ * 0.42 + Vec3(0, 0.35, 0);
        p->handL = bar + side * 0.22;
        p->handR = bar - side * 0.22;
        return;
    }
    if (move_ == Move::Ragdoll) {
        p->mode = PlayerPose::Mode::Ragdoll;
        for (int i = 0; i < BlockBody::kParts; ++i) pw.ragdollPart(ragdoll_, i, p->ragdollPos[i], p->ragdollRot[i]);
        return;
    }
    if (move_ == Move::Grapple) {
        p->mode = PlayerPose::Mode::Grapple;
        return;
    }
    if (move_ == Move::Parachute || move_ == Move::Wingsuit) {
        p->mode = move_ == Move::Parachute ? PlayerPose::Mode::Parachute : PlayerPose::Mode::Wingsuit;
        p->canopyHeading = chuteHeading_;
        p->canopyOpen = move_ == Move::Parachute ? chuteOpen_ : 0;
        return;
    }
    if (swim_.swimming) {
        p->mode = PlayerPose::Mode::Swim;
        return;
    }
    p->mode = pw.characterGroundState(cc.characterId) == GroundState::InAir ? PlayerPose::Mode::Air
                                                                           : PlayerPose::Mode::Walk;
}

// THE GEAR HUD (Glenn: "when I click 5 nothing happens. How do I know I will deploy a parachute?"): the tool row
// for a few seconds after a slot change, and while the chute or the suit is on, what to press now -- drawn like
// the interaction prompt (InteractBrokerSystem), on ImGui's foreground list until the game UI layer has text.
void PlayerSystem::drawGearHud(FrameContext& ctx) {
#ifdef RT_ENABLE_IMGUI
    const Real dt = std::clamp(static_cast<Real>(ctx.frameDelta), Real(0), Real(0.1));
    hotbarShow_ = std::max(Real(0), hotbarShow_ - dt);
    hudClock_ += dt;
    if (!camera.positionLocked || !ctx.world.alive(playerEntity) || ctx.world.has<InVehicle>(playerEntity)) return;
    const float W = static_cast<float>(ctx.windowWidth), H = static_cast<float>(ctx.windowHeight);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImFont* font = ImGui::GetFont();
    const float fs = ImGui::GetFontSize() * 1.4f;
    auto label = [&](float cx, float y, const std::string& text, ImU32 colour, ImU32 back) {
        const ImVec2 ts = font->CalcTextSizeA(fs, 1e9f, 0.0f, text.c_str());
        const float x = cx - ts.x * 0.5f;
        dl->AddRectFilled(ImVec2(x - 8, y - 4), ImVec2(x + ts.x + 8, y + ts.y + 4), back, 4.0f);
        dl->AddText(font, fs, ImVec2(x, y), colour, text.c_str());
        return ts;
    };
    // the tool row
    if (hotbarShow_ > 0) {
        static const char* names[9] = {"hands", "gun", "map", "light", "parachute", "wingsuit", "grapple", "skateboard",
                                       "scooter"};
        const float alpha = static_cast<float>(std::min(Real(1), hotbarShow_ / 0.4));
        // each box as wide as its words, 14 px apart, the row centred
        std::string words[9];
        float widths[9], total = 0;
        for (int k = 0; k < 9; ++k) {
            words[k] = std::to_string(k + 1) + "  " + names[k];
            widths[k] = font->CalcTextSizeA(fs, 1e9f, 0.0f, words[k].c_str()).x + 16.0f;
            total += widths[k] + (k ? 14.0f : 0.0f);
        }
        const float y = H - 52.0f;
        float x = W * 0.5f - total * 0.5f;
        for (int k = 0; k < 9; ++k) {
            const bool on = gear_ == k + 1;
            const std::string& t = words[k];
            const ImU32 fg = on ? IM_COL32(20, 20, 20, static_cast<int>(255 * alpha))
                                : IM_COL32(235, 235, 235, static_cast<int>(220 * alpha));
            const ImU32 bg = on ? IM_COL32(255, 220, 110, static_cast<int>(235 * alpha))
                                : IM_COL32(0, 0, 0, static_cast<int>(140 * alpha));
            label(x + widths[k] * 0.5f, y, t, fg, bg);
            x += widths[k] + 14.0f;
        }
    }
    // THE GRAPPLE: a ring where a shot would hook, and what the trigger does
    if (gear_ == 7) {
        const CameraState& cam = ctx.view.camera;
        const Vec3 at = move_ == Move::Grapple ? anchor_ : aim_.point;
        Real sx = 0, sy = 0;
        if ((move_ == Move::Grapple || aim_.ok) &&
            projectToScreen(at, cam.position, cam.target, cam.up, cam.fovDegrees, cam.aspectRatio, W, H, sx, sy)) {
            const float r = 10.0f + 2.0f * static_cast<float>(std::sin(hudClock_ * 5.0));
            const ImVec2 c(static_cast<float>(sx), static_cast<float>(sy));
            dl->AddCircle(c, r + 2.0f, IM_COL32(0, 0, 0, 140), 24, 4.0f);
            dl->AddCircle(c, r, IM_COL32(120, 230, 255, 235), 24, 2.5f);
            dl->AddCircleFilled(c, 2.5f, IM_COL32(120, 230, 255, 235));
        }
        const std::string text = move_ == Move::Grapple
                                     ? "let go to fly     Space reel in     Ctrl let out     W pump"
                                 : aim_.ok ? "hold click   hook and swing"
                                           : "nothing in reach to hook";
        label(W * 0.5f, H - (hotbarShow_ > 0 ? 100.0f : 56.0f), text,
              aim_.ok || move_ == Move::Grapple ? IM_COL32(255, 255, 255, 235) : IM_COL32(200, 200, 200, 190),
              IM_COL32(0, 0, 0, 150));
        return;
    }
    if (gear_ == 8 || gear_ == 9) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%.0f km/h     W kick     S brake     A/D carve     Space %s",
                      boardSpeed_ * 3.6, gear_ == 8 ? "ollie" : "hop");
        label(W * 0.5f, H - (hotbarShow_ > 0 ? 100.0f : 56.0f), move_ == Move::Board ? std::string(buf)
              : std::string(gear_ == 8 ? "skateboard: ride it on the ground" : "scooter: ride it on the ground"),
              IM_COL32(255, 255, 255, 235), IM_COL32(0, 0, 0, 150));
        return;
    }
    // what to press now
    if (gear_ != 5 && gear_ != 6) return;
    const bool chute = gear_ == 5;
    std::string text;
    ImU32 colour = IM_COL32(255, 255, 255, 235);
    const CharacterController* cc = ctx.world.get<CharacterController>(playerEntity);
    const Transform* t = ctx.world.get<Transform>(playerEntity);
    if (!cc || !t || cc->characterId == INVALID_CHARACTER) return;
    PhysicsWorld& pw = physicsSys.physicsWorld();
    if (move_ == Move::Parachute) {
        text = "look to steer     W dive     S flare     Ctrl cut away";
    } else if (move_ == Move::Wingsuit) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "%.0f km/h     look down to dive, up to climb     Space parachute     Ctrl fold",
                      pw.characterVelocity(cc->characterId).length() * 3.6);
        text = buf;
    } else if (pw.characterGroundState(cc->characterId) == GroundState::InAir && !swim_.swimming) {
        PhysicsWorld::RayHit g;
        const bool low = pw.castRayHit(t->position, Vec3(0, -(cc->halfHeight + cc->radius + tune_.deployMinHeight), 0), g);
        if (low) {
            text = "too low to open";
            colour = IM_COL32(200, 200, 200, 200);
        } else {
            text = chute ? "Space   open the parachute" : "Space   spread the wingsuit";
            const int a = 180 + static_cast<int>(75 * std::sin(hudClock_ * 6.0));
            colour = IM_COL32(255, 225, 120, a);
        }
    } else {
        text = chute ? "parachute on     jump off something high, then Space in the air"
                     : "wingsuit on     jump off something high, then Space in the air";
    }
    label(W * 0.5f, H - (hotbarShow_ > 0 ? 100.0f : 56.0f), text, colour, IM_COL32(0, 0, 0, 150));
#else
    (void)ctx;
#endif
}

void PlayerSystem::render(FrameContext& ctx) {
    // TELEMETRY for `move?` (the socket): what the body is doing and why it last changed.
    {
        const char* mode = move_ == Move::Mantle    ? "mantle"    : move_ == Move::Climb    ? "climb"
                         : move_ == Move::Parachute ? "parachute" : move_ == Move::Wingsuit ? "wingsuit"
                         : move_ == Move::Grapple   ? "grapple"   : move_ == Move::Ragdoll  ? "ragdoll"
                         : move_ == Move::Board     ? (gear_ == 9 ? "scooter" : "skateboard")
                         : swim_.swimming           ? "swim"      : "walk";
        Vec3 p, v;
        const char* ground = "none";
        if (ctx.world.alive(playerEntity))
            if (const CharacterController* cc = ctx.world.get<CharacterController>(playerEntity))
                if (cc->characterId != INVALID_CHARACTER) {
                    PhysicsWorld& pw = physicsSys.physicsWorld();
                    p = pw.characterPosition(cc->characterId);
                    v = pw.characterVelocity(cc->characterId);
                    switch (pw.characterGroundState(cc->characterId)) {
                        case GroundState::OnGround: ground = "ground"; break;
                        case GroundState::OnSteepGround: ground = "steep"; break;
                        case GroundState::NotSupported: ground = "unsupported"; break;
                        default: ground = "air"; break;
                    }
                }
        if (ctx.world.alive(playerEntity) && ctx.world.has<Climbing>(playerEntity)) mode = "ladder";
        char buf[320];
        std::snprintf(buf, sizeof(buf),
                      "mode %s ground %s pos %.2f %.2f %.2f vel %.2f %.2f %.2f speed %.1f stamina %.1f/%.0f gear %d "
                      "last \"%s\"",
                      mode, ground, p.x, p.y, p.z, v.x, v.y, v.z, v.length(), stamina_, tune_.staminaMax, gear_,
                      lastEvent_.c_str());
        ctx.settings.setString("player.telemetry", buf);
    }
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    drawGearHud(ctx);
    if (!ctx.debugOverlayActive) return;
    ImGui::Begin("Debug");
    if (ImGui::CollapsingHeader("Movement")) {
        ImGui::TextDisabled("live; Save keeps them in settings.json (movement.*)");
        ImGui::Text("%s", ctx.settings.getString("player.telemetry", "").c_str());
        MovementTuning t = tune_;
        bool changed = false;
        const char* group = "";
        for (const MovementTuning::Field& f : MovementTuning::fields()) {
            if (std::string(group) != f.group) {
                group = f.group;
                ImGui::SeparatorText(group);
                for (const MovementTuning::Flag& g : MovementTuning::flags())
                    if (std::string(g.group) == group) changed |= ImGui::Checkbox(g.key, &(t.*g.value));
            }
            float x = static_cast<float>(t.*f.value);
            if (ImGui::SliderFloat(f.key, &x, static_cast<float>(f.lo), static_cast<float>(f.hi))) {
                t.*f.value = x;
                changed = true;
            }
        }
        if (changed) t.save(ctx.settings);
        if (ImGui::Button("Save")) {
            tune_.save(ctx.settings);
            ctx.settings.save("settings.json");
        }
        ImGui::SameLine();
        if (ImGui::Button("Defaults")) MovementTuning{}.save(ctx.settings);
    }
    ImGui::End();
#endif
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
    // GO LIMP on purpose: G, or `ragdoll` on the socket (player.limp)
    if (camera.positionLocked && ctx.actions.pressed("player_limp")) limpRequested_ = true;
    if (ctx.settings.getDouble("player.limp", 0.0) > 0.5) {
        ctx.settings.setDouble("player.limp", 0.0);
        limpRequested_ = true;
    }
    // THE GRAPPLE'S TRIGGER: the press is staged for the step, the hold keeps the rope
    grappleHeld_ = gear_ == 7 && camera.positionLocked && ctx.actions.held("fire") && !ctx.input.uiWantsMouse;
    if (gear_ == 7 && camera.positionLocked && ctx.actions.pressed("fire") && !ctx.input.uiWantsMouse)
        grappleFire_ = true;
    // THE GEAR IN HAND (tool slots, like the gun script tracks them): 5 the parachute, 6 the wingsuit
    for (int k = 1; k <= 9; ++k)
        if (ctx.actions.pressed("slot_" + std::to_string(k)) &&
            !(ctx.world.alive(playerEntity) && ctx.world.has<InVehicle>(playerEntity))) {
            if (k != gear_ && k >= 5)
                event(k == 5 ? "parachute on your back" : k == 6 ? "wingsuit on" : k == 7 ? "grapple in hand"
                      : k == 8 ? "skateboard under your feet" : "scooter out");
            gear_ = k;
            hotbarShow_ = 3.0;
        }
    if (ctx.actions.pressed("player_jump") &&
        !(ctx.world.alive(playerEntity) &&
          (ctx.world.has<InVehicle>(playerEntity) || ctx.world.has<Seated>(playerEntity))))
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

    // THE INPUT MODE follows the body: in a vehicle, the in-vehicle actions
    // resolve (brake, handbrake, flip); on foot, the on-foot ones (jump, crouch,
    // teleport, board). One key can then mean different things in each without
    // both handlers firing. Set here because this is where "is the player in a
    // vehicle" is already known; a system reading input earlier in the frame
    // sees last frame's mode, a one-frame lag on getting in or out.
    if (ctx.world.alive(playerEntity))
        ctx.actions.setContext(ctx.world.has<InVehicle>(playerEntity)
                                   ? engine::InputContext::InVehicle
                                   : engine::InputContext::OnFoot);

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
    // SEATED or LYING (InteractionSystem, engine/interaction.h): the eye is the pose's, turned once to face along
    // it -- after that the mouse looks round as usual. First person while there: the shoulder rig frames the
    // standing capsule, not the seat.
    Seated* seated = ctx.world.get<Seated>(playerEntity);
    if (seated) {
        camera.eye = seated->eye;
        if (!seated->aimed) {
            seated->aimed = true;
            const Vec3 l = seated->look;
            camera.yaw = std::atan2(l.x, -l.z) * 180.0 / 3.14159265358979323846;
            camera.pitch = std::asin(std::clamp(l.y, Real(-1), Real(1))) * 180.0 / 3.14159265358979323846;
        }
    }

    // A placed SceneCamera owns the view this frame; keep the eye pinned above
    // so returning to first person is seamless, but don't write over it.
    if (ctx.view.activeCameraEntity.valid()) return;

    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    if ((thirdPerson || move_ == Move::Ragdoll) && !seated) {   // limp: watched from outside, whichever view
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
        zoom.zoomDelta = camera.inputSuspended ? 0.0 : ctx.input.scrollDelta;
        shoulder.update(zoom, ctx.frameDelta);
        shoulder.orbitPitch =
            std::clamp(camera.pitch, kShoulderPitchMin, kShoulderPitchMax);
        shoulder.farPlane = camera.farPlane;  // fly carries the world-extent value
        ctx.view.camera = shoulder.cameraState(aspect);
    } else {
        ctx.view.camera = camera.cameraState(aspect);
    }
    // SPEED widens the view (a wingsuit dive, a long fall): up to 14 degrees from 20 to 60 m/s, eased
    {
        Real speed = 0;
        if (const CharacterController* cc = ctx.world.get<CharacterController>(playerEntity))
            if (cc->characterId != INVALID_CHARACTER)
                speed = physicsSys.physicsWorld().characterVelocity(cc->characterId).length();
        const Real want = 14.0 * std::clamp((speed - 20.0) / 40.0, Real(0), Real(1));
        fovBoost_ += (want - fovBoost_) * (1.0 - std::exp(-ctx.frameDelta * 2.0));
        ctx.view.camera.fovDegrees += static_cast<float>(fovBoost_);
    }
}

}  // namespace engine
