#include "player_system.h"
#include "physics_system.h"
#include "../components.h"
#include "../interaction.h"   // Seated
#include "../interact_broker.h"   // projectToScreen
#include "../particles.h"     // ParticleEmitter (the body's two steady flows)
#include "../playground.h"
#include "../camera/scene_camera.h"
#include "../ui/ui_text.h"
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
    ctx.actions.bindButton("player_camera_toggle", GamepadButton::Back);   // View / Select / Share
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
    body_.tune.load(ctx.settings);   // Debug > Movement's saved numbers (movement.* in settings.json)
    // THE TOOL SLOTS past the map (engine/player/tools.h): 4 the parachute, 5 the wingsuit (jump in the air opens
    // either; jump again in the suit: its chute), 6 the grapple (hold fire to hang on the rope), 7 the skateboard
    // (ridden on the ground; W kicks), 8 the scooter
    const KeyCode keys[] = {KeyCode::Num4, KeyCode::Num5, KeyCode::Num6, KeyCode::Num7, KeyCode::Num8};
    for (int k = slotOf(Tool::Parachute); k <= kToolSlots; ++k)
        ctx.actions.bindButton(slotAction(k), keys[k - slotOf(Tool::Parachute)]);
    ctx.actions.bindButton("player_limp", KeyCode::G);  // go limp (the ragdoll); jump gets up
    ctx.actions.setActionContext("player_limp", engine::InputContext::OnFoot);
    body_.stamina = body_.tune.staminaMax;
}

// WHAT THE PLAYER WANTS this step, from the input map and the one-shots the frame staged
PlayerIntent PlayerSystem::intent(FrameContext& ctx) const {
    PlayerIntent in;
    in.active = camera.positionLocked;
    in.forward = in.active ? ctx.actions.axis("cam_forward") : Real(0);
    in.right = in.active ? ctx.actions.axis("cam_right") : Real(0);
    in.run = ctx.actions.held("player_run");
    in.jumpHeld = ctx.actions.held("player_jump");
    in.crouchHeld = ctx.actions.held("player_crouch");
    in.jump = jumpRequested;
    in.limp = limpRequested_;
    in.fire = grappleFire_;
    in.fireHeld = grappleHeld_;
    in.playTarget = playTarget_;
    in.playKind = playKind_;
    in.eye = camera.eye;
    in.look = camera.forward();
    in.lookRight = camera.right();
    return in;
}

void PlayerSystem::fixedUpdate(FrameContext& ctx) {
    const Real dt = ctx.clock.fixedStep();
    body_.tune.load(ctx.settings);   // live: the panel and `set movement.<key> <value>` both write the settings

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
            PlayerIntent in = intent(ctx);
            BodyStep s{ctx.world, ctx.settings, e, t, cc, physicsSys.physicsWorld(), in, dt};
            body_.step(s);
            // what the step did not spend waits for the next
            jumpRequested = in.jump;
            limpRequested_ = in.limp;
            grappleFire_ = in.fire;
            playTarget_ = in.playTarget;
        });
}

// THE GEAR HUD (Glenn: "when I click 5 nothing happens. How do I know I will deploy a parachute?"): the tool row
// for a few seconds after a slot change, and while the chute or the suit is on, what to press now -- the game UI
// layer's words (engine/ui/ui_text.h, ADR-0250), not ImGui's.
void PlayerSystem::drawGearHud(FrameContext& ctx) {
    const Real dt = std::clamp(static_cast<Real>(ctx.frameDelta), Real(0), Real(0.1));
    hotbarShow_ = std::max(Real(0), hotbarShow_ - dt);
    hudClock_ += dt;
    if (!camera.positionLocked || !ctx.world.alive(playerEntity) || ctx.world.has<InVehicle>(playerEntity)) return;
    if (ctx.actions.layerUp()) return;   // a screen that has the controls (the map) is the HUD now
    // framebuffer pixels; sizes in window points times the display's scale
    const float W = static_cast<float>(ctx.framebufferWidth), H = static_cast<float>(ctx.framebufferHeight);
    const float k = ctx.windowWidth > 0 ? W / static_cast<float>(ctx.windowWidth) : 1.0f;
    const float fs = 18.0f * k;
    std::vector<Renderer::UiQuad> q;
    const float white[4] = {1, 1, 1, 0.92f}, grey[4] = {0.78f, 0.78f, 0.78f, 0.75f}, back[4] = {0, 0, 0, 0.59f};
    const float promptY = H - (hotbarShow_ > 0 ? 100.0f : 56.0f) * k;
    auto prompt = [&](const std::string& text, const float* colour) {
        hud_.label(ctx.renderer, q, W * 0.5f, promptY, text, fs, colour, back, 8.0f * k);
    };
    // the tool row
    if (hotbarShow_ > 0) {
        const float alpha = static_cast<float>(std::min(Real(1), hotbarShow_ / 0.4));
        // each box as wide as its words, 14 px apart, the row centred
        std::string words[kToolSlots];
        float widths[kToolSlots], total = 0;
        for (int i = 0; i < kToolSlots; ++i) {
            words[i] = std::to_string(i + 1) + "  " + toolName(static_cast<Tool>(i + 1));
            widths[i] = hud_.measure(words[i], fs) + 16.0f * k;
            total += widths[i] + (i ? 14.0f * k : 0.0f);
        }
        float x = W * 0.5f - total * 0.5f;
        for (int i = 0; i < kToolSlots; ++i) {
            const bool on = slotOf(body_.hands.tool) == i + 1;
            const float fg[4] = {on ? 0.08f : 0.92f, on ? 0.08f : 0.92f, on ? 0.08f : 0.92f, (on ? 1.0f : 0.86f) * alpha};
            const float bg[4] = {on ? 1.0f : 0.0f, on ? 0.86f : 0.0f, on ? 0.43f : 0.0f, (on ? 0.92f : 0.55f) * alpha};
            hud_.label(ctx.renderer, q, x + widths[i] * 0.5f, H - 52.0f * k, words[i], fs, fg, bg, 8.0f * k);
            x += widths[i] + 14.0f * k;
        }
    }
    [&] {
        if (body_.is(body_.swing) || body_.is(body_.bars)) {
            prompt(body_.is(body_.swing) ? "W / S  pump in time with the swing     Space  jump off     Ctrl  get off"
                                         : "W / S  hand over hand     Space or Ctrl  drop",
                   white);
            return;
        }
        // THE GRAPPLE: a ring where a shot would hook, and what the trigger does
        if (body_.hands.holdingGrapple()) {
            const CameraState& cam = ctx.view.camera;
            const Vec3 at = body_.is(body_.grapple) ? body_.grapple.anchor : body_.hands.aim.point;
            Real sx = 0, sy = 0;
            if ((body_.is(body_.grapple) || body_.hands.aim.ok) &&
                projectToScreen(at, cam.position, cam.target, cam.up, cam.fovDegrees, cam.aspectRatio, W, H, sx, sy)) {
                const float r = (10.0f + 2.0f * static_cast<float>(std::sin(hudClock_ * 5.0))) * k;
                const float shade[4] = {0, 0, 0, 0.55f}, cyan[4] = {0.47f, 0.9f, 1.0f, 0.92f};
                const float cx = static_cast<float>(sx), cy = static_cast<float>(sy);
                hud_.ring(ctx.renderer, q, cx, cy, r + 4.0f * k, 4.0f * k, shade);
                hud_.ring(ctx.renderer, q, cx, cy, r + 1.25f * k, 2.5f * k, cyan);
                hud_.disc(ctx.renderer, q, cx, cy, 2.5f * k, cyan);
            }
            const bool live = body_.hands.aim.ok || body_.is(body_.grapple);
            prompt(body_.is(body_.grapple) ? "let go to fly     Space reel in     Ctrl let out     W pump"
                   : body_.hands.aim.ok    ? "hold click   hook and swing"
                                           : "nothing in reach to hook",
                   live ? white : grey);
            return;
        }
        if (body_.hands.holdingBoard()) {
            const bool skate = body_.hands.tool == Tool::Skateboard;
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%.0f km/h     W kick     S brake     A/D carve     Space %s",
                          body_.board.speed * 3.6, skate ? "ollie" : "hop");
            prompt(body_.is(body_.board) ? std::string(buf)
                   : std::string(skate ? "skateboard: ride it on the ground" : "scooter: ride it on the ground"),
                   white);
            return;
        }
        // what to press now
        if (!body_.hands.holdingGlider()) return;
        const bool chute = body_.hands.tool == Tool::Parachute;
        const CharacterController* cc = ctx.world.get<CharacterController>(playerEntity);
        const Transform* t = ctx.world.get<Transform>(playerEntity);
        if (!cc || !t || cc->characterId == INVALID_CHARACTER) return;
        PhysicsWorld& pw = physicsSys.physicsWorld();
        if (body_.is(body_.parachute)) {
            prompt("look to steer     W dive     S flare     Ctrl cut away", white);
        } else if (body_.is(body_.wingsuit)) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%.0f km/h     look down to dive, up to climb     Space parachute     Ctrl fold",
                          pw.characterVelocity(cc->characterId).length() * 3.6);
            prompt(buf, white);
        } else if (pw.characterGroundState(cc->characterId) == GroundState::InAir && !body_.water.swimming) {
            PhysicsWorld::RayHit g;
            const Real reach = cc->halfHeight + cc->radius + body_.tune.deployMinHeight;
            if (pw.castRayHit(t->position, Vec3(0, -reach, 0), g)) {
                const float dim[4] = {0.78f, 0.78f, 0.78f, 0.78f};
                prompt("too low to open", dim);
            } else {
                const float pulse[4] = {1.0f, 0.88f, 0.47f, static_cast<float>(0.7 + 0.29 * std::sin(hudClock_ * 6.0))};
                prompt(chute ? "Space   open the parachute" : "Space   spread the wingsuit", pulse);
            }
        } else {
            prompt(chute ? "parachute on     jump off something high, then Space in the air"
                         : "wingsuit on     jump off something high, then Space in the air",
                   white);
        }
    }();
    if (!q.empty()) ctx.renderer.submitUi(q);
}

void PlayerSystem::render(FrameContext& ctx) {
    // TELEMETRY for `move?` (the socket): what the body is doing and why it last changed.
    {
        const char* mode = body_.mode().name(body_);
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
        // the rope as drawn: its state, how long it is, its lowest point under the straight line hand-to-hook (sag)
        using Hook = PlayerHands::Hook;
        const Hook hk = body_.hands.hook;
        const char* hook = hk == Hook::Flying ? "flying" : hk == Hook::Hooked ? "hooked" : hk == Hook::Loose ? "loose" : "none";
        const RopeLine& rope = body_.hands.rope;
        Real sag = 0;
        if (rope.active()) {
            const std::vector<Vec3>& r = rope.points();
            for (std::size_t i = 1; i + 1 < r.size(); ++i) {
                const Real f = static_cast<Real>(i) / (r.size() - 1);
                sag = std::max(sag, (r.front() + (r.back() - r.front()) * f).y - r[i].y);
            }
        }
        const Real span = rope.active() ? (rope.points().back() - rope.points().front()).length() : 0;
        char buf[400];
        std::snprintf(buf, sizeof(buf),
                      "mode %s ground %s pos %.2f %.2f %.2f vel %.2f %.2f %.2f speed %.1f stamina %.1f/%.0f gear %d "
                      "hook %s rope %.2f span %.2f sag %.2f last \"%s\"",
                      mode, ground, p.x, p.y, p.z, v.x, v.y, v.z, v.length(), body_.stamina, body_.tune.staminaMax,
                      slotOf(body_.hands.tool), hook,
                      hk == Hook::Hooked ? body_.grapple.ropeLen : hk == Hook::Loose ? body_.hands.looseLen : Real(0),
                      span, sag,
                      body_.lastEvent.c_str());
        ctx.settings.setString("player.telemetry", buf);
    }
    drawGearHud(ctx);
#ifdef RT_ENABLE_IMGUI
    if (ImGui::GetCurrentContext() == nullptr) return;
    if (!ctx.debugOverlayActive) return;
    ImGui::Begin("Debug");
    if (ImGui::CollapsingHeader("Movement")) {
        ImGui::TextDisabled("live; Save keeps them in settings.json (movement.*)");
        ImGui::Text("%s", ctx.settings.getString("player.telemetry", "").c_str());
        MovementTuning t = body_.tune;
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
            body_.tune.save(ctx.settings);
            ctx.settings.save("settings.json");
        }
        ImGui::SameLine();
        if (ImGui::Button("Defaults")) MovementTuning{}.save(ctx.settings);
    }
    ImGui::End();
#endif
}

void PlayerSystem::update(FrameContext& ctx) {
    // the body's two steady particle flows (effects()): made here, outside any walk of the Transforms
    auto emitter = [&](Entity& em, const char* effect, Real ahead, const Vec3& box, bool relative) {
        if (em.valid() && ctx.world.alive(em)) return;
        em = ctx.world.create();
        ctx.world.add<Transform>(em, Transform{});
        ParticleEmitter pe;
        pe.effect = effect;
        pe.ahead = ahead;
        pe.halfExtent = box;
        pe.relative = relative;
        ctx.world.add<ParticleEmitter>(em, pe);
    };
    emitter(body_.bubbleEmitter, "bubble", 0, Vec3(0.03, 0.03, 0.03), false);
    emitter(body_.streakEmitter, "streak", 0.45, Vec3(4, 3, 4), true);
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
        body_.fall.onGrounded(stand.y);   // new baseline: a teleport is
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
    if (ctx.actions.pressed("player_respawn") && body_.spawnCaptured && ctx.world.alive(playerEntity)) {
        if (auto* cc = ctx.world.get<CharacterController>(playerEntity))
            if (cc->characterId != INVALID_CHARACTER) {
                body_.respawn(physicsSys.physicsWorld(), cc->characterId, /*manual=*/true);
                if (auto* t = ctx.world.get<Transform>(playerEntity)) t->position = body_.spawnPos;
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
    // THE PLAYGROUND: the swing seat or the bars in reach are offered (E); the broker's command is taken up at the
    // next step. Empty swings swing on and settle meanwhile.
    ctx.world.each<PlaySwing>([&](Entity, PlaySwing& s) {
        if (!s.ridden) swingStep(s.theta, s.omega, s.length, 0, std::clamp(ctx.frameDelta, 0.0, 0.1), body_.tune);
        if (Transform* st = ctx.world.get<Transform>(s.seat)) {
            if (PrevTransform* sp = ctx.world.get<PrevTransform>(s.seat)) sp->value = *st;
            st->orientation = Quat::fromAxisAngle(normalize(cross(Vec3(0, 1, 0), s.forward)), -s.theta);
            if (PrevTransform* sp = ctx.world.get<PrevTransform>(s.seat)) sp->value = *st;
        }
    });
    if (ctx.world.alive(playerEntity) && body_.is(body_.onFoot) && camera.positionLocked &&
        !ctx.world.has<InVehicle>(playerEntity) && !ctx.world.has<Seated>(playerEntity))
        if (const Transform* pt = ctx.world.get<Transform>(playerEntity)) {
            const Vec3 p = pt->position;
            Entity best;
            uint64_t kind = 0;
            Vec3 at;
            Real bestD = 1e9;
            ctx.world.each<PlaySwing>([&](Entity e, PlaySwing& s) {
                const Vec3 seat = s.pivot - Vec3(0, s.length, 0);
                const Real d = std::hypot(seat.x - p.x, seat.z - p.z);
                if (d < 1.6 && d < bestD) { bestD = d; best = e; kind = 1; at = seat; }
            });
            ctx.world.each<MonkeyBars>([&](Entity e, MonkeyBars& b) {
                const Vec3 ab = b.b - b.a;
                const Real len2 = std::max(ab.lengthSquared(), Real(1e-6));
                const Real u = std::clamp(dot(p - b.a, ab) / len2, Real(0), Real(1));
                const Vec3 q = b.a + ab * u;
                const Real d = std::hypot(q.x - p.x, q.z - p.z);
                // (the marker at reaching height: the bar itself is on the broker's "another floor" line)
                if (d < 1.2 && q.y > p.y && d < bestD) { bestD = d; best = e; kind = 2; at = q - Vec3(0, 0.8, 0); }
            });
            if (best.valid()) {
                InteractOffer o;
                o.provider = "playground";
                o.entity = best;
                o.key = kind;
                o.anchor = at;
                o.name = kind == 1 ? "swing" : "monkey bars";
                o.tap = kind == 1 ? "sit on it" : "grab the bars";
                o.reach = 1.8;
                o.needsSight = false;
                offerInteraction(ctx.world, playerEntity, o);
            }
        }
    {
        InteractCommand pc;
        if (ctx.world.alive(playerEntity) && takeInteractCommand(ctx.world, playerEntity, "playground", pc)) {
            playTarget_ = pc.entity;
            playKind_ = pc.key;
        }
    }
    // GO LIMP on purpose: G, or `ragdoll` on the socket (player.limp)
    if (camera.positionLocked && ctx.actions.pressed("player_limp")) limpRequested_ = true;
    if (ctx.settings.getDouble("player.limp", 0.0) > 0.5) {
        ctx.settings.setDouble("player.limp", 0.0);
        limpRequested_ = true;
    }
    // THE GRAPPLE'S TRIGGER: the press is staged for the step, the hold keeps the rope
    const bool grapple = body_.hands.holdingGrapple();
    grappleHeld_ = grapple && camera.positionLocked && ctx.actions.held("fire") && !ctx.input.uiWantsMouse;
    if (grapple && camera.positionLocked && ctx.actions.pressed("fire") && !ctx.input.uiWantsMouse)
        grappleFire_ = true;
    // THE TOOL IN HAND (the slots, as the gun script and the map track them too)
    for (int k = 1; k <= kToolSlots; ++k)
        if (ctx.actions.pressed(slotAction(k)) &&
            !(ctx.world.alive(playerEntity) && ctx.world.has<InVehicle>(playerEntity))) {
            const Tool t = static_cast<Tool>(k);
            if (t != body_.hands.tool && k >= slotOf(Tool::Parachute))
                body_.event(t == Tool::Parachute ? "parachute on your back" : t == Tool::Wingsuit ? "wingsuit on"
                            : t == Tool::Grapple ? "grapple in hand" : t == Tool::Skateboard ? "skateboard under your feet"
                                                                    : "scooter out");
            body_.hands.tool = t;
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
        body_.crouched && body_.standCaptured
            ? (body_.standHalfHeight * PlayerBody::kCrouchHalfScale + body_.standRadius) /
                  std::max(Real(1e-3), body_.standHalfHeight + body_.standRadius)
            : Real(1);
    camera.eye = t->position + Vec3(0, body_.eyeHeight * eyeScale, 0);
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
    if ((thirdPerson || body_.is(body_.ragdoll)) && !seated) {   // limp: watched from outside, whichever view
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
