// THE BODY'S OWN MODES (ADR-0246): on foot, afloat, on a ladder, pulling up, holding a wall.

#include "player_body.h"

#include "../ladder.h"        // Climbing
#include "../player_pose.h"
#include "../playground.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

namespace {

// ON FOOT OR AFLOAT, FIRST: the crouch (held), a teleport's new fall baseline, and the way the keys push
Vec3 footPrelude(PlayerBody& b, BodyStep& s, bool& run) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const bool wantCrouch = s.in.active && s.in.crouchHeld && !b.water.swimming;
    if (wantCrouch != b.crouched) {
        const Real targetHalf = wantCrouch ? b.standHalfHeight * PlayerBody::kCrouchHalfScale : b.standHalfHeight;
        // Standing up can FAIL (a ledge overhead) -- the physics fit test is the authority, so the player stays
        // crouched instead of popping through the ceiling.
        if (pw.setCharacterHeight(cc.characterId, targetHalf, b.standRadius)) {
            b.crouched = wantCrouch;
            // the live capsule: the third-person body is scaled from it, so the drawn person shrinks with the
            // collider instead of floating over a crouched capsule
            cc.halfHeight = targetHalf;
            t.position = pw.characterPosition(cc.characterId);
        }
    }
    // A TELEPORT IS NEVER A FALL. Something moved the player further than any physics step can -- F's fast travel,
    // the `teleport` verb, the Teleport panel -- and the fall tracker's baseline is still the OLD height: a teleport
    // to lower ground read as a long drop and put the player back at the spawn (Glenn: "When I tap F again I go
    // back to where I started"). Detected here it covers every teleport path, including ones not written yet.
    if (b.haveLastBodyPos) {
        const Real jx = t.position.x - b.lastBodyPos.x;
        const Real jy = t.position.y - b.lastBodyPos.y;
        const Real jz = t.position.z - b.lastBodyPos.z;
        if (jx * jx + jy * jy + jz * jz > 4.0 * 4.0) b.fall.onGrounded(t.position.y);
    }
    b.moveSpeed = b.tune.walkSpeed;
    run = s.in.active && s.in.run;
    Vec3 desired;
    if (s.in.active) {
        Vec3 camForward = s.in.look;
        camForward.y = 0;
        camForward = normalize(camForward);
        Vec3 moveDir = camForward * s.in.forward + s.in.lookRight * s.in.right;
        const Real len = moveDir.length();
        if (len > 1.0) moveDir = moveDir / len;
        desired = moveDir * b.moveSpeed *
                  (b.crouched ? PlayerBody::kCrouchSpeedScale : run ? b.tune.runScale : Real(1));
    }
    return desired;
}

}  // namespace

// --- on foot -------------------------------------------------------------------------------------------------------

void OnFootMode::leave(PlayerBody& b, BodyStep& s) {
    if (b.water.swimming) return b.enter(b.swim);
    // ONTO THE PLAYGROUND: the swing or the bars E chose
    if (s.in.playTarget.valid()) {
        const Entity target = s.in.playTarget;
        const uint64_t kind = s.in.playKind;
        s.in.playTarget = Entity{};
        if (kind == 1 && s.world.has<PlaySwing>(target)) {
            b.swing.seat = target;
            s.world.get<PlaySwing>(target)->ridden = true;
            b.enter(b.swing);
            b.event("on the swing");
            return;
        }
        if (kind == 2 && s.world.has<MonkeyBars>(target)) {
            BarsMode& m = b.bars;
            m.bars = target;
            const MonkeyBars& bars = *s.world.get<MonkeyBars>(target);
            const Vec3 ab = bars.b - bars.a;
            const Real len = std::max(ab.length(), Real(1e-3));
            m.at = std::clamp(dot(s.t.position - bars.a, ab) / len, Real(0), len);
            m.facing = normalize(Vec3(ab.x, 0, ab.z));
            if (dot(m.facing, s.in.look) < 0) m.facing = m.facing * -1.0;
            b.enter(b.bars);
            b.event("on the monkey bars");
            return;
        }
    }
    // ON A BOARD: with one in hand, the ground is ridden, not walked (off it again when the hand changes)
    if (b.hands.holdingBoard() && !b.crouched &&
        s.pw.characterGroundState(s.cc.characterId) == GroundState::OnGround) {
        BoardMode& m = b.board;
        Vec3 h = s.pw.characterVelocity(s.cc.characterId);
        h.y = 0;
        m.speed = h.length();
        if (m.speed < 0.5) {
            h = s.in.look;
            h.y = 0;
        }
        m.heading = h.lengthSquared() > 1e-8 ? normalize(h) : Vec3(0, 0, -1);
        m.kickClock = 0;
        b.enter(b.board);
        b.event(b.hands.gear == 8 ? "on the skateboard" : "on the scooter");
    }
}

void OnFootMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const PlayerIntent& in = s.in;
    const MovementTuning& tune = b.tune;
    const Real dt = s.dt;
    bool run = false;
    Vec3 desired = footPrelude(b, s, run);

    // THE JUMP, A PULL-UP, A GRAB (engine/locomotion.h). Facing: where the keys push, else the view.
    const GroundState before = pw.characterGroundState(cc.characterId);
    Vec3 facing(desired.x, 0, desired.z);
    const bool pushing = facing.length() > 0.5;
    if (pushing) {
        facing = normalize(facing);
    } else {
        facing = in.look;
        facing.y = 0;
        facing = facing.lengthSquared() > 1e-8 ? normalize(facing) : Vec3(0, 0, -1);
    }
    // A ledge in front when jump is pressed on the ground: pull up onto it rather than hop at it.
    if (in.active && b.gate.buffered > 0 && !b.crouched && before == GroundState::OnGround &&
        b.tryMantle(s, facing, tune.groundMantleMax)) {
        b.gate.buffered = -1;
        return;
    }
    bool launched = false;
    if (in.active && !b.crouched) {
        const JumpGate::Launch l = b.gate.take(before, pw.characterGroundNormal(cc.characterId), tune, dt);
        if (l.go) {
            const bool running = run && Vec3(desired.x, 0, desired.z).length() > b.moveSpeed * 1.2;
            pw.launchCharacter(cc.characterId, tune.jumpSpeed * (running ? tune.runJumpScale : Real(1)));
            b.jumpRising = true;
            b.airVel = Vec3(desired.x, 0, desired.z) + l.push;
            desired = b.airVel;
            launched = true;
            b.event(before == GroundState::OnGround ? "jump"
                    : before == GroundState::InAir  ? "jump (just off an edge)"
                                                    : "jump (off steep ground)");
        }
    }
    if (!launched && before == GroundState::InAir && b.gate.buffered > 0 && in.active && b.tryWallJump(s, facing)) {
        b.gate.buffered = -1;
        return;
    }
    // let go of jump on the way up: the rest of the rise is cut (a tap hops, held jumps)
    if (!launched && b.jumpRising) {
        const Real vy = pw.characterVelocity(cc.characterId).y;
        if (before != GroundState::InAir || vy <= 0) {
            b.jumpRising = false;
        } else if (!in.jumpHeld) {
            pw.launchCharacter(cc.characterId, vy * tune.jumpCut);
            b.jumpRising = false;
        }
    }
    if (before == GroundState::OnGround) b.wallJumped = false;
    if (!launched && before == GroundState::InAir && b.gate.buffered > 0 && b.tryDeploy(s)) {
        b.gate.buffered = -1;
        return;
    }
    if (!launched && before != GroundState::OnGround) {
        // in the air (or sliding): a ledge in reach is grabbed and pulled up onto, a climbable wall held
        if (in.active && pushing && b.tryMantle(s, facing, tune.mantleMaxHeight)) return;
        if (in.active && pushing && tune.grabInAir && b.tryGrab(s, facing)) return;
    }
    if (!launched && before == GroundState::OnGround && in.active && pushing && tune.grabFromGround &&
        b.tryGrab(s, facing))
        return;
    // In the air the keys only nudge what the body left the ground with (#83); on the ground they set it, and that
    // is what a jump or a step off a ledge carries.
    if (!launched) {
        if (before == GroundState::InAir) {
            b.airVel = airborneVelocity(b.airVel, desired, dt, tune.airAccel);
            desired = b.airVel;
        } else {
            b.airVel = Vec3(desired.x, 0, desired.z);
        }
    }
    const Real vyBefore = pw.characterVelocity(cc.characterId).y;
    pw.moveCharacter(cc.characterId, desired, dt);
    t.position = pw.characterPosition(cc.characterId);

    // THE SAFETY NET: fallen far below solid ground -- off a ledge, or through a level whose ground under the spawn
    // has no collider -- the body goes back to the spawn (FallRespawnTracker decides; see player_system.h)
    const GroundState gs = pw.characterGroundState(cc.characterId);
    b.gate.observe(gs, pw.characterVelocity(cc.characterId).y, dt);
    if (gs == GroundState::OnGround || gs == GroundState::OnSteepGround) b.fall.onGrounded(t.position.y);
    if (gs == GroundState::OnGround) {
        b.stamina = std::min(tune.staminaMax, b.stamina + tune.staminaRegen * dt);
        if (before == GroundState::InAir && vyBefore < -2.0) b.landed = std::min(Real(1), -vyBefore / 7.0);
        if (before == GroundState::InAir && vyBefore < -tune.limpFallSpeed) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "landed hard at %.0f m/s", -vyBefore);
            return b.goLimp(s, Vec3(b.airVel.x, vyBefore * 0.3, b.airVel.z), buf);
        }
    }
    if (b.haveLastBodyPos) {
        const Real hx = t.position.x - b.lastBodyPos.x, hz = t.position.z - b.lastBodyPos.z;
        const Real hd = std::sqrt(hx * hx + hz * hz);
        if (hd < 1.0 && gs == GroundState::OnGround) b.stride += hd / 1.6;
    }
    // ...but only when there is nothing below to land on: a jump off a tower with a chute to open is a fall of 40 m
    // and more, and the net is for falling through the world, not off it
    PhysicsWorld::RayHit below;
    if (b.spawnCaptured && b.fall.shouldRespawn(t.position.y) && !pw.castRayHit(t.position, Vec3(0, -5000, 0), below)) {
        b.respawn(pw, cc.characterId, /*manual=*/false);
        t.position = b.spawnPos;
    }
}

void OnFootMode::pose(const PlayerBody&, const BodyStep& s, PlayerPose& p) const {
    p.mode = s.pw.characterGroundState(s.cc.characterId) == GroundState::InAir ? PlayerPose::Mode::Air
                                                                               : PlayerPose::Mode::Walk;
}

// --- afloat --------------------------------------------------------------------------------------------------------

void SwimMode::leave(PlayerBody& b, BodyStep&) {
    if (!b.water.swimming) b.enter(b.onFoot);
}

// The stroke's pace; the water holds you at the surface; jump rises, crouch dives (not a smaller capsule)
void SwimMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    const MovementTuning& tune = b.tune;
    const Real dt = s.dt;
    bool run = false;
    const Vec3 desired = footPrelude(b, s, run);
    b.swimClock += dt;
    b.gate.buffered = -1;   // jump held is "rise" here, not a leap
    const Real floatY = b.swimLevel + SwimState::kEyeAbove - b.eyeHeight;
    const bool rise = s.in.active && s.in.jumpHeld;
    const bool dive = s.in.active && s.in.crouchHeld;
    const Real stroking = std::min(Real(1), desired.length() / std::max(b.moveSpeed, Real(0.1)));
    if (stroking > 0.1) b.strokePhase += dt / std::max(tune.strokePeriod, Real(0.1)) * (run ? 1.35 : 1.0);
    // STILL, YOU TREAD WATER upright; going somewhere you tip forward into the crawl (Glenn, 2026-10-24: "if you're
    // not moving in a direction you should bobble in the water and maybe he should kick his legs to float. On when
    // you go in a direction should you dive forward and swim")
    b.swimStroke += ((stroking > 0.1 ? Real(1) : Real(0)) - b.swimStroke) * (1.0 - std::exp(-dt * 4.0));
    const Real pace = tune.swimSpeed * (run ? tune.swimRunScale : Real(1)) *
                      strokeSpeedScale(strokeSurgePhase(b.strokePhase), tune.strokeSurge);
    // the water takes the way on gradually: a dive carries in, a stroke builds speed
    const Vec3 stroke = desired * (pace / std::max(b.moveSpeed, Real(0.1)));
    const Real take = 1.0 - std::exp(-dt * 3.0);
    Vec3 v(b.airVel.x + (stroke.x - b.airVel.x) * take, 0, b.airVel.z + (stroke.z - b.airVel.z) * take);
    // the bob: a slow swell, and treading, a lift with each kick (two a second, one per leg)
    const Real treading = 1.0 - b.swimStroke;
    const Real bob = (0.04 + 0.03 * treading) * std::sin(b.swimClock * 1.7) +
                     0.025 * treading * std::sin(b.swimClock * 12.566370614359172);   // 2 Hz
    b.swimVy = swimVerticalStep(b.swimVy, t.position.y, floatY + bob, rise, dive, dt, tune);
    v.y = b.swimVy;
    b.airVel = Vec3(v.x, 0, v.z);   // leaving the water keeps the stroke's speed
    pw.moveCharacterFree(s.cc.characterId, v, dt);
    const Real yWas = t.position.y;
    t.position = pw.characterPosition(s.cc.characterId);
    if (dt > 0) b.swimVy = (t.position.y - yWas) / dt;   // the pool floor stops a plunge
    b.fall.onGrounded(t.position.y);   // floating is footing: never a fall
}

void SwimMode::pose(const PlayerBody&, const BodyStep&, PlayerPose& p) const { p.mode = PlayerPose::Mode::Swim; }

// --- on a ladder ---------------------------------------------------------------------------------------------------

// CLIMBING a ladder (engine/ladder.h; Glenn, 2026-10-13: "Can we make ladders as interactable?"): forward climbs, back
// climbs down, no gravity; at the top the body steps off onto what the ladder lands on, at the foot onto the ground
// in front of it; jump lets go.
void LadderMode::step(PlayerBody& b, BodyStep& s) {
    Climbing* cl = s.world.get<Climbing>(s.e);
    if (!cl) return;
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const Real dt = s.dt;
    const LadderSpec L = cl->ladder;
    const Real feetOff = cc.halfHeight + cc.radius;
    const Real up = s.in.forward;
    const Real speed = s.in.run ? ClimbSpeed::kRun : ClimbSpeed::kClimb;
    const Vec3 out(L.out.x, 0, L.out.y);
    auto off = [&](const Vec3& feetAt) {
        pw.setCharacterPosition(cc.characterId, feetAt + Vec3(0, feetOff + 0.05, 0));
        t.position = pw.characterPosition(cc.characterId);
        b.fall.onGrounded(t.position.y);
        b.airVel = Vec3(0, 0, 0);
        s.world.remove<Climbing>(s.e);   // (another type's pool: safe inside the walk)
    };
    if (s.in.jump) {   // let go: a step back off the rungs, and down
        s.in.jump = false;
        off(Vec3(L.foot.x, cl->feetY, L.foot.z) + out * (cc.radius + 0.6));
    } else if (cl->feetY >= L.landY - 0.02 && up > 0.3) {
        off(L.exit);                                                   // over the top
    } else if (cl->feetY <= L.foot.y + 0.01 && up < -0.3) {
        off(L.foot + out * (cc.radius + 0.45));                      // down at the foot
    } else {
        cl->feetY = std::clamp(cl->feetY + up * speed * dt, L.foot.y, L.landY);
        const Vec3 target = Vec3(L.foot.x, cl->feetY + feetOff, L.foot.z) + out * (cc.radius + 0.14);
        const Vec3 v = (target - t.position) * (1.0 / std::max(dt, Real(1e-4)));
        pw.moveCharacterFree(cc.characterId, v, dt);
        t.position = pw.characterPosition(cc.characterId);
        b.fall.onGrounded(t.position.y);   // a rung is footing: never a fall
        b.climbCycle += std::fabs(up) * speed * dt / 0.6;
    }
}

void LadderMode::pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const {
    const Climbing* cl = s.world.get<Climbing>(s.e);
    if (!cl) return b.onFoot.pose(b, s, p);
    // on the rails, a hand on each, alternating a rung apart as the body goes up
    const LadderSpec& L = cl->ladder;
    const Vec3 out(L.out.x, 0, L.out.y), side(L.out.y, 0, -L.out.x);
    const Real phase = std::sin(b.climbCycle * 3.14159265358979);
    const Real y = cl->feetY + 1.35;
    p.mode = PlayerPose::Mode::Ladder;
    p.handL = Vec3(L.foot.x, y + 0.15 * phase, L.foot.z) - side * (L.width * 0.5) + out * 0.04;
    p.handR = Vec3(L.foot.x, y - 0.15 * phase, L.foot.z) + side * (L.width * 0.5) + out * 0.04;
    p.wallNormal = out;
}

// --- pulling up ----------------------------------------------------------------------------------------------------

void MantleMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    const Real dt = s.dt;
    move.t += dt;
    const Vec3 target = move.at(move.t);
    pw.moveCharacterFree(s.cc.characterId, (target - t.position) * (1.0 / std::max(dt, Real(1e-4))), dt);
    t.position = pw.characterPosition(s.cc.characterId);
    b.fall.onGrounded(t.position.y);   // hands on the edge: never a fall
    if ((t.position - target).length() > 0.35) {
        b.enter(b.onFoot);
        b.airVel = Vec3(0, 0, 0);
        b.event("pull-up blocked");
    } else if (move.done()) {
        b.enter(b.onFoot);
        b.airVel = Vec3(0, 0, 0);
        b.event("up");
    }
}

void MantleMode::pose(const PlayerBody&, const BodyStep&, PlayerPose& p) const {
    const Ledge& l = move.ledge;
    const Vec3 side(l.normal.z, 0, -l.normal.x);
    p.mode = PlayerPose::Mode::Mantle;
    p.handL = l.edge - side * 0.24 - l.normal * 0.06 + Vec3(0, 0.03, 0);
    p.handR = l.edge + side * 0.24 - l.normal * 0.06 + Vec3(0, 0.03, 0);
    p.wallNormal = l.normal;
    p.progress = move.progress();
}

// --- holding a wall ------------------------------------------------------------------------------------------------

void ClimbMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const MovementTuning& tune = b.tune;
    const Real dt = s.dt;
    const Real inF = s.in.forward;
    const Real inR = s.in.right;
    const bool run = s.in.active && s.in.run;
    Vec3 right, up;
    wallBasis(normal, right, up);
    Vec3 dir = up * inF + right * inR;
    const Real len = dir.length();
    if (len > 1.0) dir = dir / len;
    const bool moving = len > 0.15;
    const Vec3 away = normalize(Vec3(normal.x, 0, normal.z));
    auto letGo = [&](Real cooldown, const std::string& why) {
        b.enter(b.onFoot);
        b.regrab = cooldown;
        b.airVel = away * 0.6;
        b.event(why);
    };
    b.stamina -= (moving ? (run ? 2.0 : 1.0) : tune.climbIdleDrain) * dt;
    if (s.in.active && s.in.crouchHeld) return letGo(0.5, "let go of the wall");
    // JUMP on the wall: back held kicks off it; otherwise a lunge the way the keys point (up by default)
    if (b.gate.buffered > 0) {
        b.gate.buffered = -1;
        if (inF < -0.5) {
            letGo(0.6, "jump off the wall");
            b.airVel = away * tune.wallKick;
            pw.launchCharacter(cc.characterId, tune.jumpSpeed);
            b.gate.sinceLaunch = 0;
            pw.moveCharacter(cc.characterId, b.airVel, dt);
            t.position = pw.characterPosition(cc.characterId);
            return;
        }
        if (b.stamina > 0) {
            lunge = tune.climbLungeTime;
            lungeDir = moving ? dir * (1.0 / std::max(len, Real(1e-3))) : up;
            b.stamina -= tune.climbJumpCost;
            b.event("lunge");
        }
    }
    Vec3 v = dir * (tune.climbSpeed * (run ? tune.climbRunScale : Real(1)));
    if (lunge > 0) {
        v = lungeDir * tune.climbLungeSpeed;
        lunge -= dt;
    }
    const bool goingUp = inF > 0.2 || (lunge > 0 && lungeDir.y > 0.3);
    const Real tall = 2 * (cc.halfHeight + cc.radius) + 0.6;   // a ledge reached from the wall: hands over the head
    // THE TOP: the head clears the wall -- pull up over it
    const WallHit atHead = probeWall(pw, t.position + Vec3(0, cc.halfHeight + cc.radius * 0.5, 0), -away,
                                     cc.radius + 0.6, tune);
    if (goingUp && !atHead.hit && b.tryMantle(s, -away, tall)) return;
    // STICK to the wall at the chest; turn into an inside corner, reach round an outside one
    WallHit at = probeWall(pw, t.position, -away, cc.radius + 0.7, tune);
    if (std::fabs(inR) > 0.3) {
        const Vec3 side = right * (inR > 0 ? 1.0 : -1.0);
        const WallHit across = probeWall(pw, t.position, side, cc.radius + 0.2, tune);
        if (across.hit && across.climbable) {
            at = across;
        } else if (!at.hit || !at.climbable) {
            const Vec3 o = t.position + side * (cc.radius + 0.35) - away * (cc.radius + 0.45);
            const WallHit round = probeWall(pw, o, -side, cc.radius + 0.7, tune);
            if (round.hit && round.climbable) at = round;
        }
    }
    if (!at.hit || !at.climbable) {
        if (b.tryMantle(s, -away, tall)) return;
        return letGo(0.4, at.hit ? "the wall stopped being climbable" : "lost the wall");
    }
    normal = normalize(normal * 0.5 + at.normal * 0.5);
    const Real gap = dot(t.position - at.point, at.normal) - (cc.radius + 0.06);
    v = v - at.normal * std::clamp(gap * 8.0, Real(-3), Real(3));
    // THE FOOT: climbing down onto walkable ground
    if (inF < -0.2) {
        PhysicsWorld::RayHit g;
        if (pw.castRayHit(t.position, Vec3(0, -(cc.halfHeight + cc.radius + 0.15), 0), g) &&
            g.normal.y >= tune.climbSteepest)
            return letGo(0.3, "down off the wall");
    }
    if (b.stamina <= 0) return letGo(1.2, "too tired to hold on");
    pw.moveCharacterFree(cc.characterId, v, dt);
    const Vec3 before = t.position;
    t.position = pw.characterPosition(cc.characterId);
    b.fall.onGrounded(t.position.y);   // holding on is footing
    const Vec3 moved = t.position - before;
    b.climbCycle += (moved - at.normal * dot(moved, at.normal)).length() / 0.6;
}

void ClimbMode::pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const {
    Vec3 right, up;
    wallBasis(normal, right, up);
    // hand over hand: each reaches up the wall in turn as the body climbs
    const Real sn = std::sin(b.climbCycle * 3.14159265358979);
    const Vec3 wall = s.t.position - normal * (s.cc.radius + 0.04);
    p.mode = PlayerPose::Mode::Climb;
    // round eye height (the eye is 0.7 over the centre and a hand's breadth off the wall: much higher and the hands
    // leave the view)
    p.handL = wall + up * (0.78 + 0.13 * sn) - right * 0.24;
    p.handR = wall + up * (0.78 - 0.13 * sn) + right * 0.24;
    p.wallNormal = normal;
}

}  // namespace engine
