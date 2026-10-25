// THE MODES THE GEAR MAKES (ADR-0246): a canopy, a wingsuit, the grapple's rope, a board -- and the playground's
// swing and bars, and limp.

#include "player_body.h"

#include "../block_body.h"
#include "../player_pose.h"
#include "../playground.h"
#include "../systems/underwater_system.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace engine {

// --- gliding -------------------------------------------------------------------------------------------------------

void GlideMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    GlideFlight& g = b.glide;
    const MovementTuning& tune = b.tune;
    const Real dt = s.dt;
    const Real inF = s.in.forward;
    const Vec3 look = s.in.look;
    auto stop = [&](const std::string& why) {
        b.enter(b.onFoot);
        b.airVel = Vec3(g.velocity.x, 0, g.velocity.z);
        b.event(why);
    };
    if (s.in.active && s.in.crouchHeld) {   // cut away: freefall
        b.regrab = 0.5;
        return stop(canopy ? "cut away the canopy" : "folded the wingsuit");
    }
    if (b.gate.buffered > 0) {
        b.gate.buffered = -1;
        if (!canopy) {   // the suit's own chute
            b.enter(b.parachute);
            g.open = 0;
            const Vec3 h(g.velocity.x, 0, g.velocity.z);
            if (h.length() > 0.5) g.heading = normalize(h);
            b.event("parachute open (off the wingsuit)");
        }
    }
    const bool chute = b.is(b.parachute);   // (this step: the suit may just have opened its chute)
    if (chute) {
        g.open = std::min(Real(1), g.open + dt / std::max(tune.chuteOpenTime, Real(0.05)));
        g.velocity = parachuteStep(g.velocity, g.heading, look, inF, g.open, dt, tune);
    } else {
        g.velocity = wingsuitStep(g.velocity, look, dt, tune);
    }
    const Vec3 was = t.position;
    pw.moveCharacterFree(s.cc.characterId, g.velocity, dt);
    t.position = pw.characterPosition(s.cc.characterId);
    b.fall.onGrounded(t.position.y);   // flying is not falling
    // DOWN: on the ground, or into something hard enough to stop the way
    const Vec3 moved = t.position - was;
    const Real want = g.velocity.length() * dt;
    const GroundState gs = pw.characterGroundState(s.cc.characterId);
    const bool blocked = want > 0.05 && moved.length() < want * 0.4;
    if (gs == GroundState::OnGround || gs == GroundState::OnSteepGround || blocked) {
        const Real impact = g.velocity.length();
        b.landed = std::min(Real(1), impact / 8.0);
        char buf[96];
        if (!chute && impact > 20.0) {
            std::snprintf(buf, sizeof(buf), "crashed the wingsuit at %.0f m/s", impact);
            const Vec3 hit = g.velocity * 0.5;
            g.velocity = Vec3(0, 0, 0);
            return b.goLimp(s, hit, buf);
        }
        std::snprintf(buf, sizeof(buf), "%s at %.1f m/s", chute ? "landed the chute" : "landed", impact);
        g.velocity = g.velocity * 0.4;
        stop(buf);
    }
}

void GlideMode::pose(const PlayerBody& b, const BodyStep&, PlayerPose& p) const {
    p.mode = canopy ? PlayerPose::Mode::Parachute : PlayerPose::Mode::Wingsuit;
    p.canopyHeading = b.glide.heading;
    p.canopyOpen = canopy ? b.glide.open : 0;
}

// --- on the rope ---------------------------------------------------------------------------------------------------

void GrappleMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const MovementTuning& tune = b.tune;
    const Real dt = s.dt;
    const Real handUp = cc.halfHeight + cc.radius * 0.5;
    auto release = [&](const std::string& why) {
        b.enter(b.onFoot);
        b.airVel = Vec3(ropeVel.x, 0, ropeVel.z);
        b.event(why);
    };
    if (!s.in.fireHeld) return release("let go of the rope");
    const bool reelIn = s.in.active && s.in.jumpHeld;
    const bool reelOut = s.in.active && s.in.crouchHeld;
    b.gate.buffered = -1;   // jump is the reel here, not a leap
    const Real lenWas = ropeLen;
    if (yank > 0) {
        yank -= dt;
        // full speed, easing off over the last few metres so it doesn't fling the body into what it hooked
        const Real ease = std::clamp((ropeLen - yankTo) / 6.0, Real(0.35), Real(1));
        ropeLen = std::max(yankTo, ropeLen - 2.0 * tune.grappleReelIn * ease * dt);
        if (ropeLen <= yankTo + 1e-6) yank = 0;
    }
    if (reelIn) ropeLen = std::max(tune.grappleMinRope, ropeLen - tune.grappleReelIn * dt);
    if (reelOut) ropeLen = std::min(tune.grappleRange, ropeLen + tune.grappleReelOut * dt);
    // pump along the swing, steer across it
    const Real inF = s.in.forward;
    const Real inR = s.in.right;
    Vec3 along(ropeVel.x, 0, ropeVel.z);
    if (along.length() < 1.0) {
        along = s.in.look;
        along.y = 0;
    }
    along = along.lengthSquared() > 1e-8 ? normalize(along) : Vec3(0, 0, -1);
    const Vec3 across(-along.z, 0, along.x);
    Vec3 accel = along * (inF * tune.grapplePump) + across * (inR * tune.grappleSteer);
    if (yank > 0) accel.y += 9.81;   // the pull runs the body up the line, not sagging under it into the ground
    const Vec3 handWas = t.position + Vec3(0, handUp, 0);
    Vec3 hand = handWas;
    const Vec3 fromAnchor = handWas - anchor;
    if (takeUp) {   // the reel takes in what the body gives it: the rope only shortens to the hand, never pulls
        ropeLen = std::max(tune.grappleMinRope, std::min(ropeLen, fromAnchor.length()));
        const Real out = fromAnchor.length() > 1e-6 ? dot(ropeVel, normalize(fromAnchor)) : Real(0);
        if (yank <= 0 && out > 0) takeUp = false;   // swinging away from the anchor: the rope has the body now
    }
    reel = (lenWas - ropeLen) / std::max(dt, Real(1e-4));
    const bool slackWas = fromAnchor.length() < ropeLen * 0.995;
    const Real outWas = fromAnchor.length() > 1e-6 ? dot(ropeVel, normalize(fromAnchor)) : Real(0);
    ropeVel = ropeStep(hand, ropeVel, anchor, ropeLen, accel, dt, tune.grappleStretch, tune.grappleBounce);
    if (slackWas && (hand - anchor).length() >= ropeLen * 0.995 && outWas > 2.5) {   // a slack rope caught the body
        ++b.ropeSnaps;
        b.snapSpeed = outWas;
    }
    pw.moveCharacterFree(cc.characterId, (hand - handWas) * (1.0 / std::max(dt, Real(1e-4))), dt);
    const Vec3 was = t.position;
    t.position = pw.characterPosition(cc.characterId);
    const Vec3 meant = ropeVel;
    ropeVel = (t.position - was) * (1.0 / std::max(dt, Real(1e-4)));   // what walls and the ground let it do
    // swung into something, hard: the speed LOST along the way it was going (a sideways shove off a corner, or the
    // capsule pushed out of a surface, is not a slam: it was read as one at 7 m/s)
    const Real meantSpeed = meant.length();
    const Real kept = meantSpeed > 1e-6 ? std::max(Real(0), dot(ropeVel, meant * (1.0 / meantSpeed))) : Real(0);
    if (meantSpeed - kept > tune.limpImpact) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "slammed into it at %.0f m/s", meant.length());
        return b.goLimp(s, meant * 0.4, buf);
    }
    b.fall.onGrounded(t.position.y);
    // reeled right up to an edge: pull up over it
    const Vec3 toAnchor = anchor - (t.position + Vec3(0, handUp, 0));
    if (reelIn && toAnchor.length() < tune.grappleMinRope + 0.6) {
        const Vec3 face(toAnchor.x, 0, toAnchor.z);
        if (face.length() > 0.05 && b.tryMantle(s, normalize(face), 2 * (cc.halfHeight + cc.radius) + 0.6)) return;
    }
    // down on the ground with the rope slack: walking again
    const GroundState g = pw.characterGroundState(cc.characterId);
    if (g == GroundState::OnGround && (toAnchor.length() < ropeLen - 0.15 || takeUp) && !reelIn && yank <= 0)
        return release("down off the rope");
}

void GrappleMode::pose(const PlayerBody&, const BodyStep&, PlayerPose& p) const { p.mode = PlayerPose::Mode::Grapple; }

// --- on a board ----------------------------------------------------------------------------------------------------

const char* BoardMode::name(const PlayerBody& b) const { return b.hands.gear == 9 ? "scooter" : "skateboard"; }

void BoardMode::leave(PlayerBody& b, BodyStep&) {
    if (b.hands.holdingBoard()) return;
    b.enter(b.onFoot);
    b.event("off the board");
}

void BoardMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const MovementTuning& tune = b.tune;
    const Real dt = s.dt;
    const bool scooter = b.hands.gear == 9;
    const GroundState g = pw.characterGroundState(cc.characterId);
    const bool grounded = g == GroundState::OnGround || g == GroundState::OnSteepGround;
    if (grounded) {
        speed = boardStep(speed, heading, kickClock, s.in.forward, s.in.right, pw.characterGroundNormal(cc.characterId),
                          scooter ? tune.scooterPushMax : tune.boardPushMax, dt, tune);
        b.stamina = std::min(tune.staminaMax, b.stamina + tune.staminaRegen * dt);
    }
    // the ollie: up, board and all
    if (b.gate.buffered > 0 && grounded) {
        b.gate.buffered = -1;
        b.gate.sinceLaunch = 0;
        pw.launchCharacter(cc.characterId, tune.boardOllie);
        ++b.ollies;
        b.event(scooter ? "hop" : "ollie");
    }
    const Vec3 want = heading * speed;
    const Vec3 was = t.position;
    const Real vyBefore = pw.characterVelocity(cc.characterId).y;
    pw.moveCharacter(cc.characterId, want, dt);
    t.position = pw.characterPosition(cc.characterId);
    const GroundState after = pw.characterGroundState(cc.characterId);
    if (after == GroundState::OnGround || after == GroundState::OnSteepGround) b.fall.onGrounded(t.position.y);
    if (!grounded && after == GroundState::OnGround && vyBefore < -1.0) {   // the wheels back down
        ++b.boardLands;
        b.landSpeed = -vyBefore;
    }
    // into something: a wall takes the speed; hard enough and you're off and limp
    const Vec3 moved(t.position.x - was.x, 0, t.position.z - was.z);
    const Real got = dot(moved, heading) / std::max(dt, Real(1e-4));
    if (speed > 0.5 && got < speed * 0.4) {
        if (speed > tune.boardCrash) {
            char buf[64];
            std::snprintf(buf, sizeof(buf), "bailed at %.1f m/s", speed);
            const Vec3 thrown = heading * (speed * 0.7) + Vec3(0, 2.5, 0);
            speed = 0;
            return b.goLimp(s, thrown, buf);
        }
        speed = std::max(Real(0), got);
    }
    // a hard landing off a drop still drops you
    if (!grounded && after == GroundState::OnGround && vyBefore < -tune.limpFallSpeed)
        return b.goLimp(s, want + Vec3(0, vyBefore * 0.3, 0), "landed hard on the board");
    b.airVel = want;
}

void BoardMode::pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const {
    p.mode = PlayerPose::Mode::Board;
    p.board = b.hands.gear == 9 ? 1 : 0;
    p.boardHeading = heading;
    p.kick = kickClock > 0 ? 1.0 - kickClock / std::max(b.tune.boardKickTime, Real(0.05)) : 0;
    const Vec3 side(-heading.z, 0, heading.x);   // the board's left
    const Vec3 bar = s.t.position + heading * 0.42 + Vec3(0, 0.35, 0);
    p.handL = bar + side * 0.22;
    p.handR = bar - side * 0.22;
}

// --- the swing -----------------------------------------------------------------------------------------------------

void SwingMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const Real dt = s.dt;
    PlaySwing* sw = s.world.alive(seat) ? s.world.get<PlaySwing>(seat) : nullptr;
    if (!sw) return b.enter(b.onFoot);
    swingStep(sw->theta, sw->omega, sw->length, s.in.forward, dt, b.tune);
    const Vec3 up(0, 1, 0);
    const Vec3 hang = sw->forward * std::sin(sw->theta) - up * std::cos(sw->theta);
    const Vec3 at = sw->pivot + hang * sw->length;
    // sat on it: the hip on the seat, the capsule over it
    const Vec3 centre = at + up * (cc.halfHeight + cc.radius - 0.45);
    pw.setCharacterPosition(cc.characterId, centre);
    t.position = centre;
    b.fall.onGrounded(t.position.y);
    b.gate.observe(GroundState::OnGround, 0, dt);
    // off: a jump flies off with the seat's speed; crouch gets down
    if (b.gate.buffered > 0) {
        b.gate.buffered = -1;
        const Vec3 along = sw->forward * std::cos(sw->theta) + up * std::sin(sw->theta);   // the seat's way of travel
        const Vec3 v = along * (sw->omega * sw->length);
        sw->ridden = false;
        b.enter(b.onFoot);
        b.airVel = Vec3(v.x, 0, v.z);
        pw.setCharacterPosition(cc.characterId, centre + up * 0.3);
        pw.launchCharacter(cc.characterId, std::max(Real(2.0), v.y + 2.0));
        b.gate.sinceLaunch = 0;
        char buf[64];
        std::snprintf(buf, sizeof(buf), "jumped off the swing at %.1f m/s", v.length());
        b.event(buf);
        return;
    }
    if (s.in.active && s.in.crouchHeld) {
        sw->ridden = false;
        b.enter(b.onFoot);
        PhysicsWorld::RayHit g;
        Vec3 stand = sw->pivot - up * sw->length + sw->forward * 0.6;
        if (pw.castRayHit(stand + up * 1.0, Vec3(0, -3, 0), g)) stand.y = g.point.y;
        pw.setCharacterPosition(cc.characterId, stand + up * (cc.halfHeight + cc.radius + 0.05));
        b.event("off the swing");
    }
}

void SwingMode::pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const {
    const PlaySwing* sw = s.world.alive(seat) ? s.world.get<PlaySwing>(seat) : nullptr;
    if (!sw) return b.onFoot.pose(b, s, p);
    const Vec3 up(0, 1, 0), side = normalize(cross(up, sw->forward));
    const Vec3 hang = sw->forward * std::sin(sw->theta) - up * std::cos(sw->theta);
    p.mode = PlayerPose::Mode::PlaySwing;
    p.handL = sw->pivot + hang * (sw->length * 0.62) + side * 0.22;
    p.handR = sw->pivot + hang * (sw->length * 0.62) - side * 0.22;
    p.wallNormal = sw->forward;
    p.progress = sw->theta;   // (the swing's angle, for the legs)
}

// --- the monkey bars -----------------------------------------------------------------------------------------------

void BarsMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    const Real dt = s.dt;
    const MonkeyBars* mb = s.world.alive(bars) ? s.world.get<MonkeyBars>(bars) : nullptr;
    if (!mb) return b.enter(b.onFoot);
    const Vec3 ab = mb->b - mb->a;
    const Real len = std::max(ab.length(), Real(1e-3));
    const Vec3 dir = ab * (1.0 / len);
    // hand over hand the way the keys point along the bars
    Vec3 camF = s.in.look, camR = s.in.lookRight;
    camF.y = 0;
    camR.y = 0;
    const Real along = dot(normalize(camF) * s.in.forward + normalize(camR) * s.in.right, dir);
    const Real was = at;
    at += std::clamp(along, Real(-1), Real(1)) * b.tune.barsSpeed * dt;
    if (std::fabs(along) > 0.2) facing = dir * (along > 0 ? 1.0 : -1.0);
    cycle += std::fabs(at - was) / 0.4;
    const bool offEnd = at < -0.05 || at > len + 0.05;
    at = std::clamp(at, Real(0), len);
    const Vec3 grip = mb->a + dir * at;
    const Vec3 centre = grip - Vec3(0, 0.45 + b.eyeHeight, 0);
    pw.setCharacterPosition(s.cc.characterId, centre);
    t.position = centre;
    b.fall.onGrounded(t.position.y);
    b.gate.observe(GroundState::OnGround, 0, dt);
    const bool drop = b.gate.buffered > 0 || (s.in.active && s.in.crouchHeld) || offEnd;
    if (drop) {
        b.gate.buffered = -1;
        b.enter(b.onFoot);
        b.airVel = facing * (std::fabs(along) * b.tune.barsSpeed);
        b.event(offEnd ? "off the end of the bars" : "dropped off the bars");
    }
}

void BarsMode::pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const {
    const MonkeyBars* mb = s.world.alive(bars) ? s.world.get<MonkeyBars>(bars) : nullptr;
    if (!mb) return b.onFoot.pose(b, s, p);
    const Vec3 ab = mb->b - mb->a;
    const Vec3 dir = ab * (1.0 / std::max(ab.length(), Real(1e-3)));
    const Vec3 side = normalize(cross(Vec3(0, 1, 0), facing));
    const Vec3 grip = mb->a + dir * at;
    const Real reach = 0.22 * std::sin(cycle * 3.14159265358979);
    p.mode = PlayerPose::Mode::Bars;
    p.handL = grip + facing * reach + side * 0.16;
    p.handR = grip - facing * reach - side * 0.16;
    p.wallNormal = facing;
}

// --- limp ----------------------------------------------------------------------------------------------------------

void RagdollMode::step(PlayerBody& b, BodyStep& s) {
    PhysicsWorld& pw = s.pw;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    const Real dt = s.dt;
    limpFor += dt;
    Vec3 torso;
    Quat rot;
    if (!pw.ragdollPart(id, BlockBody::Torso, torso, rot)) return b.enter(b.onFoot);
    t.position = torso;   // the view follows the body
    stillFor = pw.ragdollSpeed(id) < 0.4 ? stillFor + dt : 0;
    const UnderwaterSystem::Surface w = UnderwaterSystem::surfaceAt(s.world, torso.x, torso.z);
    const bool inWater = w.kind != UnderwaterSystem::Water::None && torso.y < w.level;
    const bool wantUp = b.gate.buffered > 0 && limpFor > 0.8 && pw.ragdollSpeed(id) < 2.5;
    // fell off the world while limp: the net still catches it
    PhysicsWorld::RayHit below;
    if (b.spawnCaptured && b.fall.shouldRespawn(torso.y) && !pw.castRayHit(torso, Vec3(0, -5000, 0), below)) {
        pw.removeRagdoll(id);
        id = PhysicsWorld::INVALID_RAGDOLL;
        b.enter(b.onFoot);
        b.respawn(pw, cc.characterId, /*manual=*/false);
        t.position = b.spawnPos;
        return;
    }
    if (!(inWater || wantUp || stillFor > b.tune.getUpAfter)) return;
    // UP: stand the capsule over where the body lies (on the ground under the torso), or float it in the water
    const Real lift = cc.halfHeight + cc.radius;
    Vec3 stand = torso + Vec3(0, lift, 0);
    PhysicsWorld::RayHit ground;
    if (!inWater && pw.castRayHit(torso + Vec3(0, 1.0, 0), Vec3(0, -3.0, 0), ground)) {
        // (the ray may meet the body itself: its parts are leaving the world below, so look past them)
        stand = Vec3(torso.x, ground.point.y + lift + 0.05, torso.z);
    }
    pw.removeRagdoll(id);
    id = PhysicsWorld::INVALID_RAGDOLL;
    for (Real up : {0.0, 0.3, 0.7, 1.2})
        if (pw.characterFitsAt(cc.characterId, stand + Vec3(0, up, 0))) {
            stand = stand + Vec3(0, up, 0);
            break;
        }
    pw.setCharacterPosition(cc.characterId, stand);
    t.position = stand;
    b.fall.onGrounded(stand.y);
    b.airVel = Vec3(0, 0, 0);
    b.gate.buffered = -1;
    b.enter(b.onFoot);
    b.event(inWater ? "came up in the water" : "got up");
}

void RagdollMode::pose(const PlayerBody&, const BodyStep& s, PlayerPose& p) const {
    p.mode = PlayerPose::Mode::Ragdoll;
    for (int i = 0; i < BlockBody::kParts; ++i) s.pw.ragdollPart(id, i, p.ragdollPos[i], p.ragdollRot[i]);
}

}  // namespace engine
