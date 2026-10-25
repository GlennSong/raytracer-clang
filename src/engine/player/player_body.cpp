#include "player_body.h"

#include "../block_body.h"
#include "../ladder.h"        // Climbing
#include "../particles.h"
#include "../player_pose.h"
#include "../systems/underwater_system.h"
#include "../../log.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

namespace engine {

// --- the machine ---------------------------------------------------------------------------------------------------

void PlayerBody::step(BodyStep& s) {
    ++steps_;
    Transform& t = s.t;
    CharacterController& cc = s.cc;
    PhysicsWorld& pw = s.pw;
    PlayerIntent& in = s.in;
    auto finish = [&]() {
        publish(s);
        lastBodyPos = t.position;
        haveLastBodyPos = true;
    };
    if (!spawnCaptured) {   // the authored spawn
        spawnPos = t.position;
        spawnCaptured = true;
        fall.onSpawnCaptured(t.position.y);
    }
    // the standing capsule is whatever the level authored: captured once, the crouched one derived from it (a level
    // with a taller player crouches proportionally)
    if (!standCaptured) {
        standHalfHeight = cc.halfHeight;
        standRadius = cc.radius;
        standCaptured = true;
    }

    // THE INTERRUPTS, over every mode. A LADDER taken (LadderSystem's Climbing): nothing else applies on the rungs.
    if (s.world.has<Climbing>(s.e)) {
        enter(ladder);
        ladder.step(*this, s);
        if (!s.world.has<Climbing>(s.e)) enter(onFoot);   // stepped off it
        return finish();
    }
    // THE WATER (#43): deep enough at the feet floats the player (UnderwaterSystem knows whose water is where).
    // Falling in ends whatever the body was doing: on foot again, and that is swimming.
    {
        const Real feetY = t.position.y - (cc.halfHeight + cc.radius);
        const UnderwaterSystem::Surface w = UnderwaterSystem::surfaceAt(s.world, t.position.x, t.position.z);
        const Real depth = w.kind != UnderwaterSystem::Water::None ? Real(w.level) - feetY : Real(-1);
        const GroundState g = pw.characterGroundState(cc.characterId);
        const bool was = water.swimming;
        water.update(depth, g != GroundState::InAir);
        cc.swimming = water.swimming;
        if (water.swimming != was) {
            LOG_INFO << (water.swimming ? "[swim] afloat" : "[swim] standing again") << " at (" << t.position.x << ", "
                     << t.position.z << "), water depth " << depth << " m";
            event(water.swimming ? "afloat" : "out of the water");
            if (water.swimming) {
                enter(onFoot);   // into the water off a wall or a ledge: swim
                swimVy = pw.characterVelocity(cc.characterId).y;   // the plunge carries on in
                splashVy_ = -swimVy;
            }
        }
        swimLevel = w.level;
    }
    if (in.jump) {   // the press is the gate's to spend (a jump, a pull-up, a lunge up a wall)
        in.jump = false;
        if (in.active) gate.wantJump(tune);
    }
    // A TELEPORT (further than any step goes) ends whatever the body was doing: no canopy, wall or ledge comes
    // along to the new place
    if (haveLastBodyPos && !is(onFoot) && !is(swim) && (t.position - lastBodyPos).length() > 4.0) {
        if (ragdoll.id != PhysicsWorld::INVALID_RAGDOLL) {
            pw.removeRagdoll(ragdoll.id);
            ragdoll.id = PhysicsWorld::INVALID_RAGDOLL;
        }
        enter(onFoot);
        airVel = Vec3(0, 0, 0);
        event("teleported");
    }
    // LIMP: nothing but getting up again (G again does nothing: it waited, and dropped the body the moment it got up)
    if (is(ragdoll)) {
        in.limp = false;
        ragdoll.step(*this, s);
        return finish();
    }
    // GOING LIMP on purpose, or thrown (off a bike: the velocity the thrower handed over, else the body's own)
    if (in.limp) {
        in.limp = false;
        Vec3 v = pw.characterVelocity(cc.characterId);
        const std::string given = s.settings.getString("player.limpVelocity", "");
        if (!given.empty()) {
            s.settings.setString("player.limpVelocity", "");
            double x = 0, y = 0, z = 0;
            if (std::sscanf(given.c_str(), "%lf %lf %lf", &x, &y, &z) == 3) v = Vec3(x, y, z);
        }
        if (!water.swimming) {
            goLimp(s, v, given.empty() ? "went limp" : "thrown off");
            return finish();
        }
    }
    regrab = std::max(Real(0), regrab - s.dt);
    landed = std::max(Real(0), landed - s.dt * 3.0);
    // THE HANDS: the grapple's aim, and a shot (not with both hands on a ledge)
    if (mode_->canAim()) hands.aimAndFire(*this, s);
    // THE MODE: its ways out first (the mode they lead to steps this same step), then its step
    for (int hop = 0; hop < 4; ++hop) {
        const BodyMode* was = mode_;
        mode_->leave(*this, s);
        if (mode_ == was) break;
    }
    mode_->step(*this, s);
    finish();
}

void PlayerBody::event(const std::string& what) {
    lastEvent = what;
    LOG_INFO << "[move] " << what;
}

void PlayerBody::respawn(PhysicsWorld& pw, CharacterId id, bool manual) {
    pw.setCharacterPosition(id, spawnPos);
    fall.onRespawn(manual);
}

// --- the ways into a mode ------------------------------------------------------------------------------------------

bool PlayerBody::tryMantle(BodyStep& s, const Vec3& facing, Real maxHeight) {
    Ledge l;
    if (!findLedge(s.pw, s.cc.characterId, s.t.position, s.cc.halfHeight, s.cc.radius, facing, maxHeight, tune, l))
        return false;
    mantle.move.start(s.t.position, l, tune);
    enter(mantle);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "pull up onto a ledge %.2f m above the feet", l.height);
    event(buf);
    return true;
}

bool PlayerBody::tryGrab(BodyStep& s, const Vec3& facing) {
    if (regrab > 0 || stamina <= 0) return false;
    const WallHit w =
        probeWall(s.pw, s.t.position + Vec3(0, s.cc.halfHeight * 0.5, 0), facing, s.cc.radius + 0.3, tune);
    if (!w.hit || !w.climbable) return false;
    const Vec3 n = normalize(Vec3(w.normal.x, 0, w.normal.z));
    if (dot(n, facing) > -0.6) return false;   // brushing past it, not reaching for it
    climb.normal = w.normal;
    climb.lunge = 0;
    enter(climb);
    airVel = Vec3(0, 0, 0);
    event("grab the wall");
    return true;
}

bool PlayerBody::tryWallJump(BodyStep& s, const Vec3& facing) {
    PhysicsWorld& pw = s.pw;
    // look for the wall the way the body is going, the way it faces, and to either side: the nearest close one
    const Vec3 going = Vec3(airVel.x, 0, airVel.z).lengthSquared() > 0.25 ? normalize(Vec3(airVel.x, 0, airVel.z))
                                                                           : facing;
    const Vec3 dirs[] = {going, facing, Vec3(-going.z, 0, going.x), Vec3(going.z, 0, -going.x)};
    const Vec3 from = s.t.position + Vec3(0, s.cc.halfHeight * 0.3, 0);
    WallHit best;
    for (const Vec3& d : dirs) {
        const WallHit w = probeWall(pw, from, d, s.cc.radius + 0.45, tune);
        if (!w.hit || std::fabs(w.normal.y) > 0.5) continue;   // a wall, not a floor or a ceiling
        if (!best.hit || w.distance < best.distance) best = w;
    }
    if (!best.hit) return false;
    const Vec3 n = normalize(Vec3(best.normal.x, 0, best.normal.z));
    if (wallJumped && dot(n, wallJumpNormal) > 0.7) return false;   // the same wall again: no climbing it by kicks
    const Vec3 h(airVel.x, 0, airVel.z);
    const Vec3 along = h - n * dot(h, n);   // keep the run along the wall, lose the speed into it
    airVel = along + n * tune.wallJumpOut;
    pw.launchCharacter(s.cc.characterId, tune.wallJumpUp);
    gate.sinceLaunch = 0;
    jumpRising = false;   // the kick is a tap: always the whole of it
    wallJumped = true;
    wallJumpNormal = n;
    regrab = std::max(regrab, Real(0.4));   // a moment before reaching for the next wall grabs it
    pw.moveCharacter(s.cc.characterId, airVel, s.dt);
    s.t.position = pw.characterPosition(s.cc.characterId);
    event("jump off the wall");
    return true;
}

bool PlayerBody::tryDeploy(BodyStep& s) {
    if (!hands.holdingGlider()) return false;
    PhysicsWorld& pw = s.pw;
    const Real feet = s.cc.halfHeight + s.cc.radius;
    PhysicsWorld::RayHit ground;
    if (pw.castRayHit(s.t.position, Vec3(0, -(feet + tune.deployMinHeight), 0), ground)) {
        event("too low to open");
        return false;
    }
    glide.velocity = pw.characterVelocity(s.cc.characterId);
    const Vec3 h(glide.velocity.x, 0, glide.velocity.z);
    Vec3 look = s.in.look;
    look.y = 0;
    glide.heading = h.length() > 0.5 ? normalize(h) : look.lengthSquared() > 1e-6 ? normalize(look) : Vec3(0, 0, -1);
    glide.open = 0;
    const bool chute = hands.tool == Tool::Parachute;
    enter(chute ? static_cast<BodyMode&>(parachute) : wingsuit);
    if (chute) ++chuteOpens;
    event(chute ? "parachute open" : "wingsuit spread");
    return true;
}

bool PlayerBody::attachRope(BodyStep& s, const Vec3& at) {
    PhysicsWorld& pw = s.pw;
    CharacterController& cc = s.cc;
    GrappleMode& g = grapple;
    g.anchor = at;
    const Vec3 hand = s.t.position + Vec3(0, cc.halfHeight + cc.radius * 0.5, 0);
    g.ropeLen = std::max(tune.grappleMinRope, (g.anchor - hand).length());
    g.ropeVel = pw.characterVelocity(cc.characterId);
    if (!is(onFoot) && !is(parachute) && !is(wingsuit)) return false;
    // HOOKED FROM THE GROUND: a hop and a hard pull, reeling until the swing would clear the ground under the anchor;
    // then the reel takes up the slack as the body flies in, so the rope catches it into a swing (Glenn, 2026-10-24:
    // "If you grapple from the ground without any jump or anything you will fall over immediately" -- the pull threw
    // the body in faster than the rope shortened, the rope went slack and it came down in an arc with no swing).
    g.yank = 0;
    g.takeUp = false;
    if (pw.characterGroundState(cc.characterId) == GroundState::OnGround) {
        PhysicsWorld::RayHit below;
        const Real under = pw.castRayHit(g.anchor - Vec3(0, 0.3, 0), Vec3(0, -200, 0), below)
                               ? g.anchor.y - below.point.y : Real(200);
        g.yankTo = std::max(tune.grappleMinRope, under - (2 * (cc.halfHeight + cc.radius) + 0.5));
        g.yank = 4.0;   // at most: it ends when the rope is down to yankTo
        g.takeUp = true;
        g.ropeVel.y = std::max(g.ropeVel.y, tune.jumpSpeed);
    }
    enter(grapple);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "hooked %.0f m away", (g.anchor - hand).length());
    event(buf);
    return true;
}

void PlayerBody::goLimp(BodyStep& s, const Vec3& velocity, const std::string& why) {
    PhysicsWorld& pw = s.pw;
    if (ragdoll.id != PhysicsWorld::INVALID_RAGDOLL) pw.removeRagdoll(ragdoll.id);
    BodyPose pose;
    pose.position = s.t.position;
    pose.scale = (s.cc.halfHeight + s.cc.radius) / BlockBody::kHalfHeight;
    Vec3 face(velocity.x, 0, velocity.z);
    if (face.length() < 0.5) {
        face = s.in.look;
        face.y = 0;
    }
    face = face.lengthSquared() > 1e-8 ? normalize(face) : Vec3(0, 0, 1);
    const Real tilt = is(wingsuit) ? 1.4 : 0.0;   // a suit goes down flat
    pose.orientation = Quat::fromAxisAngle(Vec3(0, 1, 0), std::atan2(face.x, face.z)) *
                       Quat::fromAxisAngle(Vec3(1, 0, 0), tilt);
    ragdoll.id = pw.addRagdoll(blockRagdoll(pose), velocity);
    if (ragdoll.id == PhysicsWorld::INVALID_RAGDOLL) return;
    enter(ragdoll);
    ragdoll.limpFor = 0;
    ragdoll.stillFor = 0;
    gate.buffered = -1;
    event(why);
}

Real BodyMode::impactTolerance(const PlayerBody& b, const Vec3&) const { return b.tune.limpImpact; }

// what the impact took away, mostly gone: the body goes on with what it kept and a little of the rest
Vec3 BodyMode::thrown(const Vec3& meant, const Vec3& got, const Vec3&) const { return got + (meant - got) * 0.3; }

bool PlayerBody::impact(BodyStep& s, const Vec3& meant, const Vec3& got, const char* what) {
    const Real m = meant.length();
    if (m < 1e-6) return false;
    const Vec3 dir = meant * (1.0 / m);
    // ONE CONTACT, NOT ONE STEP: a body that meets a wall part way through a step loses only that part, and the rest
    // the step after (a scooter at 7 m/s: 4.6, then 2.3 -- neither past 6.5). Losses in consecutive steps add up.
    constexpr Real kStillMoving = 0.5;   // m/s lost in a step that is not a contact
    const Real lost = speedLost(meant, got);
    contactLoss_ = lost > kStillMoving && lastLossStep_ + 1 == steps_ ? contactLoss_ + lost : lost;
    lastLossStep_ = lost > kStillMoving ? steps_ : 0;
    if (contactLoss_ <= mode_->impactTolerance(*this, dir)) return false;
    const Real total = contactLoss_;
    contactLoss_ = 0;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%s at %.0f m/s", what, std::max(m, total));
    goLimp(s, mode_->thrown(meant, got, dir), buf);
    return true;
}

// --- the pose ------------------------------------------------------------------------------------------------------

// WHAT THE BODY KICKS UP (engine/particles.h): bursts as things happen, and the two steady flows turned up or off
void PlayerBody::effects(BodyStep& s) {
    const Real dt = s.dt;
    const Transform& t = s.t;
    const CharacterController& cc = s.cc;
    const Vec3 feet = t.position - Vec3(0, cc.halfHeight + cc.radius, 0);
    // THE HANDS CATCHING HOLD (for MotionAudioSystem's grab): onto a wall, a ledge, the bars or a ladder, and each
    // catch at the top of a lunge up a wall
    const bool held = is(climb) || is(mantle) || is(bars) || s.world.has<Climbing>(s.e);
    if ((held && !heldWas_) || (is(climb) && lungeWas_ > 0 && climb.lunge <= 0)) ++grabs;
    heldWas_ = held;
    lungeWas_ = climb.lunge;
    if (!fxSeen_) {
        fxOllies_ = ollies; fxLands_ = boardLands; fxBites_ = hands.hookBites;
        fxSeen_ = true;
    }
    // a board: dust off the kicking foot, a ring of it at the wheels on an ollie and a landing
    if (is(board) && board.kickClock > 0 && fxKickWas_ <= 0)
        requestParticles(s.world, "dust", feet - board.heading * 0.25, board.heading * -1.0, 6);
    fxKickWas_ = is(board) ? board.kickClock : 0;
    if (ollies != fxOllies_) requestParticles(s.world, "dust", feet, Vec3(), 8, 1.2);
    if (boardLands != fxLands_)
        requestParticles(s.world, "dust", feet, Vec3(), 6 + static_cast<int>(landSpeed * 2), 1.0 + landSpeed * 0.25);
    // the hook biting: sparks off what it hit, back toward the hand
    if (hands.hookBites != fxBites_) {
        Vec3 back = t.position - hands.hookTarget;
        back = back.lengthSquared() > 1e-6 ? normalize(back) : Vec3(0, -1, 0);
        requestParticles(s.world, "spark", hands.hookTarget + back * 0.1, back, 14);
    }
    fxOllies_ = ollies; fxLands_ = boardLands; fxBites_ = hands.hookBites;
    // into the water: a splash, bigger the harder
    if (splashVy_ > 2.0) {
        ParticleRequest r;
        r.effect = "splash";
        r.at = Vec3(t.position.x, swimLevel + 0.05, t.position.z);
        r.count = 10 + static_cast<int>(splashVy_ * 3);
        r.scale = 1.0 + splashVy_ * 0.12;
        requestParticles(s.world, r);
    }
    splashVy_ = 0;
    // under it: a trickle of bubbles from the mouth, and a breath let out every few seconds
    const bool under = water.swimming && s.in.eye.y < swimLevel - 0.05;
    const bool third = s.settings.getBool("playerThirdPerson", false);
    const Vec3 mouth = third ? t.position + Vec3(0, cc.halfHeight * 0.9, 0)
                                   : s.in.eye + s.in.look * 0.35 - Vec3(0, 0.12, 0);
    if (ParticleEmitter* b = bubbleEmitter.valid() ? s.world.get<ParticleEmitter>(bubbleEmitter) : nullptr) {
        b->rate = under ? 2.5 : 0;
        b->ceiling = swimLevel;
        if (Transform* bt = s.world.get<Transform>(bubbleEmitter)) bt->position = mouth;
    }
    if (under && (breathIn_ -= dt) <= 0) {
        breathIn_ = 3.0 + 3.0 * std::fmod(swimClock * 0.618, 1.0);
        ParticleRequest r;
        r.effect = "breath";
        r.at = mouth;
        r.ceiling = swimLevel;
        requestParticles(s.world, r);
    }
    // under a canopy, in a wingsuit: the air going past, hung ahead and stretched against the body's flight
    if (ParticleEmitter* st = streakEmitter.valid() ? s.world.get<ParticleEmitter>(streakEmitter) : nullptr) {
        const bool gliding = is(parachute) || is(wingsuit);
        st->rate = !gliding ? 0 : is(parachute) ? 24.0 : 10.0 + glide.velocity.length() * 1.2;
        st->entityVelocity = glide.velocity;
        if (Transform* tt = s.world.get<Transform>(streakEmitter)) tt->position = t.position;
    }
}

void PlayerBody::publish(BodyStep& s) {
    PlayerPose* p = s.world.get<PlayerPose>(s.e);
    if (!p) {
        s.world.add<PlayerPose>(s.e, PlayerPose{});   // (another type's pool: safe inside the walk)
        p = s.world.get<PlayerPose>(s.e);
        if (!p) return;
    }
    hands.stepHook(*this, s);
    effects(s);
    static_assert(PlayerPose::kRopePoints == RopeLine::kPoints, "the pose carries the whole rope");
    p->ropePoints = 0;
    if (hands.rope.active())
        for (const Vec3& q : hands.rope.points()) p->rope[p->ropePoints++] = q;
    p->hookFlying = hands.hook == PlayerHands::Hook::Flying;
    p->reel = is(grapple) ? grapple.reel : 0;
    p->hookShots = hands.hookShots;
    p->hookBites = hands.hookBites;
    p->ropeSnaps = ropeSnaps;
    p->snapSpeed = snapSpeed;
    p->chuteOpens = chuteOpens;
    p->ollies = ollies;
    p->boardLands = boardLands;
    p->landSpeed = landSpeed;
    p->tool = slotOf(hands.tool);
    if (!s.in.active) {   // the freecam: nobody's eye, no arms
        p->mode = PlayerPose::Mode::Hidden;
        p->handsFree = 2;
        return;
    }
    PhysicsWorld& pw = s.pw;
    const Vec3 v = pw.characterVelocity(s.cc.characterId);
    {
        const GroundState gs = pw.characterGroundState(s.cc.characterId);
        p->grounded = gs == GroundState::OnGround || gs == GroundState::OnSteepGround;
    }
    p->speed = std::sqrt(v.x * v.x + v.z * v.z);
    p->stride = stride;
    p->run = s.in.active && s.in.run;
    p->crouched = crouched;
    p->stamina = std::clamp(stamina / std::max(tune.staminaMax, Real(0.1)), Real(0), Real(1));
    p->landed = landed;
    p->climbCycle = climbCycle;
    p->swimPhase = strokePhase;
    p->swimStroke = swimStroke;
    p->waterLevel = swimLevel;
    p->swimClock = swimClock;
    p->underwater = water.swimming && s.in.eye.y < swimLevel - 0.05;
    p->grabs = grabs;
    p->progress = 0;
    p->handsFree = mode_->handsFree(*this);
    if (mode_->movesFreely()) {
        p->velocity = v;
        p->airspeed = v.length();
        p->anchor = grapple.anchor;
    }
    mode_->pose(*this, s, *p);
}

}  // namespace engine
