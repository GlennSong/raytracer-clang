#include "locomotion.h"

#include "../renderer/settings.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace engine {

namespace {

Vec3 flat(const Vec3& v) {
    const Real l = std::sqrt(v.x * v.x + v.z * v.z);
    return l > 1e-6 ? Vec3(v.x / l, 0, v.z / l) : Vec3(0, 0, 0);
}

Real smooth(Real s) {
    s = std::clamp(s, Real(0), Real(1));
    return s * s * (3 - 2 * s);
}

}  // namespace

const std::vector<MovementTuning::Field>& MovementTuning::fields() {
    using T = MovementTuning;
    static const std::vector<Field> f = {
        {"walkSpeed", &T::walkSpeed, 1, 12, "Walk"},
        {"runScale", &T::runScale, 1, 3, "Walk"},
        {"jumpSpeed", &T::jumpSpeed, 2, 9, "Jump"},
        {"coyoteTime", &T::coyoteTime, 0, 0.4, "Jump"},
        {"jumpBuffer", &T::jumpBuffer, 0, 0.4, "Jump"},
        {"steepPush", &T::steepPush, 0, 6, "Jump"},
        {"wedgedTime", &T::wedgedTime, 0, 1, "Jump"},
        {"airAccel", &T::airAccel, 0, 20, "Jump"},
        {"mantleReach", &T::mantleReach, 0.1, 1.2, "Mantle"},
        {"mantleMinHeight", &T::mantleMinHeight, 0.2, 1.5, "Mantle"},
        {"mantleMaxHeight", &T::mantleMaxHeight, 0.8, 3.0, "Mantle"},
        {"groundMantleMax", &T::groundMantleMax, 0.5, 2.5, "Mantle"},
        {"mantleRiseSpeed", &T::mantleRiseSpeed, 0.5, 8, "Mantle"},
        {"mantleOverTime", &T::mantleOverTime, 0.05, 1, "Mantle"},
        {"climbSpeed", &T::climbSpeed, 0.2, 5, "Climb"},
        {"climbRunScale", &T::climbRunScale, 1, 3, "Climb"},
        {"climbSteepest", &T::climbSteepest, 0, 0.95, "Climb"},
        {"climbOverhang", &T::climbOverhang, -0.9, 0, "Climb"},
        {"staminaMax", &T::staminaMax, 1, 60, "Climb"},
        {"staminaRegen", &T::staminaRegen, 0, 20, "Climb"},
        {"climbIdleDrain", &T::climbIdleDrain, 0, 2, "Climb"},
        {"climbJumpCost", &T::climbJumpCost, 0, 6, "Climb"},
        {"climbLungeSpeed", &T::climbLungeSpeed, 0.5, 10, "Climb"},
        {"climbLungeTime", &T::climbLungeTime, 0.05, 1, "Climb"},
        {"wallKick", &T::wallKick, 0, 10, "Climb"},
        {"swimSpeed", &T::swimSpeed, 0.5, 6, "Swim"},
        {"swimRunScale", &T::swimRunScale, 1, 3, "Swim"},
        {"strokePeriod", &T::strokePeriod, 0.4, 3, "Swim"},
        {"strokeSurge", &T::strokeSurge, 0, 0.95, "Swim"},
        {"buoyancy", &T::buoyancy, 1, 60, "Swim"},
        {"waterDamping", &T::waterDamping, 0, 12, "Swim"},
        {"waterDrag", &T::waterDrag, 0, 3, "Swim"},
        {"deployMinHeight", &T::deployMinHeight, 0, 30, "Glide"},
        {"chuteOpenTime", &T::chuteOpenTime, 0.1, 3, "Glide"},
        {"chuteSpeed", &T::chuteSpeed, 1, 25, "Glide"},
        {"chuteSink", &T::chuteSink, 0.5, 15, "Glide"},
        {"chuteDiveSpeed", &T::chuteDiveSpeed, 1, 30, "Glide"},
        {"chuteDiveSink", &T::chuteDiveSink, 0.5, 20, "Glide"},
        {"chuteFlareSpeed", &T::chuteFlareSpeed, 0, 15, "Glide"},
        {"chuteFlareSink", &T::chuteFlareSink, 0.2, 10, "Glide"},
        {"chuteTurnRate", &T::chuteTurnRate, 5, 240, "Glide"},
        {"chuteResponse", &T::chuteResponse, 0.2, 8, "Glide"},
        {"suitDrag", &T::suitDrag, 0.0003, 0.01, "Glide"},
        {"suitInducedDrag", &T::suitInducedDrag, 0, 100, "Glide"},
        {"suitStallSpeed", &T::suitStallSpeed, 2, 40, "Glide"},
        {"suitTurn", &T::suitTurn, 0.2, 10, "Glide"},
        {"suitPitchDown", &T::suitPitchDown, 10, 89, "Glide"},
        {"suitPitchUp", &T::suitPitchUp, 0, 60, "Glide"},
        {"grappleRange", &T::grappleRange, 5, 200, "Grapple"},
        {"grappleMinRange", &T::grappleMinRange, 0.5, 10, "Grapple"},
        {"grappleCone", &T::grappleCone, 0, 30, "Grapple"},
        {"grappleReelIn", &T::grappleReelIn, 0, 30, "Grapple"},
        {"grappleReelOut", &T::grappleReelOut, 0, 20, "Grapple"},
        {"grappleMinRope", &T::grappleMinRope, 0.5, 10, "Grapple"},
        {"grapplePump", &T::grapplePump, 0, 15, "Grapple"},
        {"grappleSteer", &T::grappleSteer, 0, 10, "Grapple"},
        {"boardPush", &T::boardPush, 0.2, 6, "Board"},
        {"boardPushMax", &T::boardPushMax, 2, 20, "Board"},
        {"scooterPushMax", &T::scooterPushMax, 2, 20, "Board"},
        {"boardKickTime", &T::boardKickTime, 0.1, 2, "Board"},
        {"boardRoll", &T::boardRoll, 0, 3, "Board"},
        {"boardBrake", &T::boardBrake, 0.5, 15, "Board"},
        {"boardCarve", &T::boardCarve, 1, 20, "Board"},
        {"boardTurnMax", &T::boardTurnMax, 20, 400, "Board"},
        {"boardOllie", &T::boardOllie, 0, 8, "Board"},
        {"boardCrash", &T::boardCrash, 2, 20, "Board"},
        {"swingPump", &T::swingPump, 0, 6, "Playground"},
        {"swingDamping", &T::swingDamping, 0, 1, "Playground"},
        {"swingMaxDeg", &T::swingMaxDeg, 20, 89, "Playground"},
        {"barsSpeed", &T::barsSpeed, 0.2, 4, "Playground"},
        {"limpFallSpeed", &T::limpFallSpeed, 5, 60, "Ragdoll"},
        {"limpImpact", &T::limpImpact, 3, 60, "Ragdoll"},
        {"getUpAfter", &T::getUpAfter, 0.2, 10, "Ragdoll"},
    };
    return f;
}

const std::vector<MovementTuning::Flag>& MovementTuning::flags() {
    using T = MovementTuning;
    static const std::vector<Flag> f = {
        {"grabInAir", &T::grabInAir, "Climb"},
        {"grabFromGround", &T::grabFromGround, "Climb"},
    };
    return f;
}

void MovementTuning::load(const Settings& s) {
    for (const Field& f : fields()) this->*f.value = s.getDouble(std::string("movement.") + f.key, this->*f.value);
    for (const Flag& f : flags()) this->*f.value = s.getBool(std::string("movement.") + f.key, this->*f.value);
}

void MovementTuning::save(Settings& s) const {
    for (const Field& f : fields()) s.setDouble(std::string("movement.") + f.key, this->*f.value);
    for (const Flag& f : flags()) s.setBool(std::string("movement.") + f.key, this->*f.value);
}

// --- jump gate ---------------------------------------------------------------

void JumpGate::observe(GroundState g, Real verticalSpeed, Real dt) {
    sinceLaunch += dt;
    if (buffered > 0) buffered -= dt;
    sinceGround = g == GroundState::OnGround ? 0 : sinceGround + dt;
    // Touching something that isn't walkable and not falling off it: wedged between rocks, perched on an edge.
    const bool touching = g == GroundState::OnSteepGround || g == GroundState::NotSupported;
    wedged = touching && std::fabs(verticalSpeed) < 0.6 ? wedged + dt : 0;
}

JumpGate::Launch JumpGate::take(GroundState g, const Vec3& groundNormal, const MovementTuning& t, Real) {
    Launch l;
    if (buffered <= 0 || sinceLaunch < 0.25) return l;
    const bool walkable = g == GroundState::OnGround || sinceGround <= t.coyoteTime;
    const bool steep = g == GroundState::OnSteepGround;
    const bool wedge = g == GroundState::NotSupported && wedged >= t.wedgedTime;
    if (!walkable && !steep && !wedge) return l;
    l.go = true;
    if (!walkable) l.push = flat(groundNormal) * t.steepPush;   // off the slope, not up it
    buffered = -1;
    sinceLaunch = 0;
    sinceGround = 1e9;
    return l;
}

// --- ledges ------------------------------------------------------------------

bool findLedge(const PhysicsWorld& world, CharacterId id, const Vec3& centre, Real halfHeight, Real radius,
               const Vec3& facing, Real maxHeight, const MovementTuning& t, Ledge& out) {
    const Vec3 f = flat(facing);
    if (f.lengthSquared() < 0.5) return false;
    const Real feet = centre.y - (halfHeight + radius);
    // The wall: level rays forward at a few heights up the body, the nearest one that meets a wall.
    PhysicsWorld::RayHit wall;
    bool haveWall = false;
    for (Real h : {0.35, 0.8, 1.25, 1.7}) {
        if (h >= maxHeight) break;
        PhysicsWorld::RayHit hit;
        const Vec3 o(centre.x, feet + h, centre.z);
        if (!world.castRayHit(o, f * (radius + t.mantleReach), hit)) continue;
        if (std::fabs(hit.normal.y) > 0.6 || hit.dynamic) continue;   // a slope or a crate, not a wall
        const Real d = std::hypot(hit.point.x - o.x, hit.point.z - o.z);
        if (!haveWall || d < std::hypot(wall.point.x - centre.x, wall.point.z - centre.z)) wall = hit;
        haveWall = true;
    }
    if (!haveWall) return false;
    const Vec3 n = flat(wall.normal);
    if (dot(n, f) > -0.5) return false;   // glancing along it, not facing it
    // The top: straight down just behind the wall's face, from the highest reach.
    bool haveTop = false;
    Real topY = 0;
    const Real from = feet + maxHeight + 0.05;
    for (Real inset : {0.22, 0.12, 0.35}) {
        const Vec3 o = Vec3(wall.point.x, from, wall.point.z) - n * inset;
        PhysicsWorld::RayHit hit;
        if (!world.castRayHit(o, Vec3(0, -(from - feet - t.mantleMinHeight + 0.05), 0), hit)) continue;
        if (hit.point.y >= from - 0.01) continue;   // started inside: the wall goes on up past reach
        if (hit.normal.y < 0.7 || hit.dynamic) continue;
        topY = hit.point.y;
        haveTop = true;
        break;
    }
    if (!haveTop) return false;
    const Real height = topY - feet;
    if (height < t.mantleMinHeight || height > maxHeight) return false;
    const Real lift = halfHeight + radius;
    // Standing on the top, a body's width in from the edge: there has to be floor there at the top's height.
    Vec3 stand = Vec3(wall.point.x, topY, wall.point.z) - n * (radius + 0.12);
    PhysicsWorld::RayHit under;
    if (!world.castRayHit(stand + Vec3(0, 0.4, 0), Vec3(0, -0.75, 0), under)) return false;
    if (std::fabs(under.point.y - topY) > 0.25 || under.normal.y < 0.7) return false;
    stand.y = under.point.y + lift + 0.04;
    const Vec3 rise = Vec3(wall.point.x, topY + lift + 0.06, wall.point.z) + n * (radius + 0.04);
    if (!world.characterFitsAt(id, stand) || !world.characterFitsAt(id, rise)) return false;
    out.edge = Vec3(wall.point.x, topY, wall.point.z);
    out.normal = n;
    out.topY = topY;
    out.height = height;
    out.rise = rise;
    out.stand = stand;
    return true;
}

void MantleMove::start(const Vec3& centre, const Ledge& l, const MovementTuning& tune) {
    from = centre;
    rise = l.rise;
    stand = l.stand;
    ledge = l;
    t = 0;
    const Real up = std::max(Real(0), rise.y - from.y);
    riseTime = std::clamp(up / std::max(tune.mantleRiseSpeed, Real(0.1)), Real(0.08), Real(1.0));
    overTime = std::max(tune.mantleOverTime, Real(0.02));
}

Vec3 MantleMove::at(Real time) const {
    if (time < riseTime) {
        const Real s = smooth(time / riseTime);
        // the climb comes first, the step in to the wall early: hands on the edge, body hauled up against it
        const Real sIn = smooth(std::min(Real(1), time / (riseTime * 0.5)));
        return Vec3(from.x + (rise.x - from.x) * sIn, from.y + (rise.y - from.y) * s,
                    from.z + (rise.z - from.z) * sIn);
    }
    const Real s = smooth((time - riseTime) / overTime);
    return rise + (stand - rise) * s;
}

// --- walls -------------------------------------------------------------------

WallHit probeWall(const PhysicsWorld& world, const Vec3& from, const Vec3& dir, Real length,
                  const MovementTuning& t) {
    WallHit w;
    PhysicsWorld::RayHit hit;
    if (!world.castRayHit(from, dir * length, hit)) return w;
    w.hit = true;
    w.point = hit.point;
    w.normal = hit.normal;
    w.distance = (hit.point - from).length();
    w.climbable = !hit.dynamic && hit.normal.y < t.climbSteepest && hit.normal.y > t.climbOverhang;
    return w;
}

void wallBasis(const Vec3& normal, Vec3& right, Vec3& up) {
    right = flat(Vec3(normal.z, 0, -normal.x));
    up = normalize(Vec3(0, 1, 0) - normal * normal.y);
}

Real swimVerticalStep(Real vy, Real centreY, Real floatY, bool rise, bool dive, Real dt, const MovementTuning& t) {
    Real a = t.buoyancy * (floatY - centreY) - t.waterDamping * vy - t.waterDrag * vy * std::fabs(vy);
    if (dive) a -= t.buoyancy * 1.2 + 4.0;    // kick down, against the float
    else if (rise) a += 6.0;                  // kick up: out past the float line, to look over a pool's edge
    const Real v = vy + a * dt;
    return dive ? std::max(v, Real(-2.0)) : v;
}

Vec3 parachuteStep(const Vec3& v, Vec3& heading, const Vec3& look, Real inF, Real open, Real dt,
                   const MovementTuning& t) {
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    // turn toward the look, at most the turn rate
    const Vec3 want = flat(look);
    if (want.lengthSquared() > 0.5) {
        const Real cur = std::atan2(heading.x, heading.z), tgt = std::atan2(want.x, want.z);
        Real d = std::remainder(tgt - cur, 2 * 3.14159265358979);
        const Real step = t.chuteTurnRate * kDeg * dt;
        d = std::clamp(d, -step, step);
        heading = Vec3(std::sin(cur + d), 0, std::cos(cur + d));
    }
    const Real f = std::clamp(inF, Real(-1), Real(1));
    const Real speed = f >= 0 ? t.chuteSpeed + (t.chuteDiveSpeed - t.chuteSpeed) * f
                              : t.chuteSpeed + (t.chuteFlareSpeed - t.chuteSpeed) * -f;
    const Real sink = f >= 0 ? t.chuteSink + (t.chuteDiveSink - t.chuteSink) * f
                             : t.chuteSink + (t.chuteFlareSink - t.chuteSink) * -f;
    const Vec3 target = heading * speed + Vec3(0, -sink, 0);
    // an opening canopy: mostly still falling, the drag coming on as it fills
    const Real o = std::clamp(open, Real(0), Real(1));
    const Real k = 1.0 - std::exp(-dt * (t.chuteResponse + 6.0 * o * o) * o);
    Vec3 out = v + Vec3(0, -9.81 * dt * (1 - o), 0);
    out = out + (target - out) * k;
    return out;
}

Vec3 wingsuitFacing(const Vec3& look, const MovementTuning& t) {
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    const Vec3 h = flat(look);
    if (h.lengthSquared() < 0.5) return Vec3(0, -1, 0);
    Real pitch = std::asin(std::clamp(look.y, Real(-1), Real(1)));
    pitch = std::clamp(pitch, -t.suitPitchDown * kDeg, t.suitPitchUp * kDeg);
    return h * std::cos(pitch) + Vec3(0, std::sin(pitch), 0);
}

Vec3 wingsuitStep(const Vec3& v, const Vec3& look, Real dt, const MovementTuning& t) {
    const Vec3 f = wingsuitFacing(look, t);
    Vec3 out = v + Vec3(0, -9.81 * dt, 0);
    const Real speed = out.length();
    if (speed > 1e-4) {
        // LIFT turns the way toward the facing without doing work, but it grows with the square of the speed:
        // at the stall speed it can just hold the body up (g), slower it can't, so a slow suit falls and a
        // fast one carves. What turning costs (induced drag) follows from how hard it turns.
        const Vec3 dir = out * (1.0 / speed);
        const Real c = std::clamp(dot(dir, f), Real(-1), Real(1));
        const Real angle = std::acos(c);
        if (angle > 1e-5 && c > -0.3) {
            const Real liftMax = 9.81 * (speed / t.suitStallSpeed) * (speed / t.suitStallSpeed);
            const Real phi = std::min(angle * (1.0 - std::exp(-dt * t.suitTurn)), liftMax * dt / speed);
            const Vec3 perp = normalize(f - dir * c);
            out = (dir * std::cos(phi) + perp * std::sin(phi)) * speed;
            const Real lift = speed * phi / dt;   // the acceleration it took
            const Real induced = t.suitInducedDrag * lift * lift / (speed * speed);
            out = out * std::max(Real(0), 1.0 - induced * dt / speed);
        }
    }
    // drag: light along the body, heavy broadside (a suit flat to the air is a sail)
    Real along = dot(out, f);
    Vec3 across = out - f * along;
    along -= t.suitDrag * along * std::fabs(along) * dt;
    const Real side = across.length();
    if (side > 1e-6) across = across * std::max(Real(0), 1.0 - 8.0 * t.suitDrag * side * dt);
    return f * along + across;
}

GrappleAnchor findGrappleAnchor(const PhysicsWorld& world, const Vec3& eye, const Vec3& look, const MovementTuning& t) {
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    GrappleAnchor best;
    const Vec3 f = normalize(look);
    Vec3 r = cross(f, Vec3(0, 1, 0));
    r = r.length() < 1e-4 ? Vec3(1, 0, 0) : normalize(r);
    const Vec3 u = cross(r, f);
    Real bestScore = 1e9;
    auto tryRay = [&](const Vec3& dir, Real offDeg) {
        PhysicsWorld::RayHit hit;
        if (!world.castRayHit(eye, dir * t.grappleRange, hit) || hit.dynamic) return;
        const Real d = (hit.point - eye).length();
        if (d < t.grappleMinRange) return;
        // near the aim first; a hook above the eye swings, one below only drags
        const Real above = hit.point.y - eye.y;
        const Real score = offDeg / std::max(t.grappleCone, Real(0.1)) + (above < 0 ? 0.6 : -0.2 * std::min(Real(1), above / 10));
        if (score < bestScore) {
            bestScore = score;
            best.ok = true;
            best.point = hit.point;
            best.normal = hit.normal;
            best.distance = d;
        }
    };
    tryRay(f, 0);
    if (best.ok && best.point.y >= eye.y) return best;   // the aim itself found a place above: take it
    for (Real ring : {0.5, 1.0}) {
        const Real a = t.grappleCone * ring * kDeg;
        for (int k = 0; k < 8; ++k) {
            const Real phi = k * 3.14159265358979 / 4;
            const Vec3 dir = normalize(f * std::cos(a) + (r * std::cos(phi) + u * std::sin(phi)) * std::sin(a));
            tryRay(dir, t.grappleCone * ring);
        }
    }
    return best;
}

Vec3 ropeStep(Vec3& pos, const Vec3& vel, const Vec3& anchor, Real length, const Vec3& accel, Real dt) {
    Vec3 v = vel + (Vec3(0, -9.81, 0) + accel) * dt;
    Vec3 next = pos + v * dt;
    Vec3 out = next - anchor;
    const Real d = out.length();
    if (d > length && d > 1e-6) {
        // held on the sphere: the position projected back, the speed away from the anchor removed
        const Vec3 n = out * (1.0 / d);
        next = anchor + n * length;
        const Real away = dot(v, n);
        if (away > 0) v = v - n * away;
    }
    pos = next;
    return v;
}

Real boardStep(Real speed, Vec3& heading, Real& kickClock, Real inF, Real inR, const Vec3& groundNormal,
               Real pushMax, Real dt, const MovementTuning& t) {
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    // carve: the turn rate the grip allows at this speed, capped
    const Real rate = std::min(t.boardTurnMax * kDeg, t.boardCarve / std::max(speed, Real(0.5)));
    const Real turn = -std::clamp(inR, Real(-1), Real(1)) * rate * dt;   // right: clockwise from above
    const Real c = std::cos(turn), s = std::sin(turn);
    heading = flat(Vec3(heading.x * c + heading.z * s, 0, -heading.x * s + heading.z * c));
    // downhill: gravity along the slope, the part along the board
    const Vec3 n = groundNormal.lengthSquared() > 0.5 ? normalize(groundNormal) : Vec3(0, 1, 0);
    const Vec3 downhill = Vec3(n.x, 0, n.z) * (9.81 * n.y);
    speed += dot(downhill, heading) * dt;
    kickClock = std::max(Real(0), kickClock - dt);
    if (inF > 0.3 && kickClock <= 0 && speed < pushMax) {
        speed = std::min(pushMax, speed + t.boardPush * (1.0 - 0.5 * speed / pushMax));
        kickClock = t.boardKickTime;
    }
    if (inF < -0.3) speed -= t.boardBrake * dt;
    speed -= t.boardRoll * dt;
    return std::max(Real(0), speed);
}

void swingStep(Real& theta, Real& omega, Real length, Real pump, Real dt, const MovementTuning& t) {
    constexpr Real kDeg = 3.14159265358979 / 180.0;
    Real a = -9.81 / std::max(length, Real(0.3)) * std::sin(theta) - t.swingDamping * omega;
    // pumping: pushing the way it swings adds, against it brakes (a kid leaning back and kicking out)
    const Real dir = std::fabs(omega) > 0.05 ? (omega > 0 ? 1.0 : -1.0) : 1.0;
    a += t.swingPump * std::clamp(pump, Real(-1), Real(1)) * dir;
    omega += a * dt;
    theta += omega * dt;
    const Real lim = t.swingMaxDeg * kDeg;
    if (std::fabs(theta) > lim) {   // the chains go slack: it stops there and comes back
        theta = theta > 0 ? lim : -lim;
        if (omega * theta > 0) omega = 0;
    }
}

Real strokeSurgePhase(Real strokes) {
    const Real x = 2.0 * strokes - 2.0 * 0.53 + 0.3;   // a pull lands on the surge's peak
    return x - std::floor(x);
}

Real strokeSpeedScale(Real phase, Real surge) {
    constexpr Real kTau = 6.283185307179586;
    return 1.0 + surge * std::cos(kTau * (phase - 0.3));   // the pull peaks a third of the way in
}

}  // namespace engine
