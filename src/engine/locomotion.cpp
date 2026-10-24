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

Real strokeSurgePhase(Real strokes) {
    const Real x = 2.0 * strokes - 2.0 * 0.53 + 0.3;   // a pull lands on the surge's peak
    return x - std::floor(x);
}

Real strokeSpeedScale(Real phase, Real surge) {
    constexpr Real kTau = 6.283185307179586;
    return 1.0 + surge * std::cos(kTau * (phase - 0.3));   // the pull peaks a third of the way in
}

}  // namespace engine
