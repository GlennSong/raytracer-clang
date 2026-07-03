#ifndef RAYTRACER_ENGINE_AI_DRIVER_AGENT_H
#define RAYTRACER_ENGINE_AI_DRIVER_AGENT_H

#include "../procgen/city/polygon.h"   // Vec2, dot, Real
#include <cmath>

namespace engine {

// The unified vehicle-control seam (ADR-0062). Every car in the world is ONE kind
// of object — a physics Vehicle — and whoever drives it, the player or an AI,
// speaks only in these four pedal/wheel values. So there is a single control path
// (`PhysicsWorld::setVehicleInput`) and a single physics path; an NPC car handles
// exactly like the player's because it IS the same object with a different
// controller. This replaces the old split where NPC cars were kinematic pose-
// holders that could never be driven or entered.

// What the player's input and the AI both produce, and what the vehicle consumes.
// Matches PhysicsWorld::setVehicleInput(forward, right, brake, handBrake).
struct DriverInput {
    Real throttle = 0;    // [-1,1] forward drive (negative = reverse) -> `forward`
    Real steer = 0;       // [-1,1] steering, +right / -left        -> `right`
    Real brake = 0;       // [0,1]                                   -> `brake`
    Real handBrake = 0;   // [0,1]                                   -> `handBrake`
};

// What a car currently IS in the ground plane: which way it points and how fast
// it is going forward. Built from the Jolt chassis each step (or by a test).
struct DriverState {
    Vec2 forward{1, 0};   // unit XZ heading (the chassis' +Z basis, flattened)
    Real speed = 0;       // forward speed m/s (signed; < 0 = rolling backward)
};

// What the AI brain WANTS this step: a heading to aim at and a speed to hold. The
// city brain (NavGraph follow + signals + perception + car-following) produces
// these; the controller below turns them into pedal/wheel positions. The player
// bypasses this and writes a DriverInput directly.
struct DriverCommand {
    Vec2 desiredHeading{1, 0};   // unit XZ direction to steer toward
    Real desiredSpeed = 0;       // m/s to seek (<= brakeSpeed -> come to a stop)
};

struct DriverTuning {
    Real steerGain = 1.6;        // heading error (rad) -> steer, before clamp
    Real speedGain = 0.5;        // speed deficit (m/s) -> throttle, before clamp
    Real brakeGain = 0.35;       // overspeed (m/s)     -> brake, before clamp
    Real stopSpeed = 0.4;        // desiredSpeed at/under this -> brake to a halt
    // Slow into a misaligned heading: the fraction of desiredSpeed shed as the
    // desired heading swings away from the car's (0 = charge at full speed no
    // matter what; 0.75 = a perpendicular target drops the target speed to 25%).
    // A driver brakes for a turn they haven't made yet — and slower also means a
    // shorter pursuit lookahead, which tightens the achievable turn radius.
    Real turnSlowdown = 0.75;
};

// Pure controller: (what the car is) + (what the brain wants) -> pedal/wheel. No
// engine/Jolt state, so it is unit-tested headless. Steering aims the heading at
// the command; throttle/brake close the speed gap. The steer sign follows
// setVehicleInput's `right` convention: + steers toward a heading on the car's
// right, - toward one on its left.
inline DriverInput computeDriverInput(const DriverState& s, const DriverCommand& c,
                                      const DriverTuning& t = {}) {
    DriverInput in;

    // --- steering: signed heading error, mapped to a wheel position ----------
    Real fl = std::sqrt(s.forward.x * s.forward.x + s.forward.y * s.forward.y);
    Real dl = std::sqrt(c.desiredHeading.x * c.desiredHeading.x +
                        c.desiredHeading.y * c.desiredHeading.y);
    Real align = 1.0;   // cos(heading error); 1 = pointed where it wants to go
    if (fl > 1e-6 && dl > 1e-6) {
        Vec2 f(s.forward.x / fl, s.forward.y / fl);
        Vec2 d(c.desiredHeading.x / dl, c.desiredHeading.y / dl);
        Real dot = f.x * d.x + f.y * d.y;
        if (dot > 1) dot = 1; else if (dot < -1) dot = -1;
        align = dot;
        Real ang = std::acos(dot);                       // unsigned error [0, pi]
        // cross = f x d: > 0 means the desired heading is to the car's LEFT (the
        // right vector of (fx,fy) is (fy,-fx), so dot(d,right) = -cross), so we
        // steer negative (left). Aiming right (cross < 0) steers positive.
        Real cross = f.x * d.y - f.y * d.x;
        Real signedAng = (cross >= 0) ? ang : -ang;
        Real steer = -signedAng * t.steerGain;
        in.steer = steer < -1 ? -1 : (steer > 1 ? 1 : steer);
    }

    // --- speed: throttle up to the target, brake if over it or told to stop ---
    // Shed target speed while misaligned (turnSlowdown): the car brakes INTO a
    // sharp turn and accelerates out of it, instead of carrying cruise speed
    // through a heading it hasn't matched yet.
    Real desiredSpeed = c.desiredSpeed *
        (1.0 - t.turnSlowdown * (1.0 - std::max(Real(0), align)));
    if (desiredSpeed <= t.stopSpeed) {
        in.throttle = 0;
        in.brake = 1.0;                                  // ease to a full stop
    } else {
        Real err = desiredSpeed - s.speed;               // + -> need more speed
        if (err >= 0) {
            Real thr = err * t.speedGain;
            in.throttle = thr > 1 ? 1 : thr;
            in.brake = 0;
        } else {
            in.throttle = 0;
            Real br = (-err) * t.brakeGain;
            in.brake = br > 1 ? 1 : br;
        }
    }
    return in;
}

}  // namespace engine

#endif
