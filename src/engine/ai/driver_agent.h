#ifndef RAYTRACER_ENGINE_AI_DRIVER_AGENT_H
#define RAYTRACER_ENGINE_AI_DRIVER_AGENT_H

#include "../procgen/city/polygon.h"   // Vec2, dot, Real
#include <algorithm>
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

// What a car currently IS in the ground plane: where it is, which way it points and how fast it is going forward.
// Built from the Jolt chassis each step (or by a test).
struct DriverState {
    Vec2 pos{0, 0};       // XZ position of the chassis
    Vec2 forward{1, 0};   // unit XZ heading (the chassis' +Z basis, flattened)
    Real speed = 0;       // forward speed m/s (signed; < 0 = rolling backward)
};

// What the AI brain WANTS this step: a LANE FRAME to ride -- the segment laneA -> laneB -- and a speed to hold. Every
// AI driver plans one: the physical traffic tier (the ghost's lane, or pursuit toward its lead), the possessed car
// (pursuitCommand: lane_follow.h), the driving lab. PURE PURSUIT is the frame from the car itself to its lookahead
// point (`toward`): the cross-track term is zero there and the steering is heading error alone. A frame of zero
// length steers straight; speed 0 holds the brake. The player bypasses this and writes a DriverInput directly.
struct DriverCommand {
    Vec2 laneA{0, 0}, laneB{0, 0};
    Real desiredSpeed = 0;   // m/s to seek (0: come to a stop and hold it)
    // Pure pursuit: from `pos` toward `heading` (any length).
    static DriverCommand toward(const Vec2& pos, const Vec2& heading, Real speed) {
        DriverCommand c;
        c.laneA = pos;
        c.laneB = pos + heading;
        c.desiredSpeed = speed;
        return c;
    }
};

struct DriverTuning {
    Real stanleyK = 1.6;       // cross-track gain (m/s per metre)
    Real maxSteerRad = 0.61;   // wheel lock the [-1,1] input maps to
    Real speedGain = 0.9;      // pedal response per m/s of speed error (throttle and brake alike)
    Real holdBrake = 0.6;      // the brake held when told to stop, so the car cannot idle-creep
};

// THE ONE AI DRIVING CONTROLLER (ADR-0062; one for all AI drivers since item 24, #117): (what the car is) + (what
// the brain wants) -> pedal and wheel. Pure, no engine/Jolt state, headless-tested.
//   LATERAL -- Stanley: heading error to the lane plus arctan(k * crossTrack / v), the cross-track softened at speed
//     (the 1.2 m/s floor keeps it usable from standstill). Sign convention MEASURED in the driving lab (the steer-sign
//     probe): +steer (setVehicleInput's `right`) turns toward -x facing +z; the heading-error term enters POSITIVE and
//     the cross-track term NEGATIVE.
//   LONGITUDINAL -- the pedals track the speed error; the brains (IDM, signals, junction rules, corner governors,
//     a pursuit's turn slowdown) have already folded everything into desiredSpeed.
inline DriverInput computeDriverInput(const DriverState& s, const DriverCommand& c, const DriverTuning& t = {}) {
    DriverInput in;
    Vec2 lane = c.laneB - c.laneA;
    const Real laneLen = std::sqrt(lane.x * lane.x + lane.y * lane.y);
    if (laneLen > 1e-6) {
        lane = lane * (1.0 / laneLen);
        // signed cross-track: + when the car sits LEFT of the lane (steer right to return)
        const Vec2 rel = s.pos - c.laneA;
        const Real cross = lane.x * rel.y - lane.y * rel.x;
        const Real headingErr = std::atan2(s.forward.x * lane.y - s.forward.y * lane.x,
                                           s.forward.x * lane.x + s.forward.y * lane.y);
        const Real steerRad = headingErr - std::atan2(t.stanleyK * cross, std::max(Real(1.2), std::fabs(s.speed)));
        in.steer = std::clamp(steerRad / t.maxSteerRad, Real(-1), Real(1));
    }
    const Real err = c.desiredSpeed - s.speed;
    if (err >= 0) {
        in.throttle = std::clamp(err * t.speedGain, Real(0), Real(1));
    } else {
        in.brake = std::clamp(-err * t.speedGain, Real(0), Real(1));
        if (c.desiredSpeed < 0.05) in.brake = std::max(in.brake, t.holdBrake);
    }
    return in;
}

}  // namespace engine

#endif
