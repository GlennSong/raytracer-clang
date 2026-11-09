#ifndef RAYTRACER_ENGINE_AI_DANGER_DODGE_H
#define RAYTRACER_ENGINE_AI_DANGER_DODGE_H

// GETTING OUT OF THE WAY (ADR-0355; Glenn, 2026-11-06: "if I do [hit pedestrians], they tend to just get pushed along
// with a vehicle and they don't ragdoll or they don't try to jump out of the way even. Maybe they need some kind of new
// state where they are self-aware around themselves and will try and move out of the way of danger.")
//
// Pure rules (headless-tested), for anyone on foot:
//   footprint   a car is its oriented box (half width, half length about its centre), not its centre -- the nose of a
//               car is two metres ahead of the point a radius test used, so it shoved people along before it "hit" them
//   threat      a car moving at someone: they are in its swept path (within its half width plus a clearance either
//               side of its line of travel) and its front reaches them within a lookahead; the escape is straight out
//               of the path, on the side they already stand
//   dodge       each person notices after their own reaction time and then sprints out of the path (dives when it is
//               close); a few freeze, as people do. The traits come from a seed, so the same person reacts the same way.
//   tuning      every number here is people.lua's `people.dodge` (DodgeTuning)

#include "../procgen/city/polygon.h"   // Vec2, dot
#include "../../hash.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace engine {

// THE TUNING (ADR-0355: data -- people.lua `people.dodge`; these defaults are its values)
struct DodgeTuning {
    Real lookahead = 1.8;      // s: a car whose front reaches them sooner is coming
    Real clearance = 1.0;      // m either side of its path that still counts as in it
    Real minSpeed = 3.5;       // m/s: slower is no danger
    Real reactionLo = 0.25, reactionHi = 0.65;   // s from a car coming to moving
    Real freezeShare = 1.0 / 7.0;                // who stands rooted instead
    Real sprintLo = 4.5, sprintHi = 6.5;         // m/s out of the way
    Real sprintFor = 0.75;     // s of the sprint
    Real diveBelow = 0.45;     // s: closer than this, a dive
    Real diveBoost = 1.3, diveFor = 0.45;        // a dive's speed (x the sprint) and length
    Real knockSpeed = 2.5;     // m/s: a car's box reaching someone this fast knocks them down
    Real throwCap = 30.0;      // m/s: nobody is thrown faster
};

struct DangerCar {
    Vec2 pos{0, 0};
    Vec2 fwd{0, 1};       // its nose, unit
    Vec2 vel{0, 0};
    Real halfW = 0.9, halfL = 2.3;
};

// Is `p` within the car's box grown by `margin`?
inline bool insideFootprint(const DangerCar& c, Vec2 p, Real margin) {
    const Vec2 r = p - c.pos;
    const Vec2 right(c.fwd.y, -c.fwd.x);
    return std::fabs(dot(r, c.fwd)) <= c.halfL + margin && std::fabs(dot(r, right)) <= c.halfW + margin;
}

struct Threat {
    bool coming = false;
    Real ttc = 0;          // s until its front reaches them
    Vec2 escape{0, 0};     // unit: straight out of its path
};

// Is the car coming at `p`? Its path is its box swept along its velocity: the box's reach across the travel line and
// ahead along it (whichever way it faces -- a car reversing at someone threatens with its tail).
inline Threat assessThreat(const DangerCar& c, Vec2 p, const DodgeTuning& k = DodgeTuning()) {
    const Real lookahead = k.lookahead, clearance = k.clearance, minSpeed = k.minSpeed;
    Threat t;
    const Real speed = c.vel.length();
    if (speed < minSpeed) return t;
    const Vec2 dir = c.vel * (1.0 / speed);
    const Vec2 across(dir.y, -dir.x);
    const Vec2 right(c.fwd.y, -c.fwd.x);
    // the box's half extents along the travel line and across it
    const Real reach = c.halfL * std::fabs(dot(c.fwd, dir)) + c.halfW * std::fabs(dot(right, dir));
    const Real width = c.halfL * std::fabs(dot(c.fwd, across)) + c.halfW * std::fabs(dot(right, across));
    const Vec2 r = p - c.pos;
    const Real ahead = dot(r, dir) - reach;     // from its front to them
    const Real side = dot(r, across);
    if (ahead < -reach || std::fabs(side) > width + clearance) return t;   // behind it, or clear of its path
    t.ttc = std::max(Real(0), ahead) / speed;
    if (t.ttc > lookahead) return t;
    t.coming = true;
    t.escape = side >= 0 ? across : across * -1.0;
    return t;
}

struct DodgeTraits {
    Real reaction = 0.4;   // s from a car coming to moving
    bool freezes = false;  // stands there (one in seven)
    Real sprint = 5.5;     // m/s out of the way
};

// A person's own: reaction 0.25-0.65 s, one in seven freezes, a sprint of 4.5-6.5 m/s.
inline DodgeTraits dodgeTraits(uint32_t seed, const DodgeTuning& k = DodgeTuning()) {
    const uint32_t h = hashPair(seed, 0xD0D6Eu);   // (hash.h)
    DodgeTraits d;
    d.reaction = k.reactionLo + (h & 0xFF) / 255.0 * (k.reactionHi - k.reactionLo);
    d.freezes = ((h >> 8) % 1000u) / 1000.0 < k.freezeShare;
    d.sprint = k.sprintLo + ((h >> 16) & 0xFF) / 255.0 * (k.sprintHi - k.sprintLo);
    return d;
}

struct DodgeState {
    Real noticed = 0;      // s a car has been coming
    Real left = 0;         // s of the dodge still to run
    Vec2 velocity{0, 0};   // the dodge's
    int dodges = 0;
};

// One step. `worst`: the nearest car coming (by ttc). True with `out` the velocity to move at while dodging (or a
// freeze: zero), false when this person carries on as they were.
inline bool stepDodge(DodgeState& s, const DodgeTraits& d, const Threat& worst, Real dt, Vec2& out,
                      const DodgeTuning& k = DodgeTuning()) {
    if (s.left > 0) {
        s.left -= dt;
        out = s.velocity;
        return true;
    }
    if (!worst.coming) {
        s.noticed = std::max(Real(0), s.noticed - dt);
        return false;
    }
    s.noticed += dt;
    if (s.noticed < d.reaction) return false;   // not seen it yet
    if (d.freezes) {
        out = Vec2(0, 0);   // rooted to the spot
        return true;
    }
    // out of its path: a dive (faster, shorter) when it is nearly there
    const bool dive = worst.ttc < k.diveBelow;
    s.velocity = worst.escape * (dive ? d.sprint * k.diveBoost : d.sprint);
    s.left = dive ? k.diveFor : k.sprintFor;
    s.noticed = 0;
    ++s.dodges;
    out = s.velocity;
    return true;
}

}  // namespace engine

#endif
