#ifndef RAYTRACER_ENGINE_AI_PERCEPTION_H
#define RAYTRACER_ENGINE_AI_PERCEPTION_H

#include "../procgen/city/polygon.h"   // Vec2, dot
#include <cmath>

namespace engine {

// A forward vision cone in the ground (XZ) plane (ADR-0059): an agent at `origin`
// looking along unit `forward`, seeing out to `range` metres within +/- the
// half-angle. Generic perception — reusable by any agent, so it lives in the core
// engine rather than the city-sim application.
struct VisionCone {
    Vec2 origin;
    Vec2 forward{1, 0};        // unit
    Real range = 30.0;         // metres
    Real halfAngleRad = 0.6;   // ~34 deg to each side
};

// Is point `p` within the cone (inside range AND within the half-angle)? A point
// at the origin counts as seen.
inline bool sees(const VisionCone& c, const Vec2& p) {
    Vec2 d = p - c.origin;
    Real dist = d.length();
    if (dist < 1e-6) return true;
    if (dist > c.range) return false;
    Real cosToP = dot(d * (1.0 / dist), c.forward);
    return cosToP >= std::cos(c.halfAngleRad);
}

// Signed distance to `p` projected on the cone axis: how far AHEAD p is (negative
// when behind). Use after sees() to rank obstacles by how close they are.
inline Real forwardDistance(const VisionCone& c, const Vec2& p) {
    return dot(p - c.origin, c.forward);
}

}  // namespace engine

#endif
