#ifndef RAYTRACER_ENGINE_MOTION_SYSTEM_H
#define RAYTRACER_ENGINE_MOTION_SYSTEM_H

#include "../system.h"

// Scripted kinematic motion: advances entities by their Velocity each fixed
// step, with no forces or collisions. This is NOT a placeholder for physics —
// it is the deliberate tool for cheap, exact, collision-free movement (visual
// spinners, rails, camera dollies). Entities that also carry a RigidBody are
// owned by the PhysicsSystem, so MotionSystem yields them (ADR-0012).
//
// fixedUpdate() wraps integrate(), which takes a World directly so it is
// unit-testable without a FrameContext.
class MotionSystem : public System {
public:
    void fixedUpdate(FrameContext& ctx) override;
    void integrate(World& world, Real dt);
};

#endif
