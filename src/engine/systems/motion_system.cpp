#include "motion_system.h"
#include "../components.h"

void MotionSystem::integrate(World& world, Real dt) {
    world.each<Transform, Velocity, PrevTransform>(
        [&world, dt](Entity e, Transform& t, Velocity& v, PrevTransform& prev) {
            if (world.has<RigidBody>(e)) return;  // physics owns this entity

            prev.value = t;                       // save previous for interpolation
            t.position += v.linear * dt;
            // Integrate angular velocity (axis * radians/sec) into the
            // orientation; world-frame spin composed on the left.
            Real angle = v.angular.length() * dt;
            if (angle > 0.0) {
                Quat spin = Quat::fromAxisAngle(v.angular, angle);
                t.orientation = (spin * t.orientation).normalized();
            }
        });
}

void MotionSystem::fixedUpdate(FrameContext& ctx) {
    integrate(ctx.world, ctx.clock.fixedStep());
}
