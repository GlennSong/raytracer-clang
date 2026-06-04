#include "motion_system.h"
#include "../components.h"

void MotionSystem::fixedUpdate(FrameContext& ctx) {
    Real dt = ctx.clock.fixedStep();
    ctx.world.each<Transform, Velocity, PrevTransform>(
        [dt](Entity, Transform& t, Velocity& v, PrevTransform& prev) {
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
