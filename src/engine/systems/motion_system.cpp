#include "motion_system.h"
#include "../components.h"

void MotionSystem::fixedUpdate(FrameContext& ctx) {
    Real dt = ctx.clock.fixedStep();
    ctx.world.each<Transform, Velocity, PrevTransform>(
        [dt](Entity, Transform& t, Velocity& v, PrevTransform& prev) {
            prev.value = t;                       // save previous for interpolation
            t.position += v.linear * dt;
            t.rotation += v.angular * dt;
        });
}
