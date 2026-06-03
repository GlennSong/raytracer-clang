#include "motion_system.h"

void MotionSystem::fixedUpdate(FrameContext& ctx) {
    ctx.world.step(ctx.clock.fixedStep());
}
