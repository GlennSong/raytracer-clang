#ifndef RAYTRACER_ENGINE_MOTION_SYSTEM_H
#define RAYTRACER_ENGINE_MOTION_SYSTEM_H

#include "../system.h"

// Integrates entity velocity/angular velocity on the fixed timestep. This is
// the seam a real physics/collision system replaces; today it just advances
// the world's simple kinematics.
class MotionSystem : public System {
public:
    void fixedUpdate(FrameContext& ctx) override;
};

#endif
