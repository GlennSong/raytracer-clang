#ifndef RAYTRACER_ENGINE_SYSTEMS_MODEL_MOTION_SYSTEM_H
#define RAYTRACER_ENGINE_SYSTEMS_MODEL_MOTION_SYSTEM_H

// Moves model instances (#191, step 8). Each fixed step, every ModelInstance with a target (engine/model_motion.h)
// moves toward it; its drawn pieces follow, and each of its kinematic bodies (a collision shape on a node that
// moves) is driven to where its node now is, so it pushes what it meets instead of passing through it.

#include "../system.h"
#include "../world.h"

namespace engine {

class PhysicsSystem;

class ModelMotionSystem : public System {
public:
    // `physics` may be null (a host without physics): the pieces still move.
    explicit ModelMotionSystem(PhysicsSystem* physics = nullptr) : physics_(physics) {}
    void fixedUpdate(FrameContext& ctx) override;
    // One step, for a test or a host that steps a World itself. Returns how many instances moved.
    int step(World& world, Real dt);

private:
    PhysicsSystem* physics_;
};

}  // namespace engine

#endif
