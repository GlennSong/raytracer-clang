// Moves model instances (#191, step 8). See model_motion_system.h.
#include "model_motion_system.h"

#include "../components.h"
#include "../model_motion.h"
#include "physics_system.h"

#include <vector>

namespace engine {

int ModelMotionSystem::step(World& world, Real dt) {
    // (collected first: advancing an instance writes its pieces' components)
    std::vector<Entity> moving;
    world.each<ModelInstance>([&](Entity e, ModelInstance& mi) {
        if (!mi.target.values.empty()) moving.push_back(e);
    });
    int moved = 0;
    for (Entity e : moving) {
        // The pieces drew at their old place last step: remember it, so the renderer moves them smoothly.
        const std::vector<Entity> pieces = world.get<ModelInstance>(e)->pieces;
        for (Entity pe : pieces) {
            PrevTransform* prev = world.get<PrevTransform>(pe);
            const Transform* t = world.get<Transform>(pe);
            if (prev != nullptr && t != nullptr) prev->value = *t;
        }
        if (!advanceModelMotion(world, e, dt)) continue;
        ++moved;
        const std::vector<Entity> bodies = world.get<ModelInstance>(e)->bodies;
        for (Entity be : bodies) {
            Vec3 position;
            Quat rotation;
            if (!modelBodyPlace(world, be, position, rotation)) continue;
            RigidBody* rb = world.get<RigidBody>(be);
            if (rb == nullptr || rb->motion != BodyMotion::Kinematic) continue;
            if (physics_ != nullptr && rb->bodyId != INVALID_PHYSICS_BODY) {
                // driven: the physics moves it there over this step and writes its Transform back
                physics_->physicsWorld().moveKinematic(rb->bodyId, position, rotation, dt);
            } else if (Transform* t = world.get<Transform>(be)) {
                t->position = position;
                t->orientation = rotation;
            }
        }
    }
    return moved;
}

void ModelMotionSystem::fixedUpdate(FrameContext& ctx) { step(ctx.world, ctx.clock.fixedStep()); }

}  // namespace engine
