// Moves model instances (#191, step 8). See model_motion_system.h.
#include "model_motion_system.h"

#include "../components.h"
#include "../model_motion.h"
#include "physics_system.h"

#include <vector>

namespace engine {

int ModelMotionSystem::step(World& world, Real dt) {
    // (collected first: advancing an instance writes its pieces' components)
    std::vector<Entity> moving, settling;
    world.each<ModelInstance>([&](Entity e, ModelInstance& mi) {
        if (mi.settle) settling.push_back(e);
        if (!mi.target.values.empty()) moving.push_back(e);
    });
    // THE PIECES' PREVIOUS PLACE. The renderer draws a piece between its previous place and its place. Every copy
    // whose pieces were put somewhere new since the last step (by a target approached, or by a value a script set
    // at once) has them brought up here, before this step moves anything: a part still moving is then drawn a step
    // behind, as it should be, and one that has stopped is drawn still. (Left undone, a clock's hands, set at once
    // every frame, were drawn between twelve and the time for good; and a door that had arrived trembled.)
    for (Entity e : settling) {
        ModelInstance* mi = world.get<ModelInstance>(e);
        if (mi == nullptr) continue;
        mi->settle = false;
        for (Entity pe : mi->pieces) {
            PrevTransform* prev = world.get<PrevTransform>(pe);
            const Transform* t = world.get<Transform>(pe);
            if (prev != nullptr && t != nullptr) prev->value = *t;
        }
    }
    int moved = 0;
    for (Entity e : moving) {
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
