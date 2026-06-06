#include "player_system.h"
#include "physics_system.h"
#include "../components.h"

namespace engine {

void PlayerSystem::onStart(FrameContext&) {
    camera.positionLocked = true;
}

void PlayerSystem::fixedUpdate(FrameContext& ctx) {
    ctx.world.each<Transform, RigidBody, ControlledBy>(
        [&](Entity e, Transform&, RigidBody& rb, ControlledBy&) {
            playerEntity = e;
            if (rb.bodyId == INVALID_PHYSICS_BODY) return;

            Real forward = ctx.actions.axis("cam_forward");
            Real right = ctx.actions.axis("cam_right");

            Vec3 camForward = camera.forward();
            camForward.y = 0;
            camForward = normalize(camForward);
            Vec3 camRight = camera.right();

            Vec3 moveDir = camForward * forward + camRight * right;
            Real len = moveDir.length();
            if (len > 1.0) moveDir = moveDir / len;

            Vec3 vel = moveDir * moveSpeed;
            Vec3 currentVel = physicsSys.physicsWorld().getLinearVelocity(rb.bodyId);
            vel.y = currentVel.y;

            physicsSys.physicsWorld().setLinearVelocity(rb.bodyId, vel);
        });
}

void PlayerSystem::update(FrameContext& ctx) {
    if (!ctx.world.alive(playerEntity)) return;
    auto* t = ctx.world.get<Transform>(playerEntity);
    if (!t) return;

    camera.eye = t->position + Vec3(0, eyeHeight, 0);

    float aspect = (ctx.framebufferHeight > 0)
        ? static_cast<float>(ctx.framebufferWidth) / ctx.framebufferHeight
        : 1.0f;
    ctx.view.camera = camera.cameraState(aspect);
}

}  // namespace engine
