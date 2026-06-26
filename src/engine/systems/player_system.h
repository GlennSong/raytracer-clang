#ifndef RAYTRACER_ENGINE_PLAYER_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "../physics/physics_world.h"

namespace engine {

class PhysicsSystem;

class PlayerSystem : public System {
public:
    PlayerSystem(FlyCameraController& camera, PhysicsSystem& physics)
        : camera(camera), physicsSys(physics) {}

    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

private:
    FlyCameraController& camera;
    PhysicsSystem& physicsSys;
    Entity playerEntity;
    Real moveSpeed = 6.0;
    Real eyeHeight = 0.7;
    Vec3 spawnPos{0, 0, 0};       // captured authored spawn; "respawn" returns the player here
    bool spawnCaptured = false;
};

}  // namespace engine

#endif
