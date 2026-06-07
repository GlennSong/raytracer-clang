#ifndef RAYTRACER_ENGINE_SHOOTING_SYSTEM_H
#define RAYTRACER_ENGINE_SHOOTING_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "../physics/physics_world.h"

namespace engine {

class PhysicsSystem;

// Spawns bouncing cube bullets on fire input. Also positions a gun entity
// (cylinder) to follow the camera.
class ShootingSystem : public System {
public:
    ShootingSystem(FlyCameraController& camera, PhysicsSystem&,
                   Renderer& renderer)
        : camera(camera), rendererRef(renderer) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;

private:
    void spawnBullet(FrameContext& ctx);

    FlyCameraController& camera;
    Renderer& rendererRef;
    Entity gunEntity;
    MeshHandle bulletMesh;
    MeshHandle gunMesh;
    Real bulletSpeed = 50.0;
    Real bulletSize = 0.5;
    int maxBullets = 500;
    int bulletCount = 0;
};

}  // namespace engine

#endif
