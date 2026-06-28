#ifndef RAYTRACER_ENGINE_VEHICLE_SYSTEM_H
#define RAYTRACER_ENGINE_VEHICLE_SYSTEM_H

#include "../system.h"   // System, FrameContext, and (via renderer.h) MeshHandle

namespace engine {

class PhysicsSystem;
class CameraSystem;

// Drives the player-controllable physics cars (ADR-0058). For every entity with
// a Vehicle component it creates the Jolt vehicle (PhysicsWorld::addVehicle) from
// the config + Transform, feeds the seated driver's input each fixed step, and
// writes the chassis (and wheel) transforms back for rendering. It also owns the
// "jump in / get out" interaction: press the enter key near a car to seat the
// player (suppressing on-foot movement via the InVehicle tag) and switch the view
// to the chase camera; press again to get out.
//
// UNVERIFIED submodule-gated path: the create/drive/writeback calls go through
// the Jolt vehicle wrapper, which can't be compiled in this environment. Build
// and tune on a machine with the Jolt submodule.
class VehicleSystem : public System {
public:
    VehicleSystem(PhysicsSystem& physics, CameraSystem& cameras)
        : physicsSys(physics), cameras(cameras) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;        // enter/exit edges + camera
    void fixedUpdate(FrameContext& ctx) override;   // create, drive, write back

private:
    void createVehicles(FrameContext& ctx);
    void driveVehicles(FrameContext& ctx);
    void writeBack(FrameContext& ctx);
    void handleEnterExit(FrameContext& ctx);

    PhysicsSystem& physicsSys;
    CameraSystem& cameras;
    MeshHandle wheelMesh;            // shared wheel cylinder, uploaded on first use
    Real enterRadius = 4.0;          // how close the player must be to board (m)
};

}  // namespace engine

#endif
