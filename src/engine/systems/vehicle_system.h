#ifndef RAYTRACER_ENGINE_VEHICLE_SYSTEM_H
#define RAYTRACER_ENGINE_VEHICLE_SYSTEM_H

#include "../system.h"   // System, FrameContext, and (via renderer.h) MeshHandle

namespace engine {

class PhysicsSystem;
class CameraSystem;

// Drives the player-controllable physics cars (ADR-0059). For every entity with
// a Vehicle component it creates the Jolt vehicle (PhysicsWorld::addVehicle) from
// the config + Transform, feeds the seated driver's input each fixed step, and
// writes the chassis (and wheel) transforms back for rendering. It also owns the
// "jump in / get out" interaction: press the enter key near a car to seat the
// player (suppressing on-foot movement via the InVehicle tag) and switch the view
// to the chase camera; press again to get out.
//
// The create/drive/writeback calls go through the Jolt vehicle wrapper
// (PhysicsWorld), whose behaviour is gated headlessly by tests/test_driving_lab.cpp.
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
    void spawnInFront(FrameContext& ctx);   // debug: drop a car ahead of the player
    void updateHorn(FrameContext& ctx);     // hold-to-honk (H / L3) while driving

    PhysicsSystem& physicsSys;
    CameraSystem& cameras;
    MeshHandle wheelMesh;            // shared wheel cylinder, uploaded on first use
    MeshHandle lensMesh;             // shared head/taillight lens box
    MeshHandle driverMesh;          // shared driver capsule
    Real enterRadius = 4.0;          // how close the player must be to board (m)
    int spawnCount_ = 0;             // debug-spawn seed counter
    // Horn: one looping procedural voice (sfx::horn) held while the key is
    // down, volume-ramped so press/release don't click.
    AudioClipHandle hornClip;
    AudioVoiceHandle hornVoice;
    float hornGain = 0.0f;
};

}  // namespace engine

#endif
