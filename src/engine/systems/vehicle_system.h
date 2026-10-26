#ifndef RAYTRACER_ENGINE_VEHICLE_SYSTEM_H
#define RAYTRACER_ENGINE_VEHICLE_SYSTEM_H

#include "../system.h"   // System, FrameContext, and (via renderer.h) MeshHandle
#include "../player/driver_seat.h"   // RideJolt, SeatSway

#include <vector>

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
    void render(FrameContext& ctx) override;        // the car picker / drive-mode panel

private:
    void createVehicles(FrameContext& ctx);
    void driveVehicles(FrameContext& ctx);
    void writeBack(FrameContext& ctx);
    void handleEnterExit(FrameContext& ctx);
    void spawnInFront(FrameContext& ctx);   // debug: drop a car ahead of the player
    void updateHorn(FrameContext& ctx);     // hold-to-honk (H / L3) while driving
    void updateEngines(FrameContext& ctx);  // per-car engine note (idle -> revs)
    void measureSlide(FrameContext& ctx);
    Entity smoke_[2];   // tyre smoke: an emitter (engine/particles.h) on each rear wheel of the player's ride
    void updateTyres(FrameContext& ctx);    // the player's ride: tyre squeal; a bicycle's freewheel and tyre hum

    PhysicsSystem& physicsSys;
    CameraSystem& cameras;
    // THE CAR PICKER (Glenn: "cycle through all of them somehow and then select a car to drop. It should
    // tell me if the car is 2WD or 4WD"): the drivable catalogue (vehicles.lua), the pick, and a panel
    // shown for a few seconds after a pick, a drop or a 2WD/4WD switch.
    struct Pick { std::string recipe, label, drive; };
    std::vector<Pick> catalogue_;
    bool catalogueTried_ = false;
    int pick_ = 0;
    // THE PLAYER'S SEAT (ADR-0252): the jolt it takes (a crash throws the rider off an open one) and the body's sway
    // in it, each fixed step
    void stepSeat(FrameContext& ctx);
    RideJolt jolt_;
    SeatSway sway_;
    Vec3 seatVel_;
    bool haveSeatVel_ = false;
    Real crank_ = 0;
    uint32_t driftCar_ = 0xFFFFFFFFu;   // the car with the drift switched on (a PhysicsWorld::VehicleId)
    bool wasDrifting_ = false;
    Real lastDriftTime_ = 0;
    bool bikeUpright_ = false;   // the ridden bike has stood up since you got on: only then can it go down
    std::string wantRecipe_;   // `vehicle_spawn <recipe>`: picked once the catalogue is loaded
    double clock_ = 0.0, panelUntil_ = 0.0;
    std::string panelText_;
    void loadCatalogue();
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
    // Engines: one looping procedural voice (sfx::engine) per RUNNING physical
    // car — the player's and any AI-driven one — pitched by
    // engineSoundFor(). Held per entity so a car keeps its voice (and its
    // ramped gain) across frames; the nearest few are voiced, the rest fade
    // out, because the mixer's slots are worth more than a car heard from
    // 200 m. Ambient city traffic is voiced by citysim's own adapter against
    // the SAME core.
    struct EngineVoice {
        Entity car;
        AudioVoiceHandle voice;
        float gain = 0.0f;      // ramped toward the core's target
        bool wanted = false;    // survives this frame's cull?
    };
    AudioClipHandle engineClip;
    std::vector<EngineVoice> engineVoices;
    static constexpr int kMaxEngineVoices = 6;
    static constexpr Real kEngineAudibleRange = 90.0;
    // Tyres (Glenn, 2026-10-24: "The bike sounds motorized and it shouldn't. The drifting needs some tire squeal"):
    // looping voices on the player's ride, eased by gain
    struct TyreLoop {
        AudioClipHandle clip;
        AudioVoiceHandle voice;
        float gain = 0.0f;
    };
    TyreLoop squeal_, freewheel_, roll_;
    bool tyreClips_ = false;
};

}  // namespace engine

#endif
