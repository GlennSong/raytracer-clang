#ifndef RAYTRACER_ENGINE_VEHICLE_SYSTEM_H
#define RAYTRACER_ENGINE_VEHICLE_SYSTEM_H

#include "../system.h"   // System, FrameContext, and (via renderer.h) MeshHandle
#include "../player/driver_seat.h"   // RideJolt, SeatSway
#include "../audio/engine_voices.h"   // EngineVoicePool
#include "../properties.h"
#include "../control/control_commands.h"
#include "../ui/ui_text.h"
#include <string>

#include <vector>

namespace engine {

class PhysicsSystem;
struct Vehicle;
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
// VehicleSpawnRequest (ADR-0253): drop a vehicle in front of the player (the N key from outside: the socket's
// `vehicle_spawn`); `recipe` picks one from the catalogue first (motorcycle, moped, ...).
struct VehicleSpawnRequest {
    std::string recipe;
};
inline void describeProperties(VehicleSpawnRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("recipe"), r.recipe);
}

class VehicleSystem : public System {
public:
    VehicleSystem(PhysicsSystem& physics, CameraSystem& cameras)
        : physicsSys(physics), cameras(cameras) {}

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;        // enter/exit edges + camera
    void fixedUpdate(FrameContext& ctx) override;   // create, drive, write back
    void render(FrameContext& ctx) override;        // the car picker / drive-mode panel

private:
    // `panels open|shut [name]` (ADR-0352): every car's panels, or the named one, staged for the next step
    ControlCommands::Binding panelsCmd_;
    std::vector<std::string> panelsStaged_;
    // the panels' and the dead starter's sounds (sfx::panelShut / panelOpen / starterNoCatch, ADR-0355), made once
    AudioClipHandle shutClip_[3], openClip_, starterClip_;
    Real starterCooldown_ = 0;
    void panelSound(FrameContext& ctx, const Vec3& at, int kind, float volume);   // kind 0..2 shut, 3 open, 4 starter
    EventBus::Subscription onCollision_;
    struct Hit {
        Entity car, other;
        Vec3 at, normal;
        Real speed = 0, slide = 0;
        bool persisted = false;
        bool broke = false;   // it knocked a breakable piece loose (a hydrant, a bin): a light knock (ADR-0354)
    };
    std::vector<Hit> hits_;
    // parts knocked off, waiting for their body (made on the next physics step) to be given the car's speed; and
    // every loose part, oldest first (a cap keeps the street from filling up)
    struct LooseKick { Entity e; Vec3 velocity; Real mass = 20; };
    std::vector<LooseKick> looseKick_;
    std::vector<Entity> loose_;
    EventBus::Subscription onSpawn_;
    bool spawnPending_ = false;
    std::string spawnRecipe_;
    ControlAnswers answers_;   // vehicle?
    void createVehicles(FrameContext& ctx);
    void driveVehicles(FrameContext& ctx);
    void writeBack(FrameContext& ctx);
    void swingPanels(FrameContext& ctx);
    // THE CAR's READOUT (ADR-0356): the engine's health as a bar and each zone's, bottom right while driving (the game
    // UI layer, engine/ui/ui_text.h)
    void drawCarHud(FrameContext& ctx);
    // A REPAIR (ADR-0356): every zone whole, the engine running, the dents and broken glass and lamps put back as they
    // were built; a panel that came off is a loose body by then and stays off
    void repair(FrameContext& ctx, Entity car, Vehicle& v);
    ControlCommands::Binding repairCmd_;
    bool repairStaged_ = false;
    UiText carHud_;
    // DAMAGE (ADR-0353): the step's touches (from the Collision events) spent on the cars they hit
    void applyDamage(FrameContext& ctx);
    void detachPanel(FrameContext& ctx, Entity car, Vehicle& v, int panel, const Vec3& kick);
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
    struct Pick { std::string recipe, label, drive; bool quick = false; };
    std::vector<Pick> catalogue_;
    bool catalogueTried_ = false;
    int pick_ = 0;
    // THE PLAYER'S SEAT (ADR-0252), each fixed step: the driver rides the car (engine/riding.h: a Passenger at its hip
    // point), and the jolt and the sway the riding component measures throw them off an open seat or lurch them in a
    // closed one (item 26, #117: one way of sitting in a vehicle)
    void stepSeat(FrameContext& ctx);
    Real crank_ = 0;
    uint32_t driftCar_ = 0xFFFFFFFFu;   // the car with the drift switched on (a PhysicsWorld::VehicleId)
    bool wasDrifting_ = false;
    Real lastDriftTime_ = 0;
    bool bikeUpright_ = false;   // the ridden bike has stood up since you got on: only then can it go down
    Real bikeDown_ = 0;          // how long it has lain on its side (s): down past 0.4 s
    std::string wantRecipe_;   // `vehicle_spawn <recipe>`: picked once the catalogue is loaded
    double clock_ = 0.0, panelUntil_ = 0.0;
    std::string panelText_;
    void loadCatalogue();
    MeshHandle wheelMesh;            // shared wheel cylinder, uploaded on first use
    MeshHandle driverMesh;          // shared driver capsule
    Real enterRadius = 4.0;          // how close the player must be to board (m)
    int spawnCount_ = 0;             // debug-spawn seed counter
    // Horn: one looping procedural voice (sfx::horn) held while the key is
    // down, volume-ramped so press/release don't click.
    AudioClipHandle hornClip;
    AudioVoiceHandle hornVoice;
    float hornGain = 0.0f;
    // Engines: the running physical cars -- the player's and any AI-driven one -- voiced by the one engine pool
    // (audio/engine_voices.h): the nearest 6 within 90 m, with seed 1's rasp, so your own engine still reads as
    // yours beside the city's traffic (seed 2, citysim's CityTrafficAudioSystem).
    EngineVoicePool engines_{6, 90.0, 1};
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
