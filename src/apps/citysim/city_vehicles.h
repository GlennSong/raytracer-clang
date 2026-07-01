#ifndef RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H
#define RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H

#include "../../engine/system.h"
#include "../../renderer/renderer.h"   // engine::MeshHandle
#include "city_render.h"

#include <vector>

namespace citysim {

// The one-car-system bridge (ADR-0061). Instead of drawing NPC cars as kinematic
// pose-holders, this spawns each CitySim driver agent a REAL engine Vehicle — the
// exact same Jolt-backed object the player drives — tagged with an AgentDriver.
// The CitySim keeps running as the PLANNER: each step its ghost agent produces a
// desired heading + speed, which this bridge writes into the car's AgentDriver
// command; the shared controller (computeDriverInput) turns that into the same
// {throttle, steer, brake} the player's input produces, so an NPC car handles
// identically and the player can walk up and DRIVE any of them.
//
// Because the cars are real Vehicles, the player can commandeer one (VehicleSystem
// ejects its AgentDriver on entry); this bridge notices the ejection and releases
// that ghost from the sim so the planner stops fighting the physical car.
//
// Viewer/editor only (needs Jolt via the engine VehicleSystem), not engine_core —
// like CityPhysicsSystem. UNVERIFIED on device (no Jolt build here).
class CityVehicleSystem : public engine::System {
public:
    explicit CityVehicleSystem(CityRenderSystem& city) : city_(city) {}

    void fixedUpdate(engine::FrameContext& ctx) override;
    void onStop(engine::FrameContext& ctx) override;

private:
    void spawnCars(engine::FrameContext& ctx);
    void driveCars(engine::FrameContext& ctx);

    struct NpcCar {
        engine::Entity entity;   // the engine Vehicle entity
        int agentId = -1;        // the CitySim ghost that plans for it
        bool released = false;   // player commandeered it -> ghost freed
    };

    CityRenderSystem& city_;
    std::vector<NpcCar> cars_;
    std::vector<engine::MeshHandle> bodyMesh_;   // one per fleet slot (shared)
    bool spawned_ = false;
};

}  // namespace citysim

#endif
