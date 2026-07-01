#ifndef RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H
#define RAYTRACER_APPS_CITYSIM_CITY_VEHICLES_H

#include "../../engine/system.h"
#include "../../engine/ai/lane_follow.h"   // LaneFollower (closed-loop pursuit)
#include "../../renderer/renderer.h"       // engine::MeshHandle
#include "city_render.h"

#include <vector>

namespace citysim {

// The one-car-system bridge (ADR-0061). Instead of drawing NPC cars as kinematic
// pose-holders, this spawns each CitySim driver agent a REAL engine Vehicle — the
// exact same Jolt-backed object the player drives — tagged with an AgentDriver.
//
// The control loop is CLOSED over the car's real pose (not an open-loop ghost
// chase). The CitySim ghost is the PLANNER — it decides route, signals, and the
// speed to hold — and the physical car follows the plan from where it actually is:
//   - HEADING: pure pursuit (LaneFollower) over the ghost's lane path, aimed from
//     the car's real position — so a car recovers its lane after a shove, arcs a
//     corner instead of cutting it, and stays stable at speed.
//   - SPEED: proportional station control toward the ghost (drive up to it, match
//     its planned speed, brake when overshot) — so a lagging car catches up and a
//     stopped ghost (red light / arrival) parks the car at the right spot.
//   - TETHER: the ghost may never LEAD the car by more than a leash — a car
//     knocked back or climbing finds its plan waiting just ahead, never gone.
//
// Because the cars are real Vehicles, the player can commandeer one (VehicleSystem
// ejects its AgentDriver on entry); this bridge notices the ejection and releases
// that ghost from the sim so the planner stops fighting the physical car.
//
// Viewer/editor only (needs Jolt via the engine VehicleSystem), not engine_core —
// like CityPhysicsSystem. UNVERIFIED on device (no Jolt build here); the control
// loop itself is exercised headless by tests/test_lane_follow.cpp's bicycle model.
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
        int trip = -1;           // ghost trip counter; a new trip rebuilds the path
        engine::LaneFollower follower;   // pursuit progress along the lane path
        bool released = false;   // player commandeered it -> ghost freed
    };

    CityRenderSystem& city_;
    std::vector<NpcCar> cars_;
    std::vector<engine::MeshHandle> bodyMesh_;   // one per fleet slot (shared)
    bool spawned_ = false;
};

}  // namespace citysim

#endif
