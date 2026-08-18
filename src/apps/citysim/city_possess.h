#ifndef RAYTRACER_APPS_CITYSIM_CITY_POSSESS_H
#define RAYTRACER_APPS_CITYSIM_CITY_POSSESS_H

#include "../../engine/system.h"
#include "../../engine/ai/lane_follow.h"
#include "../../engine/ai/traffic_sense.h"
#include "../../engine/camera/follow_camera_controller.h"
#include "city_possess_logic.h"

#include <vector>

namespace engine {
class PhysicsSystem;
}

namespace citysim {

class CityRenderSystem;

// The control channel's AVATAR (ADR-0079). An agent directing the engine can't
// play in realtime — its perception-action loop is seconds and stills — so the
// realtime half stays in-engine: this system spawns a car (or commandeers a
// sim pedestrian), attaches the AgentDriver brain, and follows routed
// destinations with the pursuit/sensing stack that has sat harness-tested in
// engine/ai since the possession-tier retirement. The director supplies
// INTENT (`possess.cmd` one-shots staged by the channel) and reads state back
// (`possess.status`, rewritten every frame).
//
// This is the first producer of AgentDriver — VehicleSystem has consumed it
// (vehicle_system.cpp driveVehicles) since ADR-0062 with nobody writing it.
//
// Ordering contract (arena_state.cpp): registered AFTER CityWalkerSystem so
// commandeered pedestrians already have bodies, BEFORE VehicleSystem so the
// fixedUpdate command is consumed the same tick — and its update() camera
// write runs after PlayerSystem's, so the chase view wins the frame (the
// CitySpectateSystem precedent). Transient play acts only: nothing here
// touches the level document (no SourceSpec — runtime spawns are never saved).
class CityPossessSystem : public engine::System {
public:
    CityPossessSystem(CityRenderSystem& city, engine::PhysicsSystem& physics)
        : city_(city), physicsSys_(physics) {}

    void update(engine::FrameContext& ctx) override;       // commands + camera + status
    void fixedUpdate(engine::FrameContext& ctx) override;  // the drive loop

private:
    void handleCommand(engine::FrameContext& ctx, const PossessCmd& cmd);
    void possessCar(engine::FrameContext& ctx, const PossessCmd& cmd);
    void possessWalker(engine::FrameContext& ctx, const PossessCmd& cmd);
    void driveTo(engine::FrameContext& ctx, engine::Real x, engine::Real z);
    void walkTo(engine::FrameContext& ctx, engine::Real x, engine::Real z);
    void releasePossession(engine::FrameContext& ctx);
    void driveCar(engine::FrameContext& ctx);
    void updateCamera(engine::FrameContext& ctx);
    void publishStatus(engine::FrameContext& ctx);
    // Walker pose: physical body when present, sim ghost otherwise.
    bool walkerPose(engine::Vec3& outPos, engine::Vec2& outHeading) const;

    CityRenderSystem& city_;
    engine::PhysicsSystem& physicsSys_;

    engine::Entity car_;                       // possessed car (kind == car)
    int walkerAgent_ = -1;                     // possessed pedestrian (>= 0)
    PossessState state_ = PossessState::None;
    std::string error_;                        // last command error, for status

    engine::LaneFollower follower_;            // car route
    std::vector<engine::Real> pathSpeeds_;     // per-point class speeds
    engine::StuckDetector stuck_;
    engine::Real stuckHold_ = 0;               // brake-hold left after a stuck fire
    engine::Real lastLateral_ = 0;             // follower telemetry, for status
    engine::Real lastLeadGap_ = -1;            // sensed leader gap (-1 = clear)
    engine::Vec2 dest_{0, 0};                  // active destination (both kinds)
    bool hasDest_ = false;

    engine::FollowCameraController follow_;    // owned chase rig (spectate idiom)
    int spawnSeed_ = 0;
};

}  // namespace citysim

#endif
