#ifndef RAYTRACER_APPS_CITYSIM_CITY_POSSESS_H
#define RAYTRACER_APPS_CITYSIM_CITY_POSSESS_H

#include "../../engine/system.h"
#include "../../engine/camera/follow_camera_controller.h"
#include "../../engine/procgen/city/polygon.h"   // Vec2
#include "city_possess_logic.h"
#include "../../engine/control/control_commands.h"

#include <vector>

namespace citysim {

class CityRenderSystem;

// The control channel's AVATAR (ADR-0079). An agent directing the engine can't
// play in realtime — its perception-action loop is seconds and stills — so the
// realtime half stays in-engine: this system commandeers a sim pedestrian and
// walks it to routed destinations. The director supplies INTENT
// (PossessRequests from the channel's verbs) and reads state back (`possess?`,
// `look?`, `agent?`, which this system answers). A possessed walker can get
// into a household's car of the new driving (`board`) and out of it again
// (`alight`). Its own car driver -- a spawned Jolt sedan steered along the
// street map's lanes -- was deleted with the old driving (ADR-0459).
//
// Ordering contract (arena_state.cpp): registered AFTER CityWalkerSystem so
// commandeered pedestrians already have bodies, and its update() camera write
// runs after PlayerSystem's, so the chase view wins the frame (the
// CitySpectateSystem precedent). Transient play acts only: nothing here
// touches the level document.
class CityPossessSystem : public engine::System {
public:
    explicit CityPossessSystem(CityRenderSystem& city) : city_(city) {}

    void onStart(engine::FrameContext& ctx) override;      // hears PossessRequest, answers possess?/look?/agent?
    void update(engine::FrameContext& ctx) override;       // commands + camera + status
    void fixedUpdate(engine::FrameContext& ctx) override;  // arrival

private:
    void handleCommand(engine::FrameContext& ctx, const PossessCmd& cmd);
    void possessWalker(engine::FrameContext& ctx, const PossessCmd& cmd);
    void walkTo(engine::Real x, engine::Real z);
    void releasePossession();
    void publishLook();
    void boardNearby(const PossessCmd& cmd);
    void alightHere();
    void updateCamera(engine::FrameContext& ctx);
    void publishStatus();
    // Walker pose: physical body when present, sim ghost otherwise.
    bool walkerPose(engine::Vec3& outPos, engine::Vec2& outHeading) const;

    CityRenderSystem& city_;

    int walkerAgent_ = -1;                     // possessed pedestrian (>= 0)
    bool directed_ = false;                    // goal layer held off (ADR-0091)
    PossessState state_ = PossessState::None;
    std::string error_;                        // last command error, for status
    // PossessRequests heard since the last update (ADR-0253), and what the three questions answer, measured each
    // frame by publishStatus/publishLook.
    std::vector<std::string> requests_;
    std::string statusLine_ = "none", lookLine_ = "none", agentLine_ = "none";
    engine::EventBus::Subscription onRequest_;
    engine::ControlCommands::Binding answerStatus_, answerLook_, answerAgent_;

    engine::Vec2 dest_{0, 0};                  // active destination
    bool hasDest_ = false;

    engine::FollowCameraController follow_;    // owned chase rig (spectate idiom)
};

}  // namespace citysim

#endif
