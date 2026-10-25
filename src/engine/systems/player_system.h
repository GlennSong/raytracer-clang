#ifndef RAYTRACER_ENGINE_PLAYER_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_SYSTEM_H

#include "../system.h"
#include "../camera/fly_camera_controller.h"
#include "../camera/follow_camera_controller.h"
#include "../physics/physics_world.h"
#include "../player/player_body.h"   // the body's modes (and body_rules.h: the fall net, the water, the air)

#include <string>

namespace engine {

class PhysicsSystem;

// Drives the on-foot player character and its camera. What the body DOES is PlayerBody (engine/player/,
// ADR-0246): its modes, stepped from a PlayerIntent this system fills from the input map each fixed step. This
// system owns the rest -- the keys, the camera, fast travel, the tool row and prompts, `move?`.
//
// The FLY controller is the player's HEADING either way (CameraSystem feeds it mouse/stick look): first person
// renders from its pinned eye; third person (V -- the on-foot camera toggle) frames the same heading from an
// over-the-shoulder follow rig (FollowCameraController::applyShoulderPreset), fly pitch tilting the rig. The mode is
// published to Settings ("playerThirdPerson") so the render-side body system (apps/citysim/city_player_body.*)
// shows/hides the player's person mesh without a dependency on this system.
class PlayerSystem : public System {
public:
    PlayerSystem(FlyCameraController& camera, PhysicsSystem& physics)
        : camera(camera), physicsSys(physics) {}

    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;   // Debug > Movement: the tuning, live

    bool thirdPersonActive() const { return thirdPerson; }
    bool swimming() const { return body_.water.swimming; }
    const MovementTuning& tuning() const { return body_.tune; }

private:
    FlyCameraController& camera;
    PhysicsSystem& physicsSys;
    FollowCameraController shoulder;   // third-person on-foot rig (shoulder preset)
    bool thirdPerson = false;
    // Fly pitch -> shoulder tilt clamps: enough to look up without dropping the
    // eye through the pavement, and down without flipping overhead.
    static constexpr Real kShoulderPitchMin = -70.0;
    static constexpr Real kShoulderPitchMax = 30.0;
    Entity playerEntity;
    PlayerBody body_;
    // THE ONE-SHOTS, staged on the PER-FRAME press edge and spent by one fixed step (PlayerIntent). Reading the
    // edge in fixedUpdate directly would fire twice whenever a frame runs two steps (a catch-up burst), which is a
    // double-height jump.
    bool jumpRequested = false;
    bool limpRequested_ = false;  // G / `ragdoll`: go limp on purpose
    Entity playTarget_;           // what E chose on the playground (the broker's command)
    uint64_t playKind_ = 0;       // 1 a swing, 2 the bars
    bool grappleFire_ = false;    // the trigger's press, with the grapple in hand...
    bool grappleHeld_ = false;    // ...and held: let go and the rope goes
    PlayerIntent intent(FrameContext& ctx) const;
    Real fovBoost_ = 0;           // degrees added to the view at speed (eased)
    Real hotbarShow_ = 0;         // s left showing the tool row after a slot change
    Real hudClock_ = 0;           // for the prompt's pulse
    void drawGearHud(FrameContext& ctx);
};

}  // namespace engine

#endif
