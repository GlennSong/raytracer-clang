#ifndef RAYTRACER_ENGINE_PLAYER_SYSTEM_H
#define RAYTRACER_ENGINE_PLAYER_SYSTEM_H

#include "../system.h"
#include "../properties.h"
#include "../control/control_commands.h"
#include "../camera/fly_camera_controller.h"
#include "../camera/follow_camera_controller.h"
#include "../physics/physics_world.h"
#include "../player/player_body.h"   // the body's modes (and body_rules.h: the fall net, the water, the air)
#include "../ui/ui_text.h"

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
// PersonRequest (ADR-0253): the on-foot camera, first (0) or third (1) person, or toggle (2) -- the V key from
// outside: the socket's `person` and the Teleport panel's checkbox.
struct PersonRequest {
    int mode = 2;
};
inline void describeProperties(PersonRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("mode").range(0, 2), r.mode);
}

// LimpRequest: the player goes limp (the G key from outside: the socket's `ragdoll`), or is thrown limp at a
// velocity (VehicleSystem, a rider off an open seat).
struct LimpRequest {
    bool thrown = false;
    Vec3 velocity;   // m/s, when thrown
};
inline void describeProperties(LimpRequest& r, PropertyVisitor& v) {
    v.field(FieldMeta("thrown"), r.thrown);
    v.field(FieldMeta("velocity").units("m/s"), r.velocity);
}

class PlayerSystem : public System {
public:
    PlayerSystem(FlyCameraController& camera, PhysicsSystem& physics)
        : camera(camera), physicsSys(physics) {}

    void onStart(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;   // Debug > Movement: the tuning, live

    bool thirdPersonActive() const { return thirdPerson; }
    // The movement line `move?` answers (mode, ground, position, velocity, stamina, the last change), measured in
    // render(): the trace harness reads it too.
    const std::string& movementTelemetry() const { return answers_.get("move?"); }
    bool swimming() const { return body_.water.swimming; }
    const MovementTuning& tuning() const { return body_.tune; }

private:
    EventBus::Subscription onPerson_, onLimp_;
    ControlAnswers answers_;   // move?
    bool limpThrown_ = false;     // the waiting limp is a throw, at limpVelocity_
    Vec3 limpVelocity_;
    int personPending_ = -1;   // a PersonRequest's mode, until update takes it
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
    UiText hud_;                  // the tool row and the prompts' words (the game UI layer)
public:
    void onStop(FrameContext& ctx) override { hud_.release(ctx.renderer); }
};

}  // namespace engine

#endif
