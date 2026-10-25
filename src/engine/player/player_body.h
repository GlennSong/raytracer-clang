#ifndef RAYTRACER_ENGINE_PLAYER_BODY_H
#define RAYTRACER_ENGINE_PLAYER_BODY_H

// THE PLAYER'S BODY AS A STATE MACHINE (ADR-0246). Each way of moving is a BodyMode -- on foot, swimming, on a
// ladder, pulling up, climbing, under a canopy, in a wingsuit, on the rope, on a board, on the swing, on the bars,
// limp -- holding its own state and stepping the body when it is the current one. PlayerBody runs them:
//
//   1. the INTERRUPTS, over every mode: a ladder taken, the water, a teleport, going limp;
//   2. the current mode's ways out that are taken BEFORE it steps (into the water, onto a board or a swing): the
//      mode they lead to steps this same step;
//   3. the current mode's step, which moves the body and may leave for another (go limp, pull up, let go);
//   4. the pose (PlayerPose), which is what everything that draws, sounds or reacts to the body reads.
//
// The modes read a PlayerIntent, never the keys. What the hands hold is PlayerHands, alongside: the tool in hand,
// and the grapple's hook, which flies, holds and comes back whatever the body is doing.

#include "player_intent.h"
#include "player_hands.h"
#include "../components.h"
#include "../locomotion.h"
#include "../physics/physics_world.h"
#include "body_rules.h"          // FallRespawnTracker, SwimState, ClimbSpeed, airborneVelocity

#include <cstdint>
#include <string>

namespace engine {

class PlayerBody;
class Settings;
struct PlayerPose;

// One step's world: what a mode works on.
struct BodyStep {
    World& world;
    Settings& settings;
    Entity e;
    Transform& t;
    CharacterController& cc;
    PhysicsWorld& pw;
    PlayerIntent& in;
    Real dt;
};

// The families the modes fall in (the parents of the hierarchy: what a rule over "in the air" applies to).
enum class BodyGroup : uint8_t { Ground, Water, Wall, Air, Riding, Play, Limp };

class BodyMode {
public:
    virtual ~BodyMode() = default;
    virtual const char* name(const PlayerBody& b) const = 0;   // for `move?`
    virtual BodyGroup group() const = 0;
    // The ways out taken before the step: b.enter() another mode and that one is stepped (and asked) next.
    virtual void leave(PlayerBody&, BodyStep&) {}
    virtual void step(PlayerBody& b, BodyStep& s) = 0;
    // What the pose says in this mode (the shared part is written first, by PlayerBody).
    virtual void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const = 0;
    // Hands free for something else (the gun): 2 both, 1 one, 0 none.
    virtual int handsFree() const { return 0; }
    // Can the grapple be aimed and fired from here? (Not with both hands on a ledge, nor limp.)
    virtual bool canAim() const { return true; }
    // Is the velocity worth publishing? (Hanging on a wall or a ledge the pose keeps the last one.)
    virtual bool movesFreely() const { return true; }
};

// --- on foot: walking, running, crouching, jumping, falling -----------------------------------------------------
class OnFootMode final : public BodyMode {
public:
    const char* name(const PlayerBody&) const override { return "walk"; }
    BodyGroup group() const override { return BodyGroup::Ground; }
    void leave(PlayerBody& b, BodyStep& s) override;
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
    int handsFree() const override { return 2; }
};

// --- afloat (#43) -----------------------------------------------------------------------------------------------
class SwimMode final : public BodyMode {
public:
    const char* name(const PlayerBody&) const override { return "swim"; }
    BodyGroup group() const override { return BodyGroup::Water; }
    void leave(PlayerBody& b, BodyStep& s) override;
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
};

// --- on a ladder (engine/ladder.h): the Climbing component says so, LadderSystem put it there --------------------
class LadderMode final : public BodyMode {
public:
    const char* name(const PlayerBody&) const override { return "ladder"; }
    BodyGroup group() const override { return BodyGroup::Wall; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
    bool movesFreely() const override { return false; }
};

// --- pulling up onto a ledge ------------------------------------------------------------------------------------
class MantleMode final : public BodyMode {
public:
    MantleMove move;
    const char* name(const PlayerBody&) const override { return "mantle"; }
    BodyGroup group() const override { return BodyGroup::Wall; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
    bool canAim() const override { return false; }
    bool movesFreely() const override { return false; }
};

// --- holding a wall ---------------------------------------------------------------------------------------------
class ClimbMode final : public BodyMode {
public:
    Vec3 normal{0, 0, 1};   // the wall held, facing the body
    Vec3 lungeDir{0, 1, 0};
    Real lunge = 0;         // s left on a lunge up the wall
    const char* name(const PlayerBody&) const override { return "climb"; }
    BodyGroup group() const override { return BodyGroup::Wall; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
    bool movesFreely() const override { return false; }
};

// --- gliding: the parachute, the wingsuit (one class, the two share a flight) ------------------------------------
struct GlideFlight {
    Vec3 velocity;           // integrated here (engine/locomotion.h)
    Vec3 heading{0, 0, -1};  // the canopy's
    Real open = 0;           // how far out the canopy is
};
class GlideMode final : public BodyMode {
public:
    explicit GlideMode(bool canopy) : canopy(canopy) {}
    const bool canopy;   // the parachute (else the wingsuit)
    const char* name(const PlayerBody&) const override { return canopy ? "parachute" : "wingsuit"; }
    BodyGroup group() const override { return BodyGroup::Air; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
};

// --- on the grapple's rope --------------------------------------------------------------------------------------
class GrappleMode final : public BodyMode {
public:
    Vec3 anchor;
    Real ropeLen = 0;
    Vec3 ropeVel;
    Real yank = 0;          // s left of the pull that lifts a body hooked from the ground into the swing
    Real yankTo = 0;        // m: the rope the pull reels down to, short enough that the swing clears the ground
    bool takeUp = false;    // after a pull from the ground the reel takes up the slack until the rope catches
    Real reel = 0;          // m/s reeling this step (+ in), for the winch's sound
    const char* name(const PlayerBody&) const override { return "grapple"; }
    BodyGroup group() const override { return BodyGroup::Air; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
};

// --- on a board: the skateboard (slot 8), the scooter (9) --------------------------------------------------------
class BoardMode final : public BodyMode {
public:
    Vec3 heading{0, 0, -1};
    Real speed = 0, kickClock = 0;
    const char* name(const PlayerBody& b) const override;
    BodyGroup group() const override { return BodyGroup::Riding; }
    void leave(PlayerBody& b, BodyStep& s) override;
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
};

// --- the playground (engine/playground.h) -----------------------------------------------------------------------
class SwingMode final : public BodyMode {
public:
    Entity seat;
    const char* name(const PlayerBody&) const override { return "swing"; }
    BodyGroup group() const override { return BodyGroup::Play; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
};
class BarsMode final : public BodyMode {
public:
    Entity bars;
    Real at = 0, cycle = 0;   // along the bars (m from a), hand-over-hand cycles
    Vec3 facing{0, 0, 1};
    const char* name(const PlayerBody&) const override { return "monkey bars"; }
    BodyGroup group() const override { return BodyGroup::Play; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
};

// --- limp (engine/block_body.h's ragdoll) -----------------------------------------------------------------------
class RagdollMode final : public BodyMode {
public:
    PhysicsWorld::RagdollId id = PhysicsWorld::INVALID_RAGDOLL;
    Real limpFor = 0, stillFor = 0;
    const char* name(const PlayerBody&) const override { return "ragdoll"; }
    BodyGroup group() const override { return BodyGroup::Limp; }
    void step(PlayerBody& b, BodyStep& s) override;
    void pose(const PlayerBody& b, const BodyStep& s, PlayerPose& p) const override;
    bool canAim() const override { return false; }
};

// --- the machine, and what every mode shares --------------------------------------------------------------------
class PlayerBody {
public:
    // One fixed step of the body (PlayerSystem calls it for the player's own, not while seated or riding).
    void step(BodyStep& s);
    // The pose, from the current mode (and the hook, and what the body set off: PlayerBody::effects).
    void publish(BodyStep& s);
    void respawn(PhysicsWorld& pw, CharacterId id, bool manual);
    void event(const std::string& what);

    BodyMode& mode() const { return *mode_; }
    bool is(const BodyMode& m) const { return mode_ == &m; }
    void enter(BodyMode& m) { mode_ = &m; }

    // THE WAYS INTO A MODE that more than one mode takes (each sets the mode up, enters it, says so)
    bool tryMantle(BodyStep& s, const Vec3& facing, Real maxHeight);
    bool tryGrab(BodyStep& s, const Vec3& facing);
    bool tryWallJump(BodyStep& s, const Vec3& facing);   // jump in the air beside a wall: off it
    bool tryDeploy(BodyStep& s);                          // jump in the air with a chute or a suit on
    bool attachRope(BodyStep& s, const Vec3& at);         // the hook bit: onto the rope (from foot or a glide)
    void goLimp(BodyStep& s, const Vec3& velocity, const std::string& why);

    // the modes
    OnFootMode onFoot;
    SwimMode swim;
    LadderMode ladder;
    MantleMode mantle;
    ClimbMode climb;
    GlideMode parachute{true}, wingsuit{false};
    GrappleMode grapple;
    BoardMode board;
    SwingMode swing;
    BarsMode bars;
    RagdollMode ragdoll;
    PlayerHands hands;

    // WHAT THE MODES SHARE
    MovementTuning tune;
    Real eyeHeight = 0.7;
    Real moveSpeed = 6.0;
    // the standing capsule, captured on first sight (level-authored); crouched is derived from it, and standing up
    // can be REFUSED by the physics fit test (a ledge overhead keeps you down)
    bool crouched = false;
    bool standCaptured = false;
    Real standHalfHeight = 0.4;
    Real standRadius = 0.3;
    static constexpr Real kCrouchHalfScale = 0.35;   // of the standing half-height (the radius stays)
    static constexpr Real kCrouchSpeedScale = 0.4;
    // the spawn, and the net under a fall
    Vec3 spawnPos{0, 0, 0};
    bool spawnCaptured = false;
    FallRespawnTracker fall;
    Vec3 lastBodyPos{0, 0, 0};    // where the last step left the body
    bool haveLastBodyPos = false;
    // jumping and the air
    JumpGate gate;
    Vec3 airVel{0, 0, 0};         // horizontal velocity carried through the air (airborneVelocity)
    bool jumpRising = false;      // a jump on its way up that letting go of jump can still cut short
    bool wallJumped = false;      // jumped off a wall since last on the ground...
    Vec3 wallJumpNormal;          // ...this one: the next kick must be off another (no running up one wall)
    Real stamina = 12;            // s of climbing left
    Real regrab = 0;              // s before a wall can be grabbed again (after letting go)
    Real landed = 0;              // a landing's jolt, eased
    Real stride = 0;              // walk cycles, for the arms
    Real climbCycle = 0;          // hand over hand: the wall, the ladder
    // the water
    SwimState water;              // afloat in deep water (#43): the flag the modes go by
    Real swimClock = 0;
    Real swimLevel = 0;           // the surface of the water the player is in
    Real swimVy = 0;              // vertical speed in the water: a plunge carries in, buoyancy brings it up
    Real strokePhase = 0;         // swim strokes
    Real swimStroke = 0;          // treading (0) to the crawl (1), eased
    GlideFlight glide;
    // THINGS THAT HAPPENED, counted (the pose carries them: sounds and particles edge-detect them)
    uint32_t chuteOpens = 0, ollies = 0, boardLands = 0, grabs = 0, ropeSnaps = 0;
    Real snapSpeed = 0, landSpeed = 0;
    std::string lastEvent = "none";
    // the particles the body sets off (effects): bursts, and the two steady flows PlayerSystem makes
    Entity bubbleEmitter, streakEmitter;

private:
    BodyMode* mode_ = &onFoot;
    void effects(BodyStep& s);
    uint32_t fxOllies_ = 0, fxLands_ = 0, fxBites_ = 0;
    bool fxSeen_ = false;
    Real fxKickWas_ = 0;
    Real splashVy_ = 0;           // into the water this fast (m/s down), a splash owed
    Real breathIn_ = 3.0;         // s to the next breath let out under water
    bool heldWas_ = false;        // the hands were on something last step (a grab is the step they catch)
    Real lungeWas_ = 0;
    friend class OnFootMode;
    friend class SwimMode;
};

}  // namespace engine

#endif
