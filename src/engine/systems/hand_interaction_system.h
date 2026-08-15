#ifndef RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H
#define RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H

#include <string>
#include <vector>

#include "../event_bus.h"
#include "../physics/physics_world.h"   // PhysicsBodyId, ConstraintId
#include "../system.h"
#include "../xr/xr_gestures.h"
#include "../xr/xr_grasp.h"
#include "../xr/xr_palette.h"

namespace engine {

class PhysicsSystem;

// Hands work like hands (AR sandbox). Reads the neutral hand skeletons in
// XrState and gives the sandbox its interaction loop:
//
//   PRESENCE — each tracked hand carries a kinematic capsule per finger bone
//   plus a palm sphere on the HAND layer (shove dynamic bodies, pass through
//   static room geometry — the real hand already stops on the real table),
//   driven through adaptive filters so swinging a hand through a pile sends
//   blocks flying with real Jolt impulses.
//
//   PALETTE — hold either palm facing up (XrPalette owns the FSM; pure,
//   host-tested) and the item row fans out along the forearm; the other
//   hand hovers with its index fingertip and pinches to spawn the item into
//   that hand, already held.
//
//   HOLD — every grab is ONE 6-DOF spring (PhysicsWorld::addGripSpring)
//   between an invisible per-hand GRIP body and the object, created at the
//   current relative pose: the object STAYS DYNAMIC, collides while
//   carried, presses stacks with bounded force, and leaves with its true
//   momentum when the spring is removed — no velocity handoff, no throw
//   estimator. Hooked by pinch near the object's oriented surface
//   (xr_touch) OR by CLOSURE: two-plus fingertips ON the shape (proximity
//   probe, xrProbeGrip — predictive, because contact events fire only
//   after the kinematic squeeze has ejected the object) with opposed press
//   directions (XrGripMemory::opposed — "held on each side"), which is how
//   a natural fist grip takes a pillar without any pinch. While held the
//   object rides the HELD layer (no hand-capsule collision — infinitely
//   strong fingers squeezing a held object shoot it away), restored only
//   once the fingers have cleared it.
//   Closure release is opening-based: live fingertips drifting off their
//   remembered object-local anchors (grip memory) past hysteresis. Each
//   hand hooks its own spring, so both hands can hold one object, and a
//   hand-to-hand toss is one hand's unhook and the other's closure hook —
//   no cooldowns on that path.
//
//   HAND MEMORY — the grip frame runs through XrHandMemory: brief tracking
//   dropouts (mid-throw, hand behind hand) COAST on the fitted velocity
//   instead of freezing the carried object, and reacquire blends back.
//
//   PLACEMENT ASSIST — while a held box hovers near a support (an aligned
//   box top, or whatever a downward ray finds), xrAlignToSupport proposes a
//   flush axis-aligned pose drawn as a faint ghost. It is APPLIED only at a
//   gentle release (< 0.25 m/s) — throws and drops are untouched, so the
//   assist can never fight the natural grip. Pref xr.placeAssist.
//
//   MECHANISMS — items may carry a sub-body joined by a hinge (chest lid)
//   or slider (bolt): grab either body with either hand; pulling the
//   sub-body works the joint.
//
//   PHYSICS GUN — the "gravgun" item IS the physical-hands answer to a
//   tool: grab it naturally (any orientation), and its trigger is a REAL
//   slider your index finger pushes (return motor snaps it home). Squeeze
//   once: a soft tractor spring pulls whatever the barrel points at to a
//   hold point floating ahead of the muzzle (rotation free — it dangles).
//   Squeeze again: launch along the barrel. Beam and candidate highlight
//   draw while a gun is held; an over-stretched beam lets go.
//
//   AUDIO — physics Collision events involving sandbox shapes become
//   spatial PlaySound cues (procedural sfx::impact variants; the state
//   registers the clips): knocks scale with impact speed, hand pats are
//   quieter, the gun launch thumps.
//
//   RENDERING — every object (held included — it's dynamic) draws at
//   fixed-step poses interpolated by ctx.interpolation (the RenderSystem
//   contract; raw Jolt poses stair-step 60Hz physics onto a 90Hz display).
//
// While the palette is up or a hold is active, the SYSTEM-level pinch
// (ctx.xr.pinch*) is consumed; this system registers before XrSurfaceSystem.
class HandInteractionSystem : public System {
public:
    explicit HandInteractionSystem(PhysicsSystem* physics);

    void onStart(FrameContext& ctx) override;
    void update(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    enum class JointKind { None, Hinge, Slider };

    struct ItemDef {
        const char* name;
        Vec3 halfExtent;    // box collider half-size, metres (also draw size)
        Vec3 color;
        bool sphere = false;   // sphere collider/mesh instead of box

        // Articulated items: a sub-body (always a box) joined to the main
        // body. Offsets/pivot/axis are main-body local at spawn.
        JointKind joint = JointKind::None;
        Vec3 subHalfExtent;
        Vec3 subColor;
        Vec3 subOffset;        // sub-body centre relative to main centre
        Vec3 pivotOffset;      // hinge pivot / slider anchor
        Vec3 axis{1, 0, 0};    // hinge/slide axis
        Real minLimit = 0;     // radians (hinge) / metres (slider)
        Real maxLimit = 0;
        Real jointSpring = 0;  // soft-limit frequency, 0 = hard stops
        Real jointReturn = 0;  // slider return-motor frequency (trigger)
        bool subGrabbable = true;   // false: the sub is a control, not cargo
        bool gravityGun = false;    // this item is the physics gun
    };
    static const ItemDef kItems[];
    static const int kItemCount;

    // A spawned sandbox object: dynamic Jolt bodies for life (a hold is a
    // spring, never a motion-type switch), plus the fixed-step pose pairs
    // the renderer interpolates between.
    struct SandboxObject {
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
        int item = 0;
        PhysicsBodyId subBody = INVALID_PHYSICS_BODY;   // articulated items
        ConstraintId jointId = INVALID_CONSTRAINT;
        bool triggerPressed = false;   // gravity gun: last trigger state
        Vec3 posPrev, posCurr; // fixed-step snapshots (render interpolation)
        Quat quatPrev, quatCurr;
        Vec3 subPosPrev, subPosCurr;
        Quat subQuatPrev, subQuatCurr;
    };

    // A grabbable shape: an object's main body or its sub-body.
    struct Pick {
        int obj = -1;
        bool sub = false;
        bool valid() const { return obj >= 0; }
    };

    // One hand's hold: the spring plus how it releases. pinchHold releases
    // on pinch-open; closure holds release when the live fingertips drift
    // off their remembered anchors (grip memory) past hysteresis.
    struct Hold {
        Pick pick;
        ConstraintId spring = INVALID_CONSTRAINT;
        bool pinchHold = false;
        XrGripMemory grip;
        Real hookedAt = 0;
        Real openSince = -1;
        Real lastOpening = 0;   // latest opening() — drives the anchor-dot
                                // release feedback (gold -> red)
        bool active() const { return spring != INVALID_CONSTRAINT; }
    };

    PhysicsSystem* physics_ = nullptr;
    std::vector<SandboxObject> objects_;
    MeshHandle itemMeshes_[12];
    MeshHandle subMeshes_[12];
    MeshHandle shadowMesh_;   // unit slab for the held-object drop shadow

    XrPinchTracker pinch_[2];
    Hold hold_[2];
    // Per-hand grip anchor: a tiny kinematic sphere on the GRIP layer
    // (collides with nothing), driven to the hand's grip frame — the fixed
    // end of every grip spring.
    PhysicsBodyId gripBody_[2] = {INVALID_PHYSICS_BODY, INVALID_PHYSICS_BODY};
    // Dropout coasting for the grip frame (pure, host-tested).
    XrHandMemory handMemory_[2];
    // Adaptive filter on the grip frame (heavy smoothing at rest, nearly
    // none in motion — a constant filter lagged fast carries by centimetres).
    XrAdaptiveFilter carry_[2];

    // Articulated hand presence: a kinematic capsule per finger bone plus a
    // palm sphere, driven from adaptively-filtered joint positions.
    struct HandBone {
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
        int jointA = 0;      // parent joint
        int jointB = 0;      // child joint
        Real radius = 0.009;
    };
    std::vector<HandBone> handBones_[2];   // built at first tracked sighting
    PhysicsBodyId palmBody_[2] = {INVALID_PHYSICS_BODY, INVALID_PHYSICS_BODY};
    XrAdaptiveFilter jointFilter_[2][XR_HAND_JOINT_COUNT];
    bool handActive_[2] = {false, false};   // false = parked / not created

    XrPalette palette_;
    float previewSpin_ = 0;
    double consumePinchUntil_ = 0;
    double timeSeconds_ = 0;
    bool placeAssist_ = true;              // pref xr.placeAssist

    // Toss detection (log only): the last unhook, so the other hand hooking
    // the same object right after reads as a hand-to-hand toss.
    int lastUnhookObj_ = -1;
    int lastUnhookHand_ = -1;
    Real lastUnhookTime_ = -10;

    // Per-hand: the shape this hand released last, so closure leaves it
    // alone for a beat instead of re-grabbing a departing throw whose
    // relative velocity is still ~0. The block is graduated at unhook
    // time: a fast release (a throw, whose follow-through keeps the hand
    // ON the object's path) blocks much longer than a gentle one (a
    // regrip, which should re-hook almost immediately).
    Pick recentUnhook_[2];
    Real rehookBlockUntil_[2] = {-10, -10};

    // Released shapes still on the HELD layer, waiting for the fingers to
    // clear before hand collision is restored (see applyHeldLayer).
    struct PendingClear {
        Pick pick;
        Real since = 0;
    };
    std::vector<PendingClear> layerClear_;

    // The gravity-gun tractor: one active beam — a kinematic muzzle anchor
    // (GRIP layer, collides with nothing) driven to a hold point ahead of
    // the gun, plus a soft pull-to-anchor spring on the target.
    struct Tractor {
        int gun = -1;      // objects_ index of the gravgun
        int target = -1;   // objects_ index of the pulled object
        ConstraintId spring = INVALID_CONSTRAINT;
        Real stretchedSince = -1;
        bool active() const { return spring != INVALID_CONSTRAINT; }
    };
    Tractor tractor_;
    PhysicsBodyId muzzleAnchor_ = INVALID_PHYSICS_BODY;

    // Impact audio: Collision events involving sandbox shapes become
    // spatial PlaySound cues (rate-limited; hand pats quieter).
    EventBus* events_ = nullptr;
    EventBus::SubscriptionId collisionSub_ = EventBus::INVALID_SUBSCRIPTION;
    Real lastCueAt_ = -1;

    Vec3 handWorld(FrameContext& ctx, const Vec3& originPoint) const;
    Quat handOrientation(const XrHand& hand) const;
    bool makeRoom();
    bool isHeld(int objIndex) const;

    PhysicsBodyId shapeBody(const Pick& pick) const;
    Vec3 shapeHalfExtent(const Pick& pick) const;
    bool shapeIsSphere(const Pick& pick) const;
    Real shapeSurfaceDistance(const Pick& pick, const Vec3& p) const;
    Vec3 shapeNearestPoint(const Pick& pick, const Vec3& p) const;
    // Nearest shape hand `h` could hook at `p` (its own held shapes are
    // excluded; the OTHER hand's hold is fair game — that's two-hand carry).
    Pick nearestGrabbable(int h, const Vec3& p, Real maxSurfaceDistance) const;

    void liveFingertips(FrameContext& ctx, int h,
                        std::vector<std::pair<int, Vec3>>& out) const;
    // Fingertips of hand `h` within `reach` of the shape's oriented
    // surface, as grip points (xrProbeGrip — predictive, not contact-based:
    // kinematic fingers that must penetrate to raise contacts have already
    // ejected the object by then).
    std::vector<XrGripPoint> probeShape(FrameContext& ctx, int h,
                                        const Pick& pick, Real reach) const;
    // While any spring holds a body it rides the HELD layer (no collision
    // with hand capsules — a kinematic finger squeeze is infinitely strong
    // and ejects it). The layer is restored only once every fingertip has
    // CLEARED the shape; restoring while fingers still overlap would eject
    // it at release instead.
    void applyHeldLayer(const Pick& pick);
    void releaseHeldLayer(int releasingHand, const Pick& pick);
    void settleHeldLayers(FrameContext& ctx);

    void hook(FrameContext& ctx, int h, const Pick& pick, bool pinchHold,
              const std::vector<XrGripPoint>& contacts);
    void unhook(FrameContext& ctx, int h);
    XrAlignedPose placementTarget(const Pick& pick) const;

    bool isHandBody(PhysicsBodyId body) const;
    Pick shapeForBody(PhysicsBodyId body) const;
    void gunFrame(int gunIdx, Vec3& muzzle, Vec3& dir) const;
    int tractorPick(int gunIdx) const;
    void updateGravityGuns(FrameContext& ctx);
    void releaseTractor(FrameContext& ctx, bool launch);

    void updateHandPresence(FrameContext& ctx);
    void updatePalette(FrameContext& ctx);
    void updateGrips(FrameContext& ctx);
    void spawnIntoHand(FrameContext& ctx, int item, int hand);
    MeshHandle itemMesh(FrameContext& ctx, int item);
    MeshHandle subMesh(FrameContext& ctx, int item);
};

}  // namespace engine

#endif  // RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H
