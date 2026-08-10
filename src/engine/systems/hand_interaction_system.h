#ifndef RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H
#define RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H

#include <string>
#include <vector>

#include "../physics/physics_world.h"   // PhysicsBodyId
#include "../system.h"
#include "../xr/xr_gestures.h"
#include "../xr/xr_palette.h"

namespace engine {

class PhysicsSystem;

// Hands work like hands (AR sandbox). Reads the neutral hand skeletons in
// XrState and gives the sandbox its interaction loop:
//
//   PRESENCE — each tracked hand carries six invisible kinematic sphere
//   colliders (five fingertips + palm), driven through a short low-pass so
//   swinging a hand through a pile sends blocks flying with real Jolt
//   impulses. Tracking loss parks them far below the room.
//
//   PALETTE — hold either palm facing up (XrPalette owns the FSM; pure,
//   host-tested) and the item row fans out along the forearm; the other
//   hand hovers with its index fingertip and pinches to spawn the item into
//   that hand, already grabbed. The ONLY spawn path — the gaze-drop probe
//   is arena-only now.
//
//   GRAB — pinch within ~10cm of an object's SURFACE (nearest point on the
//   oriented shape, xr_touch — a lying pillar's pick region is the lying
//   pillar) to take it kinematic; it follows the pinch with the grip's
//   position AND orientation offsets preserved through a ~50ms filter.
//   Opening the fingers releases it dynamic with the hand's fitted linear +
//   angular velocity (still hand = velocities zeroed = clean placement).
//   Motion changes are Jolt SetMotionType switches — one body per object,
//   for life.
//
//   AFFORDANCE — the grab candidate shows an ORIENTED wire box that turns
//   green in range, and the reaching hand's thumb/index tips project onto
//   its surface as dots with ruler lines — the depth cue that tells you
//   how far your fingers are from touching it.
//
//   RENDERING — free objects draw at fixed-step poses interpolated by
//   ctx.interpolation (the RenderSystem contract; raw Jolt poses
//   stair-step 60Hz physics onto a 90Hz display), held objects draw at the
//   carry filter pose directly.
//
// While the palette is up or a grab is active, the SYSTEM-level pinch
// (ctx.xr.pinch*) is consumed; this system registers before XrSurfaceSystem.
class HandInteractionSystem : public System {
public:
    explicit HandInteractionSystem(PhysicsSystem* physics);

    void update(FrameContext& ctx) override;
    void fixedUpdate(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

private:
    struct ItemDef {
        const char* name;
        Vec3 halfExtent;    // box collider half-size, metres (also draw size)
        Vec3 color;
        bool sphere;        // sphere collider/mesh instead of box
    };
    static const ItemDef kItems[];
    static const int kItemCount;

    // A spawned sandbox object: ONE dynamic Jolt body for life (held =
    // switched kinematic), plus how to draw it and the fixed-step pose pair
    // the renderer interpolates between.
    struct SandboxObject {
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
        int item = 0;
        int heldByHand = -1;   // -1 free, 0 left, 1 right
        Quat gripOffset;       // object orientation relative to the hand's
        Vec3 gripPosOffset;    // object centre relative to the pinch point,
                               // hand space — grabbing the corner of a slab
                               // must not snap its centre to the fingers
        Vec3 posPrev, posCurr; // fixed-step snapshots (render interpolation)
        Quat quatPrev, quatCurr;
    };

    PhysicsSystem* physics_ = nullptr;
    std::vector<SandboxObject> objects_;
    MeshHandle itemMeshes_[8];

    XrPinchTracker pinch_[2];
    XrPoseHistory grip_[2];
    int heldObject_[2] = {-1, -1};   // index into objects_ per hand
    // Adaptive carry filter per hand (heavy smoothing at rest, nearly none
    // in motion — a constant filter lagged fast carries by centimetres).
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

    Vec3 handWorld(FrameContext& ctx, const Vec3& originPoint) const;
    Quat handOrientation(const XrHand& hand) const;
    bool makeRoom();
    // Surface distance / nearest point of an object's oriented shape.
    Real objectSurfaceDistance(const SandboxObject& obj, const Vec3& p) const;
    Vec3 objectNearestPoint(const SandboxObject& obj, const Vec3& p) const;
    int nearestFreeObject(const Vec3& p, Real maxSurfaceDistance) const;
    void updateHandPresence(FrameContext& ctx);
    void updatePalette(FrameContext& ctx);
    void updateGrabs(FrameContext& ctx);
    void spawnIntoHand(FrameContext& ctx, int item, int hand);
    void release(FrameContext& ctx, int hand);
    MeshHandle itemMesh(FrameContext& ctx, int item);
};

}  // namespace engine

#endif  // RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H
