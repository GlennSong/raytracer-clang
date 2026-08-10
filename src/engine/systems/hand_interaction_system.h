#ifndef RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H
#define RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H

#include <string>
#include <vector>

#include "../physics/physics_world.h"   // PhysicsBodyId
#include "../system.h"
#include "../xr/xr_gestures.h"

namespace engine {

class PhysicsSystem;

// Hands work like hands (AR sandbox, ADR-0078 follow-on). Reads the neutral
// hand skeletons in XrState and gives the sandbox its interaction loop:
//
//   PALETTE — hold either palm facing up and a row of spawnable items fans
//   out above it (the other hand is then the picker, so the gesture is
//   left/right symmetric). Hover an item with the free hand's index
//   fingertip, pinch that hand to spawn the item INTO it, already grabbed.
//
//   GRAB — pinch (thumb+index) within reach of a spawned object to grab it:
//   the body goes kinematic and follows the pinch point and hand orientation,
//   so turning your wrist turns the object. Opening the fingers releases it
//   back to dynamic with the hand's velocity — linear from a least-squares
//   fit of the last 150ms, angular from the wrist's rotation — so a toss
//   arcs and a flick spins, and Jolt takes it from there (onto the room
//   colliders). Tracking loss freezes a carried object in place; it never
//   drops because you glanced away.
//
//   AFFORDANCE — the nearest grabbable object within approach range shows a
//   wire highlight, brightening when the pinch would connect, so "how close
//   am I" is visible before committing.
//
// Every gesture edge logs one `[xr]` line — the console is the verifier.
//
// While the palette is up, a grab is active, or a release just happened, the
// SYSTEM-level pinch (ctx.xr.pinchEnded, the gaze+pinch spatial event) is
// consumed so XrSurfaceSystem's gaze-drop probe doesn't also fire — this
// system must be registered BEFORE XrSurfaceSystem.
class HandInteractionSystem : public System {
public:
    explicit HandInteractionSystem(PhysicsSystem* physics)
        : physics_(physics) {}

    void update(FrameContext& ctx) override;
    void render(FrameContext& ctx) override;
    void onStop(FrameContext& ctx) override;

    // Spawn an item as a FREE dynamic body at a world position — the surface
    // system's gaze-drop delegates here so everything dropped is grabbable.
    // Cycles through the item set so gaze-dropping gives variety.
    void spawnDynamicAt(FrameContext& ctx, const Vec3& worldPos);

private:
    struct ItemDef {
        const char* name;
        Vec3 halfExtent;    // box collider half-size, metres (also draw size)
        Vec3 color;
        bool sphere;        // sphere collider/mesh instead of box
    };
    static const ItemDef kItems[];
    static const int kItemCount;

    // A spawned sandbox object: a dynamic Jolt body plus how to draw it.
    // While held the body is KINEMATIC (recreated; Jolt ids change across
    // grab/release, which is why the id lives here and nowhere else).
    struct SandboxObject {
        PhysicsBodyId body = INVALID_PHYSICS_BODY;
        int item = 0;
        int heldByHand = -1;   // -1 free, 0 left, 1 right
        Quat gripOffset;       // object orientation relative to the hand's
        Vec3 gripPosOffset;    // object centre relative to the pinch point,
                               // hand space — grabbing the corner of a slab
                               // must not snap its centre to the fingers
    };

    PhysicsSystem* physics_ = nullptr;
    std::vector<SandboxObject> objects_;
    MeshHandle itemMeshes_[8];

    XrPinchTracker pinch_[2];
    XrPoseHistory grip_[2];
    int heldObject_[2] = {-1, -1};   // index into objects_ per hand
    // Low-pass-filtered carry target per hand: raw joint poses jitter at the
    // millimetre level, and driving a kinematic body with that noise makes
    // the held object vibrate — and hammer anything it touches.
    Vec3 carryPos_[2];
    Quat carryQ_[2];

    // Palette state: which hand anchors it (-1 hidden), current hover.
    int paletteHand_ = -1;
    int hoverItem_ = -1;
    double paletteLastUp_ = -1;   // last time the anchor palm was truly up
    int gazeDropCounter_ = 0;     // cycles items for spawnDynamicAt
    double lastGazeDrop_ = -1;    // rate limit for gaze spawns
    float previewSpin_ = 0;
    double consumePinchUntil_ = 0;   // suppress the gaze-drop probe until then
    double timeSeconds_ = 0;

    Vec3 handWorld(FrameContext& ctx, const Vec3& originPoint) const;
    Quat handOrientation(const XrHand& hand) const;
    bool makeRoom();
    void updatePalette(FrameContext& ctx);
    void updateGrabs(FrameContext& ctx);
    void spawnIntoHand(FrameContext& ctx, int item, int hand);
    void release(FrameContext& ctx, int hand);
    MeshHandle itemMesh(FrameContext& ctx, int item);
};

}  // namespace engine

#endif  // RAYTRACER_ENGINE_HAND_INTERACTION_SYSTEM_H
