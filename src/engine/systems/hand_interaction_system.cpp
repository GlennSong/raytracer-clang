#include "hand_interaction_system.h"

#include <cmath>

#include "../../log.h"
#include "../mesh_builder.h"
#include "../xr/xr_touch.h"
#include "physics_system.h"

namespace engine {

namespace {

// Grab reach, measured to the object's SURFACE (nearest point on its
// oriented shape): forgiving of centimetre fingertip wander, but the pick
// region has exactly the object's shape — a lying pillar grabs along its
// whole lying length.
constexpr Real kGrabSurfaceReach = 0.10;
// Affordance range: candidates inside this of the pinch point show the
// oriented highlight + fingertip projection cues.
constexpr Real kApproachRange = 0.35;
constexpr size_t kMaxObjects = 32;
constexpr Real kPalmRadius = 0.035;

XrPalette::Config paletteConfig(int itemCount) {
    XrPalette::Config c;
    c.itemCount = itemCount;
    return c;
}

}  // namespace

// The spawnable inventory. Boxes + spheres only for now — every entry's
// collider matches its render mesh exactly, which keeps grab/rest behaviour
// honest. Procgen items (car, tree) join once their collider proxies exist.
const HandInteractionSystem::ItemDef HandInteractionSystem::kItems[] = {
    {"crate", Vec3(0.075, 0.075, 0.075), Vec3(0.80, 0.52, 0.25), false},
    {"ball", Vec3(0.075, 0.075, 0.075), Vec3(0.20, 0.55, 0.95), true},
    {"slab", Vec3(0.15, 0.025, 0.10), Vec3(0.55, 0.55, 0.60), false},
    {"pillar", Vec3(0.04, 0.18, 0.04), Vec3(0.85, 0.82, 0.70), false},
    {"wedge", Vec3(0.10, 0.05, 0.075), Vec3(0.30, 0.75, 0.40), false},
};
const int HandInteractionSystem::kItemCount =
    static_cast<int>(sizeof(HandInteractionSystem::kItems) /
                     sizeof(HandInteractionSystem::kItems[0]));

HandInteractionSystem::HandInteractionSystem(PhysicsSystem* physics)
    : physics_(physics), palette_(paletteConfig(kItemCount)) {}

Vec3 HandInteractionSystem::handWorld(FrameContext& ctx,
                                      const Vec3& originPoint) const {
    // Sandbox scale is locked to 1, so ORIGIN -> WORLD is the base
    // translation alone (xr_state.h spaces contract).
    return ctx.xr.originBase + originPoint;
}

Quat HandInteractionSystem::handOrientation(const XrHand& hand) const {
    return Quat::fromRotationMatrix(
        hand.joints[XR_JOINT_WRIST].originFromJoint);
}

Real HandInteractionSystem::objectSurfaceDistance(const SandboxObject& obj,
                                                  const Vec3& p) const {
    const ItemDef& def = kItems[obj.item];
    const Vec3 c = physics_->physicsWorld().bodyPosition(obj.body);
    if (def.sphere) return xrSphereSurfaceDistance(def.halfExtent.x, c, p);
    return xrBoxSurfaceDistance(def.halfExtent, c,
                                physics_->physicsWorld().bodyOrientation(obj.body),
                                p);
}

Vec3 HandInteractionSystem::objectNearestPoint(const SandboxObject& obj,
                                               const Vec3& p) const {
    const ItemDef& def = kItems[obj.item];
    const Vec3 c = physics_->physicsWorld().bodyPosition(obj.body);
    if (def.sphere) return xrNearestPointOnSphere(def.halfExtent.x, c, p);
    return xrNearestPointOnBox(def.halfExtent, c,
                               physics_->physicsWorld().bodyOrientation(obj.body),
                               p);
}

int HandInteractionSystem::nearestFreeObject(const Vec3& p,
                                             Real maxSurfaceDistance) const {
    int best = -1;
    Real bestD = maxSurfaceDistance;
    for (size_t i = 0; i < objects_.size(); i++) {
        if (objects_[i].heldByHand >= 0) continue;
        const Real d = objectSurfaceDistance(objects_[i], p);
        if (d < bestD) {
            bestD = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

MeshHandle HandInteractionSystem::itemMesh(FrameContext& ctx, int item) {
    if (!itemMeshes_[item].valid()) {
        const ItemDef& def = kItems[item];
        itemMeshes_[item] = ctx.renderer.uploadMesh(
            def.sphere ? MeshBuilder::sphere(static_cast<float>(def.halfExtent.x))
                       : MeshBuilder::box(def.halfExtent * 2.0));
    }
    return itemMeshes_[item];
}

// Room for one more: recycle the oldest FREE object, never a held one.
bool HandInteractionSystem::makeRoom() {
    if (objects_.size() < kMaxObjects) return true;
    for (size_t i = 0; i < objects_.size(); i++) {
        if (objects_[i].heldByHand >= 0) continue;
        physics_->physicsWorld().removeBody(objects_[i].body);
        objects_.erase(objects_.begin() + i);
        for (int h = 0; h < 2; h++)   // reindex held slots past the hole
            if (heldObject_[h] > static_cast<int>(i)) heldObject_[h]--;
        return true;
    }
    return false;   // everything held + cap (unreachable with two hands)
}

void HandInteractionSystem::spawnIntoHand(FrameContext& ctx, int item,
                                          int hand) {
    if (!makeRoom()) return;

    const ItemDef& def = kItems[item];
    const Vec3 at = handWorld(ctx, pinch_[hand].pinchPoint());
    const Quat q = handOrientation(ctx.xr.hands[hand]);

    SandboxObject obj;
    obj.item = item;
    obj.heldByHand = hand;
    obj.gripOffset = Quat::identity();   // spawned aligned to the hand
    obj.gripPosOffset = Vec3(0, 0, 0);   // spawned centred on the pinch
    obj.body = def.sphere
        ? physics_->physicsWorld().addSphere(def.halfExtent.x, at, q,
                                             BodyMotion::Kinematic)
        : physics_->physicsWorld().addBox(def.halfExtent, at, q,
                                          BodyMotion::Kinematic);
    if (obj.body == INVALID_PHYSICS_BODY) return;
    obj.posPrev = obj.posCurr = at;
    obj.quatPrev = obj.quatCurr = q;

    objects_.push_back(obj);
    heldObject_[hand] = static_cast<int>(objects_.size()) - 1;
    grip_[hand].clear();
    carry_[hand].reset(at, q);   // seed the carry filter at the spawn pose
    LOG_INFO("[xr] spawned %s into %s hand (%zu objects)", def.name,
             hand == 0 ? "left" : "right", objects_.size());
}

void HandInteractionSystem::release(FrameContext& ctx, int hand) {
    const int idx = heldObject_[hand];
    if (idx < 0) return;
    SandboxObject& obj = objects_[idx];
    const ItemDef& def = kItems[obj.item];
    (void)ctx;

    // Back to dynamic IN PLACE (SetMotionType — the body id is stable for
    // the object's life), leaving with the hand's velocities: linear from
    // the windowed fit, angular from the wrist. A hand that is HOLDING
    // STILL when it opens means "set it down" — residual estimator noise (a
    // few tenths of m/s) would knock the stack it was placed on, so below
    // the threshold both velocities zero.
    physics_->physicsWorld().setMotionType(obj.body, BodyMotion::Dynamic);
    Vec3 v = grip_[hand].linearVelocity();
    Vec3 w = grip_[hand].angularVelocity();
    if (v.length() < 0.25) {
        v = Vec3(0, 0, 0);
        w = Vec3(0, 0, 0);
    }
    physics_->physicsWorld().setLinearVelocity(obj.body, v);
    physics_->physicsWorld().setAngularVelocity(obj.body, w);
    obj.heldByHand = -1;
    heldObject_[hand] = -1;
    consumePinchUntil_ = timeSeconds_ + 0.4;
    LOG_INFO("[xr] release %s: throw v=%.2fm/s spin=%.1frad/s",
             hand == 0 ? "L" : "R", v.length(), w.length());
}

void HandInteractionSystem::updateHandPresence(FrameContext& ctx) {
    // The articulated hand: a kinematic capsule per finger bone plus a palm
    // sphere, driven from adaptively-filtered joints — the physical presence
    // that lets an open hand shove, backhand, and scatter blocks with
    // finger-shaped contacts. Kinematic drive means Jolt derives real
    // velocities: a swing imparts a swing's impulse.
    const Real dt = ctx.frameDelta;
    const Real moveDt = std::max(ctx.frameDelta, 1.0 / 240.0);
    for (int h = 0; h < 2; h++) {
        const XrHand& hand = ctx.xr.hands[h];
        if (!hand.tracked) {
            if (handActive_[h]) {
                // Park far below the room; teleport (not a kinematic sweep
                // — that would batter everything on the way down).
                const Vec3 park =
                    ctx.xr.originBase + Vec3(h == 0 ? -2 : 2, -100, 0);
                for (HandBone& bone : handBones_[h])
                    physics_->physicsWorld().teleport(bone.body, park,
                                                      Quat::identity());
                if (palmBody_[h] != INVALID_PHYSICS_BODY)
                    physics_->physicsWorld().teleport(palmBody_[h], park,
                                                      Quat::identity());
                handActive_[h] = false;
            }
            continue;
        }

        // Filter every finger joint + wrist (position only; capsule
        // orientation derives from the filtered endpoints).
        const bool reacquired = !handActive_[h];
        for (int j = XR_JOINT_WRIST; j < XR_HAND_JOINT_COUNT; j++) {
            if (!hand.joints[j].tracked) continue;
            const Vec3 target = handWorld(ctx, xrJointPosition(
                hand, static_cast<XrHandJointId>(j)));
            if (reacquired)
                jointFilter_[h][j].reset(target, Quat::identity());
            else
                jointFilter_[h][j].feed(target, Quat::identity(), dt);
        }

        // Build the bone set once, at first sighting: every finger joint
        // whose parent is another finger joint (the wrist-rooted metacarpal
        // bones span the palm — the palm sphere covers those).
        if (handBones_[h].empty()) {
            for (int j = XR_JOINT_THUMB_KNUCKLE; j < XR_HAND_JOINT_COUNT;
                 j++) {
                const int parent = xrHandJointParent(j);
                if (parent < XR_JOINT_THUMB_KNUCKLE) continue;
                if (!hand.joints[j].tracked || !hand.joints[parent].tracked)
                    continue;
                HandBone bone;
                bone.jointA = parent;
                bone.jointB = j;
                bone.radius = (j <= XR_JOINT_THUMB_TIP) ? 0.011 : 0.009;
                const XrBoneCapsule c = xrBoneCapsule(
                    jointFilter_[h][parent].position(),
                    jointFilter_[h][j].position(), bone.radius);
                bone.body = physics_->physicsWorld().addCapsule(
                    c.halfHeight, bone.radius, c.center, c.orientation,
                    BodyMotion::Kinematic);
                if (bone.body != INVALID_PHYSICS_BODY)
                    handBones_[h].push_back(bone);
            }
        }
        if (palmBody_[h] == INVALID_PHYSICS_BODY &&
            hand.joints[XR_JOINT_WRIST].tracked) {
            palmBody_[h] = physics_->physicsWorld().addSphere(
                kPalmRadius, jointFilter_[h][XR_JOINT_WRIST].position(),
                Quat::identity(), BodyMotion::Kinematic);
        }

        for (const HandBone& bone : handBones_[h]) {
            if (!hand.joints[bone.jointA].tracked ||
                !hand.joints[bone.jointB].tracked)
                continue;
            const XrBoneCapsule c = xrBoneCapsule(
                jointFilter_[h][bone.jointA].position(),
                jointFilter_[h][bone.jointB].position(), bone.radius);
            if (reacquired)
                physics_->physicsWorld().teleport(bone.body, c.center,
                                                  c.orientation);
            else
                physics_->physicsWorld().moveKinematic(bone.body, c.center,
                                                       c.orientation, moveDt);
        }
        if (palmBody_[h] != INVALID_PHYSICS_BODY &&
            hand.joints[XR_JOINT_WRIST].tracked) {
            const Vec3 p = jointFilter_[h][XR_JOINT_WRIST].position();
            if (reacquired)
                physics_->physicsWorld().teleport(palmBody_[h], p,
                                                  Quat::identity());
            else
                physics_->physicsWorld().moveKinematic(palmBody_[h], p,
                                                       Quat::identity(),
                                                       moveDt);
        }
        handActive_[h] = true;
    }
}

void HandInteractionSystem::updatePalette(FrameContext& ctx) {
    XrPalette::Inputs in;
    for (int h = 0; h < 2; h++) {
        in.palm[h] = xrPalmPose(ctx.xr.hands[h], h == 0);
        if (in.palm[h].valid)
            in.palm[h].position = handWorld(ctx, in.palm[h].position);
        in.handBusy[h] = heldObject_[h] >= 0;
        const XrHand& hand = ctx.xr.hands[h];
        in.fingertipValid[h] =
            hand.tracked && hand.joints[XR_JOINT_INDEX_TIP].tracked;
        if (in.fingertipValid[h])
            in.fingertip[h] =
                handWorld(ctx, xrJointPosition(hand, XR_JOINT_INDEX_TIP));
    }
    palette_.update(in, timeSeconds_);

    if (palette_.shownEdge())
        LOG_INFO("[xr] palette shown (%s palm)",
                 palette_.anchorHand() == 0 ? "left" : "right");
    if (palette_.hiddenEdge()) LOG_INFO("[xr] palette hidden");
    if (palette_.hoverChanged() && palette_.hoverItem() >= 0)
        LOG_INFO("[xr] palette hover %s", kItems[palette_.hoverItem()].name);

    // Free-hand pinch on a hovered item spawns it into that hand. Grabs ran
    // first this frame, so a pinch that landed near an existing object
    // already took it and set handBusy.
    const int anchor = palette_.anchorHand();
    if (anchor >= 0 && palette_.hoverItem() >= 0) {
        const int free = 1 - anchor;
        if (pinch_[free].began() && heldObject_[free] < 0) {
            spawnIntoHand(ctx, palette_.hoverItem(), free);
            consumePinchUntil_ = timeSeconds_ + 0.4;
        }
    }
}

void HandInteractionSystem::updateGrabs(FrameContext& ctx) {
    for (int h = 0; h < 2; h++) {
        const XrHand& hand = ctx.xr.hands[h];

        if (heldObject_[h] >= 0) {
            SandboxObject& obj = objects_[heldObject_[h]];
            if (pinch_[h].tracking()) {
                // Carry: drive the kinematic body to follow the pinch point
                // with the grip offsets preserved. The ADAPTIVE filter kills
                // millimetre noise at rest but gets out of the way at speed
                // — a fast carry tracks within ~1cm instead of trailing by
                // several (device: "the response can get very laggy").
                // Tracking loss skips all of it — the object freezes until
                // the hand returns.
                const Quat handQ = handOrientation(hand);
                const Quat rawQ = handQ * obj.gripOffset;
                const Vec3 rawTarget =
                    handWorld(ctx, pinch_[h].pinchPoint()) +
                    handQ.rotate(obj.gripPosOffset);
                carry_[h].feed(rawTarget, rawQ, ctx.frameDelta);
                physics_->physicsWorld().moveKinematic(
                    obj.body, carry_[h].position(), carry_[h].orientation(),
                    std::max(ctx.frameDelta, 1.0 / 240.0));
                grip_[h].push(timeSeconds_, carry_[h].position(),
                              carry_[h].orientation());
            }
            if (pinch_[h].released()) release(ctx, h);
            continue;
        }

        // Not holding: a fresh pinch within surface reach grabs. Surface
        // distance against the ORIENTED shape (xr_touch) — the pick region
        // is the object, whatever way it lies.
        if (!pinch_[h].began()) continue;
        const Vec3 at = handWorld(ctx, pinch_[h].pinchPoint());
        const int best = nearestFreeObject(at, kGrabSurfaceReach);
        if (best < 0) continue;

        SandboxObject& obj = objects_[best];
        const Vec3 p = physics_->physicsWorld().bodyPosition(obj.body);
        const Quat q = physics_->physicsWorld().bodyOrientation(obj.body);
        // Take it kinematic IN PLACE and remember the grip — orientation
        // AND position — so it carries exactly as grabbed instead of
        // snapping its centre into the fingers.
        physics_->physicsWorld().setMotionType(obj.body,
                                               BodyMotion::Kinematic);
        const Quat handQ = handOrientation(hand);
        obj.gripOffset = handQ.conjugate() * q;
        obj.gripPosOffset = handQ.conjugate().rotate(p - at);
        obj.heldByHand = h;
        heldObject_[h] = best;
        grip_[h].clear();
        carry_[h].reset(p, q);   // seed the carry filter at the grabbed pose
        consumePinchUntil_ = timeSeconds_ + 0.4;
        LOG_INFO("[xr] grab %s %s (surface %.2fm)", h == 0 ? "L" : "R",
                 kItems[obj.item].name, objectSurfaceDistance(obj, at));
    }
}

void HandInteractionSystem::update(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.active || !ctx.xr.originBaseValid) return;
    timeSeconds_ += ctx.frameDelta;
    previewSpin_ += static_cast<float>(ctx.frameDelta);

    pinch_[0].feed(ctx.xr.hands[0]);
    pinch_[1].feed(ctx.xr.hands[1]);

    updateHandPresence(ctx);
    // Grabs BEFORE the palette: a pinch that lands near an existing object
    // means "pick that up", even with the palette open.
    updateGrabs(ctx);
    updatePalette(ctx);

    // While hands are interacting, the SYSTEM gaze+pinch must not also fire
    // gaze-driven consumers (this system registers before XrSurfaceSystem).
    if (palette_.anchorHand() >= 0 || heldObject_[0] >= 0 ||
        heldObject_[1] >= 0 || timeSeconds_ < consumePinchUntil_) {
        ctx.xr.pinchEnded = false;
        ctx.xr.pinchBegan = false;
    }
}

void HandInteractionSystem::fixedUpdate(FrameContext& ctx) {
    if (!physics_) return;
    // Post-step pose snapshots (PhysicsSystem registers first, so the step
    // already ran): render() interpolates between these with
    // ctx.interpolation, exactly the Transform/PrevTransform contract
    // RenderSystem applies to entities — raw fixed-step poses stair-step
    // 60Hz physics onto a 90Hz display, which reads as jitter.
    (void)ctx;
    for (SandboxObject& obj : objects_) {
        obj.posPrev = obj.posCurr;
        obj.quatPrev = obj.quatCurr;
        obj.posCurr = physics_->physicsWorld().bodyPosition(obj.body);
        obj.quatCurr = physics_->physicsWorld().bodyOrientation(obj.body);
    }
}

void HandInteractionSystem::render(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.active || !ctx.xr.originBaseValid) return;

    // Spawned objects. Held: at the carry filter pose (90Hz smooth, zero
    // fixed-step quantisation in the hand). Free: interpolated between the
    // last two fixed steps.
    const Real alpha = ctx.interpolation;
    RenderMaterial material;
    material.metallic = 0.0f;
    material.roughness = 0.6f;
    for (const SandboxObject& obj : objects_) {
        material.albedo = kItems[obj.item].color;
        Vec3 p;
        Quat q;
        if (obj.heldByHand >= 0) {
            p = carry_[obj.heldByHand].position();
            q = carry_[obj.heldByHand].orientation();
        } else {
            p = obj.posPrev + (obj.posCurr - obj.posPrev) * alpha;
            q = Quat::slerp(obj.quatPrev, obj.quatCurr, alpha);
        }
        const Mat4 world = Mat4::translate(p.x, p.y, p.z) * q.toMat4();
        ctx.renderer.drawMesh(itemMesh(ctx, obj.item), world, material);
    }

    // Grab affordance + depth cues per reaching hand: the candidate shows
    // an ORIENTED wire box that turns green in grab range, and the thumb
    // and index tips project onto its surface — dot on the surface, faint
    // ruler line from fingertip to dot. That projection is the depth cue
    // that says how far the fingers are from touching.
    for (int h = 0; h < 2; h++) {
        if (heldObject_[h] >= 0 || !pinch_[h].tracking()) continue;
        const Vec3 at = handWorld(ctx, pinch_[h].pinchPoint());
        const int best = nearestFreeObject(at, kApproachRange);
        if (best < 0) continue;
        const SandboxObject& obj = objects_[best];
        const Real surface = objectSurfaceDistance(obj, at);
        const Real glow = 1.0 - surface / kApproachRange;
        const bool inReach = surface < kGrabSurfaceReach;
        const Vec3 color =
            (inReach ? Vec3(0.2, 1.0, 0.4) : Vec3(1.0, 1.0, 1.0)) *
            (0.35 + 0.65 * glow);
        ctx.debug.box(physics_->physicsWorld().bodyPosition(obj.body),
                      kItems[obj.item].halfExtent * 1.08,
                      physics_->physicsWorld().bodyOrientation(obj.body),
                      color);

        const XrHand& hand = ctx.xr.hands[h];
        const XrHandJointId tips[2] = {XR_JOINT_THUMB_TIP,
                                       XR_JOINT_INDEX_TIP};
        for (const XrHandJointId tip : tips) {
            if (!hand.joints[tip].tracked) continue;
            const Vec3 tipWorld = handWorld(ctx, xrJointPosition(hand, tip));
            const Vec3 onSurface = objectNearestPoint(obj, tipWorld);
            ctx.debug.sphere(onSurface, 0.008, color, 0, 8);
            ctx.debug.line(tipWorld, onSurface, color * 0.5);
        }
    }

    // The palette: item previews spinning above the anchor palm, hover ring
    // under the hovered one.
    const int anchor = palette_.anchorHand();
    if (anchor >= 0) {
        XrPalmPose palm = xrPalmPose(ctx.xr.hands[anchor], anchor == 0);
        if (palm.valid) {
            palm.position = handWorld(ctx, palm.position);
            const Quat spin =
                Quat::fromAxisAngle(Vec3(0, 1, 0), previewSpin_ * 1.2);
            for (int i = 0; i < kItemCount; i++) {
                const Vec3 slot = palette_.slotPosition(palm, i);
                const ItemDef& def = kItems[i];
                // Fit each preview into a ~6cm cell regardless of item size.
                const Real maxDim =
                    2.0 * std::max(def.halfExtent.x,
                                   std::max(def.halfExtent.y,
                                            def.halfExtent.z));
                const Real s =
                    (i == palette_.hoverItem() ? 0.075 : 0.06) / maxDim;
                material.albedo = def.color;
                const Mat4 world = Mat4::translate(slot.x, slot.y, slot.z) *
                                   spin.toMat4() * Mat4::scale(s, s, s);
                ctx.renderer.drawMesh(itemMesh(ctx, i), world, material);
                if (i == palette_.hoverItem()) {
                    const Vec3 a = palm.across * 0.035;
                    const Vec3 b = palm.fingers * 0.035;
                    const Vec3 base = slot - palm.normal * 0.03;
                    const Vec3 ring = Vec3(0.3, 1.0, 0.5);
                    ctx.debug.line(base - a - b, base + a - b, ring);
                    ctx.debug.line(base + a - b, base + a + b, ring);
                    ctx.debug.line(base + a + b, base - a + b, ring);
                    ctx.debug.line(base - a + b, base - a - b, ring);
                }
            }
        }
    }
}

void HandInteractionSystem::onStop(FrameContext& ctx) {
    if (physics_) {
        for (const SandboxObject& obj : objects_)
            physics_->physicsWorld().removeBody(obj.body);
        for (int h = 0; h < 2; h++) {
            for (const HandBone& bone : handBones_[h])
                physics_->physicsWorld().removeBody(bone.body);
            handBones_[h].clear();
            if (palmBody_[h] != INVALID_PHYSICS_BODY) {
                physics_->physicsWorld().removeBody(palmBody_[h]);
                palmBody_[h] = INVALID_PHYSICS_BODY;
            }
        }
    }
    objects_.clear();
    heldObject_[0] = heldObject_[1] = -1;
    handActive_[0] = handActive_[1] = false;
    palette_ = XrPalette(paletteConfig(kItemCount));
    for (auto& mesh : itemMeshes_) {
        if (mesh.valid()) {
            ctx.renderer.removeMesh(mesh);
            mesh = MeshHandle{};
        }
    }
}

}  // namespace engine
