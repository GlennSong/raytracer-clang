#include "hand_interaction_system.h"

#include "../../log.h"
#include "../mesh_builder.h"
#include "physics_system.h"

namespace engine {

namespace {

// Reach for a grab: pinch within this of an object's centre grabs it. Wide
// enough to be forgiving (tracked fingertips wander a centimetre or two),
// tight enough that a pinch across the room never teleports an object.
constexpr Real kGrabReach = 0.30;
// Affordance range: objects inside this of a fingertip show the highlight.
constexpr Real kApproachRange = 0.45;
constexpr Real kPaletteSlotSpacing = 0.09;
constexpr Real kPaletteLift = 0.09;       // above the palm, metres
constexpr Real kPaletteHoverRadius = 0.055;
constexpr size_t kMaxObjects = 32;

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

MeshHandle HandInteractionSystem::itemMesh(FrameContext& ctx, int item) {
    if (!itemMeshes_[item].valid()) {
        const ItemDef& def = kItems[item];
        itemMeshes_[item] = ctx.renderer.uploadMesh(
            def.sphere ? MeshBuilder::sphere(static_cast<float>(def.halfExtent.x))
                       : MeshBuilder::box(def.halfExtent * 2.0));
    }
    return itemMeshes_[item];
}

void HandInteractionSystem::spawnIntoHand(FrameContext& ctx, int item,
                                          int hand) {
    // Room for one more: recycle the oldest FREE object, never a held one.
    if (objects_.size() >= kMaxObjects) {
        for (size_t i = 0; i < objects_.size(); i++) {
            if (objects_[i].heldByHand >= 0) continue;
            physics_->physicsWorld().removeBody(objects_[i].body);
            objects_.erase(objects_.begin() + i);
            for (int h = 0; h < 2; h++)   // reindex held slots past the hole
                if (heldObject_[h] > static_cast<int>(i)) heldObject_[h]--;
            break;
        }
        if (objects_.size() >= kMaxObjects) return;   // both hands full + cap
    }

    const ItemDef& def = kItems[item];
    const Vec3 at = handWorld(ctx, pinch_[hand].pinchPoint());
    const Quat q = handOrientation(ctx.xr.hands[hand]);

    SandboxObject obj;
    obj.item = item;
    obj.heldByHand = hand;
    obj.gripOffset = Quat::identity();   // spawned aligned to the hand
    obj.body = def.sphere
        ? physics_->physicsWorld().addSphere(def.halfExtent.x, at, q,
                                             BodyMotion::Kinematic)
        : physics_->physicsWorld().addBox(def.halfExtent, at, q,
                                          BodyMotion::Kinematic);
    if (obj.body == INVALID_PHYSICS_BODY) return;

    objects_.push_back(obj);
    heldObject_[hand] = static_cast<int>(objects_.size()) - 1;
    grip_[hand].clear();
    LOG_INFO("[xr] spawned %s into %s hand (%zu objects)", def.name,
             hand == 0 ? "left" : "right", objects_.size());
}

void HandInteractionSystem::release(FrameContext& ctx, int hand) {
    const int idx = heldObject_[hand];
    if (idx < 0) return;
    SandboxObject& obj = objects_[idx];
    const ItemDef& def = kItems[obj.item];

    // Swap the kinematic carrier for a dynamic body at the same pose, leaving
    // with the hand's velocities: linear from the windowed fit, angular from
    // the wrist — the throw itself.
    const Vec3 p = physics_->physicsWorld().bodyPosition(obj.body);
    const Quat q = physics_->physicsWorld().bodyOrientation(obj.body);
    physics_->physicsWorld().removeBody(obj.body);
    obj.body = def.sphere
        ? physics_->physicsWorld().addSphere(def.halfExtent.x, p, q,
                                             BodyMotion::Dynamic, 0.3, 0.5)
        : physics_->physicsWorld().addBox(def.halfExtent, p, q,
                                          BodyMotion::Dynamic, 0.2, 0.5);
    const Vec3 v = grip_[hand].linearVelocity();
    const Vec3 w = grip_[hand].angularVelocity();
    physics_->physicsWorld().setLinearVelocity(obj.body, v);
    physics_->physicsWorld().setAngularVelocity(obj.body, w);
    obj.heldByHand = -1;
    heldObject_[hand] = -1;
    consumePinchUntil_ = timeSeconds_ + 0.4;
    LOG_INFO("[xr] release %s: throw v=%.2fm/s spin=%.1frad/s",
             hand == 0 ? "L" : "R", v.length(), w.length());
}

void HandInteractionSystem::updatePalette(FrameContext& ctx) {
    // The palette anchors to whichever palm faces up (either hand — that IS
    // the lefty support); with both up, the first wins. A hand carrying an
    // object doesn't offer a palette. Hysteresis on the angle (tight to
    // show, loose to keep) plus a short linger absorb a palm hovering right
    // at the threshold — device session two logged the palette flapping
    // shown/hidden several times a second.
    int anchor = -1;
    XrPalmPose anchorPalm;
    for (int h = 0; h < 2; h++) {
        if (heldObject_[h] >= 0) continue;
        const XrPalmPose palm = xrPalmPose(ctx.xr.hands[h], h == 0);
        const Real keepAngle = (h == paletteHand_) ? 1.05 : 0.7;
        if (xrPalmUp(palm, keepAngle)) {
            anchor = h;
            anchorPalm = palm;
            break;
        }
    }
    if (anchor < 0 && paletteHand_ >= 0 &&
        timeSeconds_ - paletteLastUp_ < 0.35) {
        // Linger: palm dipped for a beat, keep the palette where it was.
        anchor = paletteHand_;
        anchorPalm = xrPalmPose(ctx.xr.hands[anchor], anchor == 0);
        if (!anchorPalm.valid) anchor = -1;
    } else if (anchor >= 0) {
        paletteLastUp_ = timeSeconds_;
    }
    if (anchor != paletteHand_) {
        paletteHand_ = anchor;
        hoverItem_ = -1;
        if (anchor >= 0)
            LOG_INFO("[xr] palette shown (%s palm)",
                     anchor == 0 ? "left" : "right");
        else
            LOG_INFO("[xr] palette hidden");
    }
    if (paletteHand_ < 0) return;

    // Hover: the free hand's index fingertip against the slot centres.
    const int free = 1 - paletteHand_;
    const XrHand& freeHand = ctx.xr.hands[free];
    int hover = -1;
    if (freeHand.tracked && freeHand.joints[XR_JOINT_INDEX_TIP].tracked) {
        const Vec3 tip =
            handWorld(ctx, xrJointPosition(freeHand, XR_JOINT_INDEX_TIP));
        Real best = kPaletteHoverRadius;
        for (int i = 0; i < kItemCount; i++) {
            const Vec3 slot = handWorld(ctx, anchorPalm.position)
                + anchorPalm.normal * kPaletteLift
                + anchorPalm.across * ((i - (kItemCount - 1) * 0.5) *
                                       kPaletteSlotSpacing);
            const Real d = (tip - slot).length();
            if (d < best) {
                best = d;
                hover = i;
            }
        }
    }
    if (hover != hoverItem_) {
        hoverItem_ = hover;
        if (hover >= 0) LOG_INFO("[xr] palette hover %s", kItems[hover].name);
    }

    // Free-hand pinch on a hovered item spawns it into that hand.
    if (hover >= 0 && pinch_[free].began() && heldObject_[free] < 0) {
        spawnIntoHand(ctx, hover, free);
        consumePinchUntil_ = timeSeconds_ + 0.4;
    }
}

void HandInteractionSystem::updateGrabs(FrameContext& ctx) {
    for (int h = 0; h < 2; h++) {
        const XrHand& hand = ctx.xr.hands[h];

        if (heldObject_[h] >= 0) {
            SandboxObject& obj = objects_[heldObject_[h]];
            if (pinch_[h].tracking()) {
                // Carry: drive the kinematic body to the pinch point with the
                // hand's orientation (grip offset preserved), and feed the
                // throw estimator. Tracking loss skips all of it — the
                // object freezes in place until the hand returns.
                const Vec3 target = handWorld(ctx, pinch_[h].pinchPoint());
                const Quat q = handOrientation(hand) * obj.gripOffset;
                physics_->physicsWorld().moveKinematic(
                    obj.body, target, q,
                    std::max(ctx.frameDelta, 1.0 / 240.0));
                grip_[h].push(timeSeconds_, target, q);
            }
            if (pinch_[h].released()) release(ctx, h);
            continue;
        }

        // Not holding: a fresh pinch near a free object grabs it. (A pinch
        // that just spawned from the palette set heldObject_ already and
        // never reaches here.)
        if (!pinch_[h].began()) continue;
        const Vec3 at = handWorld(ctx, pinch_[h].pinchPoint());
        int best = -1;
        Real bestD = kGrabReach;
        for (size_t i = 0; i < objects_.size(); i++) {
            if (objects_[i].heldByHand >= 0) continue;
            const Real d =
                (physics_->physicsWorld().bodyPosition(objects_[i].body) - at)
                    .length();
            if (d < bestD) {
                bestD = d;
                best = static_cast<int>(i);
            }
        }
        if (best < 0) continue;

        SandboxObject& obj = objects_[best];
        const ItemDef& def = kItems[obj.item];
        const Vec3 p = physics_->physicsWorld().bodyPosition(obj.body);
        const Quat q = physics_->physicsWorld().bodyOrientation(obj.body);
        // Re-create as kinematic; remember the object's orientation relative
        // to the hand so it keeps its grip alignment while carried.
        physics_->physicsWorld().removeBody(obj.body);
        obj.body = def.sphere
            ? physics_->physicsWorld().addSphere(def.halfExtent.x, p, q,
                                                 BodyMotion::Kinematic)
            : physics_->physicsWorld().addBox(def.halfExtent, p, q,
                                              BodyMotion::Kinematic);
        obj.gripOffset = handOrientation(hand).conjugate() * q;
        obj.heldByHand = h;
        heldObject_[h] = best;
        grip_[h].clear();
        consumePinchUntil_ = timeSeconds_ + 0.4;
        LOG_INFO("[xr] grab %s %s at %.2fm", h == 0 ? "L" : "R", def.name,
                 bestD);
    }
}

void HandInteractionSystem::update(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.active || !ctx.xr.originBaseValid) return;
    timeSeconds_ += ctx.frameDelta;
    previewSpin_ += static_cast<float>(ctx.frameDelta);

    pinch_[0].feed(ctx.xr.hands[0]);
    pinch_[1].feed(ctx.xr.hands[1]);

    updatePalette(ctx);
    updateGrabs(ctx);

    // While hands are interacting, the SYSTEM gaze+pinch must not also fire
    // the surface gaze-drop probe. This system registers before
    // XrSurfaceSystem, so clearing the edge here is seen there.
    if (paletteHand_ >= 0 || heldObject_[0] >= 0 || heldObject_[1] >= 0 ||
        timeSeconds_ < consumePinchUntil_) {
        ctx.xr.pinchEnded = false;
        ctx.xr.pinchBegan = false;
    }
}

void HandInteractionSystem::render(FrameContext& ctx) {
    if (!physics_ || !ctx.xr.active || !ctx.xr.originBaseValid) return;

    // Spawned objects, at the pose Jolt says.
    RenderMaterial material;
    material.metallic = 0.0f;
    material.roughness = 0.6f;
    for (const SandboxObject& obj : objects_) {
        material.albedo = kItems[obj.item].color;
        const Vec3 p = physics_->physicsWorld().bodyPosition(obj.body);
        const Mat4 world =
            Mat4::translate(p.x, p.y, p.z) *
            physics_->physicsWorld().bodyOrientation(obj.body).toMat4();
        ctx.renderer.drawMesh(itemMesh(ctx, obj.item), world, material);
    }

    // Grab affordance: the nearest free object to each open hand's pinch
    // point shows a wire box, brightening as the fingers get in range.
    for (int h = 0; h < 2; h++) {
        if (heldObject_[h] >= 0 || !pinch_[h].tracking()) continue;
        const Vec3 at = handWorld(ctx, pinch_[h].pinchPoint());
        int best = -1;
        Real bestD = kApproachRange;
        for (size_t i = 0; i < objects_.size(); i++) {
            if (objects_[i].heldByHand >= 0) continue;
            const Real d =
                (physics_->physicsWorld().bodyPosition(objects_[i].body) - at)
                    .length();
            if (d < bestD) {
                bestD = d;
                best = static_cast<int>(i);
            }
        }
        if (best < 0) continue;
        const SandboxObject& obj = objects_[best];
        const Vec3 c = physics_->physicsWorld().bodyPosition(obj.body);
        const Vec3 he = kItems[obj.item].halfExtent * 1.15;
        const Real glow = 1.0 - bestD / kApproachRange;
        const Vec3 color = (bestD < kGrabReach ? Vec3(0.2, 1.0, 0.4)
                                               : Vec3(1.0, 1.0, 1.0))
                         * (0.35 + 0.65 * glow);
        // Axis-aligned wire box (orientation omitted on purpose: it is an
        // affordance, not a bounding proof).
        const Vec3 mn = c - he, mx = c + he;
        const Vec3 corners[8] = {
            {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mn.y, mx.z},
            {mn.x, mn.y, mx.z}, {mn.x, mx.y, mn.z}, {mx.x, mx.y, mn.z},
            {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z}};
        const int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                  {4, 5}, {5, 6}, {6, 7}, {7, 4},
                                  {0, 4}, {1, 5}, {2, 6}, {3, 7}};
        for (const auto& e : edges)
            ctx.debug.line(corners[e[0]], corners[e[1]], color);
    }

    // The palette: item previews spinning above the anchor palm, hover ring
    // under the hovered one.
    if (paletteHand_ >= 0) {
        const XrPalmPose palm =
            xrPalmPose(ctx.xr.hands[paletteHand_], paletteHand_ == 0);
        if (palm.valid) {
            const Quat spin = Quat::fromAxisAngle(Vec3(0, 1, 0),
                                                  previewSpin_ * 1.2);
            for (int i = 0; i < kItemCount; i++) {
                const Vec3 slot = handWorld(ctx, palm.position)
                    + palm.normal * kPaletteLift
                    + palm.across * ((i - (kItemCount - 1) * 0.5) *
                                     kPaletteSlotSpacing);
                const ItemDef& def = kItems[i];
                // Fit each preview into a ~6cm cell regardless of item size.
                const Real maxDim =
                    2.0 * std::max(def.halfExtent.x,
                                   std::max(def.halfExtent.y,
                                            def.halfExtent.z));
                const Real s = (i == hoverItem_ ? 0.075 : 0.06) / maxDim;
                material.albedo = def.color;
                const Mat4 world = Mat4::translate(slot.x, slot.y, slot.z) *
                                   spin.toMat4() * Mat4::scale(s, s, s);
                ctx.renderer.drawMesh(itemMesh(ctx, i), world, material);
                if (i == hoverItem_) {
                    // Hover ring: a small square of lines under the item.
                    const Vec3 a = palm.across * 0.035;
                    const Vec3 b =
                        cross(palm.normal, palm.across) * 0.035;
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
    }
    objects_.clear();
    heldObject_[0] = heldObject_[1] = -1;
    paletteHand_ = -1;
    hoverItem_ = -1;
    for (auto& mesh : itemMeshes_) {
        if (mesh.valid()) {
            ctx.renderer.removeMesh(mesh);
            mesh = MeshHandle{};
        }
    }
}

}  // namespace engine
